"""
Unit tests for pact_controls.safety.gates.
 
Tests verify each of the five safety gates triggers at the right threshold
and that gate outputs are correct for REQ-CTRL-SAFE-001 through 005.
"""
 
import pytest
 
from pact_controls.safety.gates import run_gates
from pact_controls.types.state import (
    ControlCommand,
    ControlOutput,
    GimbalMode,
    GimbalState,
)
 
 
# ---- fixtures ---------------------------------------------------------------
 
@pytest.fixture
def cfg() -> dict:
    return {
        "hard_pan_limit_deg":       180.0,
        "hard_tilt_min_deg":        -90.0,
        "hard_tilt_max_deg":         90.0,
        "soft_limit_buffer_deg":      5.0,
        "scan_slew_rate_deg_per_s":   0.5,
        "max_slew_deg_s":             0.0,   # disabled
    }
 
 
def _make_inputs(
    pan_cmd=0.0, tilt_cmd=0.0, mode=GimbalMode.IDLE,
    pan_torque=0.1, tilt_torque=0.1,
    pan_deg=0.0, tilt_deg=0.0, pan_rate=0.0, tilt_rate=0.0,
):
    cmd = ControlCommand(pan_cmd_deg=pan_cmd, tilt_cmd_deg=tilt_cmd, mode=mode)
    out = ControlOutput(pan_torque=pan_torque, tilt_torque=tilt_torque)
    state = GimbalState(pan_deg=pan_deg, tilt_deg=tilt_deg,
                        pan_rate_deg_s=pan_rate, tilt_rate_deg_s=tilt_rate)
    return cmd, out, state
 
 
# ---- Gate 1: Hard angle limits (REQ-CTRL-SAFE-001) --------------------------
 
def test_gate1_no_violation_within_limits(cfg):
    cmd, out, state = _make_inputs(pan_deg=90.0, tilt_deg=45.0)
    result = run_gates(cmd, out, state, cfg)
    assert not result.hard_limit_violated
 
 
def test_gate1_pan_violation_zeroes_pan_torque(cfg):
    cmd, out, state = _make_inputs(pan_deg=181.0)  # exceeds ±180
    result = run_gates(cmd, out, state, cfg)
    assert result.hard_limit_violated
    assert result.output.pan_torque == pytest.approx(0.0)
 
 
def test_gate1_tilt_violation_zeroes_tilt_torque(cfg):
    cmd, out, state = _make_inputs(tilt_deg=-91.0)  # below −90
    result = run_gates(cmd, out, state, cfg)
    assert result.hard_limit_violated
    assert result.output.tilt_torque == pytest.approx(0.0)
 
 
def test_gate1_violation_forces_safe_mode(cfg):
    cmd, out, state = _make_inputs(pan_deg=190.0)
    result = run_gates(cmd, out, state, cfg)
    assert result.command.mode == GimbalMode.SAFE
 
 
def test_gate1_violation_skips_remaining_gates(cfg):
    """When gate 1 fires, the output must NOT be further modified by later gates."""
    cmd, out, state = _make_inputs(pan_deg=190.0, pan_torque=5.0)  # torque > 1
    result = run_gates(cmd, out, state, cfg)
    # Gate 1 zeroed pan torque — gate 5 should not re-clamp to ±1 on top of 0
    assert result.output.pan_torque == pytest.approx(0.0)
 
 
# ---- Gate 2: Soft limit warning ---------------------------------------------
 
def test_gate2_warned_inside_soft_zone(cfg):
    # hard=180, buffer=5 → soft boundary at 175
    cmd, out, state = _make_inputs(pan_deg=176.0)
    result = run_gates(cmd, out, state, cfg)
    assert result.soft_limit_warned
    assert not result.hard_limit_violated  # warning only, not fault
 
 
def test_gate2_not_warned_outside_soft_zone(cfg):
    cmd, out, state = _make_inputs(pan_deg=100.0)
    result = run_gates(cmd, out, state, cfg)
    assert not result.soft_limit_warned
 
 
# ---- Gate 3: Scan mode slew rate lock (REQ-CTRL-SAFE-003) -------------------
 
def test_gate3_scan_rate_exceeded_reduces_torque(cfg):
    cmd, out, state = _make_inputs(
        mode=GimbalMode.SCAN,
        pan_torque=1.0,
        pan_rate=2.0,   # 2.0 > 0.5 deg/s limit
    )
    result = run_gates(cmd, out, state, cfg)
    assert result.rate_limited
    # Torque must be reduced by factor 0.5/2.0 = 0.25
    assert result.output.pan_torque == pytest.approx(0.25, abs=1e-6)
 
 
def test_gate3_scan_rate_within_limit_no_change(cfg):
    cmd, out, state = _make_inputs(
        mode=GimbalMode.SCAN,
        pan_torque=0.5,
        pan_rate=0.3,   # within 0.5 deg/s
    )
    result = run_gates(cmd, out, state, cfg)
    assert not result.rate_limited
    assert result.output.pan_torque == pytest.approx(0.5)
 
 
def test_gate3_rate_limit_not_applied_in_tracking_mode(cfg):
    """Gate 3 is SCAN mode only — TRACKING mode must not trigger it."""
    cmd, out, state = _make_inputs(
        mode=GimbalMode.TRACKING,
        pan_torque=0.8,
        pan_rate=5.0,   # high rate, but gate 3 should not fire
    )
    result = run_gates(cmd, out, state, cfg)
    # max_slew_deg_s=0 so gate 4 is also off; result.rate_limited must be False
    assert not result.rate_limited
 
 
# ---- Gate 4: General max slew cap (REQ-CTRL-SAFE-002) -----------------------
 
def test_gate4_disabled_when_max_slew_zero(cfg):
    """max_slew_deg_s=0 means gate 4 is disabled."""
    cmd, out, state = _make_inputs(pan_torque=0.9, pan_rate=100.0)
    result = run_gates(cmd, out, state, cfg)
    assert not result.rate_limited
 
 
def test_gate4_active_when_max_slew_set(cfg):
    cfg_with_slew = {**cfg, "max_slew_deg_s": 10.0}
    cmd, out, state = _make_inputs(pan_torque=1.0, pan_rate=20.0)  # 20 > 10
    result = run_gates(cmd, out, state, cfg_with_slew)
    assert result.rate_limited
    assert result.output.pan_torque == pytest.approx(0.5, abs=1e-6)  # 10/20
 
 
# ---- Gate 5: Output saturation clamp (REQ-CTRL-SAFE-005) --------------------
 
def test_gate5_clamps_torque_above_one(cfg):
    cmd, out, state = _make_inputs(pan_torque=3.5, tilt_torque=0.5)
    result = run_gates(cmd, out, state, cfg)
    assert result.output_saturated
    assert result.output.pan_torque == pytest.approx(1.0)
    assert result.output.tilt_torque == pytest.approx(0.5)
 
 
def test_gate5_clamps_torque_below_minus_one(cfg):
    cmd, out, state = _make_inputs(pan_torque=-2.0)
    result = run_gates(cmd, out, state, cfg)
    assert result.output_saturated
    assert result.output.pan_torque == pytest.approx(-1.0)
 
 
def test_gate5_no_saturation_within_limits(cfg):
    cmd, out, state = _make_inputs(pan_torque=0.8, tilt_torque=-0.6)
    result = run_gates(cmd, out, state, cfg)
    assert not result.output_saturated
    assert result.output.pan_torque  == pytest.approx(0.8)
    assert result.output.tilt_torque == pytest.approx(-0.6)
