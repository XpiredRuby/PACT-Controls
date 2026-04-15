"""
Safety gate enforcement for the gimbal controller.
 
Five gates are applied in sequence on every control cycle, after the LQR
computes a torque output but BEFORE the output reaches the gimbal HAL.
All gates are pure functions — no side effects, no stored state.
 
Gate order (must not be changed — later gates depend on earlier ones):
 
  1. Hard angle limit       — if current state exceeds hard limits, zero output on
                              the violated axis and flag for SAFE transition (REQ-CTRL-SAFE-001)
  2. Soft angle limit       — if within the soft-buffer zone, set a warning flag
                              (no command change, just a telemetry/log signal)
  3. Scan mode rate lock    — in SCAN mode, reduce torque proportionally to keep
                              estimated rate ≤ 0.5 deg/s (REQ-CTRL-SAFE-003)
  4. General max slew cap   — in any mode, cap rate to max_slew_deg_s if configured
                              (REQ-CTRL-SAFE-002; gate is disabled when max_slew_deg_s = 0)
  5. Output saturation clamp — clamp pan and tilt torque to [-1.0, 1.0] and flag
                              if clamping occurred (REQ-CTRL-SAFE-005)
 
Entry point: run_gates()
"""
 
from __future__ import annotations
 
import time
from dataclasses import dataclass, field
 
from pact_controls.types.state import ControlCommand, ControlOutput, GimbalMode, GimbalState
 
 
@dataclass
class SafetyResult:
    """
    Outcome of running all five safety gates for one cycle.
 
    The controller always uses result.command and result.output — the raw inputs
    may have been modified. The boolean flags are written to the telemetry frame.
    """
    command:             ControlCommand
    output:              ControlOutput
    hard_limit_violated: bool = False
    soft_limit_warned:   bool = False
    rate_limited:        bool = False
    output_saturated:    bool = False
 
 
def run_gates(
    cmd:    ControlCommand,
    output: ControlOutput,
    state:  GimbalState,
    cfg:    dict,
) -> SafetyResult:
    """
    Apply all five safety gates and return the (possibly modified) result.
 
    Args:
        cmd:    Proposed command from the arbiter (angles in degrees).
        output: Proposed LQR torque output (normalized, may be outside [-1,1]).
        state:  Current Kalman-estimated gimbal state.
        cfg:    Flat merged config dict.
 
    Returns:
        SafetyResult — always use result.command and result.output downstream.
    """
    result = SafetyResult(command=cmd, output=output)
 
    # ----------------------------------------------------------------
    # Gate 1: Hard angle limits (REQ-CTRL-SAFE-001)
    # ----------------------------------------------------------------
    hard_pan   = cfg["hard_pan_limit_deg"]
    hard_t_min = cfg["hard_tilt_min_deg"]
    hard_t_max = cfg["hard_tilt_max_deg"]
 
    pan_violated  = abs(state.pan_deg)  > hard_pan
    tilt_violated = not (hard_t_min <= state.tilt_deg <= hard_t_max)
 
    if pan_violated or tilt_violated:
        result.hard_limit_violated = True
 
        # Clamp command to prevent integral windup on the violated axis
        safe_pan  = max(-hard_pan,  min(hard_pan,  cmd.pan_cmd_deg))
        safe_tilt = max(hard_t_min, min(hard_t_max, cmd.tilt_cmd_deg))
        result.command = ControlCommand(
            pan_cmd_deg=safe_pan,
            tilt_cmd_deg=safe_tilt,
            mode=GimbalMode.SAFE,
            timestamp_s=cmd.timestamp_s,
        )
 
        # Zero torque on the violated axis to stop motion immediately
        result.output = ControlOutput(
            pan_torque=0.0 if pan_violated  else output.pan_torque,
            tilt_torque=0.0 if tilt_violated else output.tilt_torque,
            timestamp_s=output.timestamp_s,
        )
        # Skip remaining gates — SAFE transition is required
        return result
 
    # ----------------------------------------------------------------
    # Gate 2: Soft angle limit warning
    # ----------------------------------------------------------------
    buf       = cfg["soft_limit_buffer_deg"]
    soft_pan  = hard_pan   - buf
    soft_tmin = hard_t_min + buf
    soft_tmax = hard_t_max - buf
 
    result.soft_limit_warned = (
        abs(state.pan_deg) > soft_pan
        or not (soft_tmin <= state.tilt_deg <= soft_tmax)
    )
 
    # ----------------------------------------------------------------
    # Gate 3: Scan mode slew rate lock (REQ-CTRL-SAFE-003)
    # ----------------------------------------------------------------
    if result.command.mode == GimbalMode.SCAN:
        scan_limit = cfg["scan_slew_rate_deg_per_s"]  # 0.5 deg/s
 
        pan_r  = abs(state.pan_rate_deg_s)
        tilt_r = abs(state.tilt_rate_deg_s)
 
        if pan_r > scan_limit or tilt_r > scan_limit:
            result.rate_limited = True
            if pan_r > scan_limit and pan_r > 0.0:
                result.output.pan_torque  *= scan_limit / pan_r
            if tilt_r > scan_limit and tilt_r > 0.0:
                result.output.tilt_torque *= scan_limit / tilt_r
 
    # ----------------------------------------------------------------
    # Gate 4: General max slew rate cap (REQ-CTRL-SAFE-002)
    # max_slew_deg_s = 0 disables this gate until hardware limits are known
    # ----------------------------------------------------------------
    max_slew = cfg.get("max_slew_deg_s", 0.0)
    if max_slew > 0.0:
        pan_r  = abs(state.pan_rate_deg_s)
        tilt_r = abs(state.tilt_rate_deg_s)
 
        if pan_r > max_slew or tilt_r > max_slew:
            result.rate_limited = True
            if pan_r > max_slew and pan_r > 0.0:
                result.output.pan_torque  *= max_slew / pan_r
            if tilt_r > max_slew and tilt_r > 0.0:
                result.output.tilt_torque *= max_slew / tilt_r
 
    # ----------------------------------------------------------------
    # Gate 5: Output saturation clamp (REQ-CTRL-SAFE-005)
    # ----------------------------------------------------------------
    raw_pan  = result.output.pan_torque
    raw_tilt = result.output.tilt_torque
    clamped_pan  = max(-1.0, min(1.0, raw_pan))
    clamped_tilt = max(-1.0, min(1.0, raw_tilt))
 
    saturated = (clamped_pan != raw_pan) or (clamped_tilt != raw_tilt)
    result.output_saturated    = saturated
    result.output.pan_torque   = clamped_pan
    result.output.tilt_torque  = clamped_tilt
    result.output.saturated    = saturated
 
    return result
