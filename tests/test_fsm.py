"""
Unit tests for pact_controls.modes.fsm.
 
Tests verify all state transitions in the GimbalFSM against the transition
table defined in modes/fsm.py and required by REQ-CTRL-ARCH-003.
"""
 
import pytest
 
from pact_controls.modes.fsm import GimbalFSM
from pact_controls.types.state import FaultCode, GimbalMode
 
 
# ---- fixtures ---------------------------------------------------------------
 
@pytest.fixture
def cfg() -> dict:
    return {"scan_entry_idle_seconds": 5.0}
 
 
@pytest.fixture
def fsm(cfg) -> GimbalFSM:
    return GimbalFSM(cfg)
 
 
# ---- initial state ----------------------------------------------------------
 
def test_initial_mode_is_idle(fsm):
    assert fsm.mode == GimbalMode.IDLE
 
 
# ---- IDLE transitions -------------------------------------------------------
 
def test_idle_to_tracking_on_tracker_cmd(fsm):
    fsm.update(FaultCode.NONE, has_tracker_cmd=True, has_scan_plan=False)
    assert fsm.mode == GimbalMode.TRACKING
 
 
def test_idle_to_scan_on_scan_plan(fsm):
    fsm.update(FaultCode.NONE, has_tracker_cmd=False, has_scan_plan=True)
    assert fsm.mode == GimbalMode.SCAN
 
 
def test_idle_stays_idle_without_inputs(fsm):
    fsm.update(FaultCode.NONE, has_tracker_cmd=False, has_scan_plan=False)
    assert fsm.mode == GimbalMode.IDLE
 
 
def test_tracking_takes_priority_over_scan_from_idle(fsm):
    """When both tracker and scan plan are available, TRACKING wins."""
    fsm.update(FaultCode.NONE, has_tracker_cmd=True, has_scan_plan=True)
    assert fsm.mode == GimbalMode.TRACKING
 
 
# ---- SCAN transitions -------------------------------------------------------
 
def test_scan_to_tracking_on_detection(fsm):
    # Enter SCAN
    fsm.update(FaultCode.NONE, has_tracker_cmd=False, has_scan_plan=True)
    assert fsm.mode == GimbalMode.SCAN
 
    # Detection arrives mid-scan
    fsm.update(FaultCode.NONE, has_tracker_cmd=True, has_scan_plan=True)
    assert fsm.mode == GimbalMode.TRACKING
 
 
# ---- TRACKING transitions ---------------------------------------------------
 
def test_tracking_stays_tracking_with_active_detection(fsm):
    fsm.update(FaultCode.NONE, has_tracker_cmd=True, has_scan_plan=False)
    fsm.update(FaultCode.NONE, has_tracker_cmd=True, has_scan_plan=False)
    assert fsm.mode == GimbalMode.TRACKING
 
 
def test_tracking_to_idle_after_timeout_no_scan(monkeypatch, fsm):
    """
    When detection is lost and no scan plan exists, TRACKING → IDLE after idle timeout.
    """
    base_time = [0.0]
    monkeypatch.setattr("pact_controls.modes.fsm.time.monotonic", lambda: base_time[0])
 
    # Enter TRACKING
    base_time[0] = 0.0
    fsm.update(FaultCode.NONE, has_tracker_cmd=True, has_scan_plan=False)
 
    # Detection lost — advance past idle timeout (5s)
    base_time[0] = 6.0
    fsm.update(FaultCode.NONE, has_tracker_cmd=False, has_scan_plan=False)
    assert fsm.mode == GimbalMode.IDLE
 
 
def test_tracking_to_scan_after_timeout_with_scan(monkeypatch, fsm):
    """When detection is lost but a scan plan exists, TRACKING → SCAN."""
    base_time = [0.0]
    monkeypatch.setattr("pact_controls.modes.fsm.time.monotonic", lambda: base_time[0])
 
    base_time[0] = 0.0
    fsm.update(FaultCode.NONE, has_tracker_cmd=True, has_scan_plan=True)
 
    base_time[0] = 6.0
    fsm.update(FaultCode.NONE, has_tracker_cmd=False, has_scan_plan=True)
    assert fsm.mode == GimbalMode.SCAN
 
 
def test_tracking_does_not_leave_before_timeout(monkeypatch, fsm):
    """Detection loss within the timeout window must NOT change the mode."""
    base_time = [0.0]
    monkeypatch.setattr("pact_controls.modes.fsm.time.monotonic", lambda: base_time[0])
 
    base_time[0] = 0.0
    fsm.update(FaultCode.NONE, has_tracker_cmd=True, has_scan_plan=False)
 
    # Only 2s elapsed — still inside 5s timeout
    base_time[0] = 2.0
    fsm.update(FaultCode.NONE, has_tracker_cmd=False, has_scan_plan=False)
    assert fsm.mode == GimbalMode.TRACKING
 
 
# ---- SAFE transitions (REQ-CTRL-FAULT-*) ------------------------------------
 
def test_any_fault_transitions_to_safe_from_idle(fsm):
    fsm.update(FaultCode.ENCODER_LOSS, has_tracker_cmd=False, has_scan_plan=False)
    assert fsm.mode == GimbalMode.SAFE
 
 
def test_any_fault_transitions_to_safe_from_tracking(fsm):
    fsm.update(FaultCode.NONE, has_tracker_cmd=True, has_scan_plan=False)
    fsm.update(FaultCode.WATCHDOG_TIMEOUT, has_tracker_cmd=True, has_scan_plan=False)
    assert fsm.mode == GimbalMode.SAFE
 
 
def test_any_fault_transitions_to_safe_from_scan(fsm):
    fsm.update(FaultCode.NONE, has_tracker_cmd=False, has_scan_plan=True)
    fsm.update(FaultCode.OVER_TEMPERATURE, has_tracker_cmd=False, has_scan_plan=True)
    assert fsm.mode == GimbalMode.SAFE
 
 
def test_safe_mode_is_sticky_without_ground_reset(fsm):
    """SAFE must persist across cycles even when fault clears, until ground resets."""
    fsm.update(FaultCode.ENCODER_LOSS, has_tracker_cmd=False, has_scan_plan=False)
    assert fsm.mode == GimbalMode.SAFE
 
    # Fault clears — no ground reset → still SAFE
    fsm.update(FaultCode.NONE, has_tracker_cmd=True, has_scan_plan=True)
    assert fsm.mode == GimbalMode.SAFE
 
 
def test_safe_to_idle_on_ground_reset(fsm):
    """Only ground_reset=True can exit SAFE mode."""
    fsm.update(FaultCode.ENCODER_LOSS, has_tracker_cmd=False, has_scan_plan=False)
    assert fsm.mode == GimbalMode.SAFE
 
    fsm.update(FaultCode.NONE, has_tracker_cmd=False, has_scan_plan=False, ground_reset=True)
    assert fsm.mode == GimbalMode.IDLE
 
 
def test_ground_reset_without_prior_safe_stays_idle(fsm):
    """ground_reset=True on a non-SAFE state must not cause unexpected transitions."""
    fsm.update(FaultCode.NONE, has_tracker_cmd=False, has_scan_plan=False, ground_reset=True)
    assert fsm.mode == GimbalMode.IDLE
 
 
def test_fault_during_safe_keeps_safe(fsm):
    """Additional faults while already in SAFE must not change the mode."""
    fsm.update(FaultCode.ENCODER_LOSS, has_tracker_cmd=False, has_scan_plan=False)
    fsm.update(FaultCode.OVER_TEMPERATURE, has_tracker_cmd=False, has_scan_plan=False)
    assert fsm.mode == GimbalMode.SAFE
