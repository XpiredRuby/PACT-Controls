"""
Unit tests for pact_controls.fault.detector.
 
Tests verify each fault condition triggers at the correct threshold and that
reset clears state, as required by REQ-CTRL-FAULT-001 through 003.
"""
 
import time
 
import pytest
 
from pact_controls.fault.detector import FaultDetector
from pact_controls.types.state import EncoderReading, FaultCode
 
 
# ---- fixtures ---------------------------------------------------------------
 
@pytest.fixture
def cfg() -> dict:
    return {
        "encoder_timeout_s":    0.5,
        "watchdog_timeout_s":   0.1,
        "thermal_limit_c":     60.0,
    }
 
 
def _valid_encoder(pan_deg: float = 0.0, tilt_deg: float = 0.0) -> EncoderReading:
    return EncoderReading(pan_deg=pan_deg, tilt_deg=tilt_deg, valid=True)
 
 
def _invalid_encoder() -> EncoderReading:
    return EncoderReading(valid=False)
 
 
# ---- nominal operation (no fault) -------------------------------------------
 
def test_no_fault_when_healthy(cfg):
    detector = FaultDetector(cfg)
    code = detector.tick(_valid_encoder(), temperature_c=20.0)
    assert code == FaultCode.NONE
    assert detector.fault_code == FaultCode.NONE
 
 
def test_multiple_healthy_cycles_remain_none(cfg):
    detector = FaultDetector(cfg)
    for _ in range(10):
        code = detector.tick(_valid_encoder(), temperature_c=25.0)
        assert code == FaultCode.NONE
 
 
# ---- REQ-CTRL-FAULT-001: Encoder loss ---------------------------------------
 
def test_single_invalid_reading_does_not_fault(cfg):
    """One bad reading within the timeout window must not trigger ENCODER_LOSS."""
    detector = FaultDetector(cfg)
    code = detector.tick(_invalid_encoder(), temperature_c=20.0)
    assert code == FaultCode.NONE
 
 
def test_encoder_loss_after_timeout(monkeypatch, cfg):
    """
    After encoder_timeout_s of continuously invalid readings, ENCODER_LOSS must fire.
    We monkeypatch time.monotonic to advance time without actually sleeping.
    """
    base_time = [0.0]
 
    def fake_time():
        return base_time[0]
 
    monkeypatch.setattr("pact_controls.fault.detector.time.monotonic", fake_time)
 
    detector = FaultDetector(cfg)
 
    # First tick — invalid but within timeout
    base_time[0] = 0.1
    code = detector.tick(_invalid_encoder(), temperature_c=20.0)
    assert code == FaultCode.NONE
 
    # Advance past encoder_timeout_s (0.5s)
    base_time[0] = 0.6
    code = detector.tick(_invalid_encoder(), temperature_c=20.0)
    assert code == FaultCode.ENCODER_LOSS
 
 
def test_valid_reading_resets_encoder_timer(monkeypatch, cfg):
    """A single valid reading within the timeout window prevents ENCODER_LOSS."""
    base_time = [0.0]
    monkeypatch.setattr("pact_controls.fault.detector.time.monotonic", lambda: base_time[0])
 
    detector = FaultDetector(cfg)
 
    base_time[0] = 0.2
    detector.tick(_invalid_encoder(), temperature_c=20.0)
 
    # Valid reading resets the timer
    base_time[0] = 0.3
    detector.tick(_valid_encoder(), temperature_c=20.0)
 
    # Now advance past 0.5s from the valid reading — still safe
    base_time[0] = 0.7
    code = detector.tick(_invalid_encoder(), temperature_c=20.0)
    assert code == FaultCode.NONE
 
 
# ---- REQ-CTRL-FAULT-002: Watchdog timeout -----------------------------------
 
def test_watchdog_fires_on_large_loop_gap(monkeypatch, cfg):
    """
    If elapsed time between tick() calls exceeds watchdog_timeout_s, WATCHDOG_TIMEOUT fires.
    """
    base_time = [0.0]
    monkeypatch.setattr("pact_controls.fault.detector.time.monotonic", lambda: base_time[0])
 
    detector = FaultDetector(cfg)
 
    # Normal first tick
    base_time[0] = 0.01
    detector.tick(_valid_encoder(), temperature_c=20.0)
 
    # Simulate loop stall: jump 0.2s (> watchdog_timeout_s=0.1s)
    base_time[0] = 0.21
    code = detector.tick(_valid_encoder(), temperature_c=20.0)
    assert code == FaultCode.WATCHDOG_TIMEOUT
 
 
def test_watchdog_ok_for_normal_cycle_rate(monkeypatch, cfg):
    """Normal 50 Hz loop (dt=0.02s) must not trigger the watchdog (timeout=0.1s)."""
    base_time = [0.0]
    monkeypatch.setattr("pact_controls.fault.detector.time.monotonic", lambda: base_time[0])
 
    detector = FaultDetector(cfg)
    for i in range(20):
        base_time[0] = i * 0.02
        code = detector.tick(_valid_encoder(), temperature_c=20.0)
        assert code == FaultCode.NONE
 
 
# ---- REQ-CTRL-FAULT-003: Over-temperature -----------------------------------
 
def test_over_temperature_fault(cfg):
    detector = FaultDetector(cfg)
    code = detector.tick(_valid_encoder(), temperature_c=61.0)  # > 60°C limit
    assert code == FaultCode.OVER_TEMPERATURE
 
 
def test_at_thermal_limit_no_fault(cfg):
    """Exactly at the limit must not fault (strict greater-than)."""
    detector = FaultDetector(cfg)
    code = detector.tick(_valid_encoder(), temperature_c=60.0)
    assert code == FaultCode.NONE
 
 
def test_temperature_returns_to_normal_clears_fault(cfg):
    """Once temperature drops, the next tick must return NONE."""
    detector = FaultDetector(cfg)
    detector.tick(_valid_encoder(), temperature_c=65.0)
    assert detector.fault_code == FaultCode.OVER_TEMPERATURE
 
    code = detector.tick(_valid_encoder(), temperature_c=50.0)
    assert code == FaultCode.NONE
 
 
# ---- reset ------------------------------------------------------------------
 
def test_reset_clears_fault_code(cfg):
    detector = FaultDetector(cfg)
    detector.tick(_valid_encoder(), temperature_c=70.0)
    assert detector.fault_code != FaultCode.NONE
 
    detector.reset()
    assert detector.fault_code == FaultCode.NONE
 
 
def test_reset_allows_clean_cycle_after_fault(cfg):
    """After reset, a healthy tick must return NONE."""
    detector = FaultDetector(cfg)
    detector.tick(_valid_encoder(), temperature_c=70.0)
    detector.reset()
 
    code = detector.tick(_valid_encoder(), temperature_c=20.0)
    assert code == FaultCode.NONE
