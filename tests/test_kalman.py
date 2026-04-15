"""
Unit tests for pact_controls.controller.kalman.
 
Tests verify the filter's predict/update mechanics, covariance behavior,
and convergence properties required by REQ-CTRL-ARCH-002.
"""
 
import numpy as np
import pytest
 
from pact_controls.controller.kalman import AxisKalmanFilter, make_axis_filters
 
 
# ---- fixtures ---------------------------------------------------------------
 
DT  = 0.02     # 50 Hz
J   = 0.01     # kg·m²
Q   = 1e-4     # process noise variance
R   = 0.01     # measurement noise variance
 
 
@pytest.fixture
def kf() -> AxisKalmanFilter:
    f = AxisKalmanFilter(dt_s=DT, moment_of_inertia=J, process_noise_var=Q, meas_noise_var=R)
    f.reset(initial_angle_deg=0.0)
    return f
 
 
# ---- initialization ---------------------------------------------------------
 
def test_initial_state_is_zero(kf):
    assert kf.angle_deg   == pytest.approx(0.0)
    assert kf.rate_deg_s  == pytest.approx(0.0)
 
 
def test_reset_sets_state():
    kf = AxisKalmanFilter(DT, J, Q, R)
    kf.reset(initial_angle_deg=45.0, initial_rate_deg_s=2.0)
    assert kf.angle_deg  == pytest.approx(45.0)
    assert kf.rate_deg_s == pytest.approx(2.0)
 
 
# ---- predict step -----------------------------------------------------------
 
def test_predict_increases_covariance(kf):
    """Without a measurement, uncertainty (P trace) must grow each cycle."""
    trace_before = np.trace(kf.covariance)
    kf.predict(torque_input=0.0)
    trace_after = np.trace(kf.covariance)
    assert trace_after > trace_before
 
 
def test_predict_propagates_angle_from_rate(kf):
    """
    With a known initial rate and zero torque, the angle must advance by rate * dt.
    """
    kf.reset(initial_angle_deg=10.0, initial_rate_deg_s=5.0)
    kf.predict(torque_input=0.0)
    expected_angle = 10.0 + 5.0 * DT
    assert kf.angle_deg == pytest.approx(expected_angle, abs=1e-9)
 
 
def test_predict_torque_advances_rate(kf):
    """Nonzero torque input must change the estimated rate after predict."""
    kf.reset(initial_angle_deg=0.0, initial_rate_deg_s=0.0)
    kf.predict(torque_input=1.0)
    # alpha = torque_scale / J; rate change = alpha * dt = 1.0/0.01 * 0.02 = 2.0 deg/s
    expected_rate = (1.0 / J) * DT
    assert kf.rate_deg_s == pytest.approx(expected_rate, abs=1e-9)
 
 
# ---- update step ------------------------------------------------------------
 
def test_update_decreases_covariance(kf):
    """After predict, an update must reduce covariance (P trace shrinks)."""
    kf.predict(torque_input=0.0)
    trace_pred = np.trace(kf.covariance)
    kf.update(encoder_angle_deg=0.0)
    trace_upd = np.trace(kf.covariance)
    assert trace_upd < trace_pred
 
 
def test_update_pulls_estimate_toward_measurement(kf):
    """
    If the predicted angle is 5° but the encoder reads 0°, the updated
    estimate should move toward 0°.
    """
    kf.reset(initial_angle_deg=5.0)
    kf.predict(torque_input=0.0)
    angle_before_update = kf.angle_deg
    kf.update(encoder_angle_deg=0.0)
    # Updated angle must be closer to 0 than before the update
    assert abs(kf.angle_deg) < abs(angle_before_update)
 
 
# ---- convergence ------------------------------------------------------------
 
def test_filter_converges_to_true_angle_over_many_cycles():
    """
    Run 200 predict+update cycles on a stationary target.
    The filter should converge to within 0.5° of truth.
    """
    true_angle = 30.0
    kf = AxisKalmanFilter(DT, J, Q, R)
    kf.reset(initial_angle_deg=0.0)   # Start from wrong initial condition
 
    rng = np.random.default_rng(seed=0)
    for _ in range(200):
        kf.predict(torque_input=0.0)
        noisy_measurement = true_angle + rng.normal(0, np.sqrt(R))
        kf.update(noisy_measurement)
 
    assert abs(kf.angle_deg - true_angle) < 0.5
 
 
# ---- skip update when encoder invalid (predict-only mode) -------------------
 
def test_predict_only_mode_covariance_grows_monotonically():
    """
    When the encoder is lost, we run predict only. Covariance must grow every cycle.
    This models increasing uncertainty during ENCODER_LOSS.
    """
    kf = AxisKalmanFilter(DT, J, Q, R)
    kf.reset()
    traces = []
    for _ in range(10):
        kf.predict(torque_input=0.0)
        traces.append(np.trace(kf.covariance))
    assert all(traces[i] < traces[i + 1] for i in range(len(traces) - 1))
 
 
# ---- factory ----------------------------------------------------------------
 
def test_make_axis_filters_creates_two_independent_filters():
    cfg = {
        "control_loop_hz":                50.0,
        "kalman_process_noise_var":        1e-4,
        "kalman_meas_noise_var":           0.01,
        "moment_of_inertia_pan_kg_m2":    0.01,
        "moment_of_inertia_tilt_kg_m2":   0.015,
    }
    kf_pan, kf_tilt = make_axis_filters(cfg)
 
    # Independently advance one axis; the other must not change
    kf_pan.predict(1.0)
    kf_pan.update(5.0)
 
    assert kf_tilt.angle_deg  == pytest.approx(0.0)
    assert kf_tilt.rate_deg_s == pytest.approx(0.0)
