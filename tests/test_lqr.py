"""
Unit tests for pact_controls.controller.lqr.
 
Tests verify the LQR control law is mathematically correct and that the
controller behaves as expected for the key cases required by REQ-CTRL-ARCH-001.
"""
 
import numpy as np
import pytest
 
from pact_controls.controller.lqr import AxisLQRController, make_axis_controllers
 
 
# ---- fixtures ---------------------------------------------------------------
 
@pytest.fixture
def default_gains() -> list[float]:
    """Representative gains — sign is what matters, magnitude is secondary."""
    return [31.62, 4.47]
 
 
@pytest.fixture
def controller(default_gains) -> AxisLQRController:
    return AxisLQRController(default_gains)
 
 
# ---- basic control law ------------------------------------------------------
 
def test_zero_error_produces_zero_output(controller):
    """u = -K(x - x_ref) must be 0 when at setpoint with zero rate."""
    u = controller.compute(
        angle_est_deg=10.0,
        rate_est_deg_s=0.0,
        angle_cmd_deg=10.0,
    )
    assert u == pytest.approx(0.0, abs=1e-9)
 
 
def test_positive_angle_error_produces_negative_torque(controller):
    """
    When the gimbal is above the setpoint (positive error), the corrective
    torque must be negative (push back down).
    u = -K [e, 0]^T  with e > 0 and K_theta > 0 → u < 0
    """
    u = controller.compute(
        angle_est_deg=15.0,
        rate_est_deg_s=0.0,
        angle_cmd_deg=10.0,
    )
    assert u < 0.0
 
 
def test_negative_angle_error_produces_positive_torque(controller):
    """When the gimbal is below the setpoint, corrective torque must be positive."""
    u = controller.compute(
        angle_est_deg=5.0,
        rate_est_deg_s=0.0,
        angle_cmd_deg=10.0,
    )
    assert u > 0.0
 
 
def test_positive_rate_error_produces_negative_torque(controller):
    """
    At setpoint angle but moving away (positive rate), controller must brake.
    x_est = [10, +5],  x_ref = [10, 0]  → rate error = +5 → u < 0
    """
    u = controller.compute(
        angle_est_deg=10.0,
        rate_est_deg_s=5.0,
        angle_cmd_deg=10.0,
    )
    assert u < 0.0
 
 
def test_output_scales_with_error_magnitude(controller):
    """Larger angle error must produce proportionally larger torque."""
    u_small = abs(controller.compute(5.0, 0.0, 0.0))   # 5 deg error
    u_large = abs(controller.compute(10.0, 0.0, 0.0))  # 10 deg error
    assert u_large > u_small
 
 
def test_control_law_formula(default_gains):
    """Manual verification of u = -K(x - x_ref)."""
    K = default_gains
    ctrl = AxisLQRController(K)
 
    angle_est, rate_est, angle_cmd = 12.0, 1.5, 10.0
    expected_u = -(K[0] * (angle_est - angle_cmd) + K[1] * (rate_est - 0.0))
 
    u = ctrl.compute(angle_est, rate_est, angle_cmd)
    assert u == pytest.approx(expected_u, rel=1e-9)
 
 
# ---- gain update ------------------------------------------------------------
 
def test_update_gains_takes_effect_on_next_compute():
    ctrl = AxisLQRController([10.0, 1.0])
    u_before = ctrl.compute(5.0, 0.0, 0.0)
 
    ctrl.update_gains([50.0, 1.0])   # 5× larger K_theta
    u_after = ctrl.compute(5.0, 0.0, 0.0)
 
    assert abs(u_after) > abs(u_before)
 
 
def test_gains_property_reflects_current_k():
    K = [31.62, 4.47]
    ctrl = AxisLQRController(K)
    np.testing.assert_array_almost_equal(ctrl.gains.flatten(), K)
 
 
# ---- make_axis_controllers factory ------------------------------------------
 
def test_make_axis_controllers_loads_from_config():
    cfg = {"lqr_K_pan": [31.62, 4.47], "lqr_K_tilt": [28.0, 3.9]}
    pan_ctrl, tilt_ctrl = make_axis_controllers(cfg)
 
    np.testing.assert_array_almost_equal(pan_ctrl.gains.flatten(),  cfg["lqr_K_pan"])
    np.testing.assert_array_almost_equal(tilt_ctrl.gains.flatten(), cfg["lqr_K_tilt"])
