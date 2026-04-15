"""
Per-axis LQR state-feedback controller.
 
Implements REQ-CTRL-ARCH-001: closed-loop gimbal pointing via LQR.
 
Control law (per axis):
    u_k = -K (x_k - x_ref)
 
where:
    x_k   = [theta_est, theta_dot_est]  from Kalman filter
    x_ref = [theta_cmd, 0.0]            desired angle; desired rate at setpoint = 0
    K     = [K_theta, K_theta_dot]      pre-computed offline via DARE (gains.py)
    u_k   = normalized torque           clamped to [-1, 1] by safety gate 5
 
Both axes (pan, tilt) use separate AxisLQRController instances with separate K vectors.
Gains are loaded from config at startup — never computed inside the control loop.
"""
 
from __future__ import annotations
 
import numpy as np
 
 
class AxisLQRController:
    """
    Single-axis LQR state-feedback controller.
 
    Instantiate one for pan and one for tilt. The two are fully independent.
    """
 
    def __init__(self, K: list[float] | np.ndarray) -> None:
        """
        Args:
            K: 1×2 gain vector [K_theta, K_theta_dot].
               Load from config (lqr_K_pan or lqr_K_tilt).
               Recompute offline with scripts/compute_gains.py when plant params change.
        """
        self._K = np.asarray(K, dtype=float).reshape(1, 2)
 
    def compute(
        self,
        angle_est_deg: float,
        rate_est_deg_s: float,
        angle_cmd_deg: float,
    ) -> float:
        """
        Compute normalized torque command for one axis, one cycle.
 
        Args:
            angle_est_deg:  Kalman-estimated angle (degrees).
            rate_est_deg_s: Kalman-estimated angular rate (degrees/second).
            angle_cmd_deg:  Reference (commanded) angle (degrees).
 
        Returns:
            Raw torque u in (-inf, +inf).
            Safety gate 5 in safety/gates.py clamps this to [-1, 1] before HAL delivery.
        """
        x_est = np.array([[angle_est_deg],
                          [rate_est_deg_s]])
        x_ref = np.array([[angle_cmd_deg],
                          [0.0]])           # Zero desired rate at the setpoint
        error = x_est - x_ref
        return float(-(self._K @ error))
 
    def update_gains(self, K: list[float] | np.ndarray) -> None:
        """
        Replace the gain vector at runtime (e.g., after an uplink config update).
 
        Note: This is NOT thread-safe. Acquire a lock before calling if the control
        loop runs on a separate thread.
        """
        self._K = np.asarray(K, dtype=float).reshape(1, 2)
 
    @property
    def gains(self) -> np.ndarray:
        """Current K vector, shape (1, 2)."""
        return self._K.copy()
 
 
def make_axis_controllers(
    cfg: dict,
) -> tuple[AxisLQRController, AxisLQRController]:
    """
    Construct pan and tilt LQR controllers from the flat merged config dict.
 
    Config keys consumed:
        lqr_K_pan   — [K_theta, K_theta_dot] for pan axis
        lqr_K_tilt  — [K_theta, K_theta_dot] for tilt axis
 
    Returns:
        (lqr_pan, lqr_tilt)
    """
    return (
        AxisLQRController(cfg["lqr_K_pan"]),
        AxisLQRController(cfg["lqr_K_tilt"]),
    )
