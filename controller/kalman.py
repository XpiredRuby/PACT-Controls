"""
Discrete-time Kalman filter for single-axis gimbal state estimation.
 
Implements REQ-CTRL-ARCH-002: estimate full axis state [theta, theta_dot] from
encoder measurements (which only observe theta directly).
 
One AxisKalmanFilter is created per axis. The two axes are fully decoupled
(REQ-CTRL-ARCH-003) — there are no cross-axis terms in either filter.
 
Predict–update cycle (called every control cycle):
    1. kf.predict(torque_input)  — propagate state forward using system model
    2. kf.update(encoder_deg)    — correct prediction with encoder measurement
 
State vector:  x = [theta (deg), theta_dot (deg/s)]    shape (2, 1)
Measurement:   z = [theta_encoder (deg)]               shape (1, 1)
"""
 
from __future__ import annotations
 
import numpy as np
 
 
class AxisKalmanFilter:
    """
    Predict–update Kalman filter for one gimbal axis.
 
    Noise parameters:
        process_noise_var  — models unmodeled dynamics (vibration, structural flex).
                             Larger → filter trusts model less, reacts faster to measurement.
        meas_noise_var     — encoder quantization and electrical noise.
                             Should be set from encoder datasheet (CTRL-TODO-HAL-003).
    """
 
    def __init__(
        self,
        dt_s: float,
        moment_of_inertia: float,
        process_noise_var: float,
        meas_noise_var: float,
    ) -> None:
        """
        Args:
            dt_s:               Control loop sample period (seconds).
            moment_of_inertia:  Axis inertia J (kg·m²). TBD pending hardware selection.
            process_noise_var:  Scalar variance for Q diagonal. Same value for both
                                state components (scaled 1x for theta, 10x for theta_dot).
            meas_noise_var:     Scalar variance for encoder measurement R.
        """
        J = moment_of_inertia
 
        # Discrete-time system matrices — same derivation as gains.build_state_space()
        self._A = np.array([[1.0, dt_s],
                            [0.0,  1.0]])
        self._B = np.array([[0.5 * dt_s**2 / J],
                            [      dt_s / J   ]])
        self._C = np.array([[1.0, 0.0]])  # Only angle is directly measured
 
        # Process noise: angle and rate components scaled separately
        self._Q = np.diag([process_noise_var, process_noise_var * 10.0])
 
        # Measurement noise: single encoder
        self._R = np.array([[meas_noise_var]])
 
        # State estimate [theta; theta_dot] and error covariance P
        self._x = np.zeros((2, 1))
        self._P = np.eye(2)
 
    def reset(
        self,
        initial_angle_deg: float = 0.0,
        initial_rate_deg_s: float = 0.0,
    ) -> None:
        """Initialize the filter from a known starting state (e.g., after homing)."""
        self._x = np.array([[initial_angle_deg],
                             [initial_rate_deg_s]])
        self._P = np.eye(2)
 
    def predict(self, torque_input: float) -> tuple[np.ndarray, np.ndarray]:
        """
        Prediction step: propagate state one cycle forward using the system model.
 
        Args:
            torque_input: Normalized torque applied last cycle [-1, 1].
                          The HAL applies a hardware-specific scale factor (TBD).
 
        Returns:
            (x_pred, P_pred): Predicted state (2,1) and covariance (2,2).
        """
        u = np.array([[torque_input]])
        self._x = self._A @ self._x + self._B @ u
        self._P = self._A @ self._P @ self._A.T + self._Q
        return self._x.copy(), self._P.copy()
 
    def update(self, encoder_angle_deg: float) -> tuple[np.ndarray, np.ndarray]:
        """
        Update step: correct the prediction with the latest encoder measurement.
 
        Args:
            encoder_angle_deg: Raw angle reading from the encoder (degrees).
                               Call this only when EncoderReading.valid is True.
                               When the encoder is invalid, skip update() and run
                               predict() only — the filter covariance will grow,
                               signalling increasing uncertainty to the controller.
 
        Returns:
            (x_est, P_est): Updated state estimate (2,1) and covariance (2,2).
        """
        z = np.array([[encoder_angle_deg]])
        y = z - self._C @ self._x                         # Innovation
        S = self._C @ self._P @ self._C.T + self._R       # Innovation covariance
        K = self._P @ self._C.T @ np.linalg.inv(S)        # Kalman gain
        self._x = self._x + K @ y                         # Corrected state
        self._P = (np.eye(2) - K @ self._C) @ self._P     # Corrected covariance
        return self._x.copy(), self._P.copy()
 
    @property
    def angle_deg(self) -> float:
        """Current estimated angle (degrees)."""
        return float(self._x[0, 0])
 
    @property
    def rate_deg_s(self) -> float:
        """Current estimated angular rate (degrees/second)."""
        return float(self._x[1, 0])
 
    @property
    def covariance(self) -> np.ndarray:
        """Current 2×2 error covariance matrix (diagnostic use)."""
        return self._P.copy()
 
 
def make_axis_filters(cfg: dict) -> tuple[AxisKalmanFilter, AxisKalmanFilter]:
    """
    Construct pan and tilt Kalman filters from the merged flat config dict.
 
    Config keys consumed:
        control_loop_hz, kalman_process_noise_var, kalman_meas_noise_var,
        moment_of_inertia_pan_kg_m2, moment_of_inertia_tilt_kg_m2
 
    Returns:
        (kf_pan, kf_tilt)
    """
    dt_s   = 1.0 / cfg["control_loop_hz"]
    q      = cfg["kalman_process_noise_var"]
    r      = cfg["kalman_meas_noise_var"]
    J_pan  = cfg.get("moment_of_inertia_pan_kg_m2",  0.01)
    J_tilt = cfg.get("moment_of_inertia_tilt_kg_m2", 0.01)
 
    return (
        AxisKalmanFilter(dt_s, J_pan,  q, r),
        AxisKalmanFilter(dt_s, J_tilt, q, r),
    )
