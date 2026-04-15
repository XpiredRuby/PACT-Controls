"""
Offline LQR gain computation via the Discrete Algebraic Riccati Equation (DARE).
 
Run this module as a script (or via scripts/compute_gains.py) whenever you change
the Q/R weights or the plant inertia values. Paste the printed K gains into
config/default.toml under [lqr]. Do NOT call these functions inside the control loop.
 
Linearized state-space model (REQ-CTRL-ARCH-004), per axis:
    Continuous:  A_c = [[0, 1], [0, 0]]   B_c = [[0], [1/J]]
    Discretized via ZOH to A_d, B_d at sample period dt_s.
 
Known issue (CTRL-TODO-005):
    The manual DARE fallback (when scipy is unavailable) has a convergence bug.
    Do not use it — the function will raise RuntimeError instead.
"""
 
from __future__ import annotations
 
import numpy as np
 
try:
    from scipy.linalg import solve_discrete_are as _dare
    _HAS_SCIPY = True
except ImportError:
    _HAS_SCIPY = False
 
 
def build_state_space(
    moment_of_inertia: float,
    dt_s: float,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Return the discrete-time ZOH state matrices (A_d, B_d) for one gimbal axis.
 
    The rigid-body model for a single rotational axis is:
        theta_ddot = tau / J
 
    As a first-order state-space with x = [theta, theta_dot]:
        A_c = [[0, 1],         B_c = [[0      ],
               [0, 0]]                [1 / J  ]]
 
    For this specific A_c the ZOH discretization has an exact closed form.
 
    Args:
        moment_of_inertia: Axis moment of inertia J (kg·m²).
                           TBD until gimbal is selected — use placeholder 0.01 for now.
        dt_s:              Control loop sample period (seconds).
 
    Returns:
        (A_d, B_d): 2×2 and 2×1 discrete-time matrices.
    """
    J = moment_of_inertia
    A_d = np.array([[1.0, dt_s],
                    [0.0,  1.0]])
    B_d = np.array([[0.5 * dt_s**2 / J],
                    [      dt_s / J   ]])
    return A_d, B_d
 
 
def compute_lqr_gain(
    A_d: np.ndarray,
    B_d: np.ndarray,
    Q:   np.ndarray,
    R:   np.ndarray,
) -> np.ndarray:
    """
    Solve the DARE and return the optimal LQR feedback gain K.
 
    Optimal control law: u_k = -K (x_k - x_ref)
 
    DARE: P = A'PA - (A'PB)(R + B'PB)^{-1}(B'PA) + Q
    Gain: K = (R + B'PB)^{-1} B'PA
 
    Args:
        A_d: 2×2 discrete state matrix.
        B_d: 2×1 discrete input matrix.
        Q:   2×2 state cost matrix.
             Suggested starting point: diag([100, 1])
             — large penalty on angle error, small on rate error.
        R:   1×1 input cost matrix.
             Suggested starting point: [[0.01]]
             — allow large torque commands.
 
    Returns:
        K: shape (1, 2) optimal gain vector [K_theta, K_theta_dot].
 
    Raises:
        RuntimeError: If scipy is not installed (CTRL-TODO-005 — fallback is broken).
    """
    if not _HAS_SCIPY:
        raise RuntimeError(
            "scipy is required for DARE gain computation.\n"
            "Install it:  pip install scipy\n"
            "The manual fallback has a known convergence bug (CTRL-TODO-005) "
            "and must not be used."
        )
 
    P = _dare(A_d, B_d, Q, R)
    K = np.linalg.inv(R + B_d.T @ P @ B_d) @ (B_d.T @ P @ A_d)
    return K
 
 
def gains_from_config(cfg: dict) -> dict[str, np.ndarray]:
    """
    Compute LQR gains for both axes from the config dict.
 
    Config keys consumed (all under the flattened merged config):
        control_loop_hz               — determines dt_s
        lqr_Q_diag                   — [q_theta, q_theta_dot]
        lqr_R_diag                   — [r_torque]
        moment_of_inertia_pan_kg_m2  — TBD
        moment_of_inertia_tilt_kg_m2 — TBD
 
    Returns:
        {"pan": K_pan, "tilt": K_tilt}  — each K has shape (1, 2).
    """
    dt_s  = 1.0 / cfg["control_loop_hz"]
    Q     = np.diag(cfg["lqr_Q_diag"])
    R     = np.diag(cfg["lqr_R_diag"])
    J_pan  = cfg.get("moment_of_inertia_pan_kg_m2",  0.01)
    J_tilt = cfg.get("moment_of_inertia_tilt_kg_m2", 0.01)
 
    A_pan,  B_pan  = build_state_space(J_pan,  dt_s)
    A_tilt, B_tilt = build_state_space(J_tilt, dt_s)
 
    return {
        "pan":  compute_lqr_gain(A_pan,  B_pan,  Q, R),
        "tilt": compute_lqr_gain(A_tilt, B_tilt, Q, R),
    }
 
 
if __name__ == "__main__":
    # Quick sanity check — run via: python -m pact_controls.controller.gains
    import tomllib
    from pathlib import Path
 
    cfg_path = Path(__file__).parents[3] / "config" / "default.toml"
    with open(cfg_path, "rb") as f:
        raw = tomllib.load(f)
 
    # Flatten the nested config sections into a single dict for gains_from_config()
    flat: dict = {}
    for section in raw.values():
        if isinstance(section, dict):
            flat.update(section)
 
    gains = gains_from_config(flat)
    print("Pan  K:", gains["pan"].round(4))
    print("Tilt K:", gains["tilt"].round(4))
    print("Paste these into config/default.toml under [lqr] as lqr_K_pan and lqr_K_tilt.")
