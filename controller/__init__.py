"""
Controller subsystem.
 
Modules:
    gains    — offline DARE-based LQR gain computation (run as a script, not at runtime)
    kalman   — per-axis discrete-time Kalman filter for state estimation
    lqr      — per-axis LQR state-feedback controller
    arbiter  — mode-priority command arbiter (selects SAFE / TRACKING / SCAN / IDLE)
"""
