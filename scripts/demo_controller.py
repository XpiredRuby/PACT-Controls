#!/usr/bin/env python3
"""
Closed-loop simulation demo for the PACT gimbal control stack.
 
Runs the full control pipeline against SimulatedGimbalHAL for N seconds,
then prints a summary of pointing performance.
 
This script exercises every subsystem in the correct order:
    FaultDetector.tick()
    →  GimbalFSM.update()
    →  CommandArbiter.arbitrate()
    →  AxisKalmanFilter.predict() + update()
    →  AxisLQRController.compute()
    →  safety.gates.run_gates()
    →  SimulatedGimbalHAL.send_command()
    →  TelemetryLogger.log()
 
Usage:
    python scripts/demo_controller.py
    python scripts/demo_controller.py --seconds 30 --cmd-pan 45 --cmd-tilt 20
"""
 
from __future__ import annotations
 
import argparse
import sys
import time
import tomllib
from pathlib import Path
 
sys.path.insert(0, str(Path(__file__).parents[1]))
 
import numpy as np
 
from pact_controls.controller.arbiter  import CommandArbiter
from pact_controls.controller.kalman   import make_axis_filters
from pact_controls.controller.lqr      import make_axis_controllers
from pact_controls.fault.detector      import FaultDetector
from pact_controls.hal.gimbal          import SimulatedGimbalHAL
from pact_controls.modes.fsm           import GimbalFSM
from pact_controls.safety.gates        import run_gates
from pact_controls.telemetry.logger    import TelemetryLogger
from pact_controls.types.state import (
    ControlCommand,
    ControlOutput,
    FaultCode,
    GimbalMode,
    GimbalState,
    TelemetryFrame,
)
 
 
def load_flat_config(default_path: Path) -> dict:
    with open(default_path, "rb") as f:
        raw = tomllib.load(f)
    flat: dict = {}
    for section_values in raw.values():
        if isinstance(section_values, dict):
            flat.update(section_values)
    return flat
 
 
def run_demo(
    cmd_pan_deg:  float = 30.0,
    cmd_tilt_deg: float = 15.0,
    duration_s:   float = 10.0,
    verbose:      bool  = True,
) -> dict:
    """
    Run a closed-loop step response simulation.
 
    Args:
        cmd_pan_deg:  Target pan angle (degrees).
        cmd_tilt_deg: Target tilt angle (degrees).
        duration_s:   Simulation duration (seconds).
        verbose:      Print per-cycle status to stdout.
 
    Returns:
        Summary statistics dict.
    """
    repo_root  = Path(__file__).parents[1]
    cfg        = load_flat_config(repo_root / "config" / "default.toml")
    dt_s       = 1.0 / cfg["control_loop_hz"]
    n_cycles   = int(duration_s / dt_s)
 
    # ---- Build subsystems ---------------------------------------------------
    hal     = SimulatedGimbalHAL(dt_s=dt_s,
                                  moment_of_inertia_pan=cfg.get("moment_of_inertia_pan_kg_m2",  0.01),
                                  moment_of_inertia_tilt=cfg.get("moment_of_inertia_tilt_kg_m2", 0.01))
    kf_pan, kf_tilt = make_axis_filters(cfg)
    lqr_pan, lqr_tilt = make_axis_controllers(cfg)
    arbiter  = CommandArbiter(cfg)
    fsm      = GimbalFSM(cfg)
    detector = FaultDetector(cfg)
 
    # Telemetry goes to /tmp for demo purposes
    log_dir = Path("/tmp/pact_controls_demo")
    logger  = TelemetryLogger(cfg, log_dir)
 
    # ---- Constant tracking command (simulates NN output) --------------------
    tracker_cmd = ControlCommand(
        pan_cmd_deg=cmd_pan_deg,
        tilt_cmd_deg=cmd_tilt_deg,
        mode=GimbalMode.TRACKING,
    )
 
    # ---- Simulation state ---------------------------------------------------
    pan_errors:  list[float] = []
    tilt_errors: list[float] = []
    last_torque = ControlOutput()   # needed for Kalman predict on first cycle
 
    print(f"\nPACT Controls Demo — step to pan={cmd_pan_deg}°, tilt={cmd_tilt_deg}°")
    print(f"Duration: {duration_s}s  |  Loop rate: {cfg['control_loop_hz']} Hz  |  {n_cycles} cycles")
    print("-" * 60)
 
    for cycle in range(n_cycles):
        t_now = cycle * dt_s
 
        # 1. Read encoder
        enc = hal.read_encoders()
 
        # 2. Fault detection
        fault = detector.tick(enc, temperature_c=hal.get_temperature_c())
 
        # 3. FSM update
        mode = fsm.update(
            fault_code=fault,
            has_tracker_cmd=True,
            has_scan_plan=False,
        )
 
        # 4. Command arbitration
        cmd = arbiter.arbitrate(tracker_cmd, scan_cmd=None, fault_code=fault)
 
        # 5. Kalman predict (using last cycle's torque)
        kf_pan.predict(last_torque.pan_torque)
        kf_tilt.predict(last_torque.tilt_torque)
 
        # 6. Kalman update (with encoder measurement)
        if enc.valid:
            kf_pan.update(enc.pan_deg)
            kf_tilt.update(enc.tilt_deg)
 
        # 7. Estimated state
        state = GimbalState(
            pan_deg=kf_pan.angle_deg,
            tilt_deg=kf_tilt.angle_deg,
            pan_rate_deg_s=kf_pan.rate_deg_s,
            tilt_rate_deg_s=kf_tilt.rate_deg_s,
        )
 
        # 8. LQR compute
        raw_output = ControlOutput(
            pan_torque=lqr_pan.compute(state.pan_deg,  state.pan_rate_deg_s,  cmd.pan_cmd_deg),
            tilt_torque=lqr_tilt.compute(state.tilt_deg, state.tilt_rate_deg_s, cmd.tilt_cmd_deg),
        )
 
        # 9. Safety gates
        safety = run_gates(cmd, raw_output, state, cfg)
        last_torque = safety.output
 
        # 10. Send to HAL
        hal.send_command(safety.output)
 
        # 11. Compute errors for logging
        pan_err  = cmd.pan_cmd_deg  - state.pan_deg
        tilt_err = cmd.tilt_cmd_deg - state.tilt_deg
        pan_errors.append(pan_err)
        tilt_errors.append(tilt_err)
 
        # 12. Telemetry
        frame = TelemetryFrame(
            cycle_id=cycle,
            timestamp_s=t_now,
            mode=mode,
            pan_cmd_deg=cmd.pan_cmd_deg,
            tilt_cmd_deg=cmd.tilt_cmd_deg,
            pan_est_deg=state.pan_deg,
            tilt_est_deg=state.tilt_deg,
            pan_rate_est_deg_s=state.pan_rate_deg_s,
            tilt_rate_est_deg_s=state.tilt_rate_deg_s,
            pan_error_deg=pan_err,
            tilt_error_deg=tilt_err,
            pan_torque=safety.output.pan_torque,
            tilt_torque=safety.output.tilt_torque,
            output_saturated=safety.output_saturated,
            fault_code=fault,
            encoder_healthy=enc.valid,
            watchdog_ok=(fault != FaultCode.WATCHDOG_TIMEOUT),
            temperature_ok=(fault != FaultCode.OVER_TEMPERATURE),
        )
        logger.log(frame)
 
        # Periodic console output (every 1 second)
        if verbose and cycle % int(cfg["control_loop_hz"]) == 0:
            true = hal.true_state
            print(
                f"t={t_now:5.1f}s | "
                f"cmd=({cmd_pan_deg:6.1f},{cmd_tilt_deg:5.1f}) | "
                f"est=({state.pan_deg:6.2f},{state.tilt_deg:5.2f}) | "
                f"err=({pan_err:+5.2f},{tilt_err:+5.2f}) | "
                f"torque=({safety.output.pan_torque:+.3f},{safety.output.tilt_torque:+.3f}) | "
                f"mode={mode.name}"
            )
 
    logger.close()
 
    # ---- Summary ------------------------------------------------------------
    pan_arr  = np.array(pan_errors)
    tilt_arr = np.array(tilt_errors)
 
    # Steady-state = last 20% of simulation
    ss_start = int(0.8 * n_cycles)
    ss_pan   = np.abs(pan_arr[ss_start:]).mean()
    ss_tilt  = np.abs(tilt_arr[ss_start:]).mean()
 
    # Settling time: first time |error| < 1° and stays there
    def settling_time(errors: np.ndarray, threshold: float = 1.0) -> float | None:
        for i, e in enumerate(np.abs(errors)):
            if i + 10 < len(errors) and np.all(np.abs(errors[i:i+10]) < threshold):
                return i * dt_s
        return None
 
    print("\n" + "=" * 60)
    print("Summary")
    print("=" * 60)
    print(f"  Pan  steady-state error (mean |e|, last 20%):  {ss_pan:.3f}°")
    print(f"  Tilt steady-state error (mean |e|, last 20%):  {ss_tilt:.3f}°")
    t_settle_pan  = settling_time(pan_arr)
    t_settle_tilt = settling_time(tilt_arr)
    print(f"  Pan  settling time (<1°):  {t_settle_pan:.2f}s"  if t_settle_pan  else "  Pan  settling time (<1°):  did not settle")
    print(f"  Tilt settling time (<1°):  {t_settle_tilt:.2f}s" if t_settle_tilt else "  Tilt settling time (<1°):  did not settle")
    print(f"\n  Telemetry written to: {log_dir}")
 
    return {
        "ss_pan_deg":    ss_pan,
        "ss_tilt_deg":   ss_tilt,
        "settle_pan_s":  t_settle_pan,
        "settle_tilt_s": t_settle_tilt,
    }
 
 
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="PACT gimbal control closed-loop demo.")
    parser.add_argument("--seconds",  type=float, default=10.0, help="Simulation duration (s)")
    parser.add_argument("--cmd-pan",  type=float, default=30.0, help="Target pan angle (deg)")
    parser.add_argument("--cmd-tilt", type=float, default=15.0, help="Target tilt angle (deg)")
    parser.add_argument("--quiet",    action="store_true",      help="Suppress per-cycle output")
    args = parser.parse_args()
 
    run_demo(
        cmd_pan_deg=args.cmd_pan,
        cmd_tilt_deg=args.cmd_tilt,
        duration_s=args.seconds,
        verbose=not args.quiet,
    )
