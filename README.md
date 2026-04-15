# PACT Controls
 
Gimbal control subsystem for the **Plume Autonomous Classification and Tracking (PACT)** ISS payload.
 
This repository owns everything on the controls side: the LQR pointing controller, Kalman filter state estimator, safety gates, fault detection, and gimbal hardware abstraction layer (HAL).
 
## What It Does
 
| Subsystem | Job |
|---|---|
| `controller/` | LQR closed-loop pointing + Kalman state estimator |
| `hal/` | Hardware abstraction — gimbal driver interface (TBD pending hardware selection) |
| `safety/` | Five safety gates: hard/soft angle limits, slew rate caps, output clamping |
| `fault/` | Encoder loss, watchdog timeout, over-temperature detection |
| `modes/` | FSM managing IDLE → SCAN ↔ TRACKING → SAFE transitions |
| `telemetry/` | Per-cycle NDJSON logging for downlink |
 
## Repository Layout
 
```
pact_controls/
├── types/          # Shared dataclasses (GimbalState, ControlOutput, TelemetryFrame, …)
├── controller/     # LQR + Kalman + mode arbiter + offline gain computation
├── hal/            # Abstract HAL interface + SimulatedGimbalHAL (flight HAL: TBD)
├── safety/         # All five safety gates in one pure function
├── fault/          # FaultDetector — runs every cycle, raises FaultCode on anomaly
├── modes/          # GimbalFSM — tracks and transitions operating mode
└── telemetry/      # TelemetryLogger — buffered NDJSON writer with file rotation
config/
├── default.toml    # All tunable parameters with documented defaults
└── flight.toml     # Flight overrides (conservative limits, production thresholds)
docs/
└── architecture.md # ← Start here — full controls architecture and requirements
scripts/
├── compute_gains.py    # Offline DARE gain computation (run once, store results in config)
└── demo_controller.py  # Closed-loop simulation against SimulatedGimbalHAL
tests/              # Unit tests — one file per subsystem
```
 
## Getting Started
 
```bash
pip install -e ".[dev]"
pytest                          # run full test suite
python scripts/compute_gains.py # compute LQR gains and print to stdout
python scripts/demo_controller.py
```
 
## Platform
 
| Item | Value |
|---|---|
| Host computer | Jetson Orin NX (or equivalent, TBD) |
| Gimbal | TBD — see `docs/architecture.md §Gimbal Hardware` |
| Control loop rate | TBD Hz (configurable via `control_loop_hz`) |
| ISS power budget | ≤ 60 W total payload |
| Thermal range (on-orbit) | −40 °C to +55 °C |
 
## Ownership
 
Controls subsystem owner: **Vin Manoj Nair**  
Parent project: [PACT](https://github.com/xpiredruby/pact-controls)
