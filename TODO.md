# PACT Controls — TODO
 
Items are grouped by subsystem. `[BLOCKED]` means work cannot start until a dependency is resolved.
 
---
 
## Hardware (Gimbal Selection)
 
- [ ] `[BLOCKED]` **CTRL-TODO-HAL-001** — Implement `CobrahpxGimbalHAL` in `hal/gimbal.py`
  - Blocked on gimbal procurement and COBRA-HPX interface spec
  - When done: also update `moment_of_inertia_*` in `default.toml`
- [ ] `[BLOCKED]` **CTRL-TODO-HAL-002** — Wire `get_temperature_c()` in HAL to real sensor
  - Currently `FaultDetector` receives a dummy 0.0 from the demo script
  - Blocked on hardware selection and temperature sensor characterization
- [ ] `[BLOCKED]` **CTRL-TODO-HAL-003** — Characterize encoder noise (σ) for Kalman `R_meas`
  - The `kalman_meas_noise_var = 0.01` in `default.toml` is a placeholder
  - Blocked on encoder datasheet / bench measurement from actual hardware
---
 
## Controller
 
- [ ] **CTRL-TODO-001** — Tune LQR Q/R weights on the SITL simulation
  - Run `scripts/demo_controller.py`, adjust `lqr_Q_diag` / `lqr_R_diag` in `default.toml`
  - Target: meet REQ-CTRL-POINT-001 (steady-state error) and REQ-CTRL-POINT-002 (settling)
  - Thresholds for those requirements are still TBD — coordinate with systems team
- [ ] `[BLOCKED]` **CTRL-TODO-002** — Set `max_slew_deg_s` in `default.toml`
  - Currently 0.0 (gate disabled). Blocked on hardware slew rate spec
- [ ] `[BLOCKED]` **CTRL-TODO-003** — Confirm `control_loop_hz` value
  - Currently 50 Hz placeholder. Blocked on CPU budget analysis on Jetson
- [ ] **CTRL-TODO-004** — Implement scan pattern waypoint generator
  - The arbiter accepts `scan_cmd` but there is no scan planner yet
  - Create `pact_controls/scan/planner.py` with a raster scan generator
- [ ] **CTRL-TODO-005** — Fix DARE fallback bug in `controller/gains.py`
  - The manual DARE fallback (when scipy is unavailable) has a convergence bug
  - Current fix: raise `RuntimeError` if scipy is missing — DO NOT use the fallback
  - Long-term: remove fallback or fix it and add a regression test
---
 
## Safety
 
- [ ] **CTRL-TODO-SAFE-001** — Confirm hard angle limits with hardware team
  - `hard_pan_limit_deg = 180` and tilt `[-90, +90]` are defaults
  - Actual mechanical limits depend on the selected gimbal — update `flight.toml`
- [ ] **CTRL-TODO-SAFE-002** — Add integral windup protection to the LQR
  - Currently the controller is purely proportional+derivative (LQR state feedback)
  - If an integrator is added later, windup limits must be gated here
---
 
## Fault Detection
 
- [ ] **CTRL-TODO-FAULT-001** — Add encoder stuck-value detection
  - Encoder loss is detected (valid flag), but a stuck encoder (constant value for N cycles)
  - is not. Add a `consecutive_same_reading` check to `FaultDetector.tick()`
- [ ] **CTRL-TODO-FAULT-002** — Add power brownout detection
  - RISK-ELEC-002 from the risk register. Needs a HAL hook for bus voltage monitoring
---
 
## Telemetry
 
- [ ] **CTRL-TODO-TELEM-001** — Implement weekly downlink summary aggregation
  - REQ-CTRL-TELEM-002 requires a weekly summary (min/max/mean per field)
  - Add `TelemetryAggregator` class to `telemetry/logger.py`
- [ ] **CTRL-TODO-TELEM-002** — Validate NDJSON format against CCSDS pipeline
  - Coordinate with comms subsystem to confirm field names match expected schema
---
 
## Tests
 
- [ ] **CTRL-TODO-TEST-001** — Add parametrized boundary tests for all safety limits
  - Test values at exactly the hard/soft boundaries, not just well inside/outside
- [ ] **CTRL-TODO-TEST-002** — Add Kalman convergence rate test
  - Verify the filter converges to truth within N cycles for a step input
- [ ] **CTRL-TODO-TEST-003** — Add SITL closed-loop integration test
  - Run `SimulatedGimbalHAL` + full controller stack for 10 seconds, assert settling
- [ ] **CTRL-TODO-TEST-004** — Add hardware-in-loop test template
  - Once HAL is implemented, a HIL test fixture is needed for acceptance testing
---
 
## Documentation
 
- [ ] **CTRL-TODO-DOC-001** — Fill in TBD performance thresholds in `docs/architecture.md`
  - REQ-CTRL-POINT-001 (steady-state error), REQ-CTRL-POINT-002 (settling time),
    REQ-CTRL-POINT-003 (jitter) — all currently TBD pending systems team decision
- [ ] **CTRL-TODO-DOC-002** — Document the torque normalization convention
  - Currently normalized torque [-1, 1] is passed to HAL with no explicit scale factor
  - Once hardware is selected, document the actual N·m mapping in `docs/architecture.md`
