# PACT Controls — Claude Code Guidance
 
## What This Repo Is
 
This is the **controls subsystem** for the PACT ISS plume-detection payload.
It owns: LQR pointing controller, Kalman filter, safety gates, fault detection, gimbal HAL.
 
This is NOT the full PACT repo — imaging, ML inference, CCSDS comms, and storage live elsewhere.
 
## Architecture
 
Read `docs/architecture.md` before touching anything. It contains:
- Full requirement IDs (REQ-CTRL-ARCH-*, REQ-CTRL-SAFE-*, REQ-CTRL-FAULT-*, REQ-CTRL-TELEM-*)
- The state-space model derivation and gain computation procedure
- All hardware TBDs that are blocking (especially the gimbal hardware selection)
- Known bugs (CTRL-TODO-005: DARE fallback)
## Key Invariants — Do Not Break
 
1. **No magic numbers.** Every threshold must come from `config/default.toml`.
   Never hardcode angle limits, rates, or timeouts inside module code.
2. **Gains are computed offline.** `controller/gains.py` is a script, not runtime code.
   The LQR controller reads pre-computed gains from config. Do not call `gains_from_config()`
   inside the control loop.
3. **Axes are decoupled.** Pan and tilt are independent. Do not add cross-axis terms
   to the LQR or Kalman without updating the architecture doc and getting sign-off.
4. **HAL is the only hardware boundary.** All motor/encoder calls go through
   `pact_controls.hal.base.GimbalHALBase`. Never import hardware-specific libraries
   outside of `hal/`.
5. **Safety gates run every cycle.** `safety/gates.run_gates()` must be called on
   every LQR output before it reaches the HAL. Never bypass the gates.
6. **SAFE mode is sticky.** Once the FSM enters `GimbalMode.SAFE`, only a
   `ground_reset=True` call on `GimbalFSM.update()` can exit it.
## Hardware Status (as of repo creation)
 
The flight gimbal hardware has NOT been selected. Until it is:
- `hal/gimbal.py` contains `SimulatedGimbalHAL` only — for testing, not flight
- `moment_of_inertia_pan_kg_m2` and `moment_of_inertia_tilt_kg_m2` in `default.toml` are
  placeholder values (0.01 kg·m²) — **must be updated** once hardware is characterized
- `max_slew_deg_s` is set to 0 (disabled) until hardware slew limits are known
## Common Commands
 
```bash
pip install -e ".[dev]"         # install with dev dependencies
pytest                          # run all tests
pytest tests/test_lqr.py -v    # run one test file
python scripts/compute_gains.py # recompute LQR gains after changing Q/R
python scripts/demo_controller.py # run closed-loop simulation
```
 
## Adding a New Gimbal HAL
 
1. Create `pact_controls/hal/<gimbal_name>.py`
2. Subclass `GimbalHALBase` and implement all abstract methods
3. Update `config/default.toml` with hardware-specific params (J, encoder noise, etc.)
4. Run `scripts/compute_gains.py` with the new inertia values
5. Update the HAL TBD entries in `docs/architecture.md`
6. Add a `tests/test_hal_<gimbal_name>.py` with hardware-in-loop tests
## Open TODOs
 
See `TODO.md` for the full list. Critical blockers are prefixed `[BLOCKED]`.
