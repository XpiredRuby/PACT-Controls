"""
Gimbal HAL implementations.
 
FLIGHT HAL STATUS: Not yet implemented — pending gimbal hardware selection.
  TODO CTRL-TODO-HAL-001: Implement CobrahpxGimbalHAL once COBRA-HPX is procured.
 
This file currently contains only SimulatedGimbalHAL, which integrates rigid-body
dynamics for software-in-the-loop (SITL) testing. It is NOT for flight use.
 
When implementing a flight HAL:
  1. Add a new class here (or a new file hal/<name>.py) subclassing GimbalHALBase
  2. Implement all 6 abstract methods
  3. Update default.toml and flight.toml with hardware-specific parameters
  4. Run scripts/compute_gains.py with measured inertia values
  5. Add integration tests in tests/test_hal_<name>.py
"""
 
from __future__ import annotations
 
import time
 
import numpy as np
 
from pact_controls.hal.base import GimbalHALBase
from pact_controls.types.state import ControlOutput, EncoderReading
 
 
class SimulatedGimbalHAL(GimbalHALBase):
    """
    Software-in-the-loop (SITL) gimbal using rigid-body dynamics.
 
    Uses the same linearized state-space model as the controller so that
    gains from gains.py close the loop in simulation. This lets the full
    control stack be tested without any physical hardware.
 
    Limitations vs. real hardware:
      - No friction, backlash, or flex modeled
      - Encoder noise is white Gaussian (real encoders have structured quantization)
      - No thermal dynamics
      - Torque normalization scale factor is arbitrary (torque_scale=1.0)
 
    NOT FOR FLIGHT USE.
    """
 
    def __init__(
        self,
        dt_s: float,
        moment_of_inertia_pan:  float = 0.01,
        moment_of_inertia_tilt: float = 0.01,
        encoder_noise_std_deg:  float = 0.05,
        torque_scale:           float = 1.0,
        rng_seed:               int   = 42,
    ) -> None:
        """
        Args:
            dt_s:                    Control loop sample period (seconds).
            moment_of_inertia_pan:   Pan axis J (kg·m²) — match value in default.toml.
            moment_of_inertia_tilt:  Tilt axis J (kg·m²) — match value in default.toml.
            encoder_noise_std_deg:   Std dev of simulated encoder noise (degrees).
            torque_scale:            Converts normalized torque [-1,1] to angular
                                     acceleration via: alpha = torque_norm * torque_scale / J
                                     TBD for real hardware (CTRL-TODO-DOC-002).
            rng_seed:                Seed for reproducible noise (use None for random).
        """
        self._dt_s       = dt_s
        self._J_pan      = moment_of_inertia_pan
        self._J_tilt     = moment_of_inertia_tilt
        self._noise_std  = encoder_noise_std_deg
        self._scale      = torque_scale
        self._rng        = np.random.default_rng(seed=rng_seed)
 
        # True simulated state: [angle_deg, rate_deg_s] per axis
        self._pan_state  = np.zeros(2)
        self._tilt_state = np.zeros(2)
 
        self._temperature_c = 20.0   # Fixed nominal — no thermal model
 
    # ------------------------------------------------------------------ HAL API
 
    def send_command(self, output: ControlOutput) -> None:
        """Integrate rigid-body dynamics one sample forward for each axis."""
        pan_accel  = output.pan_torque  * self._scale / self._J_pan
        tilt_accel = output.tilt_torque * self._scale / self._J_tilt
 
        self._pan_state[1]  += pan_accel  * self._dt_s
        self._pan_state[0]  += self._pan_state[1]  * self._dt_s
 
        self._tilt_state[1] += tilt_accel * self._dt_s
        self._tilt_state[0] += self._tilt_state[1] * self._dt_s
 
    def read_encoders(self) -> EncoderReading:
        """Return true angles plus additive Gaussian noise."""
        noise = self._rng.normal(0.0, self._noise_std, size=2)
        return EncoderReading(
            pan_deg=self._pan_state[0]  + noise[0],
            tilt_deg=self._tilt_state[0] + noise[1],
            timestamp_s=time.monotonic(),
            valid=True,
        )
 
    def home(self) -> None:
        """Instantly teleport to home (simulation only — real HAL will slew)."""
        self._pan_state[:]  = 0.0
        self._tilt_state[:] = 0.0
 
    def is_healthy(self) -> bool:
        return True
 
    def get_temperature_c(self) -> float:
        return self._temperature_c
 
    def shutdown(self) -> None:
        self._pan_state[:]  = 0.0
        self._tilt_state[:] = 0.0
 
    # ------------------------------------------------------------------ helpers
 
    @property
    def true_state(self) -> dict[str, float]:
        """Ground-truth state for SITL comparison (not available on real hardware)."""
        return {
            "pan_deg":          self._pan_state[0],
            "pan_rate_deg_s":   self._pan_state[1],
            "tilt_deg":         self._tilt_state[0],
            "tilt_rate_deg_s":  self._tilt_state[1],
        }
