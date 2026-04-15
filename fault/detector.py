"""
Fault detection for the gimbal control subsystem.
 
Implements three fault checks per REQ-CTRL-FAULT-*:
  REQ-CTRL-FAULT-001  Encoder loss detection
  REQ-CTRL-FAULT-002  Control loop watchdog timeout
  REQ-CTRL-FAULT-003  Over-temperature protection
 
FaultDetector.tick() must be called once per control cycle. When any fault is
detected, it sets self.fault_code. The CommandArbiter reads this code and
overrides the command to SAFE. The GimbalFSM transitions to SAFE on the same
cycle. SAFE mode is sticky — only GimbalFSM.update(ground_reset=True) clears it.
 
The watchdog works by measuring the elapsed time since the last tick() call.
If the control loop stalls (CPU overload, deadlock, process killed + restarted),
the elapsed time on the first post-stall tick will exceed watchdog_timeout_s.
 
Known gap (CTRL-TODO-FAULT-001): stuck-value encoder detection is not yet implemented.
A frozen encoder that always returns valid=True but a constant value will not trigger
ENCODER_LOSS. Add a consecutive-same-value counter in a future iteration.
 
Known gap (CTRL-TODO-FAULT-002): power brownout detection is not implemented.
Requires a HAL hook for bus voltage monitoring (see RISK-ELEC-002 in risk register).
"""
 
from __future__ import annotations
 
import time
 
from pact_controls.types.state import EncoderReading, FaultCode
 
 
class FaultDetector:
    """
    Stateful fault detector. Instantiate once; call tick() every cycle.
 
    Usage:
        detector = FaultDetector(cfg)
        ...
        for each cycle:
            fault = detector.tick(encoder_reading, hal.get_temperature_c())
            if fault != FaultCode.NONE:
                # arbiter will override to SAFE this cycle
    """
 
    def __init__(self, cfg: dict) -> None:
        self._encoder_timeout_s:  float = cfg["encoder_timeout_s"]
        self._watchdog_timeout_s: float = cfg["watchdog_timeout_s"]
        self._thermal_limit_c:    float = cfg["thermal_limit_c"]
 
        self._last_valid_encoder_t: float = time.monotonic()
        self._last_tick_t:          float = time.monotonic()
 
        self.fault_code: FaultCode = FaultCode.NONE
 
    def tick(
        self,
        encoder:       EncoderReading,
        temperature_c: float,
    ) -> FaultCode:
        """
        Run all fault checks for this cycle.
 
        Args:
            encoder:       Latest EncoderReading from the HAL.
            temperature_c: Current gimbal temperature from hal.get_temperature_c().
                           Pass 0.0 until the HAL temperature sensor is implemented
                           (CTRL-TODO-HAL-002) — the thermal check will be inactive.
 
        Returns:
            The active FaultCode (NONE if everything is healthy).
            Also stored in self.fault_code for the arbiter to read.
        """
        now = time.monotonic()
 
        # --- REQ-CTRL-FAULT-001: Encoder loss ---
        if encoder.valid:
            self._last_valid_encoder_t = now
        else:
            elapsed_since_valid = now - self._last_valid_encoder_t
            if elapsed_since_valid > self._encoder_timeout_s:
                self.fault_code = FaultCode.ENCODER_LOSS
                return self.fault_code
 
        # --- REQ-CTRL-FAULT-002: Control loop watchdog ---
        # Measure elapsed since last tick. If the loop stalled, elapsed will be large.
        elapsed_loop = now - self._last_tick_t
        self._last_tick_t = now
 
        if elapsed_loop > self._watchdog_timeout_s:
            self.fault_code = FaultCode.WATCHDOG_TIMEOUT
            return self.fault_code
 
        # --- REQ-CTRL-FAULT-003: Over-temperature ---
        if temperature_c > self._thermal_limit_c:
            self.fault_code = FaultCode.OVER_TEMPERATURE
            return self.fault_code
 
        # All checks passed
        self.fault_code = FaultCode.NONE
        return self.fault_code
 
    def reset(self) -> None:
        """
        Clear fault state after a confirmed safe recovery.
 
        Should only be called when the FSM receives ground_reset=True.
        Do not call this autonomously — SAFE mode must require ground intervention.
        """
        self.fault_code = FaultCode.NONE
        self._last_valid_encoder_t = time.monotonic()
        self._last_tick_t          = time.monotonic()
