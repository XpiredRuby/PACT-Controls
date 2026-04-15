"""
Abstract Hardware Abstraction Layer interface for the PACT gimbal.
 
Every concrete gimbal driver MUST subclass GimbalHALBase and implement all
abstract methods. The rest of the control stack (controller, safety, fault) only
ever calls this interface — never hardware-specific code directly.
 
This decoupling means:
  - The entire control stack is testable without real hardware (use SimulatedGimbalHAL)
  - Swapping gimbal hardware (e.g., COBRA-HPX → Gremsy) only requires a new file
    in hal/ — no changes elsewhere
 
HARDWARE STATUS: Flight gimbal not yet selected (see docs/architecture.md §Gimbal Hardware).
Current candidates:
  - COBRA-HPX (Tethers Unlimited)  — highest TRL, most space-qualified
  - Gremsy Pixy U                  — COTS, lower mass, lower TRL
  - Small FPV gimbal               — lowest mass, lowest TRL
 
When the gimbal is selected:
  1. Create pact_controls/hal/<gimbal_name>.py
  2. Subclass GimbalHALBase, implement all abstract methods
  3. Update CLAUDE.md §Adding a New Gimbal HAL
"""
 
from __future__ import annotations
 
from abc import ABC, abstractmethod
 
from pact_controls.types.state import ControlOutput, EncoderReading
 
 
class GimbalHALBase(ABC):
    """Abstract interface every gimbal driver must implement."""
 
    @abstractmethod
    def send_command(self, output: ControlOutput) -> None:
        """
        Translate normalized torque output to hardware-specific motor commands
        and transmit to the gimbal.
 
        The normalization convention: [-1.0, 1.0] maps to [−max_torque, +max_torque]
        for the given hardware. The exact scale factor (N·m) is hardware-dependent
        and must be documented in docs/architecture.md once the gimbal is selected
        (CTRL-TODO-DOC-002).
 
        Args:
            output: Normalized torque for each axis. Guaranteed clamped to [-1, 1]
                    by safety gate 5 before this call.
        """
 
    @abstractmethod
    def read_encoders(self) -> EncoderReading:
        """
        Read current encoder angles from the gimbal.
 
        Returns:
            EncoderReading with pan_deg, tilt_deg, valid flag, and timestamp.
            Set valid=False if the encoder bus is unresponsive (triggers ENCODER_LOSS
            fault after encoder_timeout_s per REQ-CTRL-FAULT-001).
        """
 
    @abstractmethod
    def home(self) -> None:
        """
        Slew the gimbal to the home position (pan=0°, tilt=0°).
 
        Called by the SAFE mode handler. Whether this is blocking or fire-and-forget
        is implementation-defined — document in the concrete subclass.
        """
 
    @abstractmethod
    def is_healthy(self) -> bool:
        """
        Return True if the gimbal driver reports no internal hardware faults.
 
        This is a quick poll, not a deep self-test. Called every cycle by the
        control loop for use in go/no-go decisions.
        """
 
    @abstractmethod
    def get_temperature_c(self) -> float:
        """
        Return the current gimbal electronics/motor temperature in Celsius.
 
        Used by FaultDetector to check REQ-CTRL-FAULT-003 (over-temperature).
        Return a safe sentinel value (e.g., 0.0) if no temperature sensor is
        available on the selected hardware, and document this in CLAUDE.md.
        """
 
    @abstractmethod
    def shutdown(self) -> None:
        """
        Perform a safe shutdown: release motor torque, park gimbal, close comms.
 
        Called at program exit or on unrecoverable fault. Must not raise.
        """
