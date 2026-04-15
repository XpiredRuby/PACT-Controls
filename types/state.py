"""
Shared data types for the PACT gimbal control system.
 
All inter-module data exchange uses these types to enforce consistent interfaces
between the controller, HAL, safety gates, fault detector, FSM, and telemetry
logger. No thresholds or constants live here — those belong in config.
 
Design rule: every subsystem imports from pact_controls.types; no subsystem
imports from another subsystem's internals (except via this module).
"""
 
from __future__ import annotations
 
import time
from dataclasses import dataclass, field
from enum import Enum, auto
 
 
class GimbalMode(Enum):
    """
    Operating modes of the gimbal FSM.
 
    Managed by pact_controls.modes.fsm.GimbalFSM.
    The arbiter (pact_controls.controller.arbiter) reads this to select
    the active command source each cycle.
    """
    IDLE     = auto()   # Gimbal held at reference; no active scan or tracking
    SCAN     = auto()   # Executing pre-planned raster scan at 0.5 deg/s (REQ-CTRL-SAFE-003)
    TRACKING = auto()   # Closed-loop tracking of a plume ROI from NN output
    SAFE     = auto()   # Fault-triggered; gimbal slews to home (0,0) and holds
 
 
class FaultCode(Enum):
    """
    Fault codes raised by pact_controls.fault.detector.FaultDetector.
 
    When fault_code != NONE, the arbiter overrides all commands with a SAFE command.
    SAFE mode is sticky — only a ground-commanded reset can clear it.
    """
    NONE                 = auto()
    ENCODER_LOSS         = auto()   # REQ-CTRL-FAULT-001
    WATCHDOG_TIMEOUT     = auto()   # REQ-CTRL-FAULT-002
    OVER_TEMPERATURE     = auto()   # REQ-CTRL-FAULT-003
    HARD_LIMIT_EXCEEDED  = auto()   # Safety gate 1 triggered
    RATE_LIMIT_EXCEEDED  = auto()   # Safety gate 3/4 triggered
 
 
@dataclass
class GimbalState:
    """
    Full two-axis gimbal state as estimated by the Kalman filter.
 
    Per-axis state vector: x = [theta (deg), theta_dot (deg/s)]
    Both axes are fully decoupled (REQ-CTRL-ARCH-003).
    """
    pan_deg:          float = 0.0
    tilt_deg:         float = 0.0
    pan_rate_deg_s:   float = 0.0
    tilt_rate_deg_s:  float = 0.0
    timestamp_s:      float = field(default_factory=time.monotonic)
 
 
@dataclass
class EncoderReading:
    """
    Raw encoder output for both axes from the gimbal HAL.
 
    When the encoder bus is unresponsive, valid=False and FaultDetector
    will raise ENCODER_LOSS after encoder_timeout_s (REQ-CTRL-FAULT-001).
    """
    pan_deg:     float = 0.0
    tilt_deg:    float = 0.0
    timestamp_s: float = field(default_factory=time.monotonic)
    valid:       bool  = True
 
 
@dataclass
class ControlCommand:
    """
    Commanded (reference) gimbal angles fed to the LQR controller.
 
    Produced by pact_controls.controller.arbiter.CommandArbiter each cycle
    after applying mode priority logic. Angle values are in degrees.
    """
    pan_cmd_deg:  float      = 0.0
    tilt_cmd_deg: float      = 0.0
    mode:         GimbalMode = GimbalMode.IDLE
    timestamp_s:  float      = field(default_factory=time.monotonic)
 
 
@dataclass
class ControlOutput:
    """
    Normalized torque output from the LQR controller, destined for the gimbal HAL.
 
    Values are clamped to [-1.0, 1.0] by safety gate 5 (output saturation).
    The HAL translates these normalized values to hardware-specific motor commands.
    The actual N·m scale factor is hardware-dependent (TBD — CTRL-TODO-DOC-002).
    """
    pan_torque:  float = 0.0   # Normalized torque [-1, 1]
    tilt_torque: float = 0.0   # Normalized torque [-1, 1]
    saturated:   bool  = False  # True if output-saturation gate clamped any axis
    timestamp_s: float = field(default_factory=time.monotonic)
 
 
@dataclass
class TelemetryFrame:
    """
    Per-cycle telemetry record logged by pact_controls.telemetry.logger.
 
    Implements REQ-CTRL-TELEM-001. Every field is mandatory — no optional fields.
    The weekly downlink summary (REQ-CTRL-TELEM-002) is derived from aggregating
    these frames. Log format is NDJSON (one JSON object per line, per cycle).
    """
    cycle_id:           int        = 0
    timestamp_s:        float      = 0.0
    mode:               GimbalMode = GimbalMode.IDLE
 
    # Commanded positions (from arbiter output)
    pan_cmd_deg:        float = 0.0
    tilt_cmd_deg:       float = 0.0
 
    # Kalman-estimated state
    pan_est_deg:        float = 0.0
    tilt_est_deg:       float = 0.0
    pan_rate_est_deg_s: float = 0.0
    tilt_rate_est_deg_s: float = 0.0
 
    # Pointing error = commanded - estimated
    pan_error_deg:      float = 0.0
    tilt_error_deg:     float = 0.0
 
    # LQR torque output (after safety gates)
    pan_torque:         float = 0.0
    tilt_torque:        float = 0.0
    output_saturated:   bool  = False
 
    # Health / fault flags
    fault_code:         FaultCode = FaultCode.NONE
    encoder_healthy:    bool = True
    watchdog_ok:        bool = True
    temperature_ok:     bool = True
