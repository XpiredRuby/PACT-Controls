"""
Mode arbiter: selects the active command source each control cycle.
 
The arbiter sits between the external command sources and the LQR controller.
It applies a fixed priority order and returns exactly one ControlCommand per cycle.
 
Command priority (highest to lowest):
  1. SAFE fault override  — any FaultCode != NONE; always wins (REQ-CTRL-FAULT-*)
  2. TRACKING             — NN tracker provides a valid ROI command this cycle
  3. SCAN                 — scan planner provides a waypoint this cycle
  4. IDLE                 — fallback; hold current idle reference position
 
The arbiter is stateless beyond the IDLE reference position. Mode state is
owned by the FSM (pact_controls.modes.fsm.GimbalFSM). All outputs are still
subject to safety gate enforcement in pact_controls.safety.gates.
"""
 
from __future__ import annotations
 
import time
 
from pact_controls.types.state import ControlCommand, FaultCode, GimbalMode
 
 
class CommandArbiter:
    """
    Selects the winning ControlCommand from available sources each cycle.
 
    Usage:
        arbiter = CommandArbiter(cfg)
        cmd = arbiter.arbitrate(tracker_cmd, scan_cmd, fault_code)
    """
 
    def __init__(self, cfg: dict) -> None:
        # Scan slew rate stored here for reference; enforcement is in safety/gates.py
        self._scan_slew_rate_deg_s: float = cfg["scan_slew_rate_deg_per_s"]
 
        # IDLE hold position — updated when a non-IDLE mode transitions back to IDLE
        self._idle_pan_deg:  float = 0.0
        self._idle_tilt_deg: float = 0.0
 
    def arbitrate(
        self,
        tracker_cmd: ControlCommand | None,
        scan_cmd:    ControlCommand | None,
        fault_code:  FaultCode,
    ) -> ControlCommand:
        """
        Return the highest-priority ControlCommand for this cycle.
 
        Args:
            tracker_cmd:  Command from the NN tracker (TRACKING mode).
                          Pass None if no valid detection this cycle.
            scan_cmd:     Command from the scan planner (SCAN mode).
                          Pass None if no active scan plan.
            fault_code:   Current FaultCode from FaultDetector.
 
        Returns:
            A single ControlCommand with the winning target angles and mode.
        """
        now = time.monotonic()
 
        # Priority 1: Fault → SAFE home position
        if fault_code != FaultCode.NONE:
            return ControlCommand(
                pan_cmd_deg=0.0,
                tilt_cmd_deg=0.0,
                mode=GimbalMode.SAFE,
                timestamp_s=now,
            )
 
        # Priority 2: Valid NN tracker output → TRACKING
        if tracker_cmd is not None:
            return ControlCommand(
                pan_cmd_deg=tracker_cmd.pan_cmd_deg,
                tilt_cmd_deg=tracker_cmd.tilt_cmd_deg,
                mode=GimbalMode.TRACKING,
                timestamp_s=now,
            )
 
        # Priority 3: Scan planner active → SCAN
        if scan_cmd is not None:
            return ControlCommand(
                pan_cmd_deg=scan_cmd.pan_cmd_deg,
                tilt_cmd_deg=scan_cmd.tilt_cmd_deg,
                mode=GimbalMode.SCAN,
                timestamp_s=now,
            )
 
        # Priority 4: IDLE — hold the stored idle reference
        return ControlCommand(
            pan_cmd_deg=self._idle_pan_deg,
            tilt_cmd_deg=self._idle_tilt_deg,
            mode=GimbalMode.IDLE,
            timestamp_s=now,
        )
 
    def set_idle_reference(self, pan_deg: float, tilt_deg: float) -> None:
        """
        Update the IDLE hold position.
 
        Call this when intentionally moving to a new idle position (e.g., after
        a scan completes and the ground commands a new stow angle).
        """
        self._idle_pan_deg  = pan_deg
        self._idle_tilt_deg = tilt_deg
