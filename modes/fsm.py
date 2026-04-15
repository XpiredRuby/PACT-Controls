"""
Finite state machine for gimbal operating modes.
 
The FSM tracks which mode the system is IN and evaluates transition conditions
each cycle. It does NOT produce commands — that is the arbiter's job.
 
States:
    IDLE      — Gimbal held at reference; awaiting a scan plan or NN detection
    SCAN      — Executing pre-planned raster scan at fixed 0.5 deg/s
    TRACKING  — Closed-loop tracking of a plume ROI from NN output
    SAFE      — Fault-triggered; gimbal homes to (0, 0) and holds
 
Transition table:
    IDLE     → SCAN      : scan plan becomes available (has_scan_plan = True)
    IDLE     → TRACKING  : NN tracker provides a valid command (has_tracker_cmd = True)
    SCAN     → TRACKING  : NN tracker provides a valid command mid-scan
    TRACKING → SCAN      : NN detection lost for > scan_entry_idle_seconds AND scan plan active
    TRACKING → IDLE      : NN detection lost for > scan_entry_idle_seconds AND no scan plan
    *        → SAFE      : any FaultCode != NONE (REQ-CTRL-FAULT-*)
    SAFE     → IDLE      : ground-commanded reset only (ground_reset=True)
 
SAFE is sticky — no autonomous recovery. A POCC operator must issue a reset
command, which arrives as ground_reset=True in the CCSDS command pipeline.
"""
 
from __future__ import annotations
 
import time
 
from pact_controls.types.state import FaultCode, GimbalMode
 
 
class GimbalFSM:
    """
    Gimbal mode finite state machine.
 
    The FSM is updated once per control cycle via update(). The returned
    GimbalMode is read by the arbiter and the telemetry logger.
    """
 
    def __init__(self, cfg: dict) -> None:
        self.mode: GimbalMode = GimbalMode.IDLE
 
        # Time without a valid NN detection before TRACKING → SCAN/IDLE
        self._idle_timeout_s: float = cfg.get("scan_entry_idle_seconds", 5.0)
 
        self._last_detection_t: float | None = None
 
    def update(
        self,
        fault_code:      FaultCode,
        has_tracker_cmd: bool,
        has_scan_plan:   bool,
        ground_reset:    bool = False,
    ) -> GimbalMode:
        """
        Evaluate transition conditions and update self.mode.
 
        Call exactly once per control cycle, after FaultDetector.tick().
 
        Args:
            fault_code:      Current FaultCode from FaultDetector (this cycle).
            has_tracker_cmd: True if the NN tracker provided a valid command this cycle.
            has_scan_plan:   True if the scan planner has an active waypoint this cycle.
            ground_reset:    True if a POCC-issued reset command was received this cycle.
                             This is the only way to exit SAFE mode.
 
        Returns:
            Updated GimbalMode stored in self.mode.
        """
        now = time.monotonic()
 
        # SAFE is sticky — only ground reset can exit it
        if self.mode == GimbalMode.SAFE:
            if ground_reset:
                self.mode = GimbalMode.IDLE
            return self.mode
 
        # Fault → SAFE (highest priority, all other modes)
        if fault_code != FaultCode.NONE:
            self.mode = GimbalMode.SAFE
            return self.mode
 
        # Valid NN detection → TRACKING (takes priority over scan)
        if has_tracker_cmd:
            self._last_detection_t = now
            self.mode = GimbalMode.TRACKING
            return self.mode
 
        # Currently TRACKING but detection was lost — check idle timeout
        if self.mode == GimbalMode.TRACKING:
            last = self._last_detection_t or now
            detection_age = now - last
            if detection_age > self._idle_timeout_s:
                self.mode = GimbalMode.SCAN if has_scan_plan else GimbalMode.IDLE
 
        # IDLE → SCAN when a scan plan becomes available
        if self.mode == GimbalMode.IDLE and has_scan_plan:
            self.mode = GimbalMode.SCAN
 
        return self.mode
