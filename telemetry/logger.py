"""
Per-cycle telemetry logger for the gimbal control subsystem.
 
REQ-CTRL-TELEM-001: Log commanded position, Kalman-estimated state, pointing
error, control output, and fault flags every control cycle.
 
REQ-CTRL-TELEM-002: Produce a weekly downlink summary with min/max/mean per field.
(TODO CTRL-TODO-TELEM-001: aggregation not yet implemented — see TODO.md)
 
Log format:
    Newline-delimited JSON (NDJSON) — one JSON object per line, one line per cycle.
    This matches the format expected by the main PACT repo's storage/writer.py
    and is compatible with the CCSDS downlink pipeline.
 
File rotation:
    A new log file is opened every telemetry_rotation_period_s (default ~1 ISS orbit).
    Old files accumulate until the comm_window_days pruning pass (not yet implemented).
 
Buffering:
    Frames are buffered in memory (telemetry_buffer_frames) before flushing to disk
    to reduce I/O frequency on the shared storage bus.
"""
 
from __future__ import annotations
 
import json
import time
from pathlib import Path
 
from pact_controls.types.state import FaultCode, GimbalMode, TelemetryFrame
 
 
def _serialize(frame: TelemetryFrame) -> dict:
    """Convert a TelemetryFrame to a JSON-serializable dict."""
    return {
        "cycle_id":             frame.cycle_id,
        "timestamp_s":          round(frame.timestamp_s, 6),
        "mode":                 frame.mode.name,
        # Commanded
        "pan_cmd_deg":          round(frame.pan_cmd_deg, 4),
        "tilt_cmd_deg":         round(frame.tilt_cmd_deg, 4),
        # Estimated
        "pan_est_deg":          round(frame.pan_est_deg, 4),
        "tilt_est_deg":         round(frame.tilt_est_deg, 4),
        "pan_rate_est_deg_s":   round(frame.pan_rate_est_deg_s, 4),
        "tilt_rate_est_deg_s":  round(frame.tilt_rate_est_deg_s, 4),
        # Error
        "pan_error_deg":        round(frame.pan_error_deg, 4),
        "tilt_error_deg":       round(frame.tilt_error_deg, 4),
        # Control output
        "pan_torque":           round(frame.pan_torque, 6),
        "tilt_torque":          round(frame.tilt_torque, 6),
        "output_saturated":     frame.output_saturated,
        # Health flags
        "fault_code":           frame.fault_code.name,
        "encoder_healthy":      frame.encoder_healthy,
        "watchdog_ok":          frame.watchdog_ok,
        "temperature_ok":       frame.temperature_ok,
    }
 
 
class TelemetryLogger:
    """
    Buffered NDJSON telemetry logger with per-orbit file rotation.
 
    Usage:
        log_dir = Path("/data/telemetry/gimbal")
        logger = TelemetryLogger(cfg, log_dir)
        try:
            for each cycle:
                logger.log(frame)
        finally:
            logger.close()
    """
 
    def __init__(self, cfg: dict, log_dir: Path) -> None:
        self._log_dir = log_dir
        self._log_dir.mkdir(parents=True, exist_ok=True)
 
        self._rotation_s:   float = cfg.get("telemetry_rotation_period_s", 5400.0)
        self._buffer_limit: int   = cfg.get("telemetry_buffer_frames", 100)
 
        self._buffer:       list[dict] = []
        self._file_start_t: float      = time.monotonic()
        self._fh = self._open_new_file()
 
    def log(self, frame: TelemetryFrame) -> None:
        """
        Buffer one TelemetryFrame. Flushes when buffer is full or file rotates.
 
        Args:
            frame: Completed TelemetryFrame for this control cycle.
        """
        self._buffer.append(_serialize(frame))
 
        if len(self._buffer) >= self._buffer_limit:
            self._flush()
 
        if (time.monotonic() - self._file_start_t) > self._rotation_s:
            self._rotate()
 
    def flush(self) -> None:
        """Flush buffered frames to disk immediately (call before shutdown)."""
        self._flush()
 
    def close(self) -> None:
        """Flush remaining buffer and close the current log file."""
        self._flush()
        self._fh.close()
 
    # ------------------------------------------------------------------ private
 
    def _open_new_file(self):
        ts = int(time.time())
        path = self._log_dir / f"gimbal_telem_{ts}.ndjson"
        return open(path, "w", buffering=1)  # line-buffered for crash safety
 
    def _flush(self) -> None:
        for record in self._buffer:
            self._fh.write(json.dumps(record) + "\n")
        self._buffer.clear()
 
    def _rotate(self) -> None:
        self._flush()
        self._fh.close()
        self._fh            = self._open_new_file()
        self._file_start_t  = time.monotonic()
