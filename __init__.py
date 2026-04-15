"""
pact_controls — Gimbal control subsystem for the PACT ISS plume-detection payload.
 
Subsystems:
    pact_controls.types       — shared dataclasses and enums
    pact_controls.controller  — LQR controller, Kalman filter, mode arbiter, gain computation
    pact_controls.hal         — hardware abstraction layer (gimbal driver interface)
    pact_controls.safety      — safety gate enforcement (angle limits, slew rate, saturation)
    pact_controls.fault       — fault detection (encoder loss, watchdog, over-temperature)
    pact_controls.modes       — FSM managing IDLE / SCAN / TRACKING / SAFE transitions
    pact_controls.telemetry   — per-cycle telemetry logging
"""
