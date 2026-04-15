#!/usr/bin/env python3
"""
Offline LQR gain computation script.
 
Run this script whenever you change Q/R weights or plant inertia values.
Paste the printed K gains into config/default.toml under [lqr].
 
Usage:
    python scripts/compute_gains.py
    python scripts/compute_gains.py --config config/flight.toml
 
How it works:
    1. Load config/default.toml (plus an optional override file)
    2. Flatten all TOML sections into a single dict
    3. Call controller.gains.gains_from_config() to run DARE
    4. Print the resulting K vectors for both axes
 
After printing, manually paste the values into config/default.toml:
    [lqr]
    lqr_K_pan  = [<value>, <value>]
    lqr_K_tilt = [<value>, <value>]
 
Do NOT call this script from the control loop — gains are computed offline.
"""
 
from __future__ import annotations
 
import argparse
import sys
import tomllib
from pathlib import Path
 
# Add project root to path so we can import pact_controls without installing
sys.path.insert(0, str(Path(__file__).parents[1]))
 
from pact_controls.controller.gains import gains_from_config
 
 
def load_config(default_path: Path, override_path: Path | None) -> dict:
    """Load and merge default + optional override TOML files."""
    with open(default_path, "rb") as f:
        raw = tomllib.load(f)
 
    if override_path is not None:
        with open(override_path, "rb") as f:
            override = tomllib.load(f)
        # Deep merge sections
        for section, values in override.items():
            if isinstance(values, dict) and section in raw:
                raw[section].update(values)
            else:
                raw[section] = values
 
    # Flatten nested TOML sections into a single dict
    flat: dict = {}
    for section_values in raw.values():
        if isinstance(section_values, dict):
            flat.update(section_values)
    return flat
 
 
def main() -> None:
    parser = argparse.ArgumentParser(description="Compute offline LQR gains via DARE.")
    parser.add_argument(
        "--config",
        type=Path,
        default=None,
        help="Optional TOML override file (e.g., config/flight.toml)",
    )
    args = parser.parse_args()
 
    repo_root   = Path(__file__).parents[1]
    default_cfg = repo_root / "config" / "default.toml"
 
    cfg = load_config(default_cfg, args.config)
 
    print("=" * 60)
    print("PACT Controls — Offline LQR Gain Computation")
    print("=" * 60)
    print()
    print(f"Control loop:   {cfg['control_loop_hz']} Hz  (dt = {1/cfg['control_loop_hz']:.4f} s)")
    print(f"Q diagonal:     {cfg['lqr_Q_diag']}")
    print(f"R diagonal:     {cfg['lqr_R_diag']}")
    print(f"J_pan:          {cfg.get('moment_of_inertia_pan_kg_m2', 0.01)} kg·m²  (TBD)")
    print(f"J_tilt:         {cfg.get('moment_of_inertia_tilt_kg_m2', 0.01)} kg·m²  (TBD)")
    print()
 
    try:
        gains = gains_from_config(cfg)
    except RuntimeError as e:
        print(f"ERROR: {e}")
        sys.exit(1)
 
    K_pan  = gains["pan"].flatten().tolist()
    K_tilt = gains["tilt"].flatten().tolist()
 
    print("Computed gains:")
    print(f"  Pan  K = {[round(k, 4) for k in K_pan]}")
    print(f"  Tilt K = {[round(k, 4) for k in K_tilt]}")
    print()
    print("Paste into config/default.toml under [lqr]:")
    print(f'  lqr_K_pan  = {[round(k, 4) for k in K_pan]}')
    print(f'  lqr_K_tilt = {[round(k, 4) for k in K_tilt]}')
    print()
    print("NOTE: J values are placeholders (0.01 kg·m²).")
    print("      Update moment_of_inertia_* in config after hardware is selected,")
    print("      then re-run this script.")
 
 
if __name__ == "__main__":
    main()
