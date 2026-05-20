from __future__ import annotations

from pathlib import Path

from pyrobot.config.load_config import load_config


def main() -> None:
    cfg = load_config()
    limits = cfg.motion.soft_limits_steps

    out_path = Path(__file__).resolve().parents[1] / "vital_motion" / "Core" / "Inc" / "motion_limits.h"
    out_path.parent.mkdir(parents=True, exist_ok=True)

    order = ["a", "b", "c", "d"]
    mins = []
    maxs = []
    for axis in order:
        if axis not in limits:
            raise KeyError(f"motion.soft_limits_steps missing axis {axis!r}")
        lo, hi = limits[axis]
        mins.append(int(lo))
        maxs.append(int(hi))

    text = f"""/**
 * @file motion_limits.h
 * @brief Auto-generated from config/robot.yaml by tools/generate_motion_limits_header.py
 *        Do not edit manually.
 */
#ifndef MOTION_LIMITS_H
#define MOTION_LIMITS_H

#include <stdint.h>

#define MOTION_LIMIT_AXES 4U

static const int32_t MOTION_SOFT_LIMIT_MIN[MOTION_LIMIT_AXES] = {{{mins[0]}, {mins[1]}, {mins[2]}, {mins[3]}}};
static const int32_t MOTION_SOFT_LIMIT_MAX[MOTION_LIMIT_AXES] = {{{maxs[0]}, {maxs[1]}, {maxs[2]}, {maxs[3]}}};

#endif /* MOTION_LIMITS_H */
"""
    out_path.write_text(text, encoding="utf-8")
    print(f"generated: {out_path}")


if __name__ == "__main__":
    main()

