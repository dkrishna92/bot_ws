#!/usr/bin/env python3
"""Scale straw bale *collision* boxes (not the visual mesh/box) in the CFR
world files, relative to the canonical real bale dimensions (36x18x14in).

SDF world files have no variables/templating, so this script is the
"undo" mechanism -- it always recomputes from the canonical size below,
so re-running it with --scale 1.0 restores the real, full-size collision
boxes regardless of what's currently in the files.

Usage:
    # Shrink to 80% (default used for tight-corridor sim testing):
    python3 scale_bale_collisions.py

    # Restore real, full-size bale collisions:
    python3 scale_bale_collisions.py --scale 1.0

    # Any other factor:
    python3 scale_bale_collisions.py --scale 0.7
"""
import argparse
import re

# Real bale dimensions (36x18x14in), matching the visual mesh/box -- do not
# change this; it's the reference every --scale factor is computed from.
CANONICAL_SIZE = (0.9144, 0.4572, 0.3556)

WORLD_FILES = [
    "src/bot_gazebo/worlds/obstacle_course_cfr.world",
    "src/bot_gazebo/worlds/speed_course_cfr.world",
]

PATTERN = re.compile(
    r'(<collision name="(?:bale_\d+_collision|bale_collision)"><pose>[^<]*</pose>'
    r'<geometry><box><size>)[^<]*(</size></box></geometry>)'
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scale", type=float, default=0.8,
                         help="Fraction of the real bale size to use for collisions "
                              "(1.0 = real size, restores full-size collisions)")
    args = parser.parse_args()

    size_str = " ".join(f"{d * args.scale:.4f}" for d in CANONICAL_SIZE)

    for path in WORLD_FILES:
        with open(path) as f:
            content = f.read()
        new_content, count = PATTERN.subn(rf"\g<1>{size_str}\g<2>", content)
        with open(path, "w") as f:
            f.write(new_content)
        print(f"{path} -> {count} bale collision boxes set to {size_str} "
              f"(scale={args.scale})")


if __name__ == "__main__":
    main()
