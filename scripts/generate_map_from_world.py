#!/usr/bin/env python3
"""Rasterize a Gazebo (SDF) course world directly into a Nav2 occupancy-grid
map (.pgm + .yaml), WITHOUT running a slam_toolbox mapping pass.

Why this exists: mapping.launch.py builds the map the honest way (drive the
robot, accumulate real lidar scans), which is what you'd validate on real
hardware. But for iterating on bringup.launch.py's amcl localization in sim,
that's a slow, sometimes-flaky loop. Since the sim world's geometry is known
exactly, we can rasterize it straight into a map instead -- a "perfect" map
with none of slam's drift, good enough to get amcl + Nav2 running against a
course in seconds.

What it captures (matches what the 2D lidar actually sees, so amcl's scan
match lines up):
  - <box> and <cylinder> VISUAL geometry, at their composed world pose
    (model pose o link pose o element pose, yaw only -- a 2D map ignores
    z/roll/pitch). Visuals specifically, NOT collisions: gz-sim's gpu_lidar
    raycasts the rendered scene (visuals), not physics collision shapes, so
    that's what a scan-match map must mirror. (Collision geometry is what
    physically blocks the robot during a run -- a different role -- and can
    legitimately differ in size from the visual, e.g. after
    scale_bale_collisions.py shrinks a bale's collision box only.)
  - EXCLUDES any visual carrying <visibility_flags> -- those are the
    climbable ramp/deck surfaces bot.urdf.xacro's lidar deliberately renders
    through (see GAZEBO_WORLDS.md), so the real sim lidar never returns them
    and neither should the map.
  - EXCLUDES the 'ground' model (floor plane -- not an obstacle for a planar
    scan).

Limitation -- <mesh> visuals are NOT rasterized (would need to load+project
the STL): the speed course is entirely box visuals so it's fully covered,
but the obstacle course's mesh props (tunnel, car_wash, buckets,
gravel/pothole sections) come out as a bales-only map. Their collision-only
cylinders aren't a substitute -- the lidar can't see those either (it sees
the mesh visual), so they're correctly absent. That's still a usable
localization map (the bale rows are the dominant lidar features either way),
just not a complete one -- a real mapping run is still the way to capture
those. A count of skipped meshes is printed so you know what's missing.

The map is written in WORLD coordinates (map frame == Gazebo world frame),
so amcl's initial_pose in nav2_params.yaml must equal the robot's spawn pose
in world coords -- exactly the same constraint a slam-built map has, since
mapping.launch.py anchors the map origin at that same spawn pose.

Usage (run from workspace root):
    python3 scripts/generate_map_from_world.py \
        src/bot_gazebo/worlds/speed_course_cfr.world \
        --out src/bot_bringup/config/maps/map

    # then rebuild so bringup.launch.py sees it, and launch:
    colcon build --packages-select bot_bringup
    ros2 launch bot_bringup bringup.launch.py use_sim:=true world:=speed_course_cfr
"""
from __future__ import annotations

import argparse
import math
import sys
import xml.etree.ElementTree as ET

import numpy as np

# PGM greyscale values map_server interprets via occupied_thresh/free_thresh
# (with the default negate:0, trinary mode): 0 = occupied, 254 = free,
# 205 = unknown. A geometry-derived map has no "unknown" -- every cell is
# either a known obstacle or known-free.
OCCUPIED = 0
FREE = 254

# Skip a box/cylinder whose footprint is smaller than a single cell at this
# resolution would show anyway -- avoids spending pixels on things like the
# car-wash's 1.7cm-radius uprights that a 5cm-cell lidar map can't resolve.
MIN_FEATURE_M = 0.02


def _parse_pose(elem):
    """Return (x, y, yaw) from an SDF <pose>, or (0,0,0) if absent."""
    pose = elem.find('pose')
    if pose is None or not pose.text:
        return 0.0, 0.0, 0.0
    vals = [float(v) for v in pose.text.split()]
    x = vals[0] if len(vals) > 0 else 0.0
    y = vals[1] if len(vals) > 1 else 0.0
    yaw = vals[5] if len(vals) > 5 else 0.0
    return x, y, yaw


def _compose(parent, child):
    """Compose two 2D poses (x, y, yaw): child expressed in parent's frame."""
    px, py, pyaw = parent
    cx, cy, cyaw = child
    c, s = math.cos(pyaw), math.sin(pyaw)
    return (px + cx * c - cy * s, py + cx * s + cy * c, pyaw + cyaw)


def collect_shapes(world_path):
    """Walk the SDF and return (boxes, cylinders, skipped_mesh_count).

    boxes: list of (cx, cy, yaw, sx, sy)
    cylinders: list of (cx, cy, radius)
    """
    tree = ET.parse(world_path)
    root = tree.getroot()
    world = root.find('world')
    if world is None:
        raise SystemExit(f"no <world> element in {world_path}")

    boxes, cylinders = [], []
    skipped_mesh = 0

    for model in world.findall('model'):
        if model.get('name') == 'ground':
            continue
        model_pose = _parse_pose(model)
        for link in model.findall('link'):
            link_pose = _compose(model_pose, _parse_pose(link))
            # VISUALS, not collisions -- gz-sim's gpu_lidar raycasts against
            # the rendered scene (visual geometry), never physics collisions.
            # This matters when the two differ: scale_bale_collisions.py can
            # shrink bale *collision* boxes while leaving visuals full-size,
            # and a collision-derived map would then not match what the real
            # sim lidar reports (silent amcl mismatch). Reading visuals also
            # makes the <visibility_flags> ramp exclusion below correct-by-
            # construction, since that flag lives on the visual. Consequence:
            # collision-only geometry (e.g. the car_wash/bucket cylinders,
            # whose visible part is a <mesh>) is intentionally absent -- the
            # lidar can't see it either, so it doesn't belong in the map.
            for g in link.findall('visual'):
                # Lidar renders through visibility-flagged surfaces (ramps);
                # they must not appear in a scan-match map.
                if g.find('visibility_flags') is not None:
                    continue
                geom = g.find('geometry')
                if geom is None:
                    continue
                elem_pose = _compose(link_pose, _parse_pose(g))
                box = geom.find('box')
                cyl = geom.find('cylinder')
                if box is not None:
                    size = box.find('size')
                    if size is None or not size.text:
                        continue
                    sx, sy = (float(v) for v in size.text.split()[:2])
                    if max(sx, sy) < MIN_FEATURE_M:
                        continue
                    boxes.append((elem_pose[0], elem_pose[1], elem_pose[2], sx, sy))
                elif cyl is not None:
                    r = cyl.find('radius')
                    if r is None or not r.text:
                        continue
                    radius = float(r.text)
                    if radius < MIN_FEATURE_M:
                        continue
                    cylinders.append((elem_pose[0], elem_pose[1], radius))
                elif geom.find('mesh') is not None:
                    skipped_mesh += 1

    return boxes, cylinders, skipped_mesh


def rasterize(boxes, cylinders, resolution, margin):
    """Return (grid, origin_x, origin_y). grid is uint8, row 0 = top (max y)."""
    if not boxes and not cylinders:
        raise SystemExit("no rasterizable geometry found -- nothing to map")

    # World-space bounds over every shape's extent.
    xs, ys = [], []
    for cx, cy, yaw, sx, sy in boxes:
        half_diag = 0.5 * math.hypot(sx, sy)  # conservative (rotation-agnostic)
        xs += [cx - half_diag, cx + half_diag]
        ys += [cy - half_diag, cy + half_diag]
    for cx, cy, r in cylinders:
        xs += [cx - r, cx + r]
        ys += [cy - r, cy + r]

    min_x, max_x = min(xs) - margin, max(xs) + margin
    min_y, max_y = min(ys) - margin, max(ys) + margin

    width = int(math.ceil((max_x - min_x) / resolution))
    height = int(math.ceil((max_y - min_y) / resolution))
    grid = np.full((height, width), FREE, dtype=np.uint8)

    def world_to_px(wx, wy):
        col = int((wx - min_x) / resolution)
        row = int((max_y - wy) / resolution)  # row 0 = top = max_y
        return row, col

    for cx, cy, yaw, sx, sy in boxes:
        c, s = math.cos(-yaw), math.sin(-yaw)  # inverse rotation into box frame
        half_diag = 0.5 * math.hypot(sx, sy)
        r0, c0 = world_to_px(cx + half_diag, cy + half_diag)
        r1, c1 = world_to_px(cx - half_diag, cy - half_diag)
        for row in range(max(0, min(r0, r1)), min(height, max(r0, r1) + 1)):
            for col in range(max(0, min(c0, c1)), min(width, max(c0, c1) + 1)):
                wx = min_x + (col + 0.5) * resolution
                wy = max_y - (row + 0.5) * resolution
                dx, dy = wx - cx, wy - cy
                # Rotate the point into the box's local frame; inside iff both
                # local coords are within the half-extents.
                lx = dx * c - dy * s
                ly = dx * s + dy * c
                if abs(lx) <= sx / 2 and abs(ly) <= sy / 2:
                    grid[row, col] = OCCUPIED

    for cx, cy, radius in cylinders:
        r0, c0 = world_to_px(cx + radius, cy + radius)
        r1, c1 = world_to_px(cx - radius, cy - radius)
        for row in range(max(0, min(r0, r1)), min(height, max(r0, r1) + 1)):
            for col in range(max(0, min(c0, c1)), min(width, max(c0, c1) + 1)):
                wx = min_x + (col + 0.5) * resolution
                wy = max_y - (row + 0.5) * resolution
                if math.hypot(wx - cx, wy - cy) <= radius:
                    grid[row, col] = OCCUPIED

    return grid, min_x, min_y


def write_pgm(path, grid):
    height, width = grid.shape
    with open(path, 'wb') as f:
        f.write(f"P5\n{width} {height}\n255\n".encode('ascii'))
        f.write(grid.tobytes())


def write_yaml(path, image_name, resolution, origin_x, origin_y):
    with open(path, 'w') as f:
        f.write(f"image: {image_name}\n")
        f.write("mode: trinary\n")
        f.write(f"resolution: {resolution:.3f}\n")
        f.write(f"origin: [{origin_x:.3f}, {origin_y:.3f}, 0]\n")
        f.write("negate: 0\n")
        f.write("occupied_thresh: 0.65\n")
        f.write("free_thresh: 0.196\n")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('world', help='path to the .world SDF file')
    ap.add_argument('--out', default='src/bot_bringup/config/maps/map',
                    help='output path prefix, no extension (default: %(default)s)')
    ap.add_argument('--resolution', type=float, default=0.05,
                    help='metres per cell (default: %(default)s)')
    ap.add_argument('--margin', type=float, default=1.0,
                    help='free-space border around the geometry, metres '
                         '(default: %(default)s)')
    args = ap.parse_args()

    boxes, cylinders, skipped_mesh = collect_shapes(args.world)
    grid, origin_x, origin_y = rasterize(boxes, cylinders, args.resolution, args.margin)

    pgm_path = f"{args.out}.pgm"
    yaml_path = f"{args.out}.yaml"
    image_name = pgm_path.rsplit('/', 1)[-1]
    write_pgm(pgm_path, grid)
    write_yaml(yaml_path, image_name, args.resolution, origin_x, origin_y)

    height, width = grid.shape
    occupied = int((grid == OCCUPIED).sum())
    print(f"wrote {pgm_path} ({width}x{height} px, {occupied} occupied cells)")
    print(f"wrote {yaml_path} (origin [{origin_x:.3f}, {origin_y:.3f}])")
    print(f"rasterized {len(boxes)} boxes, {len(cylinders)} cylinders")
    if skipped_mesh:
        print(f"WARNING: skipped {skipped_mesh} <mesh> geometries -- these are "
              f"NOT in the map (see this script's docstring). If the course "
              f"relies on them for localization, do a real mapping run instead.",
              file=sys.stderr)


if __name__ == '__main__':
    main()
