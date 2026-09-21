# Gazebo Worlds

This document describes the available Gazebo simulation worlds for the CAT Robotics competition.

## Culture Club Robotics (CFR) 2026 Worlds

These worlds are imported from the [CfR-2026 repository](https://github.com/dkt01/CfR-2026) and contain the actual competition course layouts.

### Obstacle Course
The 2-lap obstacle course with various challenges.

**Launch:**
```bash
# Directly launch the obstacle course
ros2 launch bot_gazebo obstacle_course_cfr.launch.py

# Or use the main bringup with the world argument
ros2 launch bot_bringup bringup.launch.py use_sim:=true
# Then in another terminal:
ros2 param set /gazebo world obstacle_course_cfr
```

**World File:** `src/bot_gazebo/worlds/obstacle_course_cfr.world`

### Speed Course
The 3-lap speed course.

**Launch:**
```bash
# Directly launch the speed course
ros2 launch bot_gazebo speed_course_cfr.launch.py

# Or use the main bringup with the world argument
ros2 launch bot_bringup bringup.launch.py use_sim:=true world:=speed_course_cfr
```

**World File:** `src/bot_gazebo/worlds/speed_course_cfr.world`

## Other Available Worlds

### OnShape Course
The course modeled from OnShape CAD data.

**Launch:**
```bash
ros2 launch bot_gazebo gazebo_sim.launch.py world:=onshape_course
```

### Obstacle Course (Legacy)
An earlier obstacle course implementation.

**Launch:**
```bash
ros2 launch bot_gazebo gazebo_sim.launch.py world:=obstacle_course
```

### Speed Course (Legacy)
An earlier speed course implementation.

**Launch:**
```bash
ros2 launch bot_gazebo gazebo_sim.launch.py world:=speed_course
```

### Circular Course
A simple circular course for basic testing.

**Launch:**
```bash
ros2 launch bot_gazebo gazebo_sim.launch.py world:=circular_course
```

## Directory Structure

```
bot_gazebo/
├── models/
│   ├── cfr_arduino_bridge/          # CFR 2026 course models
│   │   ├── meshes/                  # 15 STL mesh files (obstacles, props)
│   │   ├── model.config
│   │   └── model.sdf
│   └── onshape_course/              # OnShape course model
│       ├── meshes/
│       ├── model.config
│       └── model.sdf
├── worlds/
│   ├── obstacle_course_cfr.world    # CFR obstacle course (new)
│   ├── speed_course_cfr.world       # CFR speed course (new)
│   ├── obstacle_course.world        # Legacy obstacle course
│   ├── speed_course.world           # Legacy speed course
│   ├── circular_course.world
│   └── onshape_course.world
└── launch/
    ├── obstacle_course_cfr.launch.py
    ├── speed_course_cfr.launch.py
    └── gazebo_sim.launch.py
```

## Mesh Files (CFR 2026)

The CFR obstacle and speed courses use the following mesh files:
- `bank.stl` - Banking turn element
- `bucket.stl` - Bucket obstacle
- `car_wash_arch.stl` - Car wash tunnel arch
- `car_wash_base.stl` - Car wash tunnel base
- `gravel_box.stl` - Gravel/terrain box
- `hoop.stl` - Hoop obstacle
- `pothole_board.stl` - Pothole board
- `pothole_bump.stl` - Pothole bump
- `ramp_narrowing.stl` - Narrowing ramp
- `ramp_pothole_entry.stl` - Pothole entry ramp
- `ramp_up.stl` - Ramp up
- `start_signal_arm_green.stl` - Start signal (green)
- `start_signal_arm_red.stl` - Start signal (red)
- `start_signal_frame.stl` - Start signal frame
- `tunnel.stl` - Tunnel obstacle

## Notes

- Meshes and worlds are automatically found via the Gazebo model path configuration set in the launch files
- The `GZ_SIM_RESOURCE_PATH` environment variable is set during launch to point to the `models/` directory
- All worlds use Gazebo Sim (Ignition Gazebo) with SDF 1.10 format
- **Climbable elements (bridge/helix ramps) are hidden from ranging sensors, not just visually driven over.** In `obstacle_course_cfr.world`, the `bridge` and `helix` models' deck-surface visuals (`ramp_visual`, `deck_visual`, `deck_N_visual`) carry `<visibility_flags>1</visibility_flags>` (bit 0 only, instead of the default all-bits mask). `bot.urdf.xacro`'s `lidar_sensor` and `oak_depth_sensor` clear that same bit in their own `<visibility_mask>`, so their gpu_lidar/depth_camera renders pass straight through those decks instead of marking them as obstacles in Nav2's costmaps — otherwise a 2D lidar at a fixed mount height sees the rising ramp surface as a wall partway up, a real limitation of 2D-lidar-based nav on ramps, not just a sim bug. Collision geometry is untouched, so physics/climbing is unaffected. Rail visuals on both models deliberately keep the default (fully visible) mask, so the local costmap still treats them as lateral boundaries. If you add another climbable structure, tag its walkable-surface visuals the same way — don't touch its rails/walls, and don't touch `camera_sensor` (the plain RGB feed), which should keep seeing everything normally.
