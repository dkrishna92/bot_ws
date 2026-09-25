"""Multi-lap course navigator.

Subscribes to bot_perception's start_trigger_node's latched start_signal
Bool and, once it fires, sends Nav2 a single NavigateThroughPoses goal
covering the whole race: the current course's per-lap checkpoints (see
_COURSE_CHECKPOINTS below) repeated num_laps times -- speed_course_cfr is
3 laps, obstacle_course_cfr is 2 laps + obstacle handling (CLAUDE.md).

Checkpoints are deliberately coarse (a handful of points per lap, not a
dense hand-traced path) -- NavigateThroughPoses hands them to Nav2's own
global planner, which fills in the actual drivable route against the live
costmap. This node only owns *which* checkpoints define the course/lap
shape, not the path between them.

Vision detection and course/lap logic are kept in separate packages
(bot_perception vs. this one) per CLAUDE.md's per-concern package
convention -- bot_perception's start_trigger_node only detects the signal
and publishes start_signal; this node is what actually drives the race.

TODO: the checkpoint positions below are a best-effort derivation from the
imported CAD world geometry (bale/obstacle mesh positions in
speed_course_cfr.world / obstacle_course_cfr.world -- see each entry's
comment for how it was picked), NOT verified against the real course (see
CLAUDE.md's open items: official course geometry is still inaccessible).
Eyeball/adjust these against the actual layout in the Gazebo GUI, and
replace them entirely once real course geometry is available.
"""
from __future__ import annotations

import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses


# Per-course checkpoints as (x, y) in the map frame -- yaw is computed
# automatically by _build_route as the heading toward the next point,
# rather than hand-picked on top of already-approximate positions.
_COURSE_CHECKPOINTS = {
    # Traces speed_course_cfr's oval: start straight (the spawn pose in
    # gazebo_sim.launch.py's _DEFAULT_SPAWN_POSES), round the west turn,
    # back along the bottom, round the east turn, and return.
    #
    # Every point here was validated against config/maps/map.yaml -- the
    # same map amcl/map_server localize against -- as free space with at
    # least 0.40m obstacle clearance (robot_radius is 0.26, inflation
    # 0.35). That check is the point: a checkpoint picked from world-file
    # coordinates alone can land on a bale, and NavfnPlanner then fails
    # every cycle with "Failed to create plan" and the robot just sits
    # there running recoveries. If these are ever re-derived, re-validate
    # them against the map rather than only against the .world file.
    # NOTE: an out-and-back shuttle along the spawn's own lane, NOT a
    # closed lap. Two separate limitations in the checked-in map force
    # this, and both are worth fixing before this counts as a real race
    # route:
    #   1. The map has no drivable cells along the bottom of the oval
    #      between x=8 and x=32 at all -- the mapping run never covered
    #      that side (consistent with CLAUDE.md's open frontier-exploration
    #      reliability problem), so there is no circuit to close.
    #   2. The top straight is two lanes split by a middle bale row, and
    #      they do not connect anywhere the planner can use at this
    #      footprint. Routing from the spawn's (inner) lane to the outer
    #      one makes NavfnPlanner fail outright, so every point here stays
    #      in the inner lane.
    # The endpoints are also pulled well inside that lane's drivable
    # extent. The raw map has >=0.30m clearance out to x=8.0, but the
    # planner still fails past roughly x=11.8: Nav2 inflates obstacles
    # (inflation_radius 0.35) and the local costmap adds live lidar
    # returns, so usable width is meaningfully less than the raw map
    # suggests. Keep checkpoints in the ~0.45m-clearance mid-section
    # rather than at the measured edge of drivable space.
    "speed_course_cfr": [
        (18.99, 4.78),
        (13.00, 4.78),
        (27.00, 4.75),
    ],
    # start -> bridge ramp entrance -> helix exit (back at ground level
    # after the climb) -> all three hoop gates in numbered order (hoop_0/
    # 1/2 -- "slide along the dashed lines in the drawing" per that
    # world's own comment, so their exact spots are provisional too) ->
    # back to start/finish.
    #
    # UNVALIDATED, unlike the speed course above: these come from the
    # .world file only. The checked-in config/maps/map.yaml was built from
    # the speed course and doesn't even cover this course's extent (its
    # map spans x >= -1.51, while the hoops sit out at x = -8.1), so these
    # points cannot be map-checked until a map is generated for this world
    # (mapping.launch.py with world:=obstacle_course_cfr). Expect planning
    # failures until that's done.
    "obstacle_course_cfr": [
        (-0.7, 0.0),
        (5.5, 0.0),
        (7.3, 1.4),
        (-3.94, -1.71),
        (-6.37, -2.40),
        (-8.14, 0.15),
        (-0.7, 0.0),
    ],
}

_COURSE_LAPS = {
    "speed_course_cfr": 3,
    "obstacle_course_cfr": 2,
}


def _build_route(checkpoints_xy: list[tuple[float, float]], num_laps: int) -> list[tuple[float, float, float]]:
    """Repeat the per-lap checkpoints num_laps times; each point's yaw is
    the heading toward the next point in the full route (wrapping after
    the last one back to the first)."""
    full = list(checkpoints_xy) * num_laps
    route = []
    for i, (x, y) in enumerate(full):
        nx, ny = full[(i + 1) % len(full)]
        route.append((x, y, math.atan2(ny - y, nx - x)))
    return route


def _route_for_course(course: str, num_laps_override: int) -> list[tuple[float, float, float]]:
    """Empty list for an unknown course; num_laps_override of 0 means "use
    the course's own default lap count"."""
    if course not in _COURSE_CHECKPOINTS:
        return []
    num_laps = num_laps_override or _COURSE_LAPS[course]
    return _build_route(_COURSE_CHECKPOINTS[course], num_laps)


class LapNavigatorNode(Node):
    def __init__(self):
        super().__init__("lap_navigator_node")

        self.declare_parameter("course", "speed_course_cfr")
        self.declare_parameter("num_laps", 0)  # 0 -> use the course's own default lap count
        self.declare_parameter("goal_frame_id", "map")

        course = self.get_parameter("course").value
        num_laps_override = self.get_parameter("num_laps").value
        self._route = _route_for_course(course, num_laps_override)
        if not self._route:
            self.get_logger().error(
                f"Unknown course '{course}' -- no checkpoints defined for it "
                f"(known: {list(_COURSE_CHECKPOINTS)}). Not navigating."
            )
        else:
            self.get_logger().info(
                f"Loaded '{course}' ({len(self._route)} checkpoints total)."
            )

        self._sent = False
        self._pending = False
        # Nav2's bt_navigator can still be coming up when the start signal
        # fires. Retry on a timer rather than blocking in the subscription
        # callback -- a blocking wait there stalls this node's executor, so
        # it would sit out the whole startup window instead of retrying.
        self.create_timer(1.0, self._try_send)

        # Matches start_trigger_node's latched publisher QoS -- this node
        # may start after start_signal already fired, and still needs the
        # last value.
        latched_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Bool, "start_signal", self._on_start_signal, latched_qos)
        self._nav_client = ActionClient(self, NavigateThroughPoses, "navigate_through_poses")

    def _on_start_signal(self, msg: Bool) -> None:
        if not msg.data or self._sent or self._pending:
            return
        if not self._route:
            self.get_logger().error("start_signal fired but no route is loaded -- not navigating.")
            return
        self.get_logger().info("Start signal received -- sending route.")
        self._pending = True
        self._try_send()

    def _try_send(self) -> None:
        if not self._pending or self._sent:
            return
        # Short active wait rather than server_is_ready(): the latter is a
        # passive check that can keep reporting "not ready" indefinitely
        # even once the server is up, because it doesn't drive the
        # discovery this client needs. 0.1s keeps the timer responsive.
        if not self._nav_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().warn(
                "navigate_through_poses action server not up yet -- retrying.",
                throttle_duration_sec=5.0,
            )
            return
        self._sent = True
        self._pending = False
        self._send_route()

    def _send_route(self) -> None:
        frame_id = self.get_parameter("goal_frame_id").value
        stamp = self.get_clock().now().to_msg()

        goal_msg = NavigateThroughPoses.Goal()
        for x, y, yaw in self._route:
            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.header.stamp = stamp
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            goal_msg.poses.append(pose)

        self.get_logger().info(f"Sending {len(self._route)}-checkpoint route to Nav2.")
        self._nav_client.send_goal_async(goal_msg).add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("Nav2 rejected the route.")
            self._sent = False
            self._pending = True  # retry on the timer
            return
        self.get_logger().info("Nav2 accepted the route -- underway.")
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future) -> None:
        self.get_logger().info("Route finished.")


def main(args=None):
    rclpy.init(args=args)
    node = LapNavigatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
