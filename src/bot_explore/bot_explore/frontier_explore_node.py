"""Autonomous frontier exploration, used by bot_bringup's mapping.launch.py.

Watches slam_toolbox's live /map, repeatedly sends Nav2 the centroid of the
nearest unexplored frontier (a free cell adjacent to unknown space), and
stops once no frontier remains -- then asks slam_toolbox to save the map.
Requires the Nav2 navigation servers (controller/planner/bt_navigator) and
slam_toolbox to already be running; both come from mapping.launch.py.

Ignores the map origin's orientation when converting grid cells to world
coordinates -- slam_toolbox always publishes an axis-aligned map, so this
doesn't lose anything here.
"""
from __future__ import annotations

import math
import threading
import time
from collections import deque

import numpy as np

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from slam_toolbox.srv import SaveMap

UNKNOWN = -1
FREE_MAX = 50  # occupancy <= this counts as free space


class FrontierExploreNode(Node):
    def __init__(self):
        super().__init__("frontier_explore_node")

        self.declare_parameter("min_frontier_size", 6)  # cells
        self.declare_parameter("goal_blacklist_radius_m", 0.4)
        self.declare_parameter("map_topic", "map")
        self.declare_parameter("save_map_name", "src/bot_bringup/config/maps/map")
        self.declare_parameter("planning_period_s", 2.0)

        self._min_frontier_size = self.get_parameter("min_frontier_size").value
        self._blacklist_radius = self.get_parameter("goal_blacklist_radius_m").value
        self._save_map_name = self.get_parameter("save_map_name").value

        self._map: OccupancyGrid | None = None
        self._blacklist: list[tuple[float, float]] = []
        self._planning_period_s = self.get_parameter("planning_period_s").value

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_subscription(OccupancyGrid, self.get_parameter("map_topic").value,
                                  self._on_map, 1)

        # BasicNavigator's goToPose()/isTaskComplete() are blocking calls
        # that internally do their own rclpy.spin_until_future_complete(self, ...)
        # -- calling them from a timer callback running inside this node's
        # own rclpy.spin() causes "RuntimeError: Executor is already
        # spinning" (nested spin on the same executor). Running the
        # exploration loop in its own thread instead lets BasicNavigator be
        # used the way it's designed to be (blocking, sequential calls),
        # while rclpy.spin(self) on the main thread keeps servicing the map
        # subscription and TF listener independently.
        self._nav = BasicNavigator()
        # Without this, goals sent before Nav2's lifecycle nodes finish
        # activating get rejected ("Action server is inactive") and
        # wrongly blacklisted, which can exhaust the few early frontier
        # candidates and falsely trigger "exploration complete".
        # localizer='robot_localization' is nav2_simple_commander's sentinel
        # for "skip the amcl-specific wait" -- this launch uses
        # robot_localization's EKF + slam_toolbox, never amcl.
        self._nav.waitUntilNav2Active(localizer='robot_localization')
        self._save_map_client = self.create_client(SaveMap, "/slam_toolbox/save_map")
        # Distinguishes "map hasn't developed any free space yet" from
        # "fully explored" -- both look like zero frontiers, but the former
        # happens on the very first /map message and must not end the loop.
        self._found_any_frontier = False

        self._explore_thread = threading.Thread(target=self._explore_loop, daemon=True)
        self._explore_thread.start()

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._map = msg

    def _robot_pose(self) -> tuple[float, float] | None:
        try:
            t = self._tf_buffer.lookup_transform("map", "base_link", Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None
        return t.transform.translation.x, t.transform.translation.y

    def _find_frontiers(self) -> list[tuple[float, float, int]]:
        """Cluster frontier cells; return (world_x, world_y, cluster_size) per cluster."""
        msg = self._map
        w, h = msg.info.width, msg.info.height
        grid = np.array(msg.data, dtype=np.int8).reshape(h, w)
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y

        free = (grid >= 0) & (grid <= FREE_MAX)
        unknown = grid == UNKNOWN

        frontier = np.zeros_like(free)
        frontier[:-1, :] |= free[:-1, :] & unknown[1:, :]
        frontier[1:, :] |= free[1:, :] & unknown[:-1, :]
        frontier[:, :-1] |= free[:, :-1] & unknown[:, 1:]
        frontier[:, 1:] |= free[:, 1:] & unknown[:, :-1]

        cells = set(zip(*np.nonzero(frontier)))
        visited: set[tuple[int, int]] = set()
        clusters = []
        for start in cells:
            if start in visited:
                continue
            visited.add(start)
            queue = deque([start])
            component = []
            while queue:
                cy, cx = queue.popleft()
                component.append((cy, cx))
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        neighbor = (cy + dy, cx + dx)
                        if neighbor in cells and neighbor not in visited:
                            visited.add(neighbor)
                            queue.append(neighbor)
            if len(component) < self._min_frontier_size:
                continue
            cy = sum(p[0] for p in component) / len(component)
            cx = sum(p[1] for p in component) / len(component)
            clusters.append((ox + (cx + 0.5) * res, oy + (cy + 0.5) * res, len(component)))
        return clusters

    def _pick_goal(self, pose: tuple[float, float]) -> tuple[float, float] | None:
        px, py = pose
        best, best_dist = None, math.inf
        for wx, wy, _size in self._find_frontiers():
            if any(math.hypot(wx - bx, wy - by) < self._blacklist_radius
                   for bx, by in self._blacklist):
                continue
            dist = math.hypot(wx - px, wy - py)
            if dist < best_dist:
                best, best_dist = (wx, wy), dist
        return best

    def _save_map(self) -> None:
        if not self._save_map_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("slam_toolbox save_map service not available; map not saved.")
            return
        request = SaveMap.Request()
        request.name = String(data=self._save_map_name)
        self._save_map_client.call_async(request)
        self.get_logger().info(f"Exploration complete -- saving map to '{self._save_map_name}'.")

    def _explore_loop(self) -> None:
        while rclpy.ok():
            if self._map is None:
                time.sleep(self._planning_period_s)
                continue

            pose = self._robot_pose()
            if pose is None:
                self.get_logger().info("Waiting for map -> base_link transform...")
                time.sleep(self._planning_period_s)
                continue

            goal = self._pick_goal(pose)
            if goal is None:
                if not self._found_any_frontier:
                    self.get_logger().info("No frontiers yet -- waiting for map to develop.")
                    time.sleep(self._planning_period_s)
                    continue
                self._save_map()
                return
            self._found_any_frontier = True

            goal_pose = PoseStamped()
            goal_pose.header.frame_id = "map"
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position.x = goal[0]
            goal_pose.pose.position.y = goal[1]
            goal_pose.pose.orientation.w = 1.0
            self.get_logger().info(f"Heading to frontier at ({goal[0]:.2f}, {goal[1]:.2f})")
            self._nav.goToPose(goal_pose)

            while not self._nav.isTaskComplete():
                time.sleep(0.5)

            result = self._nav.getResult()
            if result != TaskResult.SUCCEEDED:
                self.get_logger().warn(f"Frontier goal {goal} did not succeed "
                                        f"({result}); blacklisting.")
                self._blacklist.append(goal)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExploreNode()
    # An explicit executor here (not the rclpy.spin() convenience wrapper)
    # avoids sharing the process-wide default executor that BasicNavigator's
    # internal spin_until_future_complete() calls also fall back to -- both
    # grabbing that same default executor is what causes goToPose() to raise
    # "Executor is already spinning".
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
