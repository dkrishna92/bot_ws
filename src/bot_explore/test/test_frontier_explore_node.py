"""Pure-logic tests for frontier detection / goal selection.

Builds FrontierExploreNode via __new__ to bypass Node.__init__ (which would
start BasicNavigator and the background explore thread) -- only the grid
math and selection logic below is under test, not ROS runtime behavior.
"""
from nav_msgs.msg import OccupancyGrid

from bot_explore.frontier_explore_node import FrontierExploreNode


def _make_grid(data, width, height, resolution=1.0, origin=(0.0, 0.0)):
    msg = OccupancyGrid()
    msg.info.width = width
    msg.info.height = height
    msg.info.resolution = resolution
    msg.info.origin.position.x = origin[0]
    msg.info.origin.position.y = origin[1]
    msg.data = data
    return msg


def _make_explorer(min_frontier_size=1, blacklist_radius=0.4, min_goal_distance=0.0,
                    heading_turn_penalty=1.5, min_obstacle_clearance=0.0):
    node = FrontierExploreNode.__new__(FrontierExploreNode)
    node._min_frontier_size = min_frontier_size
    node._blacklist_radius = blacklist_radius
    node._min_goal_distance = min_goal_distance
    node._heading_turn_penalty = heading_turn_penalty
    node._min_obstacle_clearance = min_obstacle_clearance
    node._blacklist = []
    node._map = None
    return node


def test_find_frontiers_single_cell():
    node = _make_explorer()
    node._map = _make_grid([0, 0, 0, -1, -1], width=5, height=1)
    assert node._find_frontiers() == [(2.5, 0.5, 1)]


def test_find_frontiers_clusters_adjacent_cells():
    # Frontier cells (0,1) and (1,1) are equidistant from their own mean, so
    # the medoid tie-break (Python set iteration order, effectively
    # arbitrary) can land on either -- what this test actually cares about
    # is that both merge into one cluster of size 2, and the returned point
    # is one of the two real member cells rather than the mean itself
    # (world (1.5, 1.0), which is not any actual free cell's center here).
    node = _make_explorer()
    node._map = _make_grid([0, 0, -1, 0, 0, -1], width=3, height=2)
    result = node._find_frontiers()
    assert len(result) == 1
    assert result[0] in [(1.5, 0.5, 2), (1.5, 1.5, 2)]


def test_find_frontiers_below_min_size_excluded():
    node = _make_explorer(min_frontier_size=3)
    node._map = _make_grid([0, 0, -1, 0, 0, -1], width=3, height=2)
    assert node._find_frontiers() == []


def test_find_frontiers_uses_medoid_not_raw_mean():
    # L-shaped cluster: free cells at (row,col) (0,0),(0,1),(0,2),(1,2).
    # Their raw mean is (row=0.25, col=1.25) -> world (1.75, 0.75), a point
    # that isn't itself a cluster member. The nearest actual member to that
    # mean is (0,1) -> world (1.5, 0.5): that's what should be returned,
    # confirming the goal is always a real free cell, never an interpolated
    # point that can land off the free-cell region entirely (see this
    # cluster's real-world equivalent: wrapping around an obstacle corner).
    node = _make_explorer(min_frontier_size=4)
    node._map = _make_grid(
        [0, 0, 0, -1,
         -1, -1, 0, -1,
         -1, -1, -1, -1],
        width=4, height=3,
    )
    assert node._find_frontiers() == [(1.5, 0.5, 4)]


def test_find_frontiers_rejects_cluster_without_clearance():
    # Single free frontier cell at (0,0); an occupied cell (100) at (0,2),
    # 2.0m away at this grid's resolution=1.0 -- inside the requested 2.0m
    # clearance, so the whole cluster should be dropped rather than handed
    # to Nav2 as an unreachable goal sitting on/near the obstacle.
    node = _make_explorer(min_frontier_size=1, min_obstacle_clearance=2.0)
    node._map = _make_grid([0, -1, 100], width=3, height=1)
    assert node._find_frontiers() == []


def test_find_frontiers_keeps_cluster_with_sufficient_clearance():
    # Same grid, but a clearance requirement small enough (0.5m at this
    # grid's resolution=1.0) that the occupied cell 2 cells away no longer
    # falls inside the checked window -- cluster should survive.
    node = _make_explorer(min_frontier_size=1, min_obstacle_clearance=0.5)
    node._map = _make_grid([0, -1, 100], width=3, height=1)
    assert node._find_frontiers() == [(0.5, 0.5, 1)]


def test_pick_goal_prefers_nearest_non_blacklisted():
    node = _make_explorer()
    node._find_frontiers = lambda: [(10.0, 0.0, 6), (1.0, 0.0, 6)]
    assert node._pick_goal((0.0, 0.0, 0.0)) == (1.0, 0.0)


def test_pick_goal_prefers_farther_frontier_over_one_too_close():
    node = _make_explorer(min_goal_distance=0.5)
    node._find_frontiers = lambda: [(0.1, 0.0, 6), (2.0, 0.0, 6)]
    assert node._pick_goal((0.0, 0.0, 0.0)) == (2.0, 0.0)


def test_pick_goal_falls_back_to_close_frontier_if_none_far_enough():
    node = _make_explorer(min_goal_distance=0.5)
    node._find_frontiers = lambda: [(0.1, 0.0, 6), (0.2, 0.0, 6)]
    assert node._pick_goal((0.0, 0.0, 0.0)) == (0.1, 0.0)


def test_pick_goal_skips_blacklisted_frontier():
    node = _make_explorer()
    node._find_frontiers = lambda: [(1.0, 0.0, 6), (10.0, 0.0, 6)]
    node._blacklist = [(1.0, 0.0)]
    assert node._pick_goal((0.0, 0.0, 0.0)) == (10.0, 0.0)


def test_pick_goal_returns_none_when_no_frontiers():
    node = _make_explorer()
    node._find_frontiers = lambda: []
    assert node._pick_goal((0.0, 0.0, 0.0)) is None


def test_pick_goal_prefers_farther_ahead_over_nearer_behind():
    # Robot faces +x (yaw=0). A frontier behind it is nearer in raw
    # distance than one ahead, but the heading penalty should make the
    # ahead one win -- this is what keeps exploration going mostly
    # straight instead of reversing direction for every marginally
    # closer frontier.
    node = _make_explorer(heading_turn_penalty=1.5)
    node._find_frontiers = lambda: [(-1.0, 0.0, 6), (1.5, 0.0, 6)]
    assert node._pick_goal((0.0, 0.0, 0.0)) == (1.5, 0.0)


def test_pick_goal_heading_penalty_zero_falls_back_to_pure_nearest():
    # With the bias disabled, behavior matches the old distance-only picker.
    node = _make_explorer(heading_turn_penalty=0.0)
    node._find_frontiers = lambda: [(-1.0, 0.0, 6), (1.5, 0.0, 6)]
    assert node._pick_goal((0.0, 0.0, 0.0)) == (-1.0, 0.0)
