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


def _make_explorer(min_frontier_size=1, blacklist_radius=0.4, min_goal_distance=0.0):
    node = FrontierExploreNode.__new__(FrontierExploreNode)
    node._min_frontier_size = min_frontier_size
    node._blacklist_radius = blacklist_radius
    node._min_goal_distance = min_goal_distance
    node._blacklist = []
    node._map = None
    return node


def test_find_frontiers_single_cell():
    node = _make_explorer()
    node._map = _make_grid([0, 0, 0, -1, -1], width=5, height=1)
    assert node._find_frontiers() == [(2.5, 0.5, 1)]


def test_find_frontiers_clusters_adjacent_cells():
    node = _make_explorer()
    node._map = _make_grid([0, 0, -1, 0, 0, -1], width=3, height=2)
    assert node._find_frontiers() == [(1.5, 1.0, 2)]


def test_find_frontiers_below_min_size_excluded():
    node = _make_explorer(min_frontier_size=3)
    node._map = _make_grid([0, 0, -1, 0, 0, -1], width=3, height=2)
    assert node._find_frontiers() == []


def test_pick_goal_prefers_nearest_non_blacklisted():
    node = _make_explorer()
    node._find_frontiers = lambda: [(10.0, 0.0, 6), (1.0, 0.0, 6)]
    assert node._pick_goal((0.0, 0.0)) == (1.0, 0.0)


def test_pick_goal_prefers_farther_frontier_over_one_too_close():
    node = _make_explorer(min_goal_distance=0.5)
    node._find_frontiers = lambda: [(0.1, 0.0, 6), (2.0, 0.0, 6)]
    assert node._pick_goal((0.0, 0.0)) == (2.0, 0.0)


def test_pick_goal_falls_back_to_close_frontier_if_none_far_enough():
    node = _make_explorer(min_goal_distance=0.5)
    node._find_frontiers = lambda: [(0.1, 0.0, 6), (0.2, 0.0, 6)]
    assert node._pick_goal((0.0, 0.0)) == (0.1, 0.0)


def test_pick_goal_skips_blacklisted_frontier():
    node = _make_explorer()
    node._find_frontiers = lambda: [(1.0, 0.0, 6), (10.0, 0.0, 6)]
    node._blacklist = [(1.0, 0.0)]
    assert node._pick_goal((0.0, 0.0)) == (10.0, 0.0)


def test_pick_goal_returns_none_when_no_frontiers():
    node = _make_explorer()
    node._find_frontiers = lambda: []
    assert node._pick_goal((0.0, 0.0)) is None
