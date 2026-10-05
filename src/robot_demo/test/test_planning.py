import json
import math
from pathlib import Path
import unittest
from robot_demo.planning import Grid, geodetic_to_xy, wrap_angle

CONFIG = Path(__file__).parents[1] / "config"
CFG = {
    **json.loads((CONFIG / "obstacle_course.json").read_text()),
    **json.loads((CONFIG / "robots.json").read_text())["custom"],
}


def fixture_grid():
    grid = Grid(CFG)
    for o in CFG["obstacles"]:
        for y in range(grid.height):
            for x in range(grid.width):
                wx, wy = grid.world((x, y))
                if abs(wx - o["x"]) <= o["sx"] / 2 and abs(wy - o["y"]) <= o["sy"] / 2:
                    grid.static[y, x] = True
    return grid


class PlanningTests(unittest.TestCase):
    def test_gps_goal_is_finish(self):
        xy = geodetic_to_xy(*CFG["waypoints_gps"][0], CFG["datum"])
        self.assertLess(math.dist(xy, CFG["waypoints_world"][0]), 1e-5)

    def test_safe_detour_and_no_corner_cut(self):
        g = fixture_grid()
        goal = CFG["waypoints_world"][0]
        path = g.plan(CFG["start"], goal, 0)
        self.assertGreater(len(path), 2)
        self.assertEqual(path[0], CFG["start"])
        self.assertEqual(path[-1], goal)
        for a, b in zip(path, path[1:]):
            self.assertTrue(g.line_free(a, b, g.inflated(0)))

    def test_unreachable_goal(self):
        self.assertEqual(fixture_grid().plan(CFG["start"], [-2, -2], 0), [])

    def test_outside_map(self):
        self.assertEqual(Grid(CFG).plan(CFG["start"], [99, 99], 0), [])

    def test_initial_grid_has_no_prior_obstacles(self):
        self.assertFalse(Grid(CFG).static.any())

    def test_escape_newly_inflated_margin_preserves_physical_clearance(self):
        cfg = {
            **CFG,
            "bounds": [0, 6, 0, 6],
            "resolution": 0.1,
            "robot_radius": 0.2,
            "safety_margin": 0.4,
        }
        g = Grid(cfg)
        g.static[30, 30] = True
        start = (2.55, 3.05)
        goal = (1.0, 1.0)
        self.assertTrue(g.inflated(0)[g.cell(start)[1], g.cell(start)[0]])
        path = g.plan(start, goal, 0)
        self.assertTrue(path)
        self.assertTrue(g.line_free(path[0], path[1], g.inflated(0, 0.27)))
        self.assertTrue(g.line_free(path[1], path[2], g.inflated(0)))

    def test_escape_does_not_double_count_cell_padding(self):
        cfg = {**CFG, "robot_radius": 0.36, "safety_margin": 0.25}
        grid = Grid(cfg)
        x, y = grid.cell((2.35, -2.15))
        grid.static[y, x] = True
        start = (1.9869, -2.3873)
        route = grid.plan(start, (5.0, 3.0), 0)
        self.assertTrue(route)
        self.assertTrue(grid.line_free(route[0], route[1], grid.inflated(0, 0.36)))

    def test_no_diagonal_corner_cut(self):
        cfg = {
            **CFG,
            "bounds": [0, 3, 0, 3],
            "resolution": 1.0,
            "robot_radius": 0.0,
            "safety_margin": 0.0,
            "obstacles": [],
        }
        g = Grid(cfg)
        g.static[0, 1] = True
        g.static[1, 0] = True
        self.assertEqual(g.plan((0.5, 0.5), (2.5, 2.5), 0), [])

    def test_open_grid_with_zero_inflation(self):
        cfg = {**CFG, "robot_radius": 0.0, "safety_margin": 0.0}
        self.assertTrue(Grid(cfg).plan(CFG["start"], CFG["waypoints_world"][0], 0))

    def test_angle_wrap(self):
        self.assertAlmostEqual(wrap_angle(2 * math.pi + 0.3), 0.3)


if __name__ == "__main__":
    unittest.main()
