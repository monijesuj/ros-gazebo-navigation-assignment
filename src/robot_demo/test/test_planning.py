import json
import math
from pathlib import Path
import unittest
import numpy as np
from robot_demo.planning import Grid, geodetic_to_xy, wrap_angle
CFG=json.loads((Path(__file__).parents[1]/'config/mission.json').read_text())

class PlanningTests(unittest.TestCase):
    def test_gps_goal_is_finish(self):
        xy=geodetic_to_xy(*CFG['waypoints_gps'][0],CFG['datum'])
        self.assertLess(math.dist(xy,CFG['waypoints_world'][0]),1e-5)
    def test_safe_detour_and_no_corner_cut(self):
        g=Grid(CFG);goal=CFG['waypoints_world'][0]
        path=g.plan(CFG['start'],goal,0)
        self.assertGreater(len(path),2)
        self.assertEqual(path[0],CFG['start']);self.assertEqual(path[-1],goal)
        for a,b in zip(path,path[1:]):self.assertTrue(g.line_free(a,b,g.inflated(0)))
    def test_unreachable_goal(self):
        self.assertEqual(Grid(CFG).plan(CFG['start'],[-2,-2],0),[])
    def test_outside_map(self):
        self.assertEqual(Grid(CFG).plan(CFG['start'],[99,99],0),[])
    def test_observed_obstacle_and_expiry(self):
        g=Grid(CFG);g.observe((-4,-3),1)
        x,y=g.cell((-4,-3));self.assertTrue(g.occupied(2)[y,x]);self.assertFalse(g.occupied(5)[y,x])
    def test_no_diagonal_corner_cut(self):
        cfg={**CFG,'bounds':[0,3,0,3],'resolution':1.,'robot_radius':0.,'safety_margin':0.,'obstacles':[]}
        g=Grid(cfg);g.static[0,1]=True;g.static[1,0]=True
        self.assertEqual(g.plan((.5,.5),(2.5,2.5),0),[])
    def test_angle_wrap(self):self.assertAlmostEqual(wrap_angle(2*math.pi+.3),.3)

if __name__=='__main__':unittest.main()
