"""Geometry, invalid-data handling and control limits."""
import math
import unittest

import numpy as np
from f1tenth_controller.gap import FollowGap, GapParameters
from f1tenth_controller.control_gate import ControlGate


class GapTests(unittest.TestCase):
    def decide(self, core, ranges, dt=.01):
        return core.decide(ranges,-math.pi/2,math.pi/1080,0.,30.,dt)

    def test_open_road_accelerates_with_bounded_steering(self):
        core=FollowGap()
        decisions=[self.decide(core,np.full(1080,20.)) for _ in range(200)]
        self.assertAlmostEqual(decisions[0].speed,.01)
        self.assertLess(abs(decisions[-1].steering),.01)
        self.assertLessEqual(decisions[-1].speed,1.2)

    def test_invalid_front_and_empty_scan_stop(self):
        for ranges in [[], np.full(1080,np.nan),np.full(1080,-np.inf),np.zeros(1080)]:
            d=self.decide(FollowGap(),ranges)
            self.assertEqual(d.speed,0.)
            self.assertNotEqual(d.reason,'following_gap')

    def test_positive_infinity_is_a_valid_no_return(self):
        self.assertEqual(self.decide(FollowGap(),np.full(1080,np.inf)).reason,'following_gap')

    def test_blocked_space_cannot_become_a_gap(self):
        core=FollowGap();a=np.full(1080,.6)
        self.assertEqual(self.decide(core,a).reason,'no_safe_gap')
        a[540]=.1
        self.assertEqual(self.decide(core,a).speed,0.)

    def test_left_and_right_goals_produce_correct_steering_sign(self):
        for sign in [-1,1]:
            a=np.full(1080,1.1)
            a[700:950] = 6. if sign==1 else 1.1
            a[130:380] = 6. if sign==-1 else 1.1
            core=FollowGap()
            d=self.decide(core,a)
            self.assertEqual(d.reason,'following_gap')
            self.assertGreater(d.steering*sign,0.)
            self.assertLessEqual(abs(d.steering),core.p.max_steering_rate_rad_s*.01+1e-9)

    def test_obstacle_filter_preserves_narrow_invalid_obstacle(self):
        a=np.full(1080,6.);a[540]=np.nan
        self.assertEqual(self.decide(FollowGap(),a).speed,0.)

    def test_wrong_scan_geometry_is_rejected(self):
        core=FollowGap()
        self.assertEqual(core.decide(np.ones(1080),0.,0.,0.,30.,.01).speed,0.)
        self.assertEqual(core.decide(np.ones(1080),2.,.001,0.,30.,.01).speed,0.)

    def test_steering_and_speed_are_bounded_for_persistent_side_gap(self):
        a=np.full(1080,1.1);a[800:1050]=6.
        core=FollowGap()
        previous=0.
        for _ in range(200):
            d=self.decide(core,a)
            self.assertLessEqual(abs(d.steering),core.p.max_steering_rad)
            self.assertLessEqual(abs(d.steering-previous),.02+1e-9)
            self.assertTrue(0<=d.speed<=core.p.max_speed_mps)
            previous=d.steering

    def test_bad_configuration_is_rejected(self):
        with self.assertRaises(ValueError):FollowGap(GapParameters(lookahead_m=0.))


class GateTests(unittest.TestCase):
    def ready(self):
        gate=ControlGate()
        status={'run_id':'a','session_id':'1','state':'ready','paused':False,'ego_collision':False}
        gate.on_status(status,0.)
        gate.scan_received=0.
        return gate,status

    def test_startup_and_reset_require_explicit_enable(self):
        gate,status=self.ready()
        self.assertFalse(gate.armed)
        self.assertTrue(gate.enable(0.,1))
        gate.on_status({**status,'session_id':'2'},.1)
        self.assertFalse(gate.armed)
        self.assertFalse(gate.enable(.1,1))

    def test_pause_and_collision_disarm(self):
        for field in [{'paused':True},{'state':'collision','ego_collision':True}]:
            gate,status=self.ready();gate.enable(0.,1)
            gate.on_status({**status,**field},.1)
            self.assertFalse(gate.armed)

    def test_stale_status_scan_and_competing_controller_reject_enable(self):
        gate,_=self.ready()
        self.assertFalse(gate.enable(0.,2))
        self.assertFalse(gate.enable(.4,1))
        gate.scan_received=1.
        self.assertFalse(gate.enable(1.,1))

    def test_completed_target_disarms_and_cannot_be_enabled_again(self):
        gate, status = self.ready()
        self.assertTrue(gate.enable(0., 1))
        gate.on_status({**status, 'completion_requested': True}, .01)
        self.assertFalse(gate.armed)
        self.assertEqual(gate.reason, 'target_laps_completed')
        self.assertFalse(gate.enable(.01, 1))

    def test_shared_braking_disarms_and_blocks_reenable(self):
        gate, status = self.ready()
        self.assertTrue(gate.enable(0., 1))
        gate.on_status({**status, 'stopping_requested': True}, .01)
        self.assertFalse(gate.armed)
        self.assertEqual(gate.reason, 'manual_stop')
        self.assertFalse(gate.enable(.01, 1))

    def test_controlled_braking_keeps_steering_active_until_terminal_state(self):
        gate, status = self.ready()
        self.assertTrue(gate.enable(0., 1))
        braking = {**status, 'controlled_stopping': True, 'completion_requested': True,
                   'stopping_requested': True}
        gate.on_status(braking, .01)
        self.assertTrue(gate.armed)
        self.assertIsNone(gate.health(.01, 1))
        # Braking does not bypass stale-drive, pause or collision protection.
        self.assertEqual(gate.health(.4, 1), 'stale_scan')
        gate.on_status({**braking, 'state': 'ended'}, .02)
        self.assertFalse(gate.armed)
