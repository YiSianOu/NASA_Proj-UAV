"""Offline checks; no ROS, network or aircraft required."""

import math
import unittest

from keyboard_flight.flight_logic import Flight, Line, MAX_SPEED, Telemetry
from keyboard_flight.mode_select import read_request


class FlightTests(unittest.TestCase):
    def setUp(self):
        self.clock = 10.0
        self.t = Telemetry(mode="AUTO.LOITER", landed="GROUND", position=(2., 3., -4.),
                           velocity=(0., 0., 0.), yaw=0., estimate_ok=True)
        self.commands, self.targets, self.events = [], [], []
        self.f = Flight(self.t, self.commands.append,
                        lambda p, yaw: self.targets.append((p, yaw)), self.events.append)
        self.stamp()
        self.seq = 0

    def stamp(self):
        self.t.position_at = self.t.attitude_at = self.t.estimator_at = self.clock
        self.t.heartbeat_at = self.t.landed_at = self.clock

    def step(self, count=1, follow=True):
        for _ in range(count):
            self.clock += .02
            self.stamp()
            if follow and self.targets:
                self.t.position, self.t.yaw = self.targets[-1]
            self.f.tick(self.clock)

    def request(self, mode, **kwargs):
        self.seq += 1
        self.f.request(dict(id=str(self.seq), mode=mode, **kwargs), self.clock)

    def enter_offboard(self, xyz=(1., 0., 0.), yaw=0):
        self.t.landed, self.t.armed, self.t.yaw = "AIR", True, yaw
        self.request("offboard", **dict(zip("xyz", xyz)))
        self.step(59)
        self.assertNotIn("offboard", self.commands)
        self.step(4)
        self.assertEqual(self.commands, ["offboard"])
        self.t.mode = "OFFBOARD"
        self.step()
        self.assertEqual(self.f.phase, "ALIGN")

    def test_profile_collinear_and_vector_speed_cap(self):
        for delta in ((1, 1, 1), (1, 0, 0), (1, -2, 3), (0, 0, -.005)):
            path = Line((5, -2, -3), delta)
            steps = 500
            dt = path.duration / steps
            last = path.start
            for i in range(1, steps + 1):
                p = path.at(dt * i)
                self.assertLessEqual(math.dist(last, p) / dt, MAX_SPEED + 1e-10)
                progress = [(b - a) / d for b, a, d in zip(p, path.start, path.delta) if d]
                self.assertLessEqual(max(progress) - min(progress), 1e-10)
                last = p
            self.assertEqual(path.at(path.duration), path.end)
        self.assertEqual(Line((0, 0, 0), (0, 0, 0)).at(0), (0, 0, 0))

    def test_takeoff_requires_reported_mode_and_armed_airborne_hold(self):
        self.request("takeoff")
        self.assertEqual(self.commands, ["takeoff"])
        self.f.acknowledge("takeoff", True, "accepted", self.clock)
        self.step()
        self.assertEqual(self.commands, ["takeoff"])
        self.t.mode = "AUTO.TAKEOFF"
        self.step()
        self.assertEqual(self.commands, ["takeoff", "arm"])
        self.t.armed, self.t.landed = True, "TAKING_OFF"
        self.step()
        self.assertEqual(self.f.phase, "TAKING_OFF")
        self.t.landed, self.t.mode = "AIR", "AUTO.LOITER"
        self.step()
        self.assertEqual(self.f.phase, "AIRBORNE")

    def test_wait_for_measured_yaw_then_move_without_fixed_delay(self):
        self.enter_offboard(xyz=(.1, 0, 0), yaw=math.radians(-30))
        self.step(150, follow=False)
        self.assertEqual(self.f.phase, "ALIGN")
        self.assertTrue(all(p == self.f.line.start for p, _ in self.targets))
        self.t.yaw = 0.
        self.step(follow=False)
        self.assertEqual(self.f.phase, "MOVE")
        self.step(follow=False)
        self.assertGreater(self.targets[-1][0][0], self.f.line.start[0])

    def test_hold_and_chain_without_mode_change_or_setpoint_jump(self):
        self.enter_offboard(xyz=(.1, .2, .1))
        self.step(300)
        self.assertEqual(self.f.phase, "HOLD")
        self.assertEqual(self.f.target, (2.1, 3.2, -4.1))
        self.assertEqual(self.events.count("指令完成，等待下個指令"), 1)
        held = self.f.target
        self.t.position = (held[0] + .01, held[1], held[2])
        self.request("offboard", x=0, y=.2, z=0)
        self.assertEqual(self.f.line.start, held)
        self.step(230)
        self.assertEqual(self.f.phase, "HOLD")
        self.assertEqual(self.commands, ["offboard"])
        self.assertAlmostEqual(self.f.target[1], 3.4)
        self.assertEqual(self.events.count("指令完成，等待下個指令"), 2)

    def test_completion_needs_measured_position_and_velocity(self):
        self.enter_offboard(xyz=(.15, 0, 0))
        self.step(180, follow=False)
        self.assertEqual(self.f.phase, "MOVE")
        self.t.position = self.f.line.end
        self.t.velocity = (.1, 0., 0.)
        self.step(40, follow=False)
        self.assertEqual(self.f.phase, "MOVE")
        self.t.velocity = (0., 0., 0.)
        self.step(30)
        self.assertEqual(self.f.phase, "HOLD")

    def test_land_preempts_then_hands_over_on_mode_confirmation(self):
        self.enter_offboard()
        self.step(20)
        target = self.targets[-1]
        self.request("land")
        self.step(4)
        self.assertEqual(self.targets[-4:], [target] * 4)
        self.t.mode, self.t.landed = "AUTO.LAND", "LANDING"
        n = len(self.targets)
        self.step(5)
        self.assertEqual(len(self.targets), n)
        self.assertEqual(self.f.phase, "LANDING")
        self.t.armed, self.t.landed = False, "GROUND"
        self.step()
        self.assertEqual(self.f.phase, "IDLE")

    def test_land_rejection_holds_and_accepts_retry(self):
        self.enter_offboard()
        self.request("land")
        self.f.acknowledge("land", False, "denied", self.clock)
        self.step(50)
        self.assertEqual(self.f.phase, "LAND_RETRY_HOLD")
        self.request("land")
        self.assertEqual(self.commands.count("land"), 2)

    def test_stale_position_stops_stream_and_does_not_resume(self):
        self.enter_offboard()
        n = len(self.targets)
        self.t.position_at -= 2
        self.f.tick(self.clock)
        self.step(10)
        self.assertEqual(self.f.phase, "FAULT")
        self.assertEqual(len(self.targets), n)

    def test_no_premature_arm_after_rejection(self):
        self.request("takeoff")
        self.f.acknowledge("takeoff", False, "denied", self.clock)
        self.t.mode = "AUTO.TAKEOFF"
        self.step()
        self.assertEqual(self.commands, ["takeoff"])

    def test_land_cancels_outstanding_ground_takeoff(self):
        self.request("takeoff")
        self.request("land")
        self.t.mode = "AUTO.TAKEOFF"
        self.step()
        self.assertEqual(self.f.phase, "LAND_REQUEST")
        self.assertNotIn("arm", self.commands)
        self.t.mode = "AUTO.LAND"
        self.step(2)
        self.assertEqual(self.f.phase, "IDLE")

    def test_unexpected_mode_change_stops_motion(self):
        self.enter_offboard()
        n = len(self.targets)
        self.t.mode = "AUTO.RTL"
        self.step()
        self.assertEqual(self.f.phase, "FAULT")
        self.assertEqual(len(self.targets), n)

    def test_invalid_busy_and_duplicate_requests(self):
        self.enter_offboard()
        self.f.request(dict(id="1", mode="offboard", x=1, y=0, z=0), self.clock)
        with self.assertRaises(ValueError):
            self.request("offboard", x=False, y=1, z=0)
        with self.assertRaises(ValueError):
            self.request("offboard", x=1, y=2, z=3)
        self.assertEqual(self.commands, ["offboard"])

    def test_keyboard_prompts_only_for_offboard(self):
        def read(values):
            return read_request(lambda prompt: next(values), lambda text: None)
        for mode in ("takeoff", "land"):
            self.assertEqual(read(iter([mode])), {"mode": mode})
        self.assertEqual(read(iter(["offboard", "1", "nan", "-0.5", ""])),
                         dict(mode="offboard", x=1, y=-.5, z=0))
        self.assertIsNone(read(iter(["offboard", "1", "cancel"])))
        self.assertEqual(read(iter(["offboard", "land"])), {"mode": "land"})


if __name__ == "__main__":
    unittest.main()
