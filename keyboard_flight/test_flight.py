"""Offline regression checks. Run: python3 -B -m unittest keyboard_flight.test_flight -v

No ROS installation, serial connection, or aircraft is used. Adapter tests stub
ROS/MAVLink transport only; they exercise the actual controller/monitor methods.
"""
import contextlib
import importlib
import io
import json
import math
import sys
import types
import unittest
from unittest.mock import Mock, patch

from keyboard_flight.flight_logic import Flight, Line, MAX_SPEED, Telemetry
from keyboard_flight.mode_select import read_request


class FlightTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.
        self.t = Telemetry(mode="POSCTL", landed="GROUND", position=(2., 3., -4.),
                           velocity=(0., 0., 0.), yaw=0., estimate_ok=True)
        self.commands, self.targets, self.events = [], [], []
        self.f = Flight(self.t, self.commands.append,
                        lambda p, y, v: self.targets.append((p, y, v)), self.events.append)
        self.seq = 0
        self.stamp()

    def stamp(self):
        self.t.heartbeat_at = self.t.position_at = self.t.attitude_at = self.now
        self.t.landed_at = self.t.estimator_at = self.now
        self.t.parameters = {"LNDMC_Z_VEL_MAX": (.05, self.now), "MPC_Z_VEL_MAX_DN": (1.5, self.now)}

    def tick(self, dt=.02, fresh=True):
        self.now += dt
        if fresh:
            self.stamp()
        self.f.tick(self.now)

    def request(self, mode, **values):
        self.seq += 1
        self.f.request(dict(id=str(self.seq), mode=mode, **values), self.now)

    def arm(self):
        self.request("arm")
        self.tick()
        self.tick(1.21)
        self.assertEqual(self.commands, ["offboard"])
        self.t.mode = "OFFBOARD"
        self.tick()
        self.assertEqual(self.commands, ["offboard", "arm"])
        self.t.armed = True
        self.tick()
        self.assertEqual(self.f.phase, "ARMED_GROUND")

    def airborne(self, height=.25):
        self.arm()
        self.request("takeoff", height=height)
        self.t.position = self.f.line.end
        self.t.landed = "AIR"
        self.tick()
        self.assertEqual(self.f.phase, "HOLD")

    def test_small_positive_height_initialization_keyboard_and_request(self):
        Flight(self.t, self.commands.append, lambda *args: None, self.events.append, takeoff_height=.01)
        inputs = iter(["1", ".01"])
        self.assertEqual(read_request(lambda _: next(inputs)), {"mode": "takeoff", "height": .01})
        self.arm()
        self.request("takeoff", height=.01)
        self.assertAlmostEqual(self.f.takeoff_origin[2], -4.01)

    def test_zero_negative_and_nonfinite_height_still_rejected(self):
        self.arm()
        for height in (0, -.1, math.nan, math.inf, True):
            with self.subTest(height=height), self.assertRaises(ValueError):
                self.request("takeoff", height=height)
        messages = []
        inputs = iter(["1", "0", "nan", ".05"])
        self.assertEqual(read_request(lambda _: next(inputs), messages.append)["height"], .05)
        self.assertEqual(len(messages), 2)

    def test_xy_keyboard_has_no_z_prompt(self):
        prompts = []
        inputs = iter(["2", ".3", "-.2"])
        def read(prompt):
            prompts.append(prompt)
            return next(inputs)
        self.assertEqual(read_request(read, lambda _: None), {"mode": "offboard", "x": .3, "y": -.2})
        self.assertEqual(len(prompts), 3)

    def test_xy_is_fixed_origin_not_incremental_or_current_position(self):
        self.airborne()
        origin = self.f.takeoff_origin
        for x, y in ((.3, .2), (.3, .2), (.1, 0), (0, 0), (1., -.75)):
            self.t.position = (8., -9., -4.1)  # Actual error must not change the origin.
            self.t.velocity = (.4, 0., 0.)
            reference = self.f.target
            self.request("offboard", x=x, y=y)
            self.assertEqual(self.f.line.start, reference)
            for actual, expected in zip(self.f.line.end, (origin[0] + x, origin[1] + y, origin[2])):
                self.assertAlmostEqual(actual, expected)
            self.assertEqual(self.f.takeoff_origin, origin)
            self.tick()
            self.tick(.1)

    def test_xy_move_stays_on_takeoff_target_plane(self):
        self.airborne()
        self.request("offboard", x=.3, y=.2)
        self.tick()  # ALIGN -> MOVE
        for _ in range(80):
            self.tick()
            self.assertAlmostEqual(self.targets[-1][0][2], self.f.takeoff_origin[2])

    def test_z_changes_are_not_silently_ignored(self):
        self.airborne()
        for z in (.01, -.01, math.nan, math.inf, "0", True):
            with self.subTest(z=z), self.assertRaisesRegex(ValueError, "僅接受 XY"):
                self.request("offboard", x=0, y=0, z=z)
        self.request("offboard", x=0, y=0, z=0)

    def test_xy_during_airborne_takeoff_uses_fixed_altitude_target(self):
        self.arm()
        self.request("takeoff", height=.25)
        self.t.landed = "AIR"
        self.tick(.1)
        previous_xy = self.f.target[:2]
        self.request("offboard", x=.2, y=.1)
        self.assertEqual(self.f.line.start[:2], previous_xy)
        self.assertEqual(self.f.line.start[2], self.f.takeoff_origin[2])
        self.assertEqual(self.f.line.end[2], self.f.takeoff_origin[2])

    def test_new_takeoff_sets_a_new_origin(self):
        self.airborne()
        old = self.f.takeoff_origin
        self.f.phase = "IDLE"
        self.t.armed = False
        self.t.landed = "GROUND"
        self.t.position = (5., 6., -7.)
        self.request("arm")
        self.assertIsNone(self.f.takeoff_origin)
        self.f.phase = "ARMED_GROUND"
        self.t.armed = True
        self.request("takeoff", height=.2)
        self.assertNotEqual(self.f.takeoff_origin, old)
        self.assertEqual(self.f.takeoff_origin, (5., 6., -7.2))

    def test_existing_point15_target_floor_is_unchanged(self):
        self.airborne(.1)
        with self.assertRaisesRegex(ValueError, "目標過低"):
            self.request("offboard", x=.1, y=0)

    def test_position_error_and_old_timeout_warn_without_stopping(self):
        self.airborne()
        self.request("offboard", x=1., y=0.)
        self.tick()
        count = len(self.targets)
        self.tick(100.)
        self.tick(2.)
        self.assertEqual(self.f.phase, "MOVE")
        self.assertGreater(len(self.targets), count)
        self.assertEqual(self.f.target, self.f.line.end)
        self.assertTrue(any("追蹤誤差" in s for s in self.f.diagnostics(self.now)["warnings"]))
        self.request("offboard", x=.1, y=0.)
        self.assertEqual(self.f.phase, "ALIGN")

    def test_takeoff_has_no_arrival_timeout(self):
        self.arm()
        self.request("takeoff", height=.25)
        self.tick(100.)
        self.assertEqual(self.f.phase, "TAKING_OFF")
        self.assertIsNotNone(self.targets[-1][0])

    def test_arrival_needs_no_trajectory_completion_speed_or_dwell(self):
        self.airborne()
        self.request("offboard", x=.5, y=0)
        self.tick()
        self.t.position = self.f.line.end
        self.t.velocity = (.3, 0., 0.)
        self.tick(.02)
        self.assertEqual(self.f.phase, "HOLD")
        self.assertEqual(self.f.target, self.f.line.end)
        self.assertTrue(any("到點速度" in s for s in self.f.diagnostics(self.now)["warnings"]))
        self.request("offboard", x=.1, y=0)
        self.assertEqual(self.f.phase, "ALIGN")

    def test_no_stale_arrival_confirmation(self):
        self.airborne()
        self.request("offboard", x=.3, y=0)
        self.tick()
        self.t.position = self.f.line.end
        self.tick(10., fresh=False)
        self.assertEqual(self.f.phase, "MOVE")
        self.tick()
        self.assertEqual(self.f.phase, "HOLD")

    def test_stale_telemetry_is_warning_only_and_new_xy_is_accepted(self):
        self.airborne()
        before = len(self.targets)
        self.tick(20., fresh=False)
        self.assertEqual(self.f.phase, "HOLD")
        self.assertEqual(len(self.targets), before + 1)
        self.assertFalse(self.t.navigation_ok(self.now))
        self.assertTrue(self.t.navigation_valid())
        warnings = self.f.diagnostics(self.now)["warnings"]
        self.assertEqual(sum("過期" in s for s in warnings), 5)
        self.request("offboard", x=.2, y=0.)
        self.tick(10., fresh=False)
        self.assertEqual(self.f.phase, "ALIGN")  # No new attitude to confirm ready.
        self.tick()
        self.assertEqual(self.f.phase, "MOVE")

    def test_alignment_wait_is_unbounded_and_ready_is_once(self):
        self.airborne()
        self.t.yaw = self.f.yaw_ref = .8
        self.request("offboard", x=.2, y=.2)
        self.tick(100.)
        self.assertEqual(self.f.phase, "ALIGN")
        self.assertEqual(self.f.ready_count, 0)
        self.t.yaw = 0.
        self.tick()
        self.assertEqual(self.f.phase, "MOVE")
        self.tick()
        self.assertEqual(self.f.ready_count, 1)
        self.assertEqual(sum("ready" in s for s in self.events), 1)

    def test_mode_and_arming_waits_are_unbounded_but_confirmed(self):
        self.request("arm")
        self.tick()
        self.tick(1.21)
        self.tick(100.)
        self.assertEqual(self.f.phase, "SELECT_OFFBOARD")
        self.assertEqual(self.commands, ["offboard"])
        self.t.mode = "OFFBOARD"
        self.tick()
        self.tick(100.)
        self.assertEqual(self.f.phase, "ARMING")
        self.assertEqual(self.commands, ["offboard", "arm"])
        self.t.armed = True
        self.tick()
        self.assertEqual(self.f.phase, "ARMED_GROUND")

    def test_px4_rejection_keeps_existing_stream(self):
        self.request("arm")
        self.tick()
        self.tick(1.21)
        self.f.acknowledge("offboard", False, "DENIED", self.now)
        before = len(self.targets)
        self.tick(20.)
        self.assertEqual(self.f.phase, "SELECT_OFFBOARD")
        self.assertEqual(len(self.targets), before + 1)
        self.assertTrue(any("DENIED" in text for text in self.f.diagnostics(self.now)["warnings"]))
        self.assertEqual(self.commands, ["offboard"])

    def test_external_arm_in_offboard_reconciles_without_stopping(self):
        self.request("arm")
        self.t.armed = True
        self.t.mode = "OFFBOARD"
        self.tick()
        self.assertEqual(self.f.phase, "ARMED_GROUND")
        self.assertEqual(self.targets[-1][2], (0., 0., 0.))
        self.assertEqual(self.commands, [])

    def test_ground_state_change_and_estimator_flags_do_not_exit(self):
        self.request("arm")
        self.t.estimate_ok = False
        self.t.landed = "AIR"
        self.tick()
        self.assertEqual(self.f.phase, "PRIME")
        self.assertEqual(self.targets[-1][2], (0., 0., 0.))
        self.assertTrue(self.f.diagnostics(self.now)["warnings"])

    def test_external_airborne_arm_keeps_zero_velocity_until_user_command(self):
        self.request("arm")
        self.t.armed = True
        self.t.mode = "OFFBOARD"
        self.t.landed = "AIR"
        self.tick()
        self.assertEqual(self.f.phase, "WAIT_USER")
        self.tick()
        self.assertEqual(self.targets[-1][2], (0., 0., 0.))
        self.request("land")
        self.assertEqual(self.f.phase, "LAND_BRAKE")

    def test_invalid_data_replays_last_target_without_advancing_then_recovers(self):
        self.airborne()
        self.request("offboard", x=.5, y=0.)
        self.tick()
        self.tick(.5)
        last = self.targets[-1]
        old_position = self.t.position
        self.t.position = (math.nan, 0., 0.)
        self.tick()
        self.tick(20.)
        self.assertEqual(self.targets[-1], last)
        self.assertEqual(self.f.phase, "MOVE")
        self.assertTrue(self.f.diagnostics(self.now)["data_hold"])
        self.t.position = old_position
        self.tick()
        self.assertIsNone(self.f.data_paused_at)
        self.assertLess(math.dist(self.targets[-1][0], last[0]), .01)
        self.assertNotEqual(self.targets[-1][0], self.f.line.end)

    def test_estimator_and_air_state_changes_are_diagnostics_only(self):
        self.airborne()
        self.request("offboard", x=.5, y=0.)
        self.tick()
        self.t.estimate_ok = False
        self.t.landed = "UNKNOWN"
        before = len(self.targets)
        self.tick(2.)
        self.assertEqual(self.f.phase, "MOVE")
        self.assertEqual(len(self.targets), before + 1)
        self.assertIsNone(self.f.data_paused_at)
        self.assertTrue(any("非 AIR" in w for w in self.f.diagnostics(self.now)["warnings"]))

    def test_px4_mode_change_releases_without_reclaim_even_after_recovery(self):
        self.airborne()
        self.t.mode = "AUTO.LAND"
        before = len(self.targets)
        commands = list(self.commands)
        self.tick()
        self.assertEqual(self.f.phase, "PX4_CONTROL")
        self.t.mode = "OFFBOARD"
        self.tick()
        self.assertEqual(len(self.targets), before)
        self.assertEqual(self.commands, commands)

    def test_px4_disarm_releases_even_without_ground_confirmation(self):
        self.airborne()
        self.t.armed = False
        before = len(self.targets)
        self.tick()
        self.assertEqual(self.f.phase, "IDLE")
        self.assertEqual(len(self.targets), before)

    def test_mode_change_during_arming_releases_without_exit(self):
        self.arm()
        self.t.mode = "POSCTL"
        before = len(self.targets)
        self.tick()
        self.assertEqual(self.f.phase, "PX4_CONTROL")
        self.assertEqual(len(self.targets), before)

    def test_frame_reset_invalidates_origin_and_streams_zero_without_old_yaw(self):
        self.airborne()
        self.request("offboard", x=.5, y=.1)
        self.tick()
        self.f.invalidate_frame(self.now)
        self.tick()
        self.assertEqual(self.f.phase, "FRAME_RESET")
        self.assertIsNone(self.f.takeoff_origin)
        self.assertIsNone(self.f.line)
        self.assertEqual(self.targets[-1], (None, None, (0., 0., 0.)))
        with self.assertRaises(ValueError):
            self.request("offboard", x=0., y=0.)
        self.request("land")
        self.assertEqual(self.f.phase, "LAND_BRAKE")

    def test_landing_brake_timeout_is_warning_only(self):
        self.airborne()
        self.request("land")
        self.t.velocity = (.2, 0., 0.)
        before = len(self.targets)
        self.tick(20.)
        self.assertEqual(self.f.phase, "LAND_BRAKE")
        self.assertEqual(len(self.targets), before + 1)
        self.assertTrue(any("尚未穩定速度" in w for w in self.f.diagnostics(self.now)["warnings"]))

    def test_landing_timeout_continues_descent(self):
        self.airborne()
        self.request("land")
        for _ in range(30):
            self.tick()
        self.tick(150.)
        self.assertEqual(self.f.phase, "DESCEND")
        self.assertGreater(self.targets[-1][2][2], 0.)
        self.assertTrue(any("降落等待" in w for w in self.f.diagnostics(self.now)["warnings"]))

    def test_invalid_data_during_descent_replays_finite_velocity(self):
        self.airborne()
        self.request("land")
        for _ in range(30):
            self.tick()
        self.tick(1.)
        last = self.targets[-1]
        self.t.velocity = None
        self.tick()
        self.tick(2.)
        self.assertEqual(self.f.phase, "DESCEND")
        self.assertEqual(self.targets[-1], last)
        self.assertTrue(self.f.diagnostics(self.now)["data_hold"])

    def test_ground_cancel_timeout_and_changed_state_keep_stream(self):
        self.arm()
        self.request("land")
        self.tick(20.)
        self.assertEqual(self.f.phase, "GROUND_STOP")
        self.assertEqual(self.targets[-1][2], (0., 0., 0.))
        commands = list(self.commands)
        self.t.landed = "AIR"
        self.tick()
        self.assertEqual(self.commands, commands)  # Never resend ground disarm into AIR.
        self.assertEqual(self.targets[-1][2], (0., 0., 0.))
        self.request("land")  # Explicit new landing request can recover this state.
        self.assertEqual(self.f.phase, "LAND_BRAKE")

    def test_normal_disarm_timeout_continues_stream_and_never_force_disarms(self):
        self.airborne()
        self.request("land")
        for _ in range(30):
            self.tick()
        self.t.landed = "GROUND"
        for _ in range(60):
            self.tick()
        self.assertEqual(self.f.phase, "DISARMING")
        before = len(self.targets)
        self.tick(20.)
        self.assertEqual(self.f.phase, "DISARMING")
        self.assertEqual(len(self.targets), before + 1)
        self.assertNotIn("kill", self.commands)

    def test_landing_and_normal_disarm_are_preserved(self):
        self.airborne()
        self.request("land")
        for _ in range(30):
            self.tick()
        self.assertEqual(self.f.phase, "DESCEND")
        self.tick(1.)
        velocity = self.targets[-1][2]
        self.assertLessEqual(math.hypot(*velocity), .1 + 1e-12)
        self.assertGreater(velocity[2], 0.)
        self.t.landed = "GROUND"
        for _ in range(60):
            self.tick()
        self.assertEqual(self.f.phase, "DISARMING")
        self.assertIn("disarm", self.commands)
        self.assertNotIn("kill", self.commands)
        self.t.armed = False
        self.tick()
        self.assertEqual(self.f.phase, "IDLE")

    def test_stale_ground_does_not_confirm_landing(self):
        self.airborne()
        self.request("land")
        for _ in range(30):
            self.tick()
        self.t.landed = "GROUND"
        self.tick(2., fresh=False)
        self.assertEqual(self.f.phase, "DESCEND")
        self.assertNotIn("disarm", self.commands)

    def test_existing_landing_parameter_gate_is_preserved(self):
        self.t.parameters["LNDMC_Z_VEL_MAX"] = (.25, self.now)
        with self.assertRaisesRegex(ValueError, "不相容"):
            self.request("arm")

    def test_estimator_invalid_does_not_block_an_otherwise_valid_xy_request(self):
        self.airborne()
        self.t.estimate_ok = False
        self.request("offboard", x=.2, y=.1)
        self.tick()
        self.assertEqual(self.f.phase, "MOVE")
        self.assertTrue(self.f.diagnostics(self.now)["warnings"])

    def test_recovery_after_reset_requires_a_new_takeoff_to_restore_origin(self):
        self.airborne()
        self.f.invalidate_frame(self.now)
        self.t.armed = False
        self.t.landed = "GROUND"
        self.tick()
        self.assertEqual(self.f.phase, "IDLE")
        self.t.position = (8., 9., -1.)
        self.request("arm")
        self.tick()
        self.tick(1.21)
        self.tick()
        self.t.armed = True
        self.tick()
        self.request("takeoff", height=.25)
        self.assertEqual(self.f.takeoff_origin, (8., 9., -1.25))
        self.assertFalse(self.f.frame_invalid)

    def test_disarm_rejection_is_a_monitor_warning_without_interrupt(self):
        self.arm()
        self.request("land")
        self.f.acknowledge("disarm", False, "DENIED", self.now)
        before = len(self.targets)
        self.tick(10.)
        self.assertEqual(self.f.phase, "GROUND_STOP")
        self.assertEqual(len(self.targets), before + 1)
        self.assertTrue(any("DENIED" in w for w in self.f.diagnostics(self.now - .1)["warnings"]))

    def test_kill_latches_and_needs_disarm_confirmation(self):
        self.airborne()
        self.request("kill")
        self.assertEqual(self.commands[-1], "kill")
        with self.assertRaisesRegex(ValueError, "鎖存"):
            self.request("offboard", x=0, y=0)
        self.t.armed = False
        self.tick()
        self.tick(1.1)
        self.assertEqual(self.f.phase, "KILLED")

    def test_half_cosine_reference_speed_and_zero_length(self):
        for delta in ((1., 1., 1.), (.5, 0., 0.), (0., 0., .25)):
            line = Line((0., 0., 0.), delta)
            dt = line.duration / 500
            last = line.start
            for n in range(1, 501):
                current = line.at(n * dt)
                self.assertLessEqual(math.dist(last, current) / dt, MAX_SPEED + 1e-10)
                last = current
            self.assertEqual(last, line.end)
        self.assertEqual(Line((1, 2, 3), (0, 0, 0)).at(0), (1, 2, 3))


def import_adapters():
    """Import the actual application with inert ROS and MAVLink transports."""
    modules = {name: types.ModuleType(name) for name in (
        "rclpy", "rclpy.clock", "rclpy.duration", "rclpy.executors", "rclpy.node", "rclpy.qos",
        "std_msgs", "std_msgs.msg", "pymavlink", "pymavlink.mavutil")}
    modules["rclpy.node"].Node = object
    modules["rclpy.clock"].Clock = Mock()
    modules["rclpy.clock"].ClockType = types.SimpleNamespace(STEADY_TIME=1)
    modules["rclpy.duration"].Duration = Mock()
    modules["rclpy.qos"].QoSProfile = Mock()
    modules["rclpy.executors"].ExternalShutdownException = type("ExternalShutdownException", (Exception,), {})
    modules["std_msgs.msg"].String = lambda **kw: types.SimpleNamespace(**kw)
    class Constants:
        def __init__(self):
            self.values = {}
        def __getattr__(self, name):
            if name not in self.values:
                self.values[name] = 1 << len(self.values)
            return self.values[name]
    modules["pymavlink.mavutil"].mavlink = Constants()
    modules["pymavlink"].mavutil = modules["pymavlink.mavutil"]
    with patch.dict(sys.modules, modules):
        controller = importlib.import_module("keyboard_flight.main")
        monitor = importlib.import_module("keyboard_flight.px4_states_monitor")
    return controller, monitor


class AdapterTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.controller_module, cls.monitor_module = import_adapters()

    def make_controller(self):
        module = self.controller_module
        c = module.Controller.__new__(module.Controller)
        c.system, c.source = 1, 245
        c.boot_ms = 1000
        c.boot_received_at = 10.
        c.px4_setpoint = module.PX4SetpointTelemetry()
        c.px4_attitude_setpoint = module.PX4AttitudeTelemetry()
        c.px4_boot_epoch = 0
        c.t = Telemetry(mode="OFFBOARD", armed=True, landed="AIR", position=(2., 3., -4.25),
                        velocity=(.1, 0., 0.), yaw=0., estimate_ok=True,
                        heartbeat_at=10., position_at=10., attitude_at=10., landed_at=10., estimator_at=10.)
        c.states = Mock()
        c.events = Mock()
        c.last_note = ""
        c.link = Mock()
        c.flight = Flight(c.t, Mock(), Mock(), Mock())
        c.flight.phase = "HOLD"
        c.flight.target = (2.5, 3., -4.25)
        c.flight.ground_z = -4.
        c.flight.takeoff_origin = (2., 3., -4.25)
        return c

    def test_snapshot_exposes_warnings_and_takeoff_frame_even_if_px4_is_stale(self):
        c = self.make_controller()
        c.snapshot(20.)
        state = json.loads(c.states.publish.call_args.args[0].data)
        self.assertFalse(state["online"])
        self.assertEqual(state["phase"], "HOLD")
        self.assertEqual(len(state["warnings"]), 5)
        c.snapshot(10.)
        state = json.loads(c.states.publish.call_args.args[0].data)
        self.assertEqual(state["xy_from_takeoff_target"], [0., 0.])
        self.assertTrue(any("追蹤誤差" in s for s in state["warnings"]))
        self.assertTrue(any("到點速度" in s for s in state["warnings"]))

    def test_monitor_prints_warnings_even_with_old_px4_heartbeat(self):
        c = self.make_controller()
        c.snapshot(20.)
        m = self.monitor_module.Monitor.__new__(self.monitor_module.Monitor)
        m.latest = json.loads(c.states.publish.call_args.args[0].data)
        m.last_rx = 20.
        with patch.object(self.monitor_module.time, "monotonic", return_value=20.), contextlib.redirect_stdout(io.StringIO()) as output:
            m.print_state()
        self.assertIn("[WARN] HEARTBEAT", output.getvalue())
        self.assertIn("phase: HOLD", output.getvalue())

    def test_monitor_ready_printed_once(self):
        m = self.monitor_module.Monitor.__new__(self.monitor_module.Monitor)
        m.previous_phase, m.previous_ready_count = None, 0
        msg = types.SimpleNamespace(data=json.dumps(dict(phase="MOVE", ready_count=1, online=True)))
        with contextlib.redirect_stdout(io.StringIO()) as output:
            m.state(msg)
            m.state(msg)
        self.assertEqual(output.getvalue().count("ready"), 1)

    def packet(self, kind, **fields):
        return types.SimpleNamespace(get_srcSystem=lambda: 1,
                                     get_srcComponent=lambda: self.controller_module.M.MAV_COMP_ID_AUTOPILOT1,
                                     get_type=lambda: kind, **fields)

    def setpoint_packet(self, **updates):
        fields = dict(time_boot_ms=1000, coordinate_frame=self.controller_module.M.MAV_FRAME_LOCAL_NED,
                      type_mask=0, x=10., y=20., z=-.25, vx=.1, vy=.2, vz=.3,
                      afx=0., afy=0., afz=0., yaw=.5, yaw_rate=.01)
        fields.update(updates)
        return self.packet("POSITION_TARGET_LOCAL_NED", **fields)

    def test_px4_setpoint_request_and_independent_received_source(self):
        c = self.make_controller()
        self.assertIn((self.controller_module.M.MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED, 10),
                      self.controller_module.RATES)
        before = c.flight.target
        c.receive(self.setpoint_packet(), 10.)
        c.snapshot(10.)
        state = json.loads(c.states.publish.call_args.args[0].data)
        self.assertEqual(state["target_ned"], list(before))
        self.assertEqual(state["px4_setpoint"]["position_ned"], [10., 20., -.25])
        self.assertEqual(state["px4_setpoint"]["message_id"], 85)
        self.assertEqual(state["px4_setpoint"]["status"], "available")
        self.assertEqual(state["position_time_boot_ms"], 1000)
        self.assertEqual(c.flight.phase, "HOLD")
        c.flight.send_command.assert_not_called()
        c.flight.send_target.assert_not_called()

    def test_px4_partial_axes_and_nonfinite_values_are_json_null(self):
        c = self.make_controller()
        m = self.controller_module.M
        c.receive(self.setpoint_packet(type_mask=m.POSITION_TARGET_TYPEMASK_X_IGNORE,
                                      y=math.nan, vx=math.inf, yaw=math.nan), 10.)
        c.snapshot(10.)
        sp = json.loads(c.states.publish.call_args.args[0].data)["px4_setpoint"]
        self.assertEqual(sp["position_ned"], [None, None, -.25])
        self.assertEqual(sp["velocity_ned"], [None, .2, .3])
        self.assertIsNone(sp["yaw_rad"])

    def test_px4_inactive_axes_differ_from_no_packet_and_stale(self):
        c = self.make_controller()
        self.assertEqual(c.px4_setpoint.snapshot(10.)["status"], "not_received")
        m = self.controller_module.M
        mask = m.POSITION_TARGET_TYPEMASK_X_IGNORE | m.POSITION_TARGET_TYPEMASK_Y_IGNORE | m.POSITION_TARGET_TYPEMASK_Z_IGNORE
        c.receive(self.setpoint_packet(type_mask=mask), 10.)
        sp = c.px4_setpoint.snapshot(10.1)
        self.assertEqual(sp["status"], "no_position_axes")
        self.assertEqual(sp["velocity_ned"], [.1, .2, .3])
        sp = c.px4_setpoint.snapshot(10.5)
        self.assertEqual(sp["status"], "stale")
        self.assertEqual(sp["velocity_ned"], [None] * 3)
        self.assertTrue(sp["has_received"])

    def test_px4_wrong_sender_and_unsupported_frame_do_not_plot_wrong_coordinates(self):
        c = self.make_controller()
        msg = self.setpoint_packet()
        msg.get_srcSystem = lambda: 99
        c.receive(msg, 10.)
        self.assertIsNone(c.px4_setpoint.sample)
        c.receive(self.setpoint_packet(coordinate_frame=-1), 10.)
        sp = c.px4_setpoint.snapshot(10.)
        self.assertEqual(sp["status"], "unsupported_frame")
        self.assertEqual(sp["position_ned"], [None] * 3)

    def test_px4_repeated_and_old_timestamps_do_not_refresh_a_stale_target(self):
        c = self.make_controller()
        c.receive(self.setpoint_packet(), 10.)
        c.receive(self.setpoint_packet(), 10.6)
        c.receive(self.setpoint_packet(time_boot_ms=999), 10.7)
        self.assertEqual(c.px4_setpoint.snapshot(10.7)["status"], "stale")
        c.receive(self.setpoint_packet(time_boot_ms=1100), 10.8)
        self.assertTrue(c.px4_setpoint.snapshot(10.8)["fresh"])

    def test_px4_setpoint_clock_wrap_is_not_rejected(self):
        c = self.make_controller()
        c.receive(self.setpoint_packet(time_boot_ms=0xfffffff0), 10.)
        c.receive(self.setpoint_packet(time_boot_ms=16), 10.1)
        self.assertEqual(c.px4_setpoint.snapshot(10.1)["time_boot_ms"], 16)

    def test_px4_reset_clears_setpoint_and_starts_a_new_display_epoch(self):
        c = self.make_controller()
        c.receive(self.setpoint_packet(), 10.)
        c.receive(self.packet("LOCAL_POSITION_NED", time_boot_ms=100, x=2., y=3., z=-4.,
                              vx=0., vy=0., vz=0.), 10.1)
        self.assertIsNone(c.px4_setpoint.sample)
        self.assertEqual(c.px4_boot_epoch, 1)
        c.receive(self.setpoint_packet(time_boot_ms=150), 10.2)
        self.assertEqual(c.px4_setpoint.snapshot(10.2)["time_boot_ms"], 150)

    def attitude_target_packet(self, **updates):
        yaw = math.radians(60.)
        values = dict(time_boot_ms=1000, type_mask=0, q=[math.cos(yaw / 2), 0., 0., math.sin(yaw / 2)])
        values.update(updates)
        return self.packet("ATTITUDE_TARGET", **values)

    def test_attitude_target_decodes_ned_yaw_in_rc_modes(self):
        for mode in ("POSCTL", "ALTCTL", "STABILIZED", "OFFBOARD"):
            c = self.make_controller()
            c.t.mode = mode
            c.flight.phase = "PX4_CONTROL"
            c.receive(self.attitude_target_packet(), 10.)
            c.snapshot(10.)
            sp = json.loads(c.states.publish.call_args.args[0].data)["px4_attitude_setpoint"]
            self.assertAlmostEqual(sp["yaw_rad"], math.radians(60.))
            self.assertTrue(sp["fresh"])
            self.assertEqual(sp["source"], "ATTITUDE_TARGET")
            c.flight.send_command.assert_not_called()

    def test_attitude_ignore_bad_quaternion_and_nan_do_not_fabricate_yaw(self):
        for updates in ({"q": [0., 0., 0., 0.]}, {"q": [math.nan, 0., 0., 0.]},
                        {"type_mask": self.controller_module.M.ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE}):
            c = self.make_controller()
            c.receive(self.attitude_target_packet(**updates), 10.)
            sp = c.px4_attitude_setpoint.snapshot(10.)
            self.assertEqual(sp["status"], "no_yaw_angle")
            self.assertIsNone(sp["yaw_rad"])

    def test_repeated_attitude_timestamp_in_rate_only_mode_expires(self):
        c = self.make_controller()
        c.receive(self.attitude_target_packet(), 10.)
        c.receive(self.attitude_target_packet(), 10.6)
        sp = c.px4_attitude_setpoint.snapshot(10.6)
        self.assertEqual(sp["status"], "stale")
        self.assertIsNone(sp["yaw_rad"])

    def test_first_packet_with_old_px4_sample_time_is_already_stale(self):
        c = self.make_controller()
        c.receive(self.attitude_target_packet(time_boot_ms=100), 10.)
        c.receive(self.setpoint_packet(time_boot_ms=100), 10.)
        c.snapshot(10.)
        data = json.loads(c.states.publish.call_args.args[0].data)
        self.assertEqual(data["px4_attitude_setpoint"]["status"], "stale")
        self.assertIsNone(data["px4_attitude_setpoint"]["yaw_rad"])
        self.assertEqual(data["px4_setpoint"]["status"], "stale")

    def test_attitude_source_age_wrap_jitter_and_wrong_epoch(self):
        age = self.controller_module.source_age_s
        self.assertAlmostEqual(age(0xfffffff0, 16), .032)
        self.assertEqual(age(1020, 1000), 0.)
        self.assertEqual(age(2000, 1000), math.inf)

    def test_timestamp_reset_also_clears_attitude_target(self):
        c = self.make_controller()
        c.receive(self.attitude_target_packet(), 10.)
        c.receive(self.packet("LOCAL_POSITION_NED", time_boot_ms=100, x=2., y=3., z=-4.,
                              vx=0., vy=0., vz=0.), 10.1)
        self.assertIsNone(c.px4_attitude_setpoint.sample)

    def test_position_and_yaw_jump_packets_do_not_clear_stream(self):
        c = self.make_controller()
        target = c.flight.target
        c.receive(self.packet("LOCAL_POSITION_NED", time_boot_ms=1100,
                              x=4., y=3., z=-4.25, vx=0., vy=0., vz=0.), 10.1)
        c.receive(self.packet("ATTITUDE", yaw=1.), 10.1)
        c.flight.tick(10.1)
        self.assertEqual(c.flight.phase, "HOLD")
        self.assertEqual(c.flight.target, target)
        c.flight.send_target.assert_called()
        warnings = c.flight.diagnostics(10.1)["warnings"]
        self.assertTrue(any("0.3 m" in w for w in warnings))
        self.assertTrue(any("30°" in w for w in warnings))

    def test_time_reset_packet_clears_frame_and_continues_zero_stream(self):
        c = self.make_controller()
        c.receive(self.packet("LOCAL_POSITION_NED", time_boot_ms=10,
                              x=0., y=0., z=0., vx=0., vy=0., vz=0.), 10.1)
        c.flight.tick(10.1)
        self.assertEqual(c.flight.phase, "FRAME_RESET")
        self.assertIsNone(c.flight.takeoff_origin)
        c.flight.send_target.assert_called_with(None, None, (0., 0., 0.))
        c.snapshot(10.1)
        snapshot = json.loads(c.states.publish.call_args.args[0].data)
        self.assertFalse(snapshot["frame_valid"])
        self.assertIsNone(snapshot["takeoff_origin_ned"])

    def test_invalid_packet_is_json_safe_and_replays_previous_output(self):
        c = self.make_controller()
        c.flight.yaw_ref = 0.
        c.flight.tick(10.)
        last = c.flight.send_target.call_args
        c.receive(self.packet("LOCAL_POSITION_NED", time_boot_ms=1100,
                              x=math.nan, y=0., z=0., vx=0., vy=0., vz=0.), 10.1)
        c.flight.tick(10.1)
        self.assertEqual(c.flight.send_target.call_args, last)
        c.snapshot(10.1)
        snapshot = json.loads(c.states.publish.call_args.args[0].data)
        self.assertIsNone(snapshot["position_ned"])
        self.assertTrue(snapshot["data_hold"])
        with patch.object(self.controller_module.time, "monotonic", return_value=10.2):
            c.send_target((2., 3., -4.25), 0.)
        self.assertAlmostEqual(c.link.mav.set_position_target_local_ned_send.call_args.args[0], 1200, delta=1)

    def test_zero_recovery_command_ignores_yaw(self):
        c = self.make_controller()
        with patch.object(self.controller_module.time, "monotonic", return_value=10.1):
            c.send_target(None, None, (0., 0., 0.))
        args = c.link.mav.set_position_target_local_ned_send.call_args.args
        self.assertTrue(args[4] & self.controller_module.M.POSITION_TARGET_TYPEMASK_YAW_IGNORE)
        self.assertEqual(args[8:11], (0., 0., 0.))
        self.assertTrue(all(math.isfinite(v) for v in args[5:]))

    def test_timestamp_wrap_is_not_a_reset(self):
        c = self.make_controller()
        c.boot_ms = 0xfffffff0
        c.receive(self.packet("LOCAL_POSITION_NED", time_boot_ms=20,
                              x=2., y=3., z=-4.25, vx=0., vy=0., vz=0.), 10.1)
        self.assertEqual(c.flight.phase, "HOLD")
        self.assertFalse(c.flight.frame_invalid)

    def test_main_cleanup_runs_on_user_interrupt(self):
        module = self.controller_module
        c = Mock()
        with patch.object(module, "Controller", return_value=c), \
             patch.object(module.rclpy, "init", create=True), \
             patch.object(module.rclpy, "spin", side_effect=KeyboardInterrupt(), create=True), \
             patch.object(module.rclpy, "ok", return_value=True, create=True), \
             patch.object(module.rclpy, "shutdown", create=True) as shutdown:
            module.main()
        c.link.close.assert_called_once()
        c.destroy_node.assert_called_once()
        shutdown.assert_called_once()


if __name__ == "__main__":
    unittest.main()
