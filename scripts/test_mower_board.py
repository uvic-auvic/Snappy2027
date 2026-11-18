#!/usr/bin/env python3
"""Bench test for the mower board firmware over micro-ROS.

Run on the Jetson with the micro-ROS agent up and the controller stopped:

    python3 scripts/test_mower_board.py              # safe checks only
    python3 scripts/test_mower_board.py --thrusters  # also spin each thruster
    python3 scripts/test_mower_board.py --rails      # also switch 5V/12V off and on
    python3 scripts/test_mower_board.py --kill       # also walk through both kill switches
    python3 scripts/test_mower_board.py --gf-zero    # also re-zero the ground fault baseline

The board needs about 25 s after power-up before it answers the agent.
Exit code is 1 if any check failed.
"""

import argparse
import sys
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

from snappy_interfaces.msg import (
    AmbientStatus,
    DiagStatus,
    GroundFaultStatus,
    MotorStatus,
    PowerStatus,
    RailStatus,
    TempStatus,
    ThrusterCommand,
)
from snappy_interfaces.srv import MowerConfig, SetPowerState, StatusRequest

# Order matches the sector bits: bit i belongs to SECTORS[i]
SECTORS = [
    ("motors", MotorStatus),
    ("temps", TempStatus),
    ("ambient", AmbientStatus),
    ("power", PowerStatus),
    ("ground_fault", GroundFaultStatus),
    ("rails", RailStatus),
    ("diag", DiagStatus),
]
ALL = StatusRequest.Request.SECTOR_ALL
DIAG_BIT = StatusRequest.Request.SECTOR_DIAG
TEMPS_BIT = StatusRequest.Request.SECTOR_TEMPS

RESET_CAUSES = {
    DiagStatus.RESET_LOW_POWER: "low power",
    DiagStatus.RESET_WWDG: "window watchdog",
    DiagStatus.RESET_IWDG: "watchdog (firmware hung)",
    DiagStatus.RESET_SOFTWARE: "software/fault",
    DiagStatus.RESET_POWER_ON: "power on",
    DiagStatus.RESET_PIN: "reset pin",
    DiagStatus.RESET_BROWNOUT: "brownout",
}


class Abort(Exception):
    pass


def close(a, b, tol=1e-3):
    return abs(a - b) <= tol


class MowerTest:
    def __init__(self, args):
        self.args = args
        self.node = rclpy.create_node("mower_board_test")
        self.counts = {"PASS": 0, "WARN": 0, "FAIL": 0}
        self.rx = {name: [] for name, _ in SECTORS}
        self.motor_pub = None
        self.saved_rates = None

        for name, msg_type in SECTORS:
            self.node.create_subscription(
                msg_type,
                f"/mower/{name}",
                lambda msg, n=name: self.rx[n].append((time.monotonic(), msg)),
                qos_profile_sensor_data,  # board publishes best effort
            )
        self.status_cli = self.node.create_client(StatusRequest, "/mower/status_request")
        self.power_cli = self.node.create_client(SetPowerState, "/mower/set_power_state")
        self.config_cli = self.node.create_client(MowerConfig, "/mower/config")

    # ---- reporting --------------------------------------------------------
    def report(self, level, name, detail=""):
        self.counts[level] += 1
        print(f"  [{level}] {name}" + (f"  ({detail})" if detail else ""))

    def check(self, name, ok, detail=""):
        self.report("PASS" if ok else "FAIL", name, detail)
        return ok

    def warn_unless(self, name, ok, detail=""):
        self.report("PASS" if ok else "WARN", name, detail)
        return ok

    @staticmethod
    def section(title):
        print(f"\n== {title}")

    # ---- ROS helpers ------------------------------------------------------
    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def call(self, client, request, timeout=3.0):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        if not future.done():
            future.cancel()
            raise Abort(f"{client.srv_name} did not answer within {timeout:.0f} s")
        return future.result()

    def status(self, mask, repeat=False, rate=0.0):
        req = StatusRequest.Request()
        req.sector_mask = mask
        req.repeat = repeat
        req.rate_hz = float(rate)
        return self.call(self.status_cli, req)

    def snap(self, mask=ALL):
        return self.status(mask).status

    def config(self, vbat=0.0, trip=0.0, zero=False, temp_limit=0.0):
        req = MowerConfig.Request()
        req.gf_vbat_v = float(vbat)
        req.gf_trip_kohm = float(trip)
        req.gf_zero_now = zero
        req.temp_limit_c = float(temp_limit)
        return self.call(self.config_cli, req)

    def set_rail(self, rail, enable):
        req = SetPowerState.Request()
        req.target_rail = rail
        req.enable = enable
        return self.call(self.power_cli, req)

    def send_thrust(self, mask, values, seconds):
        """Publish one command at 10 Hz for `seconds`, like the controller does."""
        msg = ThrusterCommand()
        msg.thruster_mask = mask
        msg.thrust_pct = values
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.motor_pub.publish(msg)
            self.spin(0.1)

    def stop_thrusters(self):
        if self.motor_pub is not None:
            self.send_thrust(255, [0] * 8, 0.3)

    # ---- tests ------------------------------------------------------------
    def test_connection(self):
        self.section("Connection")
        for cli in (self.status_cli, self.power_cli, self.config_cli):
            if not cli.wait_for_service(timeout_sec=10.0):
                self.report("FAIL", f"{cli.srv_name} available")
                raise Abort(
                    "board not reachable: is the micro-ROS agent running on the right "
                    "serial port, and has the board had ~25 s since power-up?"
                )
            self.report("PASS", f"{cli.srv_name} available")

    def test_snapshot(self):
        self.section("On-demand snapshot (/mower/status_request, repeat=false)")
        res = self.status(ALL)
        self.saved_rates = list(res.rates_hz)
        st = res.status
        self.check("request accepted", res.accepted)
        self.check("all sectors filled", st.sector_mask == ALL, f"mask={st.sector_mask}")

        ids = list(st.temps.sensor_id)
        self.check("temp sensor ids are 11,12,21,22", ids == [11, 12, 21, 22], str(ids))
        for i, sensor in enumerate(ids):
            self.check(
                f"temp sensor {sensor} valid",
                bool(st.temps.valid[i]),
                f"{st.temps.temp_c[i]:.1f} C, over_limit={bool(st.temps.over_limit[i])}",
            )
        self.check(
            "ambient (HDC2080) valid",
            st.ambient.valid and 0.0 <= st.ambient.humidity_pct <= 100.0,
            f"{st.ambient.temp_c:.1f} C, {st.ambient.humidity_pct:.0f} %RH",
        )
        self.check(
            "5V monitor (INA260) valid",
            st.power.v5_valid,
            f"{st.power.v5_bus_v:.2f} V, {st.power.v5_current_a:.2f} A",
        )
        self.check(
            "12V monitor (INA260) valid",
            st.power.v12_valid,
            f"{st.power.v12_bus_v:.2f} V, {st.power.v12_current_a:.2f} A",
        )

        gf = st.ground_fault
        self.check(
            "ground fault reading valid",
            gf.valid,
            f"leak={gf.leak_kohm:.0f} kOhm, side={gf.side}, GFout={gf.gfout_v:.3f} V",
        )
        self.warn_unless("no ground fault flagged", not gf.fault, f"trip={gf.trip_kohm:.0f} kOhm")

        r = st.rails
        self.check(
            "5V rail state consistent",
            r.v5_on == (r.v5_requested and not r.v5_killed),
            f"on={r.v5_on} requested={r.v5_requested} killed={r.v5_killed} good={r.v5_power_good}",
        )
        self.check(
            "12V rail state consistent",
            r.v12_on == (r.v12_requested and not r.v12_killed),
            f"on={r.v12_on} requested={r.v12_requested} killed={r.v12_killed} good={r.v12_power_good}",
        )

        m = st.motors
        self.warn_unless(
            "kill switches armed",
            not m.lcd_kill_active and not m.enclosure_kill_active,
            f"lcd_kill={m.lcd_kill_active} enclosure_kill={m.enclosure_kill_active}",
        )

        d = st.diag
        causes = [text for bit, text in RESET_CAUSES.items() if d.reset_cause & bit]
        print(f"  uptime {d.uptime_s} s, last reset: {', '.join(causes) or 'unknown'}")
        print(
            f"  heap free {d.heap_free} (min {d.heap_min}), micro-ROS heap free "
            f"{d.uros_heap_free} (min {d.uros_heap_min})"
        )
        print(
            f"  stack never used: micro-ROS {d.uros_stack_min} B, sensors {d.sensor_stack_min} B; "
            f"MCU {d.mcu_temp_c:.1f} C, VDDA {d.vdda_v:.2f} V"
        )
        self.check("independent watchdog running", d.watchdog_active)
        self.warn_unless(
            "last reset was not a watchdog reset", not d.reset_cause & DiagStatus.RESET_IWDG
        )
        self.check("no micro-ROS allocation failures", d.uros_alloc_failures == 0,
                   str(d.uros_alloc_failures))
        self.check("agent connect counted", d.agent_connects >= 1, str(d.agent_connects))
        self.warn_unless(
            "no I2C errors",
            d.i2c1_errors == 0 and d.i2c2_errors == 0,
            f"i2c1={d.i2c1_errors} i2c2={d.i2c2_errors} recoveries={d.i2c_recoveries}",
        )
        self.warn_unless(
            "serial link clean",
            d.uart_rx_restarts == 0 and d.ping_failures == 0,
            f"rx_restarts={d.uart_rx_restarts} ping_failures={d.ping_failures}",
        )
        if self.check("stamps synced to ROS time", d.time_synced):
            now = self.node.get_clock().now().nanoseconds * 1e-9
            stamp = d.stamp.sec + d.stamp.nanosec * 1e-9
            self.check("stamp within 2 s of Jetson clock", abs(now - stamp) < 2.0,
                       f"offset {now - stamp:+.2f} s")

        part = self.status(TEMPS_BIT)
        self.check(
            "partial snapshot fills only the masked sector",
            part.status.sector_mask == TEMPS_BIT
            and part.status.temps.stamp.sec != 0
            and part.status.diag.stamp.sec == 0
            and part.status.diag.uptime_s == 0,
        )
        self.check("snapshot leaves stream rates alone", list(part.rates_hz) == self.saved_rates)
        self.check("empty sector mask is refused", not self.status(0).accepted)

    def measure(self, seconds):
        """Message count per sector over a window."""
        for samples in self.rx.values():
            samples.clear()
        self.spin(seconds)
        return {name: len(samples) for name, samples in self.rx.items()}

    def test_streams(self):
        self.section("Telemetry streams (/mower/*, repeat=true)")
        rate = self.args.rate
        window = 4.0

        res = self.status(ALL, repeat=True, rate=0.0)
        self.check("rate 0 reports every sector off", all(r == 0.0 for r in res.rates_hz))
        self.spin(0.5)
        counts = self.measure(1.5)
        self.check("no messages while off", sum(counts.values()) == 0, str(counts))

        res = self.status(ALL, repeat=True, rate=rate)
        self.check(f"all sectors set to {rate:g} Hz", all(close(r, rate) for r in res.rates_hz),
                   str([round(r, 2) for r in res.rates_hz]))
        counts = self.measure(window)
        expected = rate * window
        for name, _ in SECTORS:
            got = counts[name]
            self.check(
                f"/mower/{name} streams at ~{rate:g} Hz",
                0.6 * expected <= got <= 1.3 * expected + 2,
                f"{got / window:.1f} Hz measured",
            )
            stamps = [m.stamp.sec + m.stamp.nanosec * 1e-9 for _, m in self.rx[name]]
            if len(stamps) > 1:
                self.check(f"/mower/{name} stamps increase",
                           all(b > a for a, b in zip(stamps, stamps[1:])))

        # Per-sector rates: motors faster, everything else off except diag
        self.status(ALL, repeat=True, rate=0.0)
        self.status(1, repeat=True, rate=10.0)
        res = self.status(DIAG_BIT, repeat=True, rate=1.0)
        want = [10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        self.check("sectors keep independent rates",
                   all(close(a, b) for a, b in zip(res.rates_hz, want)),
                   str([round(r, 2) for r in res.rates_hz]))
        counts = self.measure(window)
        self.check("motors at ~10 Hz while diag at ~1 Hz",
                   24 <= counts["motors"] <= 54 and 2 <= counts["diag"] <= 7,
                   f"motors {counts['motors'] / window:.1f} Hz, diag {counts['diag'] / window:.1f} Hz")
        self.check("other sectors stay silent",
                   all(counts[n] == 0 for n in ("temps", "ambient", "power", "ground_fault", "rails")))

        self.status(ALL, repeat=True, rate=0.0)
        res = self.status(DIAG_BIT, repeat=True, rate=50.0)
        self.check("rate above 20 Hz clamps to 20", close(res.rates_hz[6], 20.0), f"{res.rates_hz[6]:g}")
        counts = self.measure(2.0)
        self.warn_unless("board keeps up with 20 Hz", counts["diag"] >= 30,
                         f"{counts['diag'] / 2.0:.1f} Hz measured")
        res = self.status(DIAG_BIT, repeat=True, rate=0.01)
        self.check("rate below 0.1 Hz clamps to 0.1", close(res.rates_hz[6], 0.1), f"{res.rates_hz[6]:g}")
        res = self.status(DIAG_BIT, repeat=True, rate=-5.0)
        self.check("negative rate turns the sector off", res.rates_hz[6] == 0.0)

    def test_config(self):
        self.section("Settings (/mower/config)")
        cur = self.config()
        self.check("empty request changes nothing and succeeds", cur.success)
        print(
            f"  current: vbat={cur.gf_vbat_v:.2f} V, trip={cur.gf_trip_kohm:.0f} kOhm, "
            f"baseline={cur.gf_baseline_v:.3f} V, temp limit={cur.temp_limit_c:.1f} C"
        )

        try:
            new_limit = cur.temp_limit_c - 5.0
            res = self.config(temp_limit=new_limit)
            self.check("temp limit set", res.success and close(res.temp_limit_c, new_limit))
            self.spin(0.5)
            self.check("TempStatus.limit_c follows", close(self.snap().temps.limit_c, new_limit))
            self.check(
                "other settings untouched",
                close(res.gf_vbat_v, cur.gf_vbat_v) and close(res.gf_trip_kohm, cur.gf_trip_kohm),
            )

            new_vbat = 16.8 if not close(cur.gf_vbat_v, 16.8) else 15.6
            res = self.config(vbat=new_vbat)
            self.check("ground fault battery voltage set", res.success and close(res.gf_vbat_v, new_vbat))
            self.spin(0.5)
            self.check("GroundFaultStatus.vbat_v follows", close(self.snap().ground_fault.vbat_v, new_vbat))

            new_trip = cur.gf_trip_kohm + 50.0
            res = self.config(trip=new_trip)
            self.check("ground fault trip level set", res.success and close(res.gf_trip_kohm, new_trip))
            self.spin(0.5)
            self.check("GroundFaultStatus.trip_kohm follows",
                       close(self.snap().ground_fault.trip_kohm, new_trip))

            res = self.config(vbat=100.0)
            self.check("battery voltage outside 5-60 V rejected",
                       not res.success and close(res.gf_vbat_v, new_vbat))
            res = self.config(temp_limit=200.0)
            self.check("temp limit above 125 C rejected",
                       not res.success and close(res.temp_limit_c, new_limit))
        finally:
            res = self.config(vbat=cur.gf_vbat_v, trip=cur.gf_trip_kohm, temp_limit=cur.temp_limit_c)
            self.check(
                "original settings restored",
                res.success
                and close(res.gf_vbat_v, cur.gf_vbat_v)
                and close(res.gf_trip_kohm, cur.gf_trip_kohm)
                and close(res.temp_limit_c, cur.temp_limit_c),
            )

        if self.args.gf_zero:
            gfout = self.snap().ground_fault.gfout_v
            res = self.config(zero=True)
            self.check("ground fault baseline zeroed to current GFout",
                       res.success and abs(res.gf_baseline_v - gfout) < 0.05,
                       f"baseline {res.gf_baseline_v:.3f} V, GFout {gfout:.3f} V")
        else:
            print("  [SKIP] ground fault zeroing (pass --gf-zero with the vehicle dry)")

    def test_rails(self):
        self.section("Power rails (/mower/set_power_state)")
        R = SetPowerState.Request
        res = self.set_rail(R.RAIL_JETSON, True)
        self.check("Jetson rail refused", not res.success, res.message)
        res = self.set_rail(9, True)
        self.check("unknown rail refused", not res.success, res.message)

        rails = self.snap().rails
        if rails.v5_killed or rails.v12_killed:
            print("  [SKIP] rail switching: a kill switch is pulled")
            return

        # Asking for a rail that is already on must succeed and change nothing
        if rails.v5_on and rails.v12_on:
            ok5 = self.set_rail(R.RAIL_5V, True)
            ok12 = self.set_rail(R.RAIL_12V, True)
            self.check("enable on an already-on rail succeeds", ok5.success and ok12.success,
                       f"{ok5.message} / {ok12.message}")

        if not self.args.rails:
            print("  [SKIP] switching rails off and on (pass --rails)")
            return

        for label, rail in (("12V", R.RAIL_12V), ("5V", R.RAIL_5V)):
            def state():
                r = self.snap().rails
                if rail == R.RAIL_12V:
                    return r.v12_on, r.v12_requested, r.v12_power_good
                return r.v5_on, r.v5_requested, r.v5_power_good

            try:
                res = self.set_rail(rail, False)
                self.check(f"{label} off accepted", res.success, res.message)
                self.spin(2.0)
                on, requested, good = state()
                self.check(f"{label} reports off", not on and not requested)
                self.warn_unless(f"{label} bus voltage dropped", not good)
            finally:
                res = self.set_rail(rail, True)
            self.check(f"{label} on accepted", res.success, res.message)
            self.spin(1.5)
            on, requested, good = state()
            self.check(f"{label} reports on with power good", on and requested and good)

    def test_motors(self):
        self.section("Thruster commands (/motor_cmd)")
        self.spin(1.0)  # let discovery settle before counting publishers
        others = self.node.count_publishers("/motor_cmd")
        if others > 0:
            print(f"  [SKIP] {others} other publisher(s) on /motor_cmd: stop the controller first")
            return
        self.motor_pub = self.node.create_publisher(
            ThrusterCommand, "/motor_cmd",
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.spin(1.5)
        self.check("board subscribed to /motor_cmd", self.node.count_subscribers("/motor_cmd") >= 1)

        self.send_thrust(255, [0] * 8, 0.6)
        m = self.snap().motors
        armed = not m.lcd_kill_active and not m.enclosure_kill_active
        self.check("zero command received", m.last_cmd_age_ms < 500, f"age {m.last_cmd_age_ms} ms")
        self.check("all thrusters report 0", all(v == 0 for v in m.thrust_pct))
        if armed:
            self.check("kill_since_last_cmd cleared by a command", not m.kill_since_last_cmd)
        self.spin(1.5)
        age = self.snap().motors.last_cmd_age_ms
        self.check("last_cmd_age_ms counts up without commands", 1200 <= age <= 4000, f"{age} ms")

        if not self.args.thrusters:
            print("  [SKIP] spinning thrusters (pass --thrusters)")
            return
        if not armed:
            print("  [SKIP] spinning thrusters: a kill switch is pulled")
            return
        pct = self.args.thrust_pct
        if not self.args.yes:
            answer = input(f"  Thrusters will spin at {pct}% one at a time. Type 'yes' when clear: ")
            if answer.strip().lower() != "yes":
                print("  [SKIP] spinning thrusters: not confirmed")
                return

        try:
            for i in range(8):
                for value in (pct, -pct):
                    cmd = [0] * 8
                    cmd[i] = value
                    self.send_thrust(1 << i, cmd, 1.0)
                    applied = list(self.snap().motors.thrust_pct)
                    self.check(f"thruster {i + 1} at {value:+d}%", applied == cmd, str(applied))
                    self.send_thrust(255, [0] * 8, 0.3)

            # Mask: only thruster 2's bit is set, so thruster 1 must keep running
            self.send_thrust(1, [pct, 0, 0, 0, 0, 0, 0, 0], 0.5)
            self.send_thrust(2, [0, pct, 0, 0, 0, 0, 0, 0], 0.5)
            applied = list(self.snap().motors.thrust_pct)
            self.check("unmasked thrusters keep their speed", applied[:2] == [pct, pct], str(applied))

            # One value past the limit must drop the whole message
            self.send_thrust(255, [101, 0, 0, 0, 0, 0, 0, 0], 0.2)
            applied = list(self.snap().motors.thrust_pct)
            self.check("out-of-range command rejected", applied[:2] == [pct, pct], str(applied))
            self.send_thrust(255, [0] * 8, 0.3)

            self.send_thrust(1, [pct, 0, 0, 0, 0, 0, 0, 0], 0.5)
            self.spin(1.5)
            held = self.snap().motors.thrust_pct[0]
            self.warn_unless(
                "thrusters stop when commands stop",
                held == 0,
                "no command timeout: thruster held its last speed for 1.5 s" if held else "",
            )
        finally:
            self.stop_thrusters()
        self.check("all thrusters back to 0", all(v == 0 for v in self.snap().motors.thrust_pct))

    def test_kill(self):
        self.section("Kill switches")
        if not self.args.kill:
            print("  [SKIP] kill switch walk-through (pass --kill)")
            return
        if not sys.stdin.isatty():
            print("  [SKIP] kill switch walk-through needs an interactive terminal")
            return
        R = SetPowerState.Request

        for label, is_lcd in (("LCD", True), ("enclosure", False)):
            input(f"  Pull the {label} kill switch, then press Enter... ")
            st = self.snap()
            m, r = st.motors, st.rails
            self.check(f"{label} kill reported",
                       m.lcd_kill_active if is_lcd else m.enclosure_kill_active)
            self.check("thrusters at 0", all(v == 0 for v in m.thrust_pct))
            self.check("kill_since_last_cmd set", m.kill_since_last_cmd)
            self.check("5V killed and off", r.v5_killed and not r.v5_on)
            if is_lcd:
                self.check("12V left on by the LCD switch", not r.v12_killed and r.v12_on)
            else:
                self.check("12V killed and off", r.v12_killed and not r.v12_on)
            res = self.set_rail(R.RAIL_5V, True)
            self.check("5V enable refused while killed", not res.success, res.message)
            if self.motor_pub is not None and self.args.thrusters:
                self.send_thrust(1, [self.args.thrust_pct, 0, 0, 0, 0, 0, 0, 0], 0.5)
                self.check("thruster command ignored while killed",
                           all(v == 0 for v in self.snap().motors.thrust_pct))

            input(f"  Re-arm the {label} kill switch, then press Enter... ")
            self.spin(0.5)
            st = self.snap()
            m, r = st.motors, st.rails
            self.check(f"{label} kill cleared",
                       not (m.lcd_kill_active if is_lcd else m.enclosure_kill_active))
            self.check("rails back on after re-arm",
                       r.v5_on and r.v5_requested and r.v12_on and r.v12_requested)
            self.check("kill_since_last_cmd stays set until a command", m.kill_since_last_cmd)
            if self.motor_pub is not None:
                self.send_thrust(255, [0] * 8, 0.5)
                self.check("kill_since_last_cmd cleared by the next command",
                           not self.snap().motors.kill_since_last_cmd)

    # ---- runner -----------------------------------------------------------
    def restore_rates(self):
        if self.saved_rates is None:
            return
        self.status(ALL, repeat=True, rate=0.0)
        for i, rate in enumerate(self.saved_rates):
            if rate > 0.0:
                self.status(1 << i, repeat=True, rate=rate)

    def run(self):
        try:
            self.test_connection()
            self.test_snapshot()
            self.test_streams()
            self.test_config()
            self.test_rails()
            self.test_motors()
            self.test_kill()
        except Abort as err:
            self.report("FAIL", "aborted", str(err))
        except KeyboardInterrupt:
            self.report("FAIL", "interrupted")
        finally:
            try:
                self.stop_thrusters()
                self.restore_rates()
            except Abort:
                print("  could not restore the board state: link is down")

        c = self.counts
        print(f"\n{c['PASS']} passed, {c['WARN']} warnings, {c['FAIL']} failed")
        return 1 if c["FAIL"] else 0


def main():
    parser = argparse.ArgumentParser(description="Bench test for the mower board firmware")
    parser.add_argument("--thrusters", action="store_true", help="spin each thruster briefly")
    parser.add_argument("--thrust-pct", type=int, default=10, choices=range(1, 31), metavar="1-30",
                        help="thrust used with --thrusters (default 10)")
    parser.add_argument("--yes", action="store_true", help="skip the thruster confirmation prompt")
    parser.add_argument("--rails", action="store_true",
                        help="switch 12V then 5V off and on (powers down whatever they feed)")
    parser.add_argument("--kill", action="store_true", help="interactive kill switch walk-through")
    parser.add_argument("--gf-zero", action="store_true",
                        help="re-zero the ground fault baseline (vehicle must be dry)")
    parser.add_argument("--rate", type=float, default=5.0, help="stream test rate in Hz (default 5)")
    args = parser.parse_args()
    if not 1.0 <= args.rate <= 10.0:
        parser.error("--rate must be between 1 and 10 Hz")

    rclpy.init()
    test = MowerTest(args)
    try:
        code = test.run()
    finally:
        test.node.destroy_node()
        rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
