# Feeder module using AMS AS5048B I2C angle sensor
#
# Copyright (C) 2024  Your Name <your@email.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import math
import logging
from . import bus

# AS5048B I2C default address and register map
AS5048B_I2C_ADDR = 0x40

AS5048B_REGS = {
    'ANGLMSB': 0xFE,
    'ANGLLSB': 0xFF,
    'ZEROMSB': 0x16,
    'ZEROLSB': 0x17,
    'ADDR':    0x15,
    'GAIN':    0xFA,
    'DIAG':    0xFB,
    'MAGNMSB': 0xFC,
    'MAGNLSB': 0xFD,
    'PROG':    0x89,
}
AS5048B_RESOLUTION = 16384.0  # 14 bits

class AS5048B:
    EXP_MOVAVG_N = 10

    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.i2c = bus.MCU_I2C_from_config(
            config, default_addr=AS5048B_I2C_ADDR, default_speed=400000)
        self.address = AS5048B_I2C_ADDR
        self.last_angle_raw = 0
        self.clockwise = config.getboolean("clockwise", False)
        self.logger = logging.getLogger("AS5048B")
        self._angle_offset = 0.0
        self.reset_moving_avg_exp()

    def read_reg16(self, msb_reg):
        result = self.i2c.i2c_read([msb_reg], 2)
        b0, b1 = result['response']
        return ((b0 << 6) | (b1 & 0x3F))

    def get_angle_raw(self):
        angle = self.read_reg16(AS5048B_REGS['ANGLMSB'])
        if self.clockwise:
            angle = int(0b11111111111111 - angle)
        self.last_angle_raw = angle
        return angle

    def get_angle_deg(self):
        return (self.get_angle_raw() / AS5048B_RESOLUTION) * 360.0

    def get_angle(self):
        # Return angle in radians, applying calibration offset
        angle = (self.get_angle_raw() / AS5048B_RESOLUTION) * 2 * math.pi
        return angle + self.get_offset()

    def set_offset(self, offset):
        self._angle_offset = offset

    def get_offset(self):
        return getattr(self, "_angle_offset", 0.0)

    def get_diag(self):
        result = self.i2c.i2c_read([AS5048B_REGS['DIAG']], 1)
        return result['response'][0]

    def get_gain(self):
        result = self.i2c.i2c_read([AS5048B_REGS['GAIN']], 1)
        return result['response'][0]

    def set_zero(self):
        angle = self.get_angle_raw()
        msb = (angle >> 6) & 0xFF
        lsb = angle & 0x3F
        self.i2c.i2c_write([AS5048B_REGS['ZEROMSB'], msb])
        self.i2c.i2c_write([AS5048B_REGS['ZEROLSB'], lsb])

    def get_status(self):
        return {
            "angle_raw": self.last_angle_raw,
            "angle_deg": round(self.get_angle_deg(), 2),
            "diag": self.get_diag(),
            "gain": self.get_gain(),
            "offset": self.get_offset(),
        }

    # ----------------Exponential Moving Average (EMA) for angle------------------
    def reset_moving_avg_exp(self):
        self._moving_avg_exp_angle = 0.0
        self._moving_avg_count = 0
        self._moving_avg_exp_alpha = 2.0 / (self.EXP_MOVAVG_N + 1.0)
        self._moving_avg_exp_sin = 0.0
        self._moving_avg_exp_cos = 0.0

    def update_moving_avg_exp(self):
        angle_rad = (self.get_angle_raw() / AS5048B_RESOLUTION) * 2 * math.pi
        if self._moving_avg_count < self.EXP_MOVAVG_N:
            self._moving_avg_exp_sin += math.sin(angle_rad)
            self._moving_avg_exp_cos += math.cos(angle_rad)
            if self._moving_avg_count == (self.EXP_MOVAVG_N - 1):
                self._moving_avg_exp_sin /= self.EXP_MOVAVG_N
                self._moving_avg_exp_cos /= self.EXP_MOVAVG_N
            self._moving_avg_count += 1
        else:
            alpha = self._moving_avg_exp_alpha
            self._moving_avg_exp_sin += alpha * (math.sin(angle_rad) - self._moving_avg_exp_sin)
            self._moving_avg_exp_cos += alpha * (math.cos(angle_rad) - self._moving_avg_exp_cos)
        self._moving_avg_exp_angle = self._get_exp_avg_raw_angle()

    def _get_exp_avg_raw_angle(self):
        twopi = 2 * math.pi
        if self._moving_avg_exp_sin < 0.0:
            angle = twopi - math.acos(self._moving_avg_exp_cos)
        else:
            angle = math.acos(self._moving_avg_exp_cos)
        return (angle / twopi) * AS5048B_RESOLUTION

    def get_moving_avg_exp(self, unit='deg'):
        raw = self._moving_avg_exp_angle
        if unit == 'deg':
            return (raw / AS5048B_RESOLUTION) * 360.0
        elif unit == 'rad':
            return (raw / AS5048B_RESOLUTION) * 2 * math.pi
        else:
            return raw

class Feeder:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.sensor = AS5048B(config)
        self.gcode = self.printer.lookup_object("gcode")
        self.logger = logging.getLogger("Feeder")
        self.printer.add_object("feeder " + self.name, self)
        self.sensor.reset_moving_avg_exp()

        # Motor driver pins (DRV8876)
        ppins = self.printer.lookup_object("pins")
        self.pwm_pin = ppins.lookup_pin(config.get("pwm_pin", "feeder:GPIO15"))
        self.dir_pin = ppins.lookup_pin(config.get("dir_pin", "feeder:GPIO16"))
        self.fault_pin = ppins.lookup_pin(config.get("fault_pin", "feeder:GPIO22"))

        # Endstop (optical switch)
        self.limit_pin = ppins.lookup_pin(config.get("limit_pin", "feeder:gpio24"))

        # Tape pitch and encoder config
        self.pocket_length = config.getfloat("pocket_length", 4.0)  # mm between pockets
        self.mm_per_rev = config.getfloat("mm_per_rev", 40.0)       # mm per full rotation
        self.ticks_per_rev = config.getint("ticks_per_rev", 16384)  # AS5048B resolution
        self.ticks_per_mm = self.ticks_per_rev / self.mm_per_rev

        # PID state for velocity control
        self.target_velocity = 0.0  # mm/sec
        self._pid = {"Kp": 0.1, "Ki": 0.01, "Kd": 0.001}
        self._pid_state = {"integral": 0.0, "last_error": 0.0}
        self._last_angle = self.sensor.get_angle_raw()
        self._last_time = self.printer.get_reactor().monotonic()
        self._pwm = 0.0
        self._pid_enabled = False
        self._pid_timer = self.printer.get_reactor().register_timer(self._pid_update)

        # Register G-code commands
        self.gcode.register_command("FEEDER_STATUS", self.cmd_FEEDER_STATUS, desc="Report feeder and sensor status")
        self.gcode.register_command("FEEDER_ZERO", self.cmd_FEEDER_ZERO, desc="Set current position as zero for AS5048B")
        self.gcode.register_command("FEEDER_MOTOR", self.cmd_FEEDER_MOTOR, desc="Control feeder motor: FEEDER_MOTOR PWM=<0..1> DIR=<0|1>")
        self.gcode.register_command("FEEDER_LIMIT", self.cmd_FEEDER_LIMIT, desc="Report endstop state")
        self.gcode.register_command("FEEDER_ADVANCE_CUT_TAPE", self.cmd_ADVANCE_CUT_TAPE, desc="Advance tape by LEN=<mm> or POCKETS=<n>")
        self.gcode.register_command("FEEDER_SET_VEL", self.cmd_FEEDER_SET_VEL, desc="Set target velocity (mm/s) and enable PID")
        self.gcode.register_command("FEEDER_STOP", self.cmd_FEEDER_STOP, desc="Stop feeder and disable PID")
        self.gcode.register_command("FEEDER_PID", self.cmd_FEEDER_PID, desc="Tweak PID gains")


    def _pid_update(self, eventtime):
        # Called periodically by reactor
        if not self._pid_enabled:
            self.pwm_pin.set_pwm(0)
            return eventtime + 0.05  # check again in 50ms

        now = self.printer.get_reactor().monotonic()
        dt = now - self._last_time
        if dt < 0.01:
            return now + 0.01

        angle = self.sensor.get_angle_raw()
        # Convert ticks to mm
        delta_ticks = angle - self._last_angle
        # Handle wrap-around
        if delta_ticks > self.ticks_per_rev / 2:
            delta_ticks -= self.ticks_per_rev
        elif delta_ticks < -self.ticks_per_rev / 2:
            delta_ticks += self.ticks_per_rev
        velocity = (delta_ticks / self.ticks_per_mm) / dt  # mm/sec

        error = self.target_velocity - velocity
        self._pid_state["integral"] += error * dt
        derivative = (error - self._pid_state["last_error"]) / dt if dt > 0 else 0.0

        # PID output
        output = (
            self._pid["Kp"] * error +
            self._pid["Ki"] * self._pid_state["integral"] +
            self._pid["Kd"] * derivative
        )
        self._pwm = max(0.0, min(1.0, output))
        self.pwm_pin.setup_pin("pwm")
        self.pwm_pin.set_pwm(self._pwm)

        self._pid_state["last_error"] = error
        self._last_angle = angle
        self._last_time = now
        return now + 0.01  # schedule next update

    def cmd_FEEDER_SET_VEL(self, gcmd):
        vel = gcmd.get_float("VEL", 0.0)
        self.target_velocity = vel
        self._pid_enabled = True
        self._pid_state = {"integral": 0.0, "last_error": 0.0}
        self._last_angle = self.sensor.get_angle_raw()
        self._last_time = self.printer.get_reactor().monotonic()
        gcmd.respond_info(f"Target velocity set to {vel:.2f} mm/s, PID enabled.")

    def cmd_FEEDER_STOP(self, gcmd):
        self._pid_enabled = False
        self.pwm_pin.set_pwm(0)
        gcmd.respond_info("Feeder stopped and PID disabled.")

    def cmd_FEEDER_PID(self, gcmd):
        for param in ["Kp", "Ki", "Kd"]:
            if gcmd.has(param):
                self._pid[param] = gcmd.get_float(param)
        gcmd.respond_info(f"PID gains updated: {self._pid}")

    def cmd_FEEDER_STATUS(self, gcmd):
        status = self.sensor.get_status()
        # Add velocity and PWM to status
        now = self.printer.get_reactor().monotonic()
        dt = now - getattr(self, "_last_status_time", now)
        angle = self.sensor.get_angle_raw()
        last_angle = getattr(self, "_last_status_angle", angle)
        delta_ticks = angle - last_angle
        if delta_ticks > self.ticks_per_rev / 2:
            delta_ticks -= self.ticks_per_rev
        elif delta_ticks < -self.ticks_per_rev / 2:
            delta_ticks += self.ticks_per_rev
        velocity = (delta_ticks / self.ticks_per_mm) / dt if dt > 0 else 0.0
        self._last_status_time = now
        self._last_status_angle = angle
        status.update({
            "velocity": round(velocity, 3),
            "pwm": round(self._pwm, 3),
            "pid_enabled": self._pid_enabled,
        })
        gcmd.respond_info(str(status))

    def cmd_FEEDER_ZERO(self, gcmd):
        self.sensor.set_zero()
        gcmd.respond_info("AS5048B zero position set.")

    def cmd_FEEDER_MOTOR(self, gcmd):
        # Control the motor: PWM=<0..1> DIR=<0|1>
        pwm = gcmd.get_float("PWM", 0.0, minval=0.0, maxval=1.0)
        direction = gcmd.get_int("DIR", 0, minval=0, maxval=1)
        # Set direction pin
        self.dir_pin.setup_pin("digital_out")
        self.dir_pin.set_value(direction)
        # Set PWM pin
        self.pwm_pin.setup_pin("pwm")
        self.pwm_pin.set_pwm(pwm)
        gcmd.respond_info(f"Motor set: PWM={pwm:.2f}, DIR={direction}")

    def cmd_FEEDER_LIMIT(self, gcmd):
        # Report endstop state
        self.limit_pin.setup_pin("digital_in")
        state = self.limit_pin.query_value()
        gcmd.respond_info(f"Endstop (limit switch) state: {'TRIGGERED' if state else 'open'}")

    def cmd_ADVANCE_CUT_TAPE(self, gcmd):
        mm = gcmd.get_float("LEN", None)
        pockets = gcmd.get_int("POCKETS", None)
        if mm is None and pockets is None:
            raise gcmd.error("Specify LEN=<mm> or POCKETS=<n>.")
        if pockets is not None:
            mm = pockets * self.pocket_length
        ticks = mm * self.ticks_per_mm
        self.dir_pin.setup_pin("digital_out")
        self.dir_pin.set_value(1)  # Forward
        self.pwm_pin.setup_pin("pwm")
        self.pwm_pin.set_pwm(0.5)
        start_ticks = self.sensor.get_angle_raw()
        while abs(self.sensor.get_angle_raw() - start_ticks) < ticks:
            self.printer.get_reactor().pause(0.01)
        self.pwm_pin.set_pwm(0)
        gcmd.respond_info(f"Advanced tape by {mm:.2f} mm.")

def load_config(config):
    printer = config.get_printer()
    # Register the AS5048B as an angle sensor if angle.py is loaded
    # angle_module = getattr(printer, "lookup_object", lambda x: None)("angle")
    # if angle_module is not None:
    #     angle_module.add_sensor_factory("AS5048B", AS5048B)
    # Register the feeder object as usual
    return Feeder(config)