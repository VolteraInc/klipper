# Feeder module using AMS AS5048B I2C angle sensor
#
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
            config, default_addr=AS5048B_I2C_ADDR, default_speed=100000)
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
        pwm_pin_name = config.get("pwm_pin", "feeder_pwm")
        self.pwm_pin = self.printer.lookup_object("output_pin " + pwm_pin_name)

        dir_pin_name = config.get("dir_pin", "feeder_dir")
        self.dir_pin = self.printer.lookup_object("output_pin " + dir_pin_name)

        # Digital input pins (limit and fault) as endstop pins using ^ for pullup
        ppins = self.printer.lookup_object("pins")
        limit_pin_desc = config.get("limit_pin", "^pico:gpio24")
        try:
            self.limit_pin = ppins.setup_pin("endstop", limit_pin_desc)
        except Exception as e:
            raise config.error(f"Failed to setup limit_pin '{limit_pin_desc}': {e}")
        fault_pin_desc = config.get("fault_pin", None)
        self.fault_pin = None
        if fault_pin_desc:
            try:
                self.fault_pin = ppins.setup_pin("endstop", fault_pin_desc)
            except Exception as e:
                self.logger.warning(f"Failed to setup fault_pin '{fault_pin_desc}': {e}")

        # Tape pitch and encoder config
        self.pocket_length = config.getfloat("pocket_length", 4.0)  # mm between pockets
        self.mm_per_rev = config.getfloat("mm_per_rev", 40.0)       # mm per full rotation
        self.ticks_per_rev = config.getint("ticks_per_rev", 16384)  # AS5048B resolution
        self.ticks_per_mm = self.ticks_per_rev / self.mm_per_rev

        # PID state for velocity control
        self.target_velocity = 0.0  # mm/sec
        self._pid = {"Kp": 0.1, "Ki": 0.01, "Kd": 0.001}
        self._pid_state = {"integral": 0.0, "last_error": 0.0}
        self._last_angle = None  # Will be set after connect
        self._last_time = None   # Will be set after connect
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
        self.gcode.register_command("FEEDER_FAULT", self.cmd_FEEDER_FAULT, desc="Report DRV8876 fault pin state")

        # Register event handler for klippy:connect
        self.printer.register_event_handler("klippy:connect", self.handle_connect)

    def handle_connect(self):
        # Set initial state for PWM and DIR pins using the public PrinterOutputPin API
        reactor = self.printer.get_reactor()
        print_time = reactor.monotonic()
        # Set PWM to 0 (off)
        if hasattr(self.pwm_pin, 'set_pwm'):
            self.pwm_pin.set_pwm(print_time, 0.0)
        elif hasattr(self.pwm_pin, 'set_digital'):
            self.pwm_pin.set_digital(print_time, 0)
        # Set DIR to 0 (default direction)
        if hasattr(self.dir_pin, 'set_digital'):
            self.dir_pin.set_digital(print_time, 0)

    def _pid_update(self, eventtime):
        # Called periodically by reactor
        if not self._pid_enabled or self._last_angle is None or self._last_time is None:
            return eventtime + 0.5
        if self.fault_pin is not None and self.is_fault_triggered():
            self._pid_enabled = False
            self.pwm_pin.set_pwm(0.0)
            self.logger.warning("DRV8876 fault detected! Motor stopped for safety.")
            return eventtime + 1.0
        if self.is_limit_triggered():
            self._pid_enabled = False
            self.pwm_pin.set_pwm(0.0)
            self.logger.info("Feeder endstop triggered. Motor stopped.")
            return eventtime + 1.0

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
        # Only access sensor after klippy:connect
        self._last_angle = self.sensor.get_angle_raw()
        self._last_time = self.printer.get_reactor().monotonic()
        gcmd.respond_info(f"Target velocity set to {vel:.2f} mm/s, PID enabled.")

    def cmd_FEEDER_STOP(self, gcmd):
        # Stop the feeder motor by setting PWM to 0
        self.pwm_pin.gcrq.queue_gcode_request(0.0)
        self._pwm = 0.0
        self._pid_enabled = False
        gcmd.respond_info("Feeder stopped (PWM=0.0, PID disabled)")

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
            "limit": self.is_limit_triggered(),
            "fault": self.is_fault_triggered() if self.fault_pin else None,
        })
        gcmd.respond_info(str(status))

    def cmd_FEEDER_ZERO(self, gcmd):
        self.sensor.set_zero()
        gcmd.respond_info("AS5048B zero position set.")

    def cmd_FEEDER_MOTOR(self, gcmd):
        pwm = gcmd.get_float("PWM", None)
        dir_val = gcmd.get_int("DIR", None)
        if pwm is not None:
            # Clamp PWM to [0, 1]
            pwm = max(0.0, min(1.0, pwm))
            self.pwm_pin.gcrq.queue_gcode_request(pwm)
            self._pwm = pwm
        if dir_val is not None:
            self.dir_pin.gcrq.queue_gcode_request(1 if dir_val else 0)
        gcmd.respond_info(f"Set PWM={self._pwm:.2f}, DIR={dir_val}")

    def cmd_FEEDER_LIMIT(self, gcmd):
        """Report the state of the limit (endstop) pin."""
        state = self.is_limit_triggered()
        gcmd.respond_info(f"Feeder limit (endstop) triggered: {state}")

    def is_limit_triggered(self):
        # Use .query_endstop() for digital input pin state
        try:
            return bool(self.limit_pin.query_endstop(self.printer.get_reactor().monotonic()))
        except Exception as e:
            self.logger.warning(f"Error reading limit pin: {e}")
            return False

    def is_fault_triggered(self):
        if not self.fault_pin:
            return False
        try:
            return bool(self.fault_pin.query_endstop(self.printer.get_reactor().monotonic()))
        except Exception as e:
            self.logger.warning(f"Error reading fault pin: {e}")
            return False

    def cmd_FEEDER_FAULT(self, gcmd):
        if self.fault_pin is None:
            gcmd.respond_info("Feeder fault pin not configured.")
            return
        state = self.is_fault_triggered()
        gcmd.respond_info(f"Feeder fault pin triggered: {state}")

    def cmd_ADVANCE_CUT_TAPE(self, gcmd):
        # Advance tape by LEN=<mm> or POCKETS=<n>
        pockets = gcmd.get_int("POCKETS", None)
        length = gcmd.get_float("LEN", None)
        if pockets is not None:
            length = pockets * self.pocket_length
        elif length is None:
            length = self.pocket_length
        # Move the tape by 'length' mm, or until endstop/fault triggered
        start_angle = self.sensor.get_angle_raw()
        target_ticks = start_angle + int(length * self.ticks_per_mm)
        # Handle wrap-around
        target_ticks = target_ticks % self.ticks_per_rev
        # Set direction
        self.dir_pin.gcrq.queue_gcode_request(1)  # Assume 1 is forward
        # Start motor
        self.pwm_pin.gcrq.queue_gcode_request(0.7)  # Use a fixed PWM for advance
        reactor = self.printer.get_reactor()
        timeout = reactor.monotonic() + 5.0  # 5s timeout for safety
        while True:
            now = reactor.monotonic()
            angle = self.sensor.get_angle_raw()
            # Check for wrap-around
            delta = (angle - start_angle) % self.ticks_per_rev
            if delta < 0:
                delta += self.ticks_per_rev
            self.logger.info(f"ADVANCE_LOOP: angle={angle}, start_angle={start_angle}, delta={delta}, target={(length * self.ticks_per_mm)}")
            if delta >= (length * self.ticks_per_mm):
                break
            if self.is_limit_triggered() or (self.fault_pin and self.is_fault_triggered()):
                self.logger.info("ADVANCE_LOOP: Endstop or fault triggered, breaking.")
                break
            if now > timeout:
                self.logger.warning("Advance cut tape timed out!")
                break
            reactor.pause(now + 0.01)
        # Stop motor
        self.pwm_pin.gcrq.queue_gcode_request(0.0)
        gcmd.respond_info(f"Advanced tape by {length:.2f} mm or until endstop/fault triggered.")

def load_config(config):
    printer = config.get_printer()
    # Register the AS5048B as an angle sensor if angle.py is loaded
    # angle_module = getattr(printer, "lookup_object", lambda x: None)("angle")
    # if angle_module is not None:
    #     angle_module.add_sensor_factory("AS5048B", AS5048B)
    # Register the feeder object as usual
    return Feeder(config)