# feeder.py — host helper for the RP2040 tape-feeder, using bus.py abstractions
import json, time, logging
from . import bus, angle

class TapeFeeder:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object("gcode")
        self.log = logging.getLogger(config.get_name())

        # ───── SPI setup for encoder ─────
        self.encoder = self.printer.lookup_object(config.get("encoder", "feeder_rotary"))
        self.spi = self.encoder.spi
        self.spi_oid = self.spi.get_oid()
        self.mcu = self.spi.get_mcu()
        self.oid = self.mcu.create_oid()

        # ───── GPIO pins setup ─────
        ppins = self.printer.lookup_object("pins")
        self.pwm_pin = ppins.lookup_pin(config.get("pwm_pin", "feeder:gpio15"))
        self.dir_pin = ppins.lookup_pin(config.get("dir_pin", "feeder:gpio16"))
        self.fault_pin = ppins.lookup_pin(config.get("fault_pin", "feeder:gpio22"))
        self.limit_pin = ppins.lookup_pin(config.get("limit_pin", "feeder:gpio24"))

        # ───── Mechanics ─────
        self.mm_per_rev = config.getfloat("mm_per_rev", 40.0)
        self.ticks_per_rev = config.getint("ticks_per_rev", 16384)
        self.ticks_per_mm = self.ticks_per_rev / self.mm_per_rev
        self.pocket_length = config.getfloat("pocket_length", 4.0)

        self.load_pwm = config.getfloat("load_pwm", 0.40)
        self.unload_pwm = config.getfloat("unload_pwm", 0.40)
        self.advance_pwm = config.getfloat("advance_pwm", 0.45)

        # ───── PID Parameters ─────
        self.pid_params = {
            "vel": {"Kp": 0.1, "Ki": 0.01, "Kd": 0.001},
            "pos": {"Kp": 0.2, "Ki": 0.02, "Kd": 0.002},
        }

        # ───── MCU Commands ─────
        self.cfg_hw_cmd = self.mcu.lookup_command(
            "config_feeder oid=%c spi_oid=%c pwm_pin=%u dir_pin=%u fault_pin=%u limit_pin=%u"
        )
        self.start_feeder_cmd = self.mcu.lookup_command(
            "start_feeder oid=%c pwm_duty=%u direction=%u"
        )
        self.stop_feeder_cmd = self.mcu.lookup_command(
            "stop_feeder oid=%c"
        )

        # ───── G-code Commands ─────
        self.gcode.register_command("FEEDER_INIT", self.cmd_INIT_FEEDER, desc="Initialize the feeder")
        self.gcode.register_command("FEEDER_START", self.cmd_START_FEEDER, desc="Start the feeder")
        self.gcode.register_command("FEEDER_STOP", self.cmd_STOP_FEEDER, desc="Stop the feeder")
        self.gcode.register_command("FEEDER_STATUS", self.cmd_FEEDER_STATUS, desc="Get feeder status")
        self.gcode.register_command("FEEDER_LOAD_CUT_TAPE", self.cmd_LOAD_CUT_TAPE, desc="Home to optical switch, park at first pocket")
        self.gcode.register_command("FEEDER_UNLOAD_CUT_TAPE", self.cmd_UNLOAD_CUT_TAPE, desc="Reverse and eject tape from cutter")
        self.gcode.register_command("FEEDER_ADVANCE_CUT_TAPE", self.cmd_ADVANCE_CUT_TAPE, desc="Step tape forward (LEN=<mm>|POCKETS=<n>)")
        self.gcode.register_command("FEEDER_PID", self.cmd_PID, desc="Runtime tweak of PID gains")
        self.gcode.register_command("FEEDER_PID_SAVE", self.cmd_PID_SAVE, desc="Persist current PID gains to printer.cfg")

        self.configured = False
        self.printer.register_event_handler("klippy:ready", self._handle_on_ready)

    def _do_initialization_commands(self):
        # Send SPI initialization commands
        self.encoder.setup()  # Perform encoder-specific setup
        self.cfg_hw_cmd.send([
            self.oid, self.spi_oid, self.pwm_pin['pin'], self.dir_pin['pin'],
            self.fault_pin['pin'], self.limit_pin['pin']
        ])
        return True

    def _handle_on_ready(self):
        # Perform initialization when Klipper is ready
        self.configured = self._do_initialization_commands()

    def cmd_LOAD_CUT_TAPE(self, gcmd):
        """Home to optical switch, park at first pocket."""
        self._set_dir(False)
        self._set_pwm(self.load_pwm)
        start = time.monotonic()
        while not self._limit() and time.monotonic() - start < 6:
            self.printer.get_reactor().pause(0.05)
        self._set_pwm(0)
        if not self._limit():
            raise gcmd.error("Limit switch not triggered!")
        gcmd.respond_info("Feeder homed and parked at first pocket.")

    def cmd_UNLOAD_CUT_TAPE(self, gcmd):
        """Reverse and eject tape from cutter."""
        self._set_dir(False)
        self._set_pwm(self.unload_pwm)
        time.sleep(2)  # Example duration
        self._set_pwm(0)
        gcmd.respond_info("Tape unloaded successfully.")

    def cmd_ADVANCE_CUT_TAPE(self, gcmd):
        """Step tape forward."""
        mm = gcmd.get_float("LEN", None)
        pockets = gcmd.get_int("POCKETS", None)
        if mm is None and pockets is None:
            raise gcmd.error("Specify LEN=<mm> or POCKETS=<n>.")
        if pockets is not None:
            mm = pockets * self.pocket_length
        ticks = mm * self.ticks_per_mm
        self._set_dir(True)
        self._set_pwm(self.advance_pwm)
        start_ticks = self.encoder.get_position()
        while self.encoder.get_position() - start_ticks < ticks:
            self.printer.get_reactor().pause(0.01)
        self._set_pwm(0)
        gcmd.respond_info(f"Advanced tape by {mm:.2f} mm.")

    def cmd_PID(self, gcmd):
        """Runtime tweak of PID gains."""
        pid_type = gcmd.get("TYPE", "vel")
        if pid_type not in self.pid_params:
            raise gcmd.error(f"Invalid PID type: {pid_type}")
        for param in ["Kp", "Ki", "Kd"]:
            if gcmd.has(param):
                self.pid_params[pid_type][param] = gcmd.get_float(param)
        gcmd.respond_info(f"Updated PID gains for {pid_type}: {self.pid_params[pid_type]}")

    def cmd_PID_SAVE(self, gcmd):
        """Persist current PID gains to printer.cfg."""
        # Example implementation: Save to printer.cfg (requires additional logic)
        gcmd.respond_info("PID gains saved to printer.cfg (not implemented).")

    def cmd_FEEDER_STATUS(self, gcmd):
        """Get feeder status."""
        gcmd.respond_info(json.dumps({
            "configured": self.configured,
            "pwm_pin": self.pwm_pin['pin'],
            "dir_pin": self.dir_pin['pin'],
            "fault_pin": self.fault_pin['pin'],
            "limit_pin": self.limit_pin['pin'],
            "pid_params": self.pid_params,
        }))

def load_config(config):
    return TapeFeeder(config)