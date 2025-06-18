# feeder.py — host helper for the RP2040 tape-feeder, “ppins” style
import json, time, logging

def load_config(config):
    return TapeFeeder(config)

class TapeFeeder:
    def __init__(self, config):
        pr = config.get_printer()
        self.reactor = pr.get_reactor()
        self.gcode   = pr.lookup_object("gcode")
        self.log     = logging.getLogger(config.get_name())

        ppins = pr.lookup_object("pins")

        # ───── MCU pins from cfg ─────
        self.pwm_pin   = ppins.setup_pin(
            "pwm",  config.get("pwm_pin",  "feeder:gpio15"))
        self.dir_pin   = ppins.setup_pin(
            "digital_out", config.get("dir_pin",   "feeder:gpio16"))
        self.fault_pin = ppins.setup_pin(
            "endstop", config.get("fault_pin", "feeder:gpio22"))
        self.limit_pin = ppins.setup_pin(
            "endstop", config.get("limit_pin", "feeder:gpio24"))

        # ───── angle-sensor object (defined in printer.cfg) ─────
        self.encoder = pr.lookup_object(config.get("encoder", "feeder_rotary"))

        # ───── mechanics ─────
        self.mm_per_rev     = config.getfloat("mm_per_rev",     40.0)
        self.ticks_per_rev  = config.getint  ("ticks_per_rev",  16384)
        self.ticks_per_mm   = self.ticks_per_rev / self.mm_per_rev
        self.pocket_length  = config.getfloat("pocket_length",  4.0)

        self.load_pwm   = config.getfloat("load_pwm",    0.40)
        self.unload_pwm = config.getfloat("unload_pwm",  0.40)
        self.advance_pwm= config.getfloat("advance_pwm", 0.45)

        # ───── state ─────
        self._vel = 0.0
        self._last_ticks = None
        self._last_time  = None
        self._fault_latched = False
        self._jam_latched   = False

        # ───── G-codes ─────
        g = self.gcode
        g.register_command("FEEDER_LOAD_CUT_TAPE", self._cmd_load,
                           desc="Home to optical switch, park at first pocket")
        g.register_command("FEEDER_UNLOAD_CUT_TAPE", self._cmd_unload,
                           desc="Reverse and eject tape from cutter")
        g.register_command("FEEDER_ADVANCE_CUT_TAPE", self._cmd_advance,
                           desc="Step tape forward (LEN=<mm>|POCKETS=<n>)")
        g.register_command("FEEDER_STOP",   self._cmd_stop)
        g.register_command("FEEDER_STATUS", self._cmd_status)

        self.reactor.register_timer(self._timer_vel, self.reactor.NOW)
        self.log.info("TapeFeeder helper initialised")

    # ───────────────── helpers ─────────────────
    def _set_pwm(self, duty): self.pwm_pin.set_duty_cycle(max(0,min(1,duty)))
    def _set_dir(self, fwd):  self.dir_pin.set_digital(bool(fwd))
    def _ticks(self):         return self.encoder.get_position()  # ANGLE helper
    def _limit(self):         return self.limit_pin.query_endstop()
    # polling the nFAULT line (optional, not yet latched):
    def _fault(self):         return self.fault_pin.query_endstop()

    # velocity estimator
    def _timer_vel(self, eventtime):
        t = self._ticks(); now = time.monotonic()
        if self._last_ticks is not None:
            dt = now - self._last_time
            if dt>0: self._vel = (t-self._last_ticks)/dt/self.ticks_per_mm
        self._last_ticks, self._last_time = t, now
        return eventtime + 0.10

    # ───────────────── G-codes ─────────────────
    def _cmd_load(self,g):
        self._set_dir(False); self._set_pwm(self.load_pwm)
        start = time.monotonic()
        while not self._limit() and time.monotonic()-start<6:
            self.reactor.pause(.05)
        self._set_pwm(0)
        if not self._limit(): return g.respond_error("Limit never hit!")
        self._last_ticks = self._ticks()
        g.respond_info("Feeder homed OK")

    def _cmd_unload(self,g):
        mm = g.get_float("LEN",50.0)
        tgt = abs(mm)*self.ticks_per_mm
        self._set_dir(False); self._set_pwm(self.unload_pwm)
        start = self._ticks()
        while abs(self._ticks()-start)<tgt: self.reactor.pause(.02)
        self._set_pwm(0); g.respond_info(f"Unloaded {mm:.1f} mm")

    def _cmd_advance(self,g):
        mm = g.get_float("LEN",None)
        pk = g.get_int("POCKETS",None)
        if mm is None and pk is None:
            return g.respond_error("Need LEN=<mm> or POCKETS=<n>")
        if mm is None: mm = pk*self.pocket_length
        tgt = mm*self.ticks_per_mm
        self._set_dir(True); self._set_pwm(self.advance_pwm)
        start = self._ticks()
        while (self._ticks()-start)<tgt: self.reactor.pause(.01)
        self._set_pwm(0); g.respond_info(f"Advanced {mm:.2f} mm")

    def _cmd_stop(self,g): self._set_pwm(0); g.respond_info("Feeder stopped")

    def _cmd_status(self,g):
        g.respond_info(json.dumps({
            "pos_mm": self._ticks()/self.ticks_per_mm,
            "vel_mm_s": self._vel,
            "limit": self._limit(),
            "fault": self._fault()
        }))
