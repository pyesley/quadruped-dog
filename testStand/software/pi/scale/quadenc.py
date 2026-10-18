"""Quadrature encoder count, without losing edges.

Two back ends, picked automatically:

  kernel   The `rotary-encoder` device-tree overlay decodes A/B in interrupt
           context and emits one REL_X event per step on an input device. We
           just drain that device. Nothing runs in Python per edge, so the
           count stays right through an impact no matter what else the
           process is doing. This is the one to use for logging.

               sudo dtoverlay rotary-encoder pin_a=5 pin_b=6 relative_axis=1 steps-per-period=1

  gpiozero Falls back to gpiozero.RotaryEncoder if no such device exists.
           Fine for slow hand-turned checks; at high rates its Python
           callbacks drop edges while the ADC loop holds the GIL, and a
           dropped edge is a permanent position error.

Step semantics are the same for both (one count per full quadrature period),
so a counts-per-metre calibration made with one is valid for the other.
"""
import glob
import os
import struct

EV_REL, REL_X = 0x02, 0x00
_EVT = struct.Struct("llHHi")          # struct input_event on 64-bit ARM: 24 bytes


class Encoder:
    def __init__(self, pin_a=5, pin_b=6, prefer="kernel"):
        self.count = 0
        self.backend = None
        self._fd = None
        self._gz = None
        if prefer == "kernel":
            dev = self._find_device()
            if dev:
                self._fd = os.open(dev, os.O_RDONLY | os.O_NONBLOCK)
                self.backend = f"kernel ({dev})"
        if self.backend is None:
            from gpiozero import RotaryEncoder, Device
            from gpiozero.pins.lgpio import LGPIOFactory
            Device.pin_factory = LGPIOFactory()
            self._gz = RotaryEncoder(pin_a, pin_b, max_steps=0)
            self.backend = f"gpiozero (GPIO{pin_a}/{pin_b}) -- WARNING: drops edges at speed"

    @staticmethod
    def _find_device():
        for link in glob.glob("/dev/input/by-path/platform-rotary@*-event"):
            return os.path.realpath(link)
        for d in glob.glob("/sys/class/input/event*"):
            try:
                with open(os.path.join(d, "device", "name")) as f:
                    if f.read().strip().startswith("rotary@"):
                        return "/dev/input/" + os.path.basename(d)
            except OSError:
                pass
        return None

    def poll(self):
        """Fold any pending steps into .count. Cheap; call it every sample."""
        if self._fd is not None:
            while True:
                try:
                    buf = os.read(self._fd, _EVT.size * 512)
                except BlockingIOError:
                    break
                if not buf:
                    break
                for _, _, typ, code, val in _EVT.iter_unpack(buf):
                    if typ == EV_REL and code == REL_X:
                        self.count += val
                if len(buf) < _EVT.size * 512:
                    break
        else:
            self.count = self._gz.steps
        return self.count

    def zero(self):
        self.poll()
        if self._gz is not None:
            self._gz.steps = 0
        self.count = 0

    def close(self):
        if self._fd is not None:
            os.close(self._fd)
        if self._gz is not None:
            self._gz.close()
