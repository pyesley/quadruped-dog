"""Driver for the TI ADS131M0x family (M02/M04/M06/M08).

24-bit simultaneous-sampling delta-sigma ADC with a per-channel PGA -- one load
cell per channel, every channel sampled at the same instant, which is what you
want for a multi-corner scale.

Wire protocol: fixed-length frames of 24-bit words, 16-bit values LEFT-aligned
inside each word.

    we send:     [command][pad]...[pad]
    we receive:  [status][ch0]...[chN-1][crc]

The reply to a command lands in the *next* frame, so a register read is always
a two-frame operation.

Two things on this part will silently give you plausible-looking garbage if you
get them wrong, so both are handled here:

  frame length   Guess the channel count wrong and you truncate the frame
                 mid-stream. The reads still look like data. detect() settles it
                 by brute force: only the true length produces a valid CRC.

  sample pacing  SPI runs far faster than the conversion rate, so back-to-back
                 reads return the SAME conversion over and over. Averaging those
                 buys you nothing. read() waits for the data to actually change.
"""

import time

try:
    import spidev
except ImportError:  # pragma: no cover
    spidev = None

# ---------------------------------------------------------------- registers
REG_ID = 0x00
REG_STATUS = 0x01
REG_MODE = 0x02
REG_CLOCK = 0x03
REG_GAIN1 = 0x04
REG_GAIN2 = 0x05
REG_CFG = 0x06
REG_CH0_CFG = 0x09
CH_BLOCK = 0x05

CMD_NULL = 0x0000
CMD_RESET = 0x0011
CMD_STANDBY = 0x0022
CMD_WAKEUP = 0x0033

OSR_MAP = {128: 0, 256: 1, 512: 2, 1024: 3, 2048: 4, 4096: 5, 8192: 6, 16384: 7}
GAIN_MAP = {1: 0, 2: 1, 4: 2, 8: 3, 16: 4, 32: 5, 64: 6, 128: 7}
PWR_HIGH_RES = 0b10


def crc16_ccitt(data: bytes) -> int:
    """CCITT-CRC16, poly 0x1021, init 0xFFFF -- what the ADS131M0x appends."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def detect(bus=0, device=0, speed_hz=1_000_000, tries=20):
    """Brute-force the channel count: only the right frame length CRCs clean."""
    if spidev is None:
        raise RuntimeError("spidev not installed: sudo apt install python3-spidev")
    s = spidev.SpiDev()
    s.open(bus, device)
    s.mode = 1
    s.max_speed_hz = speed_hz
    try:
        for nch in (8, 6, 4, 2):
            n = (nch + 2) * 3
            ok = 0
            for _ in range(tries):
                r = bytes(s.xfer2([0] * n))
                if crc16_ccitt(r[:-3]) == int.from_bytes(r[-3:], "big") >> 8:
                    ok += 1
            if ok >= tries * 0.9:
                return nch
        return None
    finally:
        s.close()


class ADS131M0x:
    def __init__(self, bus=0, device=0, nch=8, speed_hz=2_000_000,
                 drdy_pin=None, gpio_chip=0):
        if spidev is None:
            raise RuntimeError("spidev not installed: sudo apt install python3-spidev")
        self.nch = nch
        self.words = nch + 2
        self.frame_len = self.words * 3
        self.gain = [1] * nch
        self.osr = 1024
        self._last = None
        self.crc_errors = 0
        self.reg_read_retries = 0
        self.reg_write_retries = 0
        self._resp_offset = 1
        self.resyncs = 0

        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)
        self.spi.max_speed_hz = speed_hz
        self.spi.mode = 1                 # CPOL=0, CPHA=1
        self.spi.bits_per_word = 8

        self._h = None
        self._drdy = drdy_pin
        if drdy_pin is not None:
            import lgpio
            self._lgpio = lgpio
            self._h = lgpio.gpiochip_open(gpio_chip)
            lgpio.gpio_claim_input(self._h, drdy_pin, lgpio.SET_PULL_UP)

    # ------------------------------------------------------------- plumbing
    def _xfer(self, cmd=CMD_NULL, payload=()):
        out = bytearray()
        out += (cmd << 8).to_bytes(3, "big")      # 16-bit cmd LEFT-aligned in 24
        for w in payload:
            out += (w << 8).to_bytes(3, "big")
        out += b"\x00" * (self.frame_len - len(out))
        raw = bytes(self.spi.xfer2(list(out)))
        return [int.from_bytes(raw[i:i + 3], "big") for i in range(0, len(raw), 3)], raw

    def _cmd(self, cmd, payload=()):
        self._xfer(cmd, payload)
        for _ in range(self._resp_offset):
            words, _ = self._xfer(CMD_NULL)
        return words[0] >> 8

    def _resync(self):
        """Idle frames, to park the response pipeline at a known point."""
        self._xfer(CMD_NULL)
        self._xfer(CMD_NULL)

    def read_reg(self, addr, count=1, tries=6):
        """Read a register, confirmed by a second identical read.

        The value comes back _resp_offset frames after the command. That is
        normally 1, but a process killed mid-transfer leaves the device a frame
        behind, and then every read returns the STATUS word -- 0x0500 or 0x05FF,
        both of which pass for believable register contents. sync_pipeline()
        measures the real offset rather than trusting it.
        """
        cmd = 0xA000 | ((addr & 0x3F) << 7) | ((count - 1) & 0x7F)
        prev = None
        for _ in range(tries):
            self._resync()
            self._xfer(cmd)
            for _ in range(self._resp_offset):
                words, _ = self._xfer(CMD_NULL)
            vals = [w >> 8 for w in words[:count]]
            if vals == prev:
                return vals[0] if count == 1 else vals
            prev = vals
        self.reg_read_retries += 1
        return prev[0] if count == 1 else prev

    def sync_pipeline(self, tries=3):
        """Find how many frames behind the command a response actually lands.

        Anchored on MODE, whose post-reset value (0x0510) is known, so there is
        a right answer to search for rather than a guess to trust.
        """
        for _ in range(tries):
            self._xfer(CMD_RESET)
            time.sleep(0.01)
            self._resync()
            cmd = 0xA000 | (REG_MODE << 7)
            self._resync()
            self._xfer(cmd)
            for off in range(1, 5):
                words, _ = self._xfer(CMD_NULL)
                if (words[0] >> 8) == 0x0510:
                    if off != self._resp_offset:
                        self.resyncs += 1
                    self._resp_offset = off
                    self._last = None
                    return off
        raise IOError("cannot locate the ADS131 response pipeline "
                      "(MODE never read back as 0x0510)")

    def write_reg(self, addr, value, verify=True, tries=4):
        """Write, then read back. Never trust a register write silently."""
        value &= 0xFFFF
        for attempt in range(tries):
            self._resync()
            self._cmd(0x6000 | ((addr & 0x3F) << 7), [value])
            if not verify:
                return True
            if self.read_reg(addr) == value:
                return True
            self.reg_write_retries += 1
            if attempt == 0:
                self.sync_pipeline()   # wrong offset is the usual cause
        raise IOError(f"register 0x{addr:02X} would not take 0x{value:04X} "
                      f"(reads back 0x{self.read_reg(addr):04X})")

    # ------------------------------------------------------------- lifecycle
    def reset(self):
        self._xfer(CMD_RESET)
        time.sleep(0.01)
        words, _ = self._xfer(CMD_NULL)
        self._last = None
        return words[0] >> 8                # 0xFF08 for M08, 0xFF04 for M04

    def configure(self, gain=64, osr=4096, channels=None):
        """gain: 1..128, an int for all channels or a list of one per channel."""
        self.sync_pipeline()
        if isinstance(gain, int):
            gain = [gain] * self.nch
        g1 = 0
        for i, g in enumerate(gain[:4]):
            g1 |= GAIN_MAP[g] << (4 * i)
        self.write_reg(REG_GAIN1, g1)
        if self.nch > 4:
            g2 = 0
            for i, g in enumerate(gain[4:8]):
                g2 |= GAIN_MAP[g] << (4 * i)
            self.write_reg(REG_GAIN2, g2)

        # Build CLOCK from scratch rather than read-modify-write. The top byte
        # is the per-channel enable mask, so a single bad read fed back into a
        # RMW silently switches channels off -- they then read as exact zeros,
        # which is easy to mistake for a dead load cell.
        if channels is None:
            channels = (1 << self.nch) - 1          # every channel on
        clock = ((channels & 0xFF) << 8) | (OSR_MAP[osr] << 2) | PWR_HIGH_RES
        self.write_reg(REG_CLOCK, clock)
        self.gain = gain
        self.osr = osr
        self._last = None

    # ----------------------------------------------------------------- reads
    def wait_drdy(self, timeout=1.0):
        if self._drdy is None:
            return True
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if self._lgpio.gpio_read(self._h, self._drdy) == 0:
                return True
            time.sleep(0.0002)
        return False

    def read(self, wait_new=True, check_crc=True, timeout=0.5):
        """One simultaneous conversion -> signed counts, one per channel.

        Blocks until the ADC has actually produced a new conversion, so N reads
        really are N independent samples and averaging behaves as advertised.
        """
        end = time.monotonic() + timeout
        bad = 0
        while True:
            self.wait_drdy()
            words, raw = self._xfer(CMD_NULL)
            if check_crc and crc16_ccitt(raw[:-3]) != (words[-1] >> 8):
                # Persistent CRC failure means frame alignment is gone, not that
                # one frame was unlucky. Retrying forever spins the caller
                # solid -- try to resync, then give up loudly.
                self.crc_errors += 1
                bad += 1
                if bad % 8 == 0:
                    try:
                        self.sync_pipeline()
                    except IOError:
                        pass
                if time.monotonic() > end:
                    raise IOError(f"ADS131 frames failing CRC "
                                  f"({bad} in a row, {self.crc_errors} total)")
                continue
            cur = tuple(words[1:1 + self.nch])
            if not wait_new or cur != self._last or time.monotonic() > end:
                self._last = cur
                break
        return [w - (1 << 24) if w & 0x800000 else w for w in cur]

    def read_avg(self, n=16, **kw):
        acc = [0] * self.nch
        for _ in range(n):
            for i, v in enumerate(self.read(**kw)):
                acc[i] += v
        return [a / n for a in acc]

    def measure_odr(self, seconds=1.0):
        """Actual output data rate, by counting how often the data changes."""
        prev, changes = None, 0
        t = time.monotonic()
        while time.monotonic() - t < seconds:
            words, _ = self._xfer(CMD_NULL)
            cur = tuple(words[1:1 + self.nch])
            if cur != prev:
                changes += 1
            prev = cur
        self._last = None
        return changes / (time.monotonic() - t)

    def measure_clkin(self, seconds=1.0):
        """Infer CLKIN from the observed rate: f_DATA = f_CLKIN / 2 / OSR."""
        return self.measure_odr(seconds) * 2 * self.osr

    def counts_to_volts(self, counts, ch=0, vref=1.2):
        return counts / (1 << 23) * vref / self.gain[ch]

    def close(self):
        try:
            self.spi.close()
        finally:
            if self._h is not None:
                self._lgpio.gpiochip_close(self._h)
