"""
Minimal driver for the mjbots fdcanusb / mjcanfd-usb-1x adapter.

The adapter appears as a USB CDC-ACM serial port (typically /dev/ttyACM0 on
Linux).  It speaks a simple newline-terminated 7-bit ASCII text protocol.  The
baud rate is irrelevant for a CDC-ACM device -- any value works.

Protocol summary (see https://github.com/mjbots/fdcanusb):

  Host -> device:
    can send <HEXID> <HEXDATA> <flags>   auto-detect standard/extended ID
    can std  <HEXID> <HEXDATA> <flags>   force standard (11-bit) ID
    can ext  <HEXID> <HEXDATA> <flags>   force extended (29-bit) ID
    can on | can off | can status        bus state control

    flags (space separated, optional):
      B/b  require / disable bitrate switching
      F/f  require / disable FDCAN (CAN-FD) framing
      R/r  require / disable remote frame

  Device -> host:
    rcv <HEXID> <HEXDATA> <flags>         a received CAN frame (async)
    OK                                    command succeeded
    ERR <details>                         command failed

Every command is acknowledged with a line ending in "OK" or "ERR ...".
Received CAN frames arrive spontaneously as "rcv ..." lines while bus is on.

This module deliberately implements only what is needed to (a) prove we can
talk to the adapter and (b) push raw CAN frames.  For the full moteus register
protocol use the official `moteus` package (see moteus_position.py), which can
drive this same adapter via `moteus.Fdcanusb`.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

import serial  # pyserial


DEFAULT_PORT = "/dev/ttyACM0"


@dataclass
class RcvFrame:
    """A CAN frame received from the bus via a `rcv` line."""
    arbitration_id: int
    data: bytes
    flags: str
    raw: str

    def __str__(self) -> str:
        return f"rcv id=0x{self.arbitration_id:X} data={self.data.hex().upper()} flags='{self.flags}'"


class Fdcanusb:
    """Raw text-protocol wrapper around the fdcanusb serial device."""

    def __init__(self, port: str = DEFAULT_PORT, timeout: float = 0.5):
        self.port = port
        self._ser = serial.Serial(port=port, baudrate=115200, timeout=timeout)

    # -- context manager -------------------------------------------------
    def __enter__(self) -> "Fdcanusb":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def close(self) -> None:
        if self._ser and self._ser.is_open:
            self._ser.close()

    # -- low level -------------------------------------------------------
    def _write_line(self, line: str) -> None:
        self._ser.write((line + "\n").encode("latin1"))
        self._ser.flush()

    def _read_line(self) -> str:
        return self._ser.readline().decode("latin1", errors="replace").strip()

    def command(self, line: str, read_until_ok: bool = True) -> list[str]:
        """Send one command line; collect reply lines up to OK/ERR.

        Returns every line received (including any interleaved `rcv` frames
        and the final OK/ERR line).
        """
        self._write_line(line)
        lines: list[str] = []
        if not read_until_ok:
            return lines
        deadline = time.time() + 1.0
        while time.time() < deadline:
            resp = self._read_line()
            if resp == "":
                continue
            lines.append(resp)
            if resp == "OK" or resp.startswith("ERR"):
                break
        return lines

    # -- convenience -----------------------------------------------------
    def status(self) -> list[str]:
        """Query adapter/bus status. Safe: does not touch any motor."""
        return self.command("can status")

    def bus_on(self) -> list[str]:
        return self.command("can on")

    def bus_off(self) -> list[str]:
        return self.command("can off")

    def send_frame(
        self,
        arbitration_id: int,
        data: bytes,
        *,
        fdcan: bool = True,
        bitrate_switch: bool = True,
        extended: bool | None = None,
    ) -> list[str]:
        """Send a raw CAN frame.

        arbitration_id : CAN ID (for moteus this is 0x8000 | dest_id to also
                         request a reply, or just dest_id for no reply).
        data           : payload bytes.
        fdcan          : use CAN-FD framing (moteus uses FD).
        bitrate_switch : use BRS (moteus uses BRS).
        extended       : force extended/standard ID. None => let adapter decide
                         via `can send`.
        """
        verb = "can send"
        if extended is True:
            verb = "can ext"
        elif extended is False:
            verb = "can std"

        flags = []
        flags.append("F" if fdcan else "f")
        flags.append("B" if bitrate_switch else "b")

        line = f"{verb} {arbitration_id:X} {data.hex().upper()} {' '.join(flags)}".strip()
        return self.command(line)

    def poll_rcv(self, timeout: float = 0.5) -> RcvFrame | None:
        """Read one spontaneous `rcv` frame, if any arrives within timeout."""
        old = self._ser.timeout
        self._ser.timeout = timeout
        try:
            line = self._read_line()
        finally:
            self._ser.timeout = old
        return parse_rcv(line)


def parse_rcv(line: str) -> RcvFrame | None:
    """Parse a `rcv <HEXID> <HEXDATA> <flags...>` line into an RcvFrame."""
    if not line or not line.startswith("rcv"):
        return None
    parts = line.split()
    # parts[0] == 'rcv', parts[1] == hex id, parts[2] == hex data, rest = flags
    if len(parts) < 3:
        return None
    try:
        arb_id = int(parts[1], 16)
        data = bytes.fromhex(parts[2])
    except ValueError:
        return None
    flags = " ".join(parts[3:])
    return RcvFrame(arbitration_id=arb_id, data=data, flags=flags, raw=line)
