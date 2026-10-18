#!/usr/bin/env python3
"""Drive ADS131M0x CLKIN from the Pi's GPCLK0 on GPIO4 (header pin 7).

The ADS131M0x has no internal oscillator -- CLKIN clocks the modulator AND the
digital/SPI core, so with it floating the chip cannot answer SPI at all.
PLLD is 500 MHz; /62 = 8.065 MHz, comfortably inside the part's CLKIN range and
an exact integer divide, so no MASH jitter.  Run with sudo.
"""
import mmap, os, struct, subprocess, sys, time

DIVI = int(sys.argv[1]) if len(sys.argv) > 1 else 62
CM_BASE, PASSWD = 0xFE101000, 0x5A000000
CTL, DIV = 0x70, 0x74
SRC_PLLD = 6

subprocess.run(['pinctrl', 'set', '4', 'a0'], check=False)   # GPIO4 -> GPCLK0
fd = os.open('/dev/mem', os.O_RDWR | os.O_SYNC)
m = mmap.mmap(fd, 4096, offset=CM_BASE)

def w(off, val):
    m.seek(off); m.write(struct.pack('<I', PASSWD | val))

w(CTL, SRC_PLLD)                       # disable, keep source
while True:
    m.seek(CTL)
    if not (struct.unpack('<I', m.read(4))[0] & (1 << 7)):   # wait !BUSY
        break
    time.sleep(0.001)
w(DIV, DIVI << 12)                     # DIVI, DIVF=0
w(CTL, (1 << 4) | SRC_PLLD)            # ENAB
time.sleep(0.01)
m.seek(CTL)
print('CM_GPCLK0_CTL = 0x%08X' % struct.unpack('<I', m.read(4))[0])
print('GPIO4 (header pin 7) now outputs %.3f MHz' % (500.0 / DIVI))
