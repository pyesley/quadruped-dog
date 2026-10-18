#!/bin/sh
# One-shot hardware bring-up for the force plate + carriage encoder. Needs sudo.
#
#   sudo ./plate_setup.sh           start ADC clock, load kernel quadrature decoder
#   sudo ./plate_setup.sh release   unload the decoder so gpiozero scripts can use GPIO5/6
#
# Neither survives a reboot.
set -e
cd "$(dirname "$0")"

if [ "$1" = "release" ]; then
    dtoverlay -r rotary-encoder 2>/dev/null && echo "rotary-encoder overlay removed" || echo "no overlay loaded"
    exit 0
fi

# Always (re)program the clock: GPIO4 can read as GPCLK0 after boot without running at
# the ADC's 8.065 MHz, so checking the pin function is not enough.  Reprogramming
# stops the clock for a moment, which would upset a running platelog -- skip then.
if pgrep -x platelog >/dev/null; then
    echo "platelog is running -- leaving CLKIN alone (stop platelog first to reprogram it)"
else
    python3 clkin.py
fi

if dtoverlay -l 2>/dev/null | grep -q rotary-encoder; then
    echo "rotary-encoder overlay already loaded"
else
    dtoverlay rotary-encoder pin_a=5 pin_b=6 relative_axis=1 steps-per-period=4
    sleep 0.5
    echo "rotary-encoder overlay loaded on GPIO5/6"
fi
ls -l /dev/input/by-path/platform-rotary@*-event 2>/dev/null || echo "WARNING: no rotary input device appeared"
