#!/bin/sh
# (Re)start platelog detached, streaming to the dashboard host.
#   ./plate_start.sh LAPTOP_IP[:PORT] [extra platelog args]
# Needs plate_setup.sh to have been run since boot (CLKIN + encoder overlay).
cd "$(dirname "$0")" || exit 1
DEST="$1"; shift
[ -z "$DEST" ] && { echo "usage: $0 HOST[:PORT] [platelog args]"; exit 2; }
case "$DEST" in *:*) ;; *) DEST="$DEST:9870";; esac
pinctrl get 4 | grep -q GPCLK0 || { echo "CLKIN not running -- run: sudo ./plate_setup.sh"; exit 1; }
pkill -x platelog 2>/dev/null && sleep 0.5
mkdir -p logs
NEWTON=$(python3 -c 'import json;c=json.load(open("scale_cal.json"));print(",".join(f"{k*9.80665/1000:.9g}" for k in c["gains"]))')
MPC=$(python3 -c 'import json,os;p="leglog_config.json";print(json.load(open(p))["m_per_count"] if os.path.exists(p) else -1.735e-4)')
nohup setsid ./platelog --gain 64 --osr "${OSR:-4096}" --newton "$NEWTON" --m-per-count "$MPC" \
    --tare-file plate_tare.json --udp "$DEST" --out-dir logs --status 0 "$@" \
    > logs/platelog.log 2>&1 < /dev/null &
sleep 1.5
if pgrep -x platelog > /dev/null; then
    echo "platelog running (pid $(pgrep -x platelog)), feed -> $DEST"; tail -3 logs/platelog.log
else
    echo "platelog failed to start:"; cat logs/platelog.log; exit 1
fi
