~/stringpot -- string pot on GPIO5/6 (quadrature, kernel rotary-encoder overlay)

  cal.json      calibration 2026-10-03: y_mm = sqrt(alpha*(n+N0)^2 + k), rms 1.1 mm, 130-570 mm (ruler read in cm)
  stringpot.py  python3 stringpot.py --y0 <ruler mm now>   -> t count y_mm dy_mm
                python3 stringpot.py --raw                 -> t count   (laptop cal app)
                or: from stringpot import StringPot; sp.poll(); sp.dy

Per boot (plate_setup.sh does it):  sudo dtoverlay rotary-encoder pin_a=5 pin_b=6 relative_axis=1 steps-per-period=4
Recalibrate from the laptop:  python3 ~/quadruped-dog/testStand/software/laptop/stringcal/stringcal.py  -> http://localhost:8090
