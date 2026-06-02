#!/usr/bin/env python3
"""Move + left-click at (x,y) on DISPLAY :20 via XTEST (no sudo/xdotool)."""
import sys, time
from Xlib import display, X
from Xlib.ext import xtest

x = int(sys.argv[1]); y = int(sys.argv[2])
d = display.Display()
root = d.screen().root
root.warp_pointer(x, y)
d.sync(); time.sleep(0.2)
xtest.fake_input(d, X.ButtonPress, 1); d.sync(); time.sleep(0.08)
xtest.fake_input(d, X.ButtonRelease, 1); d.sync()
print(f"clicked ({x},{y})")
