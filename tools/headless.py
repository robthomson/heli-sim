"""Run HeliSim headless: stubbed Ethos lcd/system, scripted sticks.

Counts Lua instructions per wakeup/paint (Ethos aborts a script that uses
its whole per-cycle budget) and saves chosen frames as PNG.

    python tools/headless.py [--out DIR] [--frames 40,120,...] [--scenario NAME]

Needs: pip install lupa pillow
"""
import argparse
import os
import sys

from lupa import lua54
from PIL import Image, ImageDraw, ImageFont

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(ROOT, "src", "helisim")
W, H = 800, 480
WAKE_DT = 0.05

# Stick scripts: (start time s, rud, ele, thr, ail) in -1..1, held until the next row.
SCENARIOS = {
    "hover": [
        (0.0, 0, 0, -1, 0),
        (0.5, 0, 0, -0.6, 0),      # spool up, on the ground
        (4.5, 0, 0, 0.55, 0),     # climb
        (6.0, 0, 0, 0.33, 0),     # hover
        (7.0, 0, 0.25, 0.36, 0),  # forward
        (8.5, 0, 0, 0.35, 0.4),   # roll right
        (9.2, 0, 0, 0.35, 0),
        (10.0, 0.5, 0, 0.35, 0),  # yaw
        (11.0, 0, 0, 0.35, 0),
    ],
    "circuit": [                  # self-level: take off, fly away, turn, come back
        (0.0, 0, 0, -1, 0),
        (0.5, 0, 0, -0.6, 0),
        (4.5, 0, 0, 0.6, 0),
        (5.5, 0, 0, 0.34, 0),
        (6.5, 0, 0.5, 0.37, 0),    # forward, away from the pilot
        (9.0, 0, 0.3, 0.37, 0.6),  # banked turn right
        (12.0, 0, 0.4, 0.36, 0),
        (15.0, 0, -0.3, 0.33, 0),  # slow down
        (17.0, 0, 0, 0.28, 0),     # descend
    ],
    "flip": [
        (0.0, 0, 0, -1, 0),
        (0.5, 0, 0, -0.6, 0),
        (4.5, 0, 0, 0.9, 0),      # punch out
        (6.0, 0, 0, 0.3, 0),
        (6.5, 0, 0, 0.0, 1.0),    # roll to inverted
        (6.9, 0, 0, -0.35, 0),    # inverted hover: negative collective
        (8.5, 0, 0, -0.35, 0),
    ],
}

STUBS = r"""
local W, H = %d, %d
local T = 0
local sticks = {[0] = 0, 0, 0, 0}
draw = {}       -- recorded ops for the current paint
local colour = 0
local function rec(op) draw[#draw + 1] = op end
lcd = {
  RGB = function(r, g, b, a)
    a = a or 1
    return math.floor(r) * 16777216 + math.floor(g) * 65536 + math.floor(b) * 256 + math.floor(a * 255)
  end,
  color = function(c) colour = c end,
  getWindowSize = function() return W, H end,
  drawFilledRectangle = function(x, y, w, h) rec({"rect", colour, x, y, w, h}) end,
  drawFilledTriangle = function(a, b, c, d, e, f) rec({"tri", colour, a, b, c, d, e, f}) end,
  drawLine = function(a, b, c, d) rec({"line", colour, a, b, c, d}) end,
  drawFilledCircle = function(x, y, r) rec({"circle", colour, x, y, r}) end,
  drawText = function(x, y, s, flags) rec({"text", colour, x, y, s, flags or 0}) end,
  font = function() end,
  invalidate = function() end,
  loadMask = function() return nil end,
  darkMode = function() return true end,
}
BLACK = lcd.RGB(0, 0, 0); WHITE = lcd.RGB(255, 255, 255)
FONT_S, FONT_STD, FONT_L = 1, 2, 3
CENTERED, RIGHT = 1, 2
CATEGORY_ANALOG = 1
EVT_KEY, EVT_CLOSE = 0, 1
KEY_ENTER_BREAK, KEY_ENTER_LONG, KEY_ENTER_FIRST = 10, 11, 12
KEY_EXIT_BREAK, KEY_EXIT_FIRST = 20, 21
KEY_ROTARY_LEFT, KEY_ROTARY_RIGHT = 30, 31
system = {
  getSource = function(t)
    return {value = function() return math.floor(sticks[t.member] * 1024) end}
  end,
  playTone = function() end,
}
os.clock = function() return T end
function setTime(t) T = t end
function setSticks(r, e, t, a) sticks[0], sticks[1], sticks[2], sticks[3] = r, e, t, a end

counted = 0
function countCall(fn, ...)
  local n = 0
  debug.sethook(function() n = n + 1 end, "", 1)
  fn(...)
  debug.sethook()
  return n
end
""" % (W, H)


def unpack_rgba(c):
    c = int(c)
    return ((c >> 24) & 255, (c >> 16) & 255, (c >> 8) & 255, c & 255)


def render_png(ops, path):
    img = Image.new("RGBA", (W, H), (0, 0, 0, 255))
    font = ImageFont.load_default()
    for op in ops:
        kind = op[1]
        col = unpack_rgba(op[2])
        layer = Image.new("RGBA", (W, H), (0, 0, 0, 0)) if col[3] < 255 else None
        d = ImageDraw.Draw(layer if layer else img)
        if kind == "rect":
            x, y, w, h = op[3], op[4], op[5], op[6]
            d.rectangle([x, y, x + w - 1, y + h - 1], fill=col)
        elif kind == "tri":
            d.polygon([(op[3], op[4]), (op[5], op[6]), (op[7], op[8])], fill=col)
        elif kind == "line":
            d.line([(op[3], op[4]), (op[5], op[6])], fill=col)
        elif kind == "circle":
            x, y, r = op[3], op[4], op[5]
            d.ellipse([x - r, y - r, x + r, y + r], fill=col)
        elif kind == "text":
            x, y, s, flags = op[3], op[4], str(op[5]), op[6]
            tw = d.textlength(s, font=font)
            if flags == 1:
                x -= tw / 2
            elif flags == 2:
                x -= tw
            d.text((x, y), s, fill=col, font=font)
        if layer:
            img = Image.alpha_composite(img, layer)
    img.convert("RGB").save(path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(ROOT, "tools", "out"))
    ap.add_argument("--frames", default="")
    ap.add_argument("--scenario", default="hover")
    ap.add_argument("--seconds", type=float, default=12.0)
    ap.add_argument("--level", action="store_true", help="self-level flight mode")
    ap.add_argument("--normal", action="store_true", help="normal (not 3D) collective")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    want = {int(x) for x in args.frames.split(",") if x}

    os.chdir(SRC)  # Ethos runs a tool with its own folder as the current directory
    L = lua54.LuaRuntime()
    L.execute(STUBS)
    sim = L.eval('assert(loadfile("sim.lua"))()')
    g = L.globals()
    state = sim.create(None)
    state.cfg.level = args.level
    state.cfg.col3d = not args.normal
    state.cfg.spoolTime = 3     # the scenarios expect a quick spool-up

    script = SCENARIOS[args.scenario]
    worst_w = worst_p = 0
    total_w = total_p = 0
    frames = int(args.seconds / WAKE_DT)
    for f in range(frames):
        t = f * WAKE_DT
        row = [r for r in script if r[0] <= t][-1]
        g.setSticks(row[1], row[2], row[3], row[4])
        g.setTime(t)
        nw = g.countCall(sim.wakeup, state)
        g.draw = L.table()
        np_ = g.countCall(sim.paint, state)
        worst_w, worst_p = max(worst_w, nw), max(worst_p, np_)
        total_w += nw
        total_p += np_
        h = state.h
        if f % 20 == 0 or f in want:
            print(f"f{f:4d} t={t:5.2f} {state.phase:8s} wake={nw:6d} paint={np_:5d} "
                  f"pos=({h.px:6.2f},{h.py:6.2f},{h.pz:6.2f}) up_z={h.r33:5.2f} "
                  f"spool={h.spool:4.2f} landed={h.landed} {h.crashReason or ''}")
        if f in want:
            ops = [tuple(op.values()) for op in g.draw.values()]
            path = os.path.join(args.out, f"{args.scenario}_{f:04d}.png")
            render_png([(None,) + op for op in ops], path)
            print("  saved", path)
    print(f"worst wakeup {worst_w}, worst paint {worst_p}; "
          f"mean wakeup {total_w // frames}, mean paint {total_p // frames}")


if __name__ == "__main__":
    sys.exit(main())
