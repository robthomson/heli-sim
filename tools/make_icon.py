"""Draw src/helisim/gfx/icon.png: the mesh's side view as an Ethos mask
(greyscale: black is drawn, white is clear, as lcd.loadMask expects).

    python tools/make_icon.py      (needs lupa and pillow)
"""
import os

from lupa import lua54
from PIL import Image, ImageDraw

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SIZE, SS = 75, 4   # final size, supersampling

os.chdir(os.path.join(ROOT, "src", "helisim"))
L = lua54.LuaRuntime()
h = L.eval('assert(loadfile("lib/heli.lua"))()')
h.build("cw")
while not h.buildStep():
    pass

xs = [h.MX[i] for i in range(1, h.vertexCount + 1)]
zs = [h.MZ[i] for i in range(1, h.vertexCount + 1)]
x0, x1, z0, z1 = min(xs), max(xs), min(zs), max(zs)
scale = (SIZE * SS * 0.92) / max(x1 - x0, z1 - z0)
ox = (SIZE * SS - (x1 - x0) * scale) / 2
oz = (SIZE * SS - (z1 - z0) * scale) / 2

img = Image.new("RGBA", (SIZE * SS, SIZE * SS), (0, 0, 0, 0))
d = ImageDraw.Draw(img)


def pt(i):
    # Nose to the right.
    return (ox + (h.MX[i] - x0) * scale, SIZE * SS - (oz + (h.MZ[i] - z0) * scale))


for f in range(1, h.faceCount + 1):
    ids = [h.FA[f], h.FB[f], h.FC[f]] + ([h.FD[f]] if h.FD[f] else [])
    if h.FMAT[f] == h.MAT_DISC:
        # The main rotor reads better as a thin blade line in a side view.
        continue
    d.polygon([pt(i) for i in ids], fill=(255, 255, 255, 255))
# Main rotor: a line across the top at hub height.
hz = SIZE * SS - (oz + (h.hubZ - z0) * scale)
d.rectangle([ox, hz - SS * 1.5, SIZE * SS - ox, hz + SS * 1.5], fill=(255, 255, 255, 255))

img = img.resize((SIZE, SIZE), Image.LANCZOS)
# White shape on clear -> black shape on white, greyscale.
img = Image.eval(img.getchannel("A"), lambda a: 255 - a)
out = os.path.join(ROOT, "src", "helisim", "gfx", "icon.png")
img.save(out)
print("wrote", out)
