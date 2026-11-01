-- The helicopter mesh: a low-poly Bell 206 style scale fuselage.
--
-- The geometry comes from rotorflight-lua-ethos-suite lib/heli3d.lua, which
-- copies the Rotorflight Configurator's bell_cw.gltf. That mesh has about 190
-- faces and takes several wakeups per frame; this one is cut down to about
-- 110 faces so a whole frame (project, cull, shade, sort) fits in one wakeup
-- next to the physics and the scenery.
--
-- Sizes are written in the glTF file's units (x = right side, y = up,
-- z = aft; the nose is at z -43, the main rotor disc has a radius of 78) and
-- converted at build to the sim's body axes in metres:
--   X = forward (nose), Y = left, Z = up, origin on the mast at glTF y 0.
-- The rotor disc radius becomes 0.8 m: a 700 size helicopter.
--
-- Built in stages (heli.buildStep() until it returns true) because Ethos
-- stops a callback that runs too many Lua instructions. The last stage
-- precomputes each face's unit normal, centroid and plane offset, so the
-- renderer can cull and light faces in model space without touching their
-- vertices.

local cos, sin, sqrt, pi = math.cos, math.sin, math.sqrt, math.pi
local min, max = math.min, math.max

local heli = {}

local S = 0.8 / 78     -- glTF units -> metres
local GY0 = 0.0        -- glTF y of the body origin

heli.ROTOR_RADIUS = 78 * S

-- Materials. A translucent material (the rotor discs) is drawn unshaded.
heli.MAT_BODY = 1
heli.MAT_TRIM = 2
heli.MAT_GLASS = 3
heli.MAT_METAL = 4
heli.MAT_BELLY = 5
heli.MAT_DISC = 6
local MAT_BODY, MAT_TRIM, MAT_GLASS, MAT_METAL, MAT_BELLY, MAT_DISC = 1, 2, 3, 4, 5, 6

-- Vertices (model space, metres).
local MX, MY, MZ = {}, {}, {}
heli.MX, heli.MY, heli.MZ = MX, MY, MZ
heli.vertexCount = 0
-- The low detail model: the first coreVertices vertices and coreFaces faces
-- (fuselage, boom, fin, stabiliser, main rotor disc).
heli.coreVertices, heli.coreFaces = 0, 0

-- Faces: up to four vertex indices (FD = 0 for a triangle), the material,
-- whether it is a flat plate seen from both sides (discs, fins), and the
-- precomputed unit normal (FNX..), centroid (FCX..) and plane offset FPD.
local FA, FB, FC, FD, FMAT, FTWO = {}, {}, {}, {}, {}, {}
local FNX, FNY, FNZ, FCX, FCY, FCZ, FPD = {}, {}, {}, {}, {}, {}, {}
heli.FA, heli.FB, heli.FC, heli.FD, heli.FMAT, heli.FTWO = FA, FB, FC, FD, FMAT, FTWO
heli.FNX, heli.FNY, heli.FNZ = FNX, FNY, FNZ
heli.FCX, heli.FCY, heli.FCZ, heli.FPD = FCX, FCY, FCZ, FPD
heli.faceCount = 0

-- The main rotor disc: its vertex range and rest positions, so the renderer
-- can tilt it with the cyclic.
heli.discFirst, heli.discLast = 0, 0
heli.DRX, heli.DRY, heli.DRZ = {}, {}, {}
heli.hubX, heli.hubY, heli.hubZ = 0, 0, 0

-- Contact points for the physics (body axes, metres).
heli.SKID_X_FRONT = 25.5 * S * 0.8   -- a little behind the upturn
heli.SKID_X_BACK = -14.3 * S
heli.SKID_Y = 17.7 * S
heli.SKID_Z = (-21.4 - 0.9 - GY0) * S
heli.HUB_Z = (25.6 - GY0) * S
heli.TAIL_X = -101 * S
heli.TAIL_Z = (9.45 - GY0) * S
heli.FIN_Z = (-6.2 - GY0) * S

local vertexCount, faceCount = 0, 0
local tailSide = 1

local function vtx(gx, gy, gz)
  vertexCount = vertexCount + 1
  MX[vertexCount] = -gz * S
  MY[vertexCount] = -gx * S
  MZ[vertexCount] = (gy - GY0) * S
  return vertexCount
end

local function pushFace(a, b, c, d, mat, twoSided)
  faceCount = faceCount + 1
  FA[faceCount], FB[faceCount], FC[faceCount], FD[faceCount] = a, b, c, d or 0
  FMAT[faceCount] = mat
  FTWO[faceCount] = twoSided
end

local function plate(a, b, c, d, mat)
  pushFace(a, b, c, d, mat, true)
end

-- One face of a closed part. (rx, ry, rz) is a glTF point inside the part
-- next to this face; the winding is flipped if needed so the normal points
-- away from it, which is all the back-face test needs.
local function solid(a, b, c, d, mat, rx, ry, rz)
  local ix, iy, iz = -rz * S, -rx * S, (ry - GY0) * S
  local ux, uy, uz = MX[b] - MX[a], MY[b] - MY[a], MZ[b] - MZ[a]
  local vx, vy, vz = MX[c] - MX[a], MY[c] - MY[a], MZ[c] - MZ[a]
  local nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
  local n = d and 4 or 3
  local gx = (MX[a] + MX[b] + MX[c] + (d and MX[d] or 0)) / n - ix
  local gy = (MY[a] + MY[b] + MY[c] + (d and MY[d] or 0)) / n - iy
  local gz = (MZ[a] + MZ[b] + MZ[c] + (d and MZ[d] or 0)) / n - iz
  if nx * gx + ny * gy + nz * gz < 0 then
    if d then a, b, c, d = a, d, c, b else b, c = c, b end
  end
  pushFace(a, b, c, d, mat, false)
end

-- Fuselage and boom stations (a subset of heli3d's): {z, top half width,
-- top y, widest half width, shoulder y, hip y, bottom half width, bottom y}.
-- Each becomes a ring of eight points: top left, top right, shoulder right,
-- hip right, bottom right, bottom left, hip left, shoulder left.
local STATIONS = {
  {-43, 0.8, -6.8, 1.6, -7.3, -8.6, 0.8, -9.2},
  {-34, 3.4, -1.0, 8.4, -5.0, -9.6, 4.5, -13.0},
  {-18, 4.4, 9.5, 10.2, 3.0, -9.8, 7.6, -14.3},
  {-4, 4.0, 15.7, 10.3, 4.6, -9.0, 9.3, -14.0},
  {16, 5.2, 14.9, 8.5, 4.7, -6.7, 6.7, -12.4},
  {29, 3.2, 11.4, 4.8, 4.5, 0.0, 2.2, -8.0},
  {33, 2.0, 9.0, 3.0, 6.5, 3.5, 1.6, 0.2},
  {86, 1.0, 10.4, 1.5, 9.5, 8.0, 0.9, 6.8},
}

-- Materials of each segment's eight bands (top, upper right, side right,
-- lower right, bottom, lower left, side left, upper left). The belly is
-- dark so upright and inverted read apart at a distance; the nose and a
-- stripe down the boom are trim coloured.
local B, T, G, D = MAT_BODY, MAT_TRIM, MAT_GLASS, MAT_BELLY
local SEGMENT_MATS = {
  {T, T, T, G, D, G, T, T},
  {G, G, G, G, D, G, G, G},
  {B, B, G, B, D, B, G, B},
  {T, B, B, B, D, B, B, B},
  {T, B, T, B, D, B, T, B},
  {B, B, T, B, D, B, T, B},
  {T, B, T, B, D, B, T, B},
}

local rings = {}

local function ring(k)
  local s = STATIONS[k]
  local z, tw, yt, hw, ys, yh, bw, yb = s[1], s[2], s[3], s[4], s[5], s[6], s[7], s[8]
  local r = rings[k] or {}
  r[1], r[2], r[3], r[4] = vtx(-tw, yt, z), vtx(tw, yt, z), vtx(hw, ys, z), vtx(hw, yh, z)
  r[5], r[6], r[7], r[8] = vtx(bw, yb, z), vtx(-bw, yb, z), vtx(-hw, yh, z), vtx(-hw, ys, z)
  rings[k] = r
end

local function segment(k)
  local r0, r1 = rings[k], rings[k + 1]
  local s0, s1 = STATIONS[k], STATIONS[k + 1]
  local rz = (s0[1] + s1[1]) * 0.5
  local ry = (s0[5] + s0[6] + s1[5] + s1[6]) * 0.25
  local mats = SEGMENT_MATS[k]
  for j = 1, 8 do
    local j2 = (j % 8) + 1
    solid(r0[j], r0[j2], r1[j2], r1[j], mats[j], 0, ry, rz)
  end
end

local function cap(k, mat, rz)
  local r = rings[k]
  local s = STATIONS[k]
  local ry = (s[5] + s[6]) * 0.5
  solid(r[1], r[2], r[3], r[8], mat, 0, ry, rz)
  solid(r[8], r[3], r[4], r[7], mat, 0, ry, rz)
  solid(r[7], r[4], r[5], r[6], mat, 0, ry, rz)
end

-- An n-sided upright prism around the y axis, y0 to y1, top closed if asked.
local function prism(cx, cz, radius, y0, y1, n, mat, top)
  local lo, hi = {}, {}
  for k = 1, n do
    local a = (k - 0.5) * 2 * pi / n
    local px, pz = cx + radius * cos(a), cz + radius * sin(a)
    lo[k], hi[k] = vtx(px, y0, pz), vtx(px, y1, pz)
  end
  local ym = (y0 + y1) * 0.5
  for k = 1, n do
    local k2 = (k % n) + 1
    solid(lo[k], lo[k2], hi[k2], hi[k], mat, cx, ym, cz)
  end
  if top then
    if n == 4 then
      solid(hi[1], hi[2], hi[3], hi[4], mat, cx, ym, cz)
    else
      for k = 2, n - 1 do solid(hi[1], hi[k], hi[k + 1], nil, mat, cx, ym, cz) end
    end
  end
end

-- A translucent disc: a fan of n triangles. axis "y" lies flat (the main
-- rotor), axis "x" stands upright across the boom (the tail rotor).
local function disc(cx, cy, cz, radius, n, axis, keep)
  local centre = vtx(cx, cy, cz)
  local first = vertexCount + 1
  for k = 1, n do
    local a = (k - 1) * 2 * pi / n
    if axis == "y" then
      vtx(cx + radius * cos(a), cy, cz + radius * sin(a))
    else
      vtx(cx, cy + radius * cos(a), cz + radius * sin(a))
    end
  end
  for k = 0, n - 1 do
    plate(centre, first + k, first + ((k + 1) % n), nil, MAT_DISC)
  end
  if keep then
    heli.discFirst, heli.discLast = centre, first + n - 1
    heli.hubX, heli.hubY, heli.hubZ = MX[centre], MY[centre], MZ[centre]
    for v = centre, first + n - 1 do
      heli.DRX[v], heli.DRY[v], heli.DRZ[v] = MX[v], MY[v], MZ[v]
    end
  end
end

-- A skid: a triangular tube along a path of {y, z} points at a given x.
local TUBE = 1.0
local function skid(x, path)
  local prev
  for i = 1, #path do
    local y, z = path[i][1], path[i][2]
    local q = {vtx(x, y + TUBE, z), vtx(x + TUBE, y - TUBE, z), vtx(x - TUBE, y - TUBE, z)}
    if prev then
      local my, mz = (y + path[i - 1][1]) * 0.5, (z + path[i - 1][2]) * 0.5
      for k = 1, 3 do
        local k2 = (k % 3) + 1
        solid(prev[k], prev[k2], q[k2], q[k], MAT_METAL, x, my, mz)
      end
    end
    prev = q
  end
end

-- A cross tube from the belly to a skid, as an upright strip.
local function crossTube(z)
  for s = -1, 1, 2 do
    plate(vtx(s * 7.5, -13.0, z - 1.0), vtx(s * 17.7, -21.4, z - 1.0),
      vtx(s * 17.7, -21.4, z + 1.0), vtx(s * 7.5, -13.0, z + 1.0), MAT_METAL)
  end
end

local function finishFaces(from, to)
  for i = from, to do
    local a, b, c, d = FA[i], FB[i], FC[i], FD[i]
    local nx, ny, nz
    local cx, cy, cz
    if d ~= 0 then
      local ux, uy, uz = MX[c] - MX[a], MY[c] - MY[a], MZ[c] - MZ[a]
      local vx, vy, vz = MX[d] - MX[b], MY[d] - MY[b], MZ[d] - MZ[b]
      nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
      cx = (MX[a] + MX[b] + MX[c] + MX[d]) * 0.25
      cy = (MY[a] + MY[b] + MY[c] + MY[d]) * 0.25
      cz = (MZ[a] + MZ[b] + MZ[c] + MZ[d]) * 0.25
    else
      local ux, uy, uz = MX[b] - MX[a], MY[b] - MY[a], MZ[b] - MZ[a]
      local vx, vy, vz = MX[c] - MX[a], MY[c] - MY[a], MZ[c] - MZ[a]
      nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
      cx = (MX[a] + MX[b] + MX[c]) / 3
      cy = (MY[a] + MY[b] + MY[c]) / 3
      cz = (MZ[a] + MZ[b] + MZ[c]) / 3
    end
    local len = sqrt(nx * nx + ny * ny + nz * nz)
    if len > 0 then nx, ny, nz = nx / len, ny / len, nz / len end
    FNX[i], FNY[i], FNZ[i] = nx, ny, nz
    FCX[i], FCY[i], FCZ[i] = cx, cy, cz
    FPD[i] = nx * cx + ny * cy + nz * cz
  end
end

local oldVertices, oldFaces = 0, 0

local STAGES = {
  function()
    oldVertices, oldFaces = vertexCount, faceCount
    vertexCount, faceCount = 0, 0
    for k = 1, #STATIONS do ring(k) end
    cap(1, MAT_TRIM, -40)
    cap(#STATIONS, MAT_BODY, 80)
  end,
  function() for k = 1, #STATIONS - 1 do segment(k) end end,
  function()
    -- Horizontal stabiliser, the fin on the side of the boom away from the
    -- tail rotor (upper and ventral part), the main rotor disc.
    for s = -1, 1, 2 do
      plate(vtx(s * 2.0, 6.4, 55.5), vtx(s * 12.5, 6.4, 57.0), vtx(s * 12.5, 6.4, 63.0), vtx(s * 2.0, 6.4, 63.0), MAT_TRIM)
    end
    local fx = -2.9 * tailSide
    plate(vtx(fx, 10.3, 83.5), vtx(fx, 27.8, 97.6), vtx(fx, 27.8, 101.3), vtx(fx, 10.3, 92.0), MAT_TRIM)
    plate(vtx(fx, 7.5, 85.0), vtx(fx, 7.5, 92.6), vtx(fx, -6.2, 96.9), vtx(fx, -6.2, 92.6), MAT_BODY)
    disc(0, 25.6, 0, 78, 12, "y", true)
    -- Everything up to here is the low detail model (see heli.coreVertices);
    -- the parts after it are only drawn when the helicopter is big enough on
    -- screen to show them.
    heli.coreVertices, heli.coreFaces = vertexCount, faceCount
    -- Mast and rotor head, the tail rotor disc.
    prism(0, 0, 2.6, 15.0, 26.0, 4, MAT_METAL, true)
    disc(3.6 * tailSide, 9.45, 88.65, 12.4, 8, "x", false)
  end,
  function()
    for s = -1, 1, 2 do
      skid(s * 17.7, {{-18.6, -25.5}, {-21.4, -18.0}, {-21.4, 14.3}})
    end
    crossTube(-15.5)
    crossTube(7.0)
    -- Free what a bigger mesh used before.
    for i = vertexCount + 1, oldVertices do MX[i], MY[i], MZ[i] = nil, nil, nil end
    for i = faceCount + 1, oldFaces do
      FA[i], FB[i], FC[i], FD[i], FMAT[i], FTWO[i] = nil, nil, nil, nil, nil, nil
      FNX[i], FNY[i], FNZ[i], FCX[i], FCY[i], FCZ[i], FPD[i] = nil, nil, nil, nil, nil, nil, nil
    end
    heli.vertexCount, heli.faceCount = vertexCount, faceCount
  end,
  function() finishFaces(1, faceCount) end,
}

local stage = 0

-- Start (re)building the mesh; rotor "cw" (default) puts the tail rotor on
-- the right of the boom, "ccw" on the left.
function heli.build(rotor)
  tailSide = (rotor == "ccw") and -1 or 1
  stage = 1
  heli.ready = false
end

-- One build stage; true once the mesh is complete.
function heli.buildStep()
  if stage == 0 then return heli.ready end
  STAGES[stage]()
  if stage < #STAGES then
    stage = stage + 1
    return false
  end
  stage = 0
  heli.ready = true
  return true
end

heli.ready = false

return heli
