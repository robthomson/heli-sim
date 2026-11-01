-- Flight model: a rigid body with a rate-stabilised (flybarless) rotor head.
--
-- World axes: X east, Y north, Z up, metres; the pilot stands at the origin
-- facing north. Body axes: X nose, Y left, Z up. The attitude is a rotation
-- matrix R (body -> world): its columns are the nose, left and up axes in the
-- world, stored as h.r11 .. h.r33 (row, column).
--
-- Body rates p, q, r turn about the body X, Y, Z axes (right-handed), so
-- p > 0 rolls right, q > 0 pitches the nose down and r > 0 yaws the nose
-- left. Sticks give rate commands (acro) or tilt commands (self-level); the
-- rates follow with a short lag, like a flybarless gyro.
--
-- The collective gives thrust along the body up axis, negative collective
-- pushes the other way (3D: inverted flight). Drag is stronger through the
-- rotor disc than edgewise. Near the ground the rotor gets some ground
-- effect. step() is called with a fixed dt; no tables are allocated.

local sqrt, abs, min, max = math.sqrt, math.abs, math.min, math.max

local physics = {}

local G = 9.81
local THRUST_MAX = 3.0 * G       -- full positive collective: hover at ~33 %, like a 3D setup
local COLLECTIVE_RESPONSE = 45   -- 1/s: pitch follows the stick in ~20 ms; no expo
local LEVEL_GAIN = 7             -- self-level: rad/s of rate per rad of error
local LEVEL_MAX_TILT = 0.75      -- self-level: tan of the full-stick tilt (~37 deg)

-- Drag per unit mass: quadratic and linear, edgewise (body X/Y) and through
-- the disc (body Z).
local DRAG_XY_Q, DRAG_XY_L = 0.035, 0.20
local DRAG_Z_Q, DRAG_Z_L = 0.05, 0.15

local GROUND_EFFECT = 0.22       -- extra thrust right at the ground
local GROUND_EFFECT_H = 1.6      -- metres (about one rotor diameter)

-- Landing limits.
local LAND_MAX_SINK = 3.0        -- m/s
local LAND_MAX_SLIDE = 3.5       -- m/s
local LAND_MIN_UPRIGHT = 0.80    -- cos of the tilt (~37 deg)

physics.G = G
physics.HOVER_COLLECTIVE = G / THRUST_MAX

-- Contact points (body axes), set from the mesh by physics.setGeometry().
local SKX1, SKX2, SKY, SKZ = 0.2, -0.15, 0.18, -0.23
local HUBZ, RR, TAILX, TAILZ = 0.26, 0.8, -1.03, 0.1

function physics.setGeometry(g)
  SKX1, SKX2, SKY, SKZ = g.SKID_X_FRONT, g.SKID_X_BACK, g.SKID_Y, g.SKID_Z
  HUBZ, RR, TAILX, TAILZ = g.HUB_Z, g.ROTOR_RADIUS, g.TAIL_X, g.FIN_Z
end

-- Put the helicopter on the ground at (x, y), nose towards heading (radians,
-- 0 = north, positive = towards east).
function physics.reset(h, x, y, heading)
  local s, c = math.sin(heading or 0), math.cos(heading or 0)
  -- nose = (s, c, 0); left = (-c, s, 0); up = (0, 0, 1)
  h.r11, h.r12, h.r13 = s, -c, 0
  h.r21, h.r22, h.r23 = c, s, 0
  h.r31, h.r32, h.r33 = 0, 0, 1
  h.px, h.py, h.pz = x or 0, y or 0, -SKZ
  h.vx, h.vy, h.vz = 0, 0, 0
  h.p, h.q, h.r = 0, 0, 0
  h.coll = 0
  h.spool = 0
  h.landed = true
  h.crashed = false
  h.crashReason = nil
end

local function crash(h, reason)
  h.crashed = true
  h.crashReason = reason
  h.vx, h.vy, h.vz = 0, 0, 0
  h.p, h.q, h.r = 0, 0, 0
end

-- Level the attitude, keeping the heading.
local function levelAttitude(h)
  local nx, ny = h.r11, h.r21
  local n = sqrt(nx * nx + ny * ny)
  if n < 1e-3 then nx, ny, n = -h.r12, -h.r22, 1 end
  nx, ny = nx / n, ny / n
  h.r11, h.r12, h.r13 = nx, -ny, 0
  h.r21, h.r22, h.r23 = ny, nx, 0
  h.r31, h.r32, h.r33 = 0, 0, 1
end

-- in: roll, pitch, yaw, coll in -1..1 (already shaped), plus
-- in.rate (rad/s at full stick), in.yawRate, in.response (1/s: how fast the
-- rates follow the commands), in.spoolTime (s), in.level (bool), in.spoolUp
-- (bool: rotor running).
function physics.step(h, inp, dt)
  if h.crashed then return end

  -- Rotor speed: spools up over a few seconds, down when stopped.
  local spoolTarget = inp.spoolUp and 1 or 0
  if h.spool < spoolTarget then
    h.spool = min(1, h.spool + dt / inp.spoolTime)
  elseif h.spool > spoolTarget then
    h.spool = max(0, h.spool - dt * 0.5)
  end
  local spool = h.spool

  local r11, r12, r13 = h.r11, h.r12, h.r13
  local r21, r22, r23 = h.r21, h.r22, h.r23
  local r31, r32, r33 = h.r31, h.r32, h.r33

  -- Rate commands.
  local pc, qc
  if inp.level then
    -- Target up vector from the stick tilt, in the heading frame.
    local fx, fy = r11, r21                     -- nose, flattened
    local fn = sqrt(fx * fx + fy * fy)
    if fn < 1e-3 then fx, fy, fn = r13, r23, sqrt(r13 * r13 + r23 * r23) + 1e-6 end
    fx, fy = fx / fn, fy / fn
    local rx, ry = fy, -fx                      -- right of the heading
    local tp, tr = inp.pitch * LEVEL_MAX_TILT, inp.roll * LEVEL_MAX_TILT
    local ux, uy, uz = fx * tp + rx * tr, fy * tp + ry * tr, 1
    local un = sqrt(ux * ux + uy * uy + uz * uz)
    ux, uy, uz = ux / un, uy / un, uz / un
    -- Error axis: body up x target up (world), into body axes.
    local ex = r23 * uz - r33 * uy
    local ey = r33 * ux - r13 * uz
    local ez = r13 * uy - r23 * ux
    -- Fully inverted: the cross product vanishes; roll out.
    if r33 < -0.95 and abs(ex) + abs(ey) + abs(ez) < 0.3 then ex, ey, ez = r11, r21, r31 end
    pc = (r11 * ex + r21 * ey + r31 * ez) * LEVEL_GAIN
    qc = (r12 * ex + r22 * ey + r32 * ez) * LEVEL_GAIN
    local lim = inp.rate
    if pc > lim then pc = lim elseif pc < -lim then pc = -lim end
    if qc > lim then qc = lim elseif qc < -lim then qc = -lim end
  else
    pc = inp.roll * inp.rate
    qc = inp.pitch * inp.rate
  end
  local rc = -inp.yaw * inp.yawRate

  -- Head response scales with rotor speed.
  local kc = min(1, dt * inp.response * spool)
  local ky = min(1, dt * inp.response * 1.3 * spool)
  h.coll = h.coll + (inp.coll - h.coll) * min(1, dt * COLLECTIVE_RESPONSE)

  local thrust = h.coll * THRUST_MAX * spool * spool
  if h.pz < GROUND_EFFECT_H + 1 and r33 > 0.5 then
    local hh = h.pz + SKZ
    if hh < GROUND_EFFECT_H then
      local k = 1 - max(0, hh) / GROUND_EFFECT_H
      thrust = thrust * (1 + GROUND_EFFECT * k * k)
    end
  end

  if h.landed then
    h.p, h.q = 0, 0
    h.r = h.r + (0 - h.r) * min(1, dt * 20)
    -- Lift off when the rotor carries more than the weight.
    if thrust * r33 > G * 1.02 then
      h.landed = false
      h.pz = h.pz + 0.01
    else
      return
    end
  end

  local p = h.p + (pc - h.p) * kc
  local q = h.q + (qc - h.q) * kc
  local r = h.r + (rc - h.r) * ky
  h.p, h.q, h.r = p, q, r

  -- Attitude: R = R * (I + [w]x dt), then re-orthonormalise.
  local wx, wy, wz = p * dt, q * dt, r * dt
  r11, r12, r13 = r11 + r12 * wz - r13 * wy, r12 - r11 * wz + r13 * wx, r13 + r11 * wy - r12 * wx
  r21, r22, r23 = r21 + r22 * wz - r23 * wy, r22 - r21 * wz + r23 * wx, r23 + r21 * wy - r22 * wx
  r31, r32, r33 = r31 + r32 * wz - r33 * wy, r32 - r31 * wz + r33 * wx, r33 + r31 * wy - r32 * wx
  local n = 1 / sqrt(r11 * r11 + r21 * r21 + r31 * r31)
  r11, r21, r31 = r11 * n, r21 * n, r31 * n
  local d = r11 * r12 + r21 * r22 + r31 * r32
  r12, r22, r32 = r12 - d * r11, r22 - d * r21, r32 - d * r31
  n = 1 / sqrt(r12 * r12 + r22 * r22 + r32 * r32)
  r12, r22, r32 = r12 * n, r22 * n, r32 * n
  r13 = r21 * r32 - r31 * r22
  r23 = r31 * r12 - r11 * r32
  r33 = r11 * r22 - r21 * r12
  h.r11, h.r12, h.r13 = r11, r12, r13
  h.r21, h.r22, h.r23 = r21, r22, r23
  h.r31, h.r32, h.r33 = r31, r32, r33

  -- Forces.
  local vx, vy, vz = h.vx, h.vy, h.vz
  local bx = r11 * vx + r21 * vy + r31 * vz
  local by = r12 * vx + r22 * vy + r32 * vz
  local bz = r13 * vx + r23 * vy + r33 * vz
  local fx = -(DRAG_XY_Q * abs(bx) + DRAG_XY_L) * bx
  local fy = -(DRAG_XY_Q * abs(by) + DRAG_XY_L) * by
  local fz = thrust - (DRAG_Z_Q * abs(bz) + DRAG_Z_L) * bz
  local ax = r11 * fx + r12 * fy + r13 * fz
  local ay = r21 * fx + r22 * fy + r23 * fz
  local az = r31 * fx + r32 * fy + r33 * fz - G

  vx, vy, vz = vx + ax * dt, vy + ay * dt, vz + az * dt
  h.vx, h.vy, h.vz = vx, vy, vz
  local px, py, pz = h.px + vx * dt, h.py + vy * dt, h.pz + vz * dt
  h.px, h.py, h.pz = px, py, pz

  -- Ground contact. Only worth checking near the ground.
  if pz > 1.6 then return end

  -- The lowest point of the rotor disc, then the tail.
  local hz = pz + r33 * HUBZ
  if hz - sqrt(r31 * r31 + r32 * r32) * RR < 0.0 then
    crash(h, "Blade strike")
    return
  end
  if pz + r31 * TAILX + r33 * TAILZ < 0 then
    crash(h, "Tail strike")
    return
  end
  -- Skids: lowest of the four ends.
  local low = pz + r33 * SKZ
  local zx1, zx2, zy = r31 * SKX1, r31 * SKX2, abs(r32 * SKY)
  low = low + min(zx1, zx2) - zy
  if low < 0 then
    local slide = sqrt(vx * vx + vy * vy)
    if r33 < LAND_MIN_UPRIGHT then
      crash(h, "Tipped over")
    elseif vz < -LAND_MAX_SINK then
      crash(h, "Hard landing")
    elseif slide > LAND_MAX_SLIDE then
      crash(h, "Skid catch")
    else
      h.landed = true
      levelAttitude(h)
      h.pz = -SKZ
      h.vx, h.vy, h.vz = 0, 0, 0
      h.p, h.q = 0, 0
    end
    if h.crashed then h.pz = max(pz, 0.05) end
  end
end

return physics
