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
-- effect. run() steps with a fixed dt; no tables are allocated.

local sqrt, abs = math.sqrt, math.abs

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

-- in: roll, pitch, yaw, coll in -1..1 (already shaped), plus
-- in.rate (rad/s at full stick), in.yawRate, in.response (1/s: how fast the
-- rates follow the commands), in.spoolTime (s), in.level (bool), in.spoolUp
-- (bool: rotor running).
--
-- Runs n steps of dt. The state lives in locals for all of them and goes
-- back into h once at the end: on the radio a Lua call and a table field
-- cost far more than a local, and this used to be one call per step.
function physics.run(h, inp, dt, n)
  if h.crashed or n < 1 then return end

  -- Inputs, the same for every step.
  local spoolTarget = inp.spoolUp and 1 or 0
  local spoolTime = inp.spoolTime
  local level = inp.level
  local roll, pitch, rate = inp.roll, inp.pitch, inp.rate
  local rc = -inp.yaw * inp.yawRate
  local response = inp.response
  local collIn = inp.coll
  local kColl = dt * COLLECTIVE_RESPONSE
  if kColl > 1 then kColl = 1 end
  local kSettle = dt * 20
  if kSettle > 1 then kSettle = 1 end

  -- State.
  local spool, coll, landed = h.spool, h.coll, h.landed
  local r11, r12, r13 = h.r11, h.r12, h.r13
  local r21, r22, r23 = h.r21, h.r22, h.r23
  local r31, r32, r33 = h.r31, h.r32, h.r33
  local p, q, r = h.p, h.q, h.r
  local vx, vy, vz = h.vx, h.vy, h.vz
  local px, py, pz = h.px, h.py, h.pz
  local crashReason

  for _ = 1, n do
    -- Rotor speed: spools up over a few seconds, down when stopped.
    if spool < spoolTarget then
      spool = spool + dt / spoolTime
      if spool > 1 then spool = 1 end
    elseif spool > spoolTarget then
      spool = spool - dt * 0.5
      if spool < 0 then spool = 0 end
    end

    -- Rate commands.
    local pc, qc
    if level then
      -- Target up vector from the stick tilt, in the heading frame.
      local fx, fy = r11, r21                     -- nose, flattened
      local fn = sqrt(fx * fx + fy * fy)
      if fn < 1e-3 then fx, fy, fn = r13, r23, sqrt(r13 * r13 + r23 * r23) + 1e-6 end
      fx, fy = fx / fn, fy / fn
      local rx, ry = fy, -fx                      -- right of the heading
      local tp, tr = pitch * LEVEL_MAX_TILT, roll * LEVEL_MAX_TILT
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
      if pc > rate then pc = rate elseif pc < -rate then pc = -rate end
      if qc > rate then qc = rate elseif qc < -rate then qc = -rate end
    else
      pc = roll * rate
      qc = pitch * rate
    end

    -- Head response scales with rotor speed.
    local kc = dt * response * spool
    if kc > 1 then kc = 1 end
    local ky = dt * response * 1.3 * spool
    if ky > 1 then ky = 1 end
    coll = coll + (collIn - coll) * kColl

    local thrust = coll * THRUST_MAX * spool * spool
    if pz < GROUND_EFFECT_H + 1 and r33 > 0.5 then
      local hh = pz + SKZ
      if hh < GROUND_EFFECT_H then
        local k = 1 - (hh > 0 and hh or 0) / GROUND_EFFECT_H
        thrust = thrust * (1 + GROUND_EFFECT * k * k)
      end
    end

    if landed then
      p, q = 0, 0
      r = r + (0 - r) * kSettle
      -- Lift off when the rotor carries more than the weight.
      if thrust * r33 > G * 1.02 then
        landed = false
        pz = pz + 0.01
      end
    end

    if not landed then
      p = p + (pc - p) * kc
      q = q + (qc - q) * kc
      r = r + (rc - r) * ky

      -- Attitude: R = R * (I + [w]x dt), then re-orthonormalise.
      local wx, wy, wz = p * dt, q * dt, r * dt
      r11, r12, r13 = r11 + r12 * wz - r13 * wy, r12 - r11 * wz + r13 * wx, r13 + r11 * wy - r12 * wx
      r21, r22, r23 = r21 + r22 * wz - r23 * wy, r22 - r21 * wz + r23 * wx, r23 + r21 * wy - r22 * wx
      r31, r32, r33 = r31 + r32 * wz - r33 * wy, r32 - r31 * wz + r33 * wx, r33 + r31 * wy - r32 * wx
      local nn = 1 / sqrt(r11 * r11 + r21 * r21 + r31 * r31)
      r11, r21, r31 = r11 * nn, r21 * nn, r31 * nn
      local d = r11 * r12 + r21 * r22 + r31 * r32
      r12, r22, r32 = r12 - d * r11, r22 - d * r21, r32 - d * r31
      nn = 1 / sqrt(r12 * r12 + r22 * r22 + r32 * r32)
      r12, r22, r32 = r12 * nn, r22 * nn, r32 * nn
      r13 = r21 * r32 - r31 * r22
      r23 = r31 * r12 - r11 * r32
      r33 = r11 * r22 - r21 * r12

      -- Forces: drag per unit mass in body axes, thrust along body up.
      local bx = r11 * vx + r21 * vy + r31 * vz
      local by = r12 * vx + r22 * vy + r32 * vz
      local bz = r13 * vx + r23 * vy + r33 * vz
      local fx = -(DRAG_XY_Q * (bx < 0 and -bx or bx) + DRAG_XY_L) * bx
      local fy = -(DRAG_XY_Q * (by < 0 and -by or by) + DRAG_XY_L) * by
      local fz = thrust - (DRAG_Z_Q * (bz < 0 and -bz or bz) + DRAG_Z_L) * bz
      local ax = r11 * fx + r12 * fy + r13 * fz
      local ay = r21 * fx + r22 * fy + r23 * fz
      local az = r31 * fx + r32 * fy + r33 * fz - G

      vx, vy, vz = vx + ax * dt, vy + ay * dt, vz + az * dt
      px, py, pz = px + vx * dt, py + vy * dt, pz + vz * dt

      -- Ground contact. Only worth checking near the ground.
      if pz <= 1.6 then
        -- The lowest point of the rotor disc, then the tail.
        if pz + r33 * HUBZ - sqrt(r31 * r31 + r32 * r32) * RR < 0.0 then
          crashReason = "Blade strike"
          break
        end
        if pz + r31 * TAILX + r33 * TAILZ < 0 then
          crashReason = "Tail strike"
          break
        end
        -- Skids: lowest of the four ends.
        local zx1, zx2, zy = r31 * SKX1, r31 * SKX2, r32 * SKY
        if zy < 0 then zy = -zy end
        local low = pz + r33 * SKZ
        low = low + (zx2 < zx1 and zx2 or zx1) - zy
        if low < 0 then
          if r33 < LAND_MIN_UPRIGHT then
            crashReason = "Tipped over"
          elseif vz < -LAND_MAX_SINK then
            crashReason = "Hard landing"
          elseif sqrt(vx * vx + vy * vy) > LAND_MAX_SLIDE then
            crashReason = "Skid catch"
          end
          if crashReason then
            if pz < 0.05 then pz = 0.05 end
            break
          end
          -- Touch down: level the attitude, keeping the heading.
          landed = true
          local nx, ny = r11, r21
          local nl = sqrt(nx * nx + ny * ny)
          if nl < 1e-3 then nx, ny, nl = -r12, -r22, 1 end
          nx, ny = nx / nl, ny / nl
          r11, r12, r13 = nx, -ny, 0
          r21, r22, r23 = ny, nx, 0
          r31, r32, r33 = 0, 0, 1
          pz = -SKZ
          vx, vy, vz = 0, 0, 0
          p, q = 0, 0
        end
      end
    end
  end

  h.spool, h.coll, h.landed = spool, coll, landed
  h.r11, h.r12, h.r13 = r11, r12, r13
  h.r21, h.r22, h.r23 = r21, r22, r23
  h.r31, h.r32, h.r33 = r31, r32, r33
  h.p, h.q, h.r = p, q, r
  h.vx, h.vy, h.vz = vx, vy, vz
  h.px, h.py, h.pz = px, py, pz
  if crashReason then
    h.crashed, h.crashReason = true, crashReason
    h.vx, h.vy, h.vz = 0, 0, 0
    h.p, h.q, h.r = 0, 0, 0
  end
end

return physics
