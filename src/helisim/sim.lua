-- HeliSim: fly a 3D helicopter with the radio's sticks.
--
-- The pilot stands at the edge of a flying field; the helicopter starts on
-- the pad in front, tail towards the pilot. Lower the throttle to start the
-- rotor, then fly. Enter resets to the pad, Page opens the settings, Exit
-- closes the tool.
--
-- wakeup does all the work (sticks, physics, camera, the frame's display
-- lists); paint only replays the lists and draws the HUD text.

local heli = assert(loadfile("lib/heli.lua"))()
local physics = assert(loadfile("lib/physics.lua"))()
local render = assert(loadfile("lib/render.lua"))()

local floor, abs, sqrt, min, max, rad = math.floor, math.abs, math.sqrt, math.min, math.max, math.rad

local sim = {}

local STEP = 0.02                -- physics step (s)
local MAX_STEPS = 5              -- per wakeup; a longer gap slows time instead
local RANGE_LIMIT = 220          -- metres from the pilot
-- The HUD text sits on opaque bars above (and, with the stats, below) the
-- scene, so it needs no shadow and is only drawn every other frame:
-- between, only the scene is invalidated and Ethos keeps the bars.
local BAR_H, STATS_H = 22, 40
local TOO_CLOSE = 1.3

-- Analog sources: the sticks by function, as Ethos numbers them.
local MEMBER_RUD, MEMBER_ELE, MEMBER_THR, MEMBER_AIL = 0, 1, 2, 3

local CONFIG_FILE = "helisim.cfg"

-- Head response: how fast the rates follow the sticks (1/s), like gyro gain.
local RESPONSES = {8, 14, 24, 40}
local RESPONSE_NAMES = {"Soft", "Normal", "Crisp", "Locked in"}

local DEFAULTS = {
  level = false,      -- self-level (true) or acro
  col3d = true,       -- 3D collective (centre = zero pitch) or normal
  cyclicRate = 450,  -- deg/s at full roll/pitch stick
  yawRate = 600,     -- deg/s at full rudder
  expo = 30,         -- percent
  response = 3,      -- index into RESPONSES
  spoolTime = 3,     -- seconds from stopped to full head speed
  autoZoom = true,
  rotor = "cw",
  detail = 1,         -- 1 low (flat backdrop, simple model), 2 high
  stats = false,
}

---------------------------------------------------------------------------
-- Helpers
---------------------------------------------------------------------------

local function now()
  return os.clock()
end

local function clamp(v, lo, hi)
  if v < lo then return lo elseif v > hi then return hi end
  return v
end

local function scriptDir()
  if not (debug and debug.getinfo) then return "" end
  local ok, info = pcall(debug.getinfo, 1, "S")
  if not ok or not info or type(info.source) ~= "string" then return "" end
  local source = info.source
  if source:sub(1, 1) == "@" then source = source:sub(2) end
  return source:match("^(.*[/\\])") or ""
end

local SCRIPT_DIR = scriptDir()

local function configPaths()
  return {SCRIPT_DIR .. CONFIG_FILE, "/scripts/helisim/" .. CONFIG_FILE, CONFIG_FILE}
end

local function loadConfig()
  local cfg = {}
  for k, v in pairs(DEFAULTS) do cfg[k] = v end
  if not (io and io.open) then return cfg end
  local f
  for _, p in ipairs(configPaths()) do
    f = io.open(p, "r")
    if f then break end
  end
  if not f then return cfg end
  local lines = {}
  while true do
    local ok, line = pcall(f.read, f, "*l")
    if not ok or not line then break end
    lines[#lines + 1] = line
  end
  f:close()
  for _, line in ipairs(lines) do
    local key, value = line:match("^%s*([%w_]+)%s*=%s*(.-)%s*$")
    local d = key and DEFAULTS[key]
    if type(d) == "boolean" then
      cfg[key] = (value == "1" or value == "true")
    elseif type(d) == "number" then
      cfg[key] = tonumber(value) or d
    elseif d ~= nil then
      cfg[key] = value
    end
  end
  cfg.cyclicRate = clamp(floor(cfg.cyclicRate), 50, 1200)
  cfg.yawRate = clamp(floor(cfg.yawRate), 50, 1500)
  cfg.expo = clamp(floor(cfg.expo), 0, 90)
  cfg.spoolTime = clamp(floor(cfg.spoolTime), 1, 20)
  if not RESPONSES[cfg.response] then cfg.response = DEFAULTS.response end
  if cfg.rotor ~= "ccw" then cfg.rotor = "cw" end
  if cfg.detail ~= 2 then cfg.detail = 1 end
  return cfg
end

local function saveConfig(cfg)
  if not (io and io.open) then return end
  local f
  for _, p in ipairs(configPaths()) do
    f = io.open(p, "w")
    if f then break end
  end
  if not f then return end
  for _, key in ipairs({"level", "col3d", "cyclicRate", "yawRate", "expo", "response", "spoolTime", "autoZoom", "rotor", "detail", "stats"}) do
    local v = cfg[key]
    if type(v) == "boolean" then v = v and "1" or "0" end
    f:write(key, "=", tostring(v), "\n")
  end
  f:close()
end

local function analogSource(member)
  if not (system and system.getSource) then return nil end
  local ok, src = pcall(system.getSource, {category = CATEGORY_ANALOG, member = member})
  if ok then return src end
  return nil
end

-- A stick in -1..1 (sources give -1024..1024).
local function stick(src)
  if not src then return 0 end
  local v = src:value()
  if type(v) ~= "number" then return 0 end
  v = v / 1024
  if v > 1 then return 1 elseif v < -1 then return -1 end
  return v
end

local function shape(v, expo)
  if v > -0.02 and v < 0.02 then return 0 end
  return v * (1 - expo) + v * v * v * expo
end

local function keyIs(value, ...)
  for i = 1, select("#", ...) do
    local k = select(i, ...)
    if k ~= nil and value == k then return true end
  end
  return false
end

---------------------------------------------------------------------------
-- State
---------------------------------------------------------------------------

local function resetHeli(state)
  physics.reset(state.h, render.PAD_X, render.PAD_Y, 0)
  state.phase = "arm"
  state.flightTime = 0
  state.message = nil
  render.snap(state.h, state.cfg.autoZoom, state.manualZoom)
  render.dirty()
end

local function setFont(f)
  if f then lcd.font(f) end
end

-- HUD strings, rebuilt only when what they show changes. Returns whether
-- the bar texts changed and whether the centre message (drawn over the
-- scene) changed.
local function updateHud(state)
  local h = state.h
  local changed = false
  local alt = floor(max(0, h.pz - 0.23) * 10 + 0.5)
  local spd = floor(sqrt(h.vx * h.vx + h.vy * h.vy + h.vz * h.vz) * 10 + 0.5)
  local col = floor(h.coll * 100 + 0.5)
  if alt ~= state.hudAlt or spd ~= state.hudSpd or col ~= state.hudCol then
    state.hudAlt, state.hudSpd, state.hudCol = alt, spd, col
    state.hudLeft = string.format("ALT %.1fm  SPD %.1fm/s  COL %+d%%", alt / 10, spd / 10, col)
    changed = true
  end
  local secs = floor(state.flightTime)
  if secs ~= state.hudSecs then
    state.hudSecs = secs
    state.hudTime = string.format("%d:%02d", floor(secs / 60), secs % 60)
    changed = true
  end
  local level, col3d = state.cfg.level, state.cfg.col3d
  if level ~= state.hudLevel or col3d ~= state.hudCol3d then
    state.hudLevel, state.hudCol3d = level, col3d
    state.hudMode = (level and "Level" or "Acro") .. (col3d and " 3D" or " Normal")
    changed = true
  end

  -- The centre message: a key for what it says, so it is only formatted
  -- when that changes.
  local key
  if state.phase == "arm" then
    key = -1
  elseif state.phase == "crashed" then
    key = -2
  elseif h.spool < 1 then
    key = floor(h.spool * 100)
  else
    key = -3
  end
  if key ~= state.hudMsgKey then
    state.hudMsgKey = key
    if key == -1 then
      state.message, state.subMessage = "Throttle low to start", "Enter: reset   Page: settings"
    elseif key == -2 then
      state.message, state.subMessage = "CRASH: " .. (h.crashReason or ""), "Enter: back to the pad"
    elseif key >= 0 then
      state.message, state.subMessage = string.format("Spooling up %d%%", key), nil
    else
      state.message, state.subMessage = nil, nil
    end
    return changed, true
  end
  return changed, false
end

---------------------------------------------------------------------------
-- Settings
---------------------------------------------------------------------------

local function closeSettings(state)
  state.settingsOpen = false
  pcall(form.clear)
  state.lastTime = nil
  state.paintScene, state.paintBar = true, true
  lcd.invalidate()
end

local function openSettings(state)
  if not (form and form.clear and form.addLine and form.addChoiceField) then return end
  form.clear()
  state.settingsOpen = true
  local cfg = state.cfg
  local function set(key, v)
    cfg[key] = v
    saveConfig(cfg)
  end

  form.addChoiceField(form.addLine("Flight mode"), nil, {{"Acro", 0}, {"Self-level", 1}},
    function() return cfg.level and 1 or 0 end, function(v) set("level", v == 1) end)
  form.addChoiceField(form.addLine("Collective"), nil, {{"3D (centre = 0 pitch)", 1}, {"Normal", 0}},
    function() return cfg.col3d and 1 or 0 end, function(v) set("col3d", v == 1) end)
  local function number(label, lo, hi, step, suffix, key)
    local f = form.addNumberField(form.addLine(label), nil, lo, hi,
      function() return cfg[key] end, function(v) set(key, v) end)
    if f then
      if f.step then f:step(step) end
      if f.suffix then f:suffix(suffix) end
      if f.default then f:default(DEFAULTS[key]) end
    end
  end
  number("Cyclic rate", 50, 1200, 10, " deg/s", "cyclicRate")
  number("Yaw rate", 50, 1500, 10, " deg/s", "yawRate")
  number("Expo", 0, 90, 5, "%", "expo")
  local responseChoices = {}
  for i = 1, #RESPONSES do responseChoices[i] = {RESPONSE_NAMES[i], i} end
  form.addChoiceField(form.addLine("Head response"), nil, responseChoices,
    function() return cfg.response end, function(v) set("response", v) end)
  number("Spool-up time", 1, 20, 1, " s", "spoolTime")
  form.addChoiceField(form.addLine("Rotor direction"), nil, {{"Clockwise", 0}, {"Counter-clockwise", 1}},
    function() return cfg.rotor == "ccw" and 1 or 0 end,
    function(v)
      set("rotor", v == 1 and "ccw" or "cw")
      heli.build(cfg.rotor, cfg.detail)
      state.phase = "boot"
    end)
  -- Low detail is for the radio, where the screen fills set the frame rate.
  -- The rotor disc has fewer segments at low detail, so the mesh is rebuilt.
  form.addChoiceField(form.addLine("Detail"), nil, {{"Low", 1}, {"High", 2}},
    function() return cfg.detail end,
    function(v)
      set("detail", v)
      render.setDetail(v)
      heli.build(cfg.rotor, v)
      state.phase = "boot"
    end)
  if form.addBooleanField then
    form.addBooleanField(form.addLine("Auto zoom"), nil,
      function() return cfg.autoZoom end, function(v) set("autoZoom", v) end)
    form.addBooleanField(form.addLine("Show CPU stats"), nil,
      function() return cfg.stats end, function(v) set("stats", v) end)
  end
  local back = function() closeSettings(state) end
  if form.addButton then
    form.addButton(form.addLine(""), nil, {text = "Back to flying", press = back})
  elseif form.addTextButton then
    form.addTextButton(form.addLine(""), nil, "Back to flying", back)
  end
end

---------------------------------------------------------------------------
-- Callbacks
---------------------------------------------------------------------------

function sim.create()
  local state = {
    cfg = loadConfig(),
    h = {},
    inp = {roll = 0, pitch = 0, yaw = 0, coll = 0, rate = 0, yawRate = 0, response = 14, spoolTime = 3, level = false, spoolUp = false},
    phase = "boot",
    manualZoom = 1,
    acc = 0,
    flightTime = 0,
    nextAwake = 0,
    -- Stats: counters for the current second, and the last second's values.
    wakes = 0, paints = 0, tickSum = 0, paintSum = 0, rateAt = 0,
    rate = 0, fps = 0, tickMs = 0, paintMs = 0,
    -- What the next paint draws: the scene, the bars; barDirty: the bars
    -- changed since they were last drawn.
    paintScene = true, paintBar = true, barDirty = true, frameNo = 0,
    hudLeft = "", hudTime = "0:00", hudMode = "",
    srcRud = analogSource(MEMBER_RUD),
    srcEle = analogSource(MEMBER_ELE),
    srcThr = analogSource(MEMBER_THR),
    srcAil = analogSource(MEMBER_AIL),
  }
  heli.build(state.cfg.rotor, state.cfg.detail)
  physics.setGeometry(heli)
  render.init(heli)
  render.setDetail(state.cfg.detail)
  physics.reset(state.h, render.PAD_X, render.PAD_Y, 0)
  return state
end

local function keepAwake(state, t)
  if t < state.nextAwake then return end
  state.nextAwake = t + 1
  if system.resetBacklightTimeout then pcall(system.resetBacklightTimeout) end
  if system.resetFocusTimeout then pcall(system.resetFocusTimeout) end
end

local function usage()
  return system.getInstructionsUsage and system.getInstructionsUsage() or 0
end

function sim.wakeup(state)
  if state.settingsOpen then return end
  local t = now()
  keepAwake(state, t)

  if state.phase == "boot" then
    if heli.buildStep() then resetHeli(state) end
    lcd.invalidate()
    return
  end

  local cfg = state.cfg
  if cfg.stats then
    state.wakes = state.wakes + 1
    if t >= state.rateAt then
      -- Once a second: wakeups and paints per second, mean wakeup and
      -- paint time (ms).
      local wakes = state.wakes
      state.rate, state.fps = wakes, state.paints
      state.tickMs = state.tickSum * 1000 / wakes
      state.paintMs = state.paints > 0 and state.paintSum * 1000 / state.paints or 0
      state.wakes, state.paints, state.tickSum, state.paintSum = 0, 0, 0, 0
      state.rateAt = t + 1
    end
  end

  local dt = state.lastTime and (t - state.lastTime) or 0
  state.lastTime = t
  if dt < 0 then dt = 0 elseif dt > MAX_STEPS * STEP then dt = MAX_STEPS * STEP end

  local h, inp = state.h, state.inp
  local expo = cfg.expo / 100
  local thr = stick(state.srcThr)
  state.thr = thr
  inp.roll = shape(stick(state.srcAil), expo)
  inp.pitch = shape(stick(state.srcEle), expo)
  inp.yaw = shape(stick(state.srcRud), expo)
  if cfg.col3d then
    inp.coll = thr
  else
    inp.coll = -0.25 + (thr + 1) * 0.625
  end
  inp.rate = rad(cfg.cyclicRate)
  inp.yawRate = rad(cfg.yawRate)
  inp.response = RESPONSES[cfg.response]
  inp.spoolTime = cfg.spoolTime
  inp.level = cfg.level

  if state.phase == "arm" then
    inp.spoolUp = false
    if thr < -0.8 then
      state.phase = "fly"
    end
  elseif state.phase == "fly" then
    inp.spoolUp = true
  end

  if state.phase ~= "crashed" then
    state.acc = state.acc + dt
    local n = 0
    while state.acc >= STEP and n < MAX_STEPS do
      physics.step(h, inp, STEP)
      state.acc = state.acc - STEP
      n = n + 1
    end
    if not h.landed then state.flightTime = state.flightTime + dt end
    local d2 = h.px * h.px + h.py * h.py
    if d2 > RANGE_LIMIT * RANGE_LIMIT then
      h.crashed, h.crashReason = true, "Out of range"
    elseif d2 + (h.pz - 1.7) * (h.pz - 1.7) < TOO_CLOSE * TOO_CLOSE then
      h.crashed, h.crashReason = true, "Too close to the pilot!"
    end
    if h.crashed then
      state.phase = "crashed"
      if system.playTone then pcall(system.playTone, 300, 400, 0) end
    end
  end

  local w, wh = lcd.getWindowSize()
  local sceneH = wh - BAR_H - (cfg.stats and STATS_H or 0)
  render.track(h, cfg.autoZoom, state.manualZoom, dt)
  local discTilt = rad(8)
  local drew = render.frame(h, BAR_H, w, sceneH, inp.roll * discTilt * h.spool, inp.pitch * discTilt * h.spool)
  local barChanged, msgChanged = updateHud(state)
  if barChanged or cfg.stats then state.barDirty = true end

  -- Repaint only what changed: the scene, and the bars every other frame
  -- (every frame when the scene is still). Both bars with a still scene
  -- (the stats on the pad) repaint the whole window.
  local scene = drew or msgChanged
  local bar = false
  if state.barDirty then
    local n = state.frameNo + 1
    state.frameNo = n
    bar = not scene or n % 2 == 0
  end
  if bar then
    state.barDirty = false
    if cfg.stats then scene = true end
  end
  if cfg.stats then state.tickSum = state.tickSum + (now() - t) end
  if not (scene or bar) then return end
  state.paintScene, state.paintBar = scene, bar
  -- Invalidate rectangles only: a plain invalidate() makes Ethos clear the
  -- window before the paint (measured on an X20: 1 fps at High detail, 2-3
  -- at Low), and the scene repaints every pixel of its rectangle anyway.
  if scene and bar then
    lcd.invalidate(0, 0, w, wh)
  elseif scene then
    lcd.invalidate(0, BAR_H, w, sceneH)
  else
    lcd.invalidate(0, 0, w, BAR_H)
  end
end

local function shadowText(x, y, text, flags)
  lcd.color(BLACK)
  lcd.drawText(x + 1, y + 1, text, flags)
  lcd.color(WHITE)
  lcd.drawText(x, y, text, flags)
end

function sim.paint(state)
  if state.settingsOpen then return end
  local w, h = lcd.getWindowSize()
  if state.phase == "boot" then
    lcd.color(lcd.RGB(30, 40, 60))
    lcd.drawFilledRectangle(0, 0, w, h)
    setFont(FONT_L)
    shadowText(w // 2, h // 2 - 12, "HeliSim: loading", CENTERED)
    return
  end
  local stats = state.cfg.stats
  local t0 = stats and now()
  -- A paint Ethos asked for by itself draws everything.
  local scene, bar = state.paintScene, state.paintBar
  if not (scene or bar) then scene, bar = true, true end
  state.paintScene, state.paintBar = false, false

  if scene then
    render.paint(w, h)
    if state.message then
      local y = BAR_H + floor(h * 0.14)
      setFont(FONT_L)
      shadowText(w // 2, y, state.message, CENTERED)
      if state.subMessage then
        setFont(FONT_S)
        shadowText(w // 2, y + 30, state.subMessage, CENTERED)
      end
    end
  end

  if bar then
    lcd.color(BLACK)
    lcd.drawFilledRectangle(0, 0, w, BAR_H)
    setFont(FONT_S)
    lcd.color(WHITE)
    lcd.drawText(6, 2, state.hudLeft)
    lcd.drawText(w // 2, 2, state.hudMode, CENTERED)
    lcd.drawText(w - 6, 2, state.hudTime, RIGHT)
    if stats then
      -- Per second: paints (fps), wakeups, mean wakeup and paint time; the
      -- paint's instruction budget (Ethos 26.1.3+, last frame).
      local y = h - STATS_H
      lcd.color(BLACK)
      lcd.drawFilledRectangle(0, y, w, STATS_H)
      lcd.color(WHITE)
      local tris, lines, faces = render.stats()
      local inp = state.inp
      lcd.drawText(6, y + 1, string.format("fps %d  wake %d/s  tick %.1f ms  paint %.1f ms  budget %d%%",
        state.fps, state.rate, state.tickMs, state.paintMs, state.cpuPaint or 0))
      lcd.drawText(6, y + 20, string.format("tri %d  faces %d  mem %dk  R%+d E%+d T%+d A%+d",
        tris, faces, floor(collectgarbage("count")),
        floor(inp.yaw * 100), floor(inp.pitch * 100), floor((state.thr or 0) * 100), floor(inp.roll * 100)))
    end
  end

  if stats then
    if system.getInstructionsUsage then state.cpuPaint = usage() end
    state.paints = state.paints + 1
    state.paintSum = state.paintSum + (now() - t0)
  end
end

-- Page keys, with the values Ethos sends as a fallback.
local PAGE_KEYS = {KEY_PAGE_BREAK or 32, KEY_PAGE_LONG or 128}

function sim.event(state, category, value)
  if state.settingsOpen then
    if category == EVT_CLOSE or (category == EVT_KEY and keyIs(value, KEY_EXIT_BREAK, KEY_EXIT_FIRST)) then
      closeSettings(state)
      -- Exit arrives as FIRST then BREAK: the BREAK would close the tool.
      state.swallowExit = (value == KEY_EXIT_FIRST)
      return true
    end
    return false
  end
  if category ~= EVT_KEY then return false end

  if state.swallowExit and keyIs(value, KEY_EXIT_BREAK) then
    state.swallowExit = false
    return true
  end
  state.swallowExit = false

  -- Enter acts on the press: once a settings form has been open, Ethos no
  -- longer passes the Enter release (KEY_ENTER_BREAK) to the tool.
  if keyIs(value, KEY_ENTER_FIRST) then
    if state.phase ~= "boot" then resetHeli(state) end
    return true
  end
  if keyIs(value, KEY_ENTER_BREAK, KEY_ENTER_LONG) then
    return true
  end
  if keyIs(value, PAGE_KEYS[1], PAGE_KEYS[2]) then
    openSettings(state)
    return true
  end
  if keyIs(value, KEY_PAGE_FIRST or 96) then
    return true
  end
  if keyIs(value, KEY_ROTARY_RIGHT) then
    state.manualZoom = min(3, state.manualZoom * 1.1)
    return true
  end
  if keyIs(value, KEY_ROTARY_LEFT) then
    state.manualZoom = max(0.5, state.manualZoom / 1.1)
    return true
  end
  return false
end

function sim.close(state)
  if state and state.settingsOpen then pcall(form.clear) end
  collectgarbage("collect")
end

return sim
