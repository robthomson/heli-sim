-- HeliSim system tool entry point.

local sim = assert(loadfile("sim.lua"))()

local function init()
  local ok, icon = pcall(lcd.loadMask, "gfx/icon.png")
  system.registerSystemTool({
    name = "HeliSim",
    icon = ok and icon or nil,
    create = sim.create,
    wakeup = sim.wakeup,
    event = sim.event,
    paint = sim.paint,
    close = sim.close,
  })
end

return {init = init}
