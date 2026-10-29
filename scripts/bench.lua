-- Bench test for calibrating the motor/prop model, on output 1.
-- Arm and it runs every test once, in order (~3 min), then hands the
-- output back. Disarm to stop at any time; arm again to rerun from the start.
-- Run it once with the prop off and once with the prop on.

local phase = 0                            -- position in the sine wave (radians)
local function sweep(base, amp)            -- makes a 1 -> 50 Hz sweep over 60 s
  return function(t, dt)
    local f = 50 ^ (t / 60)
    phase = phase + 2 * math.pi * f * dt
    return math.floor(base + amp * math.sin(phase)), f
  end
end

-- each test: {id, seconds, function(t, dt) returning throttle (us), sweep freq (Hz)}
local TESTS = {
  {1, 24, function(t) return 1100 + 100 * math.floor(t / 3), 0 end},  -- steps 1100..1800
  {2, 24, function(t) if t % 8 < 4 then return 1500, 0 end return 1000, 0 end}, -- coast-downs
  {3, 60, sweep(1300, 200)},               -- small sweep
  {4, 60, sweep(1500, 400)},               -- big sweep
}
local GAP = 5                              -- seconds at zero throttle before each test

local TOTAL = 0                            -- length of the whole run
for _, test in ipairs(TESTS) do TOTAL = TOTAL + GAP + test[2] end

local t_arm = nil
local t_last = 0

function update()
  if not arming:is_armed() then
    t_arm = nil
    return update, 100
  end

  local now = millis():tofloat() / 1000
  if t_arm == nil then t_arm = now; t_last = now end
  local dt = now - t_last
  t_last = now

  local t = now - t_arm
  if t >= TOTAL then return update, 100 end  -- all done: stop overriding

  -- walk through the tests to find where we are
  local id, pwm, f = 0, 1000, 0              -- default: zero-throttle gap
  for _, test in ipairs(TESTS) do
    if t < GAP then break end
    t = t - GAP
    if t < test[2] then
      if t < dt then phase = 0 end           -- each sweep starts fresh, at the base throttle
      id = test[1]
      pwm, f = test[3](t, dt)
      break
    end
    t = t - test[2]
  end

  SRV_Channels:set_output_pwm_chan_timeout(0, pwm, 50)
  local rpm = esc_telem:get_rpm(0) or -1
  logger:write('BNCH', 'test,pwm,f,rpm', 'BHff', id, pwm, f, rpm)
  return update, 2
end

return update, 1000
