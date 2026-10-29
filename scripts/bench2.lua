-- Blind validation test ("bench2"), PROP ON, output 1.
-- The simulator predicted this run before it was flown: notes/sim/blind/.
-- Arm and it runs once (~2 min), then hands the output back. Disarm to stop.

local phase = 0                            -- position in the sine wave (radians)
local function sine(base, amp, f)          -- fixed-frequency sine
  return function(t, dt)
    phase = phase + 2 * math.pi * f * dt
    return math.floor(base + amp * math.sin(phase)), f
  end
end
local function hold(pwm)                   -- constant throttle
  return function() return pwm, 0 end
end

-- each segment: {id, seconds, function(t, dt) returning throttle (us), freq (Hz)}
-- ids: 11 = steps, 12 = coast-down, 13 = sines 1600+/-250, 14 = sine 1400+/-150
local TESTS = {
  {11, 4, hold(1150)}, {11, 4, hold(1450)}, {11, 4, hold(1750)},
  {12, 9, function(t) if t < 4 then return 1650, 0 end return 1000, 0 end},
  {13, 8, sine(1600, 250, 5)},
  {13, 8, sine(1600, 250, 15)},
  {13, 8, sine(1600, 250, 35)},
  {13, 8, sine(1600, 250, 50)},
  {14, 8, sine(1400, 150, 25)},
}
local GAP = 5                              -- seconds at zero throttle before each segment
local SYNC = {1.0, 1.6, 2.2}               -- start times (s after arming) of the video-sync blips

local TOTAL = 0
for _, test in ipairs(TESTS) do TOTAL = TOTAL + GAP + test[2] end

local t_arm = nil
local t_last = 0

function update()
  if not arming:is_armed() then
    t_arm = nil
    return update, 100
  end

  local now = millis():tofloat() / 1000
  if t_arm == nil then
    t_arm = now; t_last = now
    gcs:send_text(6, string.format('bench2: start, %d s', TOTAL))
  end
  local dt = now - t_last
  t_last = now

  local t = now - t_arm
  if t >= TOTAL then
    if t - dt < TOTAL then gcs:send_text(6, 'bench2: done, disarm now') end
    return update, 100
  end

  local id, pwm, f = 0, 1000, 0              -- default: zero-throttle gap
  for _, test in ipairs(TESTS) do
    if t < GAP then break end
    t = t - GAP
    if t < test[2] then
      if t < dt then phase = 0 end           -- each sine starts fresh
      id = test[1]
      pwm, f = test[3](t, dt)
      break
    end
    t = t - test[2]
  end

  -- video sync: three short motor blips early in the first gap (logged as test 10),
  -- well before segment A so they don't affect the measured tests
  local ta = now - t_arm
  if ta < GAP then
    for _, s in ipairs(SYNC) do
      if ta >= s and ta < s + 0.2 then id, pwm = 10, 1200 end
    end
    if ta >= SYNC[1] and ta - dt < SYNC[1] then
      notify:play_tune('MFT240L16C')       -- also beep the FC buzzer, if one is fitted
      gcs:send_text(6, 'bench2: sync blips')
    end
  end

  SRV_Channels:set_output_pwm_chan_timeout(0, pwm, 50)
  local rpm = esc_telem:get_rpm(0) or -1
  logger:write('BNCH', 'test,pwm,f,rpm', 'BHff', id, pwm, f, rpm)
  return update, 2
end

return update, 1000
