-- Sine sweep on output 1: after arming, hold BASE for 3 s,
-- then sweep 1 Hz -> 50 Hz over 60 s, then stop.

local BASE = 1500   -- centre throttle (us)
local AMP = 400     -- swing either side (us)
local SWEEP_S = 60  -- sweep length (s)

local t_arm = nil   -- time we armed (s)
local t_last = 0    -- time of the previous update (s)
local phase = 0     -- position in the sine wave (radians)

function update()
  if not arming:is_armed() then
    t_arm = nil                       -- reset, ready for the next run
    return update, 100
  end

  local now = millis():tofloat() / 1000
  if t_arm == nil then t_arm = now; t_last = now; phase = 0 end
  local dt = now - t_last             -- time since the previous update
  t_last = now
  local t = now - t_arm - 3           -- seconds since the sweep started

  if t > SWEEP_S then
    return update, 100                -- finished: stop overriding the output
  end

  local f = 0
  if t > 0 then
    f = 1 * 50 ^ (t / SWEEP_S)        -- 1 Hz at t=0, 50 Hz at t=SWEEP_S
    phase = phase + 2 * math.pi * f * dt
  end

  local pwm = math.floor(BASE + AMP * math.sin(phase))
  SRV_Channels:set_output_pwm_chan_timeout(0, pwm, 50)

  local rpm = esc_telem:get_rpm(0) or -1
  logger:write('SWP', 'pwm,f,rpm', 'Hff', pwm, f, rpm)

  return update, 2                    -- run again in 2 ms
end

return update, 1000
