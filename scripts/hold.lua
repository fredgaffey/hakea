-- Hold a steady base throttle on output 1 while the ESC runs its PROFILE
-- programme (the pulsing is generated inside the ESC, not here).
-- Arm and it runs once (~45 s), then hands the output back. Disarm to stop.

local BASE = 1500        -- base throttle (us); the ESC adds its sine on top
local HOLD_S = 45        -- PROFILE programme is ~31 s after the motor settles
local GAP = 3            -- zero throttle first, so the ESC arms cleanly
local SYNC = {1.0, 1.6}  -- two short video-sync blips during the gap

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
    gcs:send_text(6, string.format('hold: start, %d s at %d', GAP + HOLD_S, BASE))
  end
  local dt = now - t_last
  t_last = now

  local t = now - t_arm
  if t >= GAP + HOLD_S then
    if t - dt < GAP + HOLD_S then gcs:send_text(6, 'hold: done, disarm now') end
    return update, 100
  end

  local id, pwm = 0, 1000
  if t >= GAP then
    id, pwm = 20, BASE
  else
    for _, s in ipairs(SYNC) do
      if t >= s and t < s + 0.2 then id, pwm = 10, 1200 end
    end
  end

  SRV_Channels:set_output_pwm_chan_timeout(0, pwm, 50)
  local rpm = esc_telem:get_rpm(0) or -1
  logger:write('BNCH', 'test,pwm,f,rpm', 'BHff', id, pwm, 0, rpm)
  return update, 2
end

return update, 1000
