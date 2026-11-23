-- First spin of a new printed rotor ("spin1"), PROP ON, output 1, NO cyclic.
-- Arm and it ramps throttle in small steps, holding each one, until the ESC reports
-- RPM_CAP (or PWM_CAP is reached), holds there, then steps back down. Disarm to stop at
-- any time; arm again to rerun from the start.
--
-- Why capped: a blade's centrifugal pull grows with rpm squared. RPM_CAP 9000 is ~50 % of
-- the pull at hover (~12 750 rpm), so the first run loads the printed webs to half of what
-- flight will. Raise RPM_CAP only after a clean run (no whitening at the webs, no change
-- of blade pitch at rest, no growing vibration).
-- Stand out of the rotor plane. Guard round the rotor. Eye protection.

local RPM_CAP = 9000        -- stop climbing once the ESC reports this (rpm)
local PWM_CAP = 1700        -- never command more than this (us), even with no rpm telemetry
local PWM_START = 1100
local PWM_STEP = 50
local HOLD = 4              -- seconds per step on the way up
local TOP_HOLD = 15         -- seconds at the top step
local DOWN_HOLD = 2         -- seconds per step on the way down
local ARM_WAIT = 3          -- seconds at zero throttle after arming

local t_arm, t_last, t_step = nil, 0, 0
local pwm, top, state = 1000, nil, 'wait'

local function out(p, rpm)
  SRV_Channels:set_output_pwm_chan_timeout(0, p, 50)
  logger:write('SPN1', 'pwm,rpm,st', 'Hfb', p, rpm, ({wait = 0, up = 1, top = 2, down = 3, done = 4})[state])
end

function update()
  if not arming:is_armed() then
    t_arm, state, pwm, top = nil, 'wait', 1000, nil
    return update, 100
  end
  local now = millis():tofloat() / 1000
  if t_arm == nil then
    t_arm, t_step = now, now
    gcs:send_text(6, string.format('spin1: start, cap %d rpm / %d us', RPM_CAP, PWM_CAP))
  end
  local rpm = esc_telem:get_rpm(0) or -1
  local held = now - t_step

  if state == 'wait' and held >= ARM_WAIT then
    state, pwm, t_step = 'up', PWM_START, now
  elseif state == 'up' and held >= HOLD then
    if rpm >= RPM_CAP or pwm + PWM_STEP > PWM_CAP then
      state, top, t_step = 'top', pwm, now
      gcs:send_text(6, string.format('spin1: top %d us, %.0f rpm', pwm, rpm))
    else
      pwm, t_step = pwm + PWM_STEP, now
      gcs:send_text(6, string.format('spin1: %d us, %.0f rpm', pwm, rpm))
    end
  elseif state == 'top' and held >= TOP_HOLD then
    state, t_step = 'down', now
  elseif state == 'down' and held >= DOWN_HOLD then
    pwm, t_step = pwm - 2 * PWM_STEP, now
    if pwm < PWM_START then
      state, pwm = 'done', 1000
      gcs:send_text(6, 'spin1: done, disarm now')
    end
  end
  -- overspeed guard: the rpm cap is a ceiling, not a target
  if rpm > RPM_CAP * 1.1 and state ~= 'done' then
    state, pwm = 'done', 1000
    gcs:send_text(3, string.format('spin1: OVERSPEED %.0f rpm, throttle cut', rpm))
  end
  out(state == 'wait' and 1000 or pwm, rpm)
  return update, 10
end

gcs:send_text(6, 'spin1 loaded: arm to start')
return update, 1000
