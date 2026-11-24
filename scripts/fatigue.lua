-- Rotor fatigue run ("fatigue"), PROP ON, output 1, NO cyclic.
-- Arm and it ramps to FTG_PWM and holds there until the rotor breaks, FTG_MINS runs out, or
-- you disarm. It learns the normal rpm and vibration for FTG_BASE seconds, then watches:
--   rpm up   > FTG_DRPM %  -> a blade (or part of one) has gone: less load, motor speeds up
--   rpm down > FTG_DRPM %  -> something is rubbing / a blade has gone to its stop / ESC trouble
--   vibration > FTG_VIBX x normal (and > FTG_VMIN m/s/s) for 0.3 s -> out of balance
--   rpm telemetry missing for 1 s, or rpm > FTG_RMAX -> stop (can't watch it, or overspeed)
-- Any of these: throttle to zero, DISARM, message + tune, and the revolutions so far are
-- reported (that is the fatigue life). The rpm baseline follows slow drift (battery sag) with a
-- 60 s time constant, so only sudden changes trip it.
-- Only ONE bench script (spin1 / fatigue / bench*) may be on the FC: they all drive output 1.
-- Stand out of the rotor plane. Guard round the rotor. Eye protection.

local KEY = 73
assert(param:add_table(KEY, 'FTG_', 10), 'fatigue: param table')
assert(param:add_param(KEY, 1, 'PWM', 1500))   -- hold throttle (us)
assert(param:add_param(KEY, 2, 'MINS', 0))     -- stop after this many minutes (0 = until break)
assert(param:add_param(KEY, 3, 'DRPM', 8))     -- rpm change that trips (%)
assert(param:add_param(KEY, 4, 'VIBX', 3))     -- vibration that trips (x normal)
assert(param:add_param(KEY, 5, 'VMIN', 8))     -- ... and at least this (m/s/s)
assert(param:add_param(KEY, 6, 'RMAX', 21000)) -- overspeed (rpm)
assert(param:add_param(KEY, 7, 'BASE', 10))    -- seconds to learn normal rpm and vibration
assert(param:add_param(KEY, 8, 'RAMP', 6))     -- seconds to ramp from 1100 us to FTG_PWM
assert(param:add_param(KEY, 10, 'TMAX', 90))   -- stop if ESC temperature exceeds this (deg C)
assert(param:add_param(KEY, 9, 'VLOW', 14.0))  -- stop below this battery voltage from ESC telemetry (4S: 3.5 V/cell)

local P = {}
for _, n in ipairs({'PWM', 'MINS', 'DRPM', 'VIBX', 'VMIN', 'RMAX', 'BASE', 'RAMP', 'VLOW', 'TMAX'}) do
  P[n] = Parameter('FTG_' .. n)
end

local SETTLE = 3            -- seconds at FTG_PWM before learning starts
local WIN = 25              -- rpm median window (samples at 50 Hz = 0.5 s)

local state, t_arm, t_last, t_state = 'idle', nil, 0, 0
local revs = 0
local rbuf, rpm_base, vib_base, vib_n, vib_sum, rpm_hist = {}, nil, nil, 0, 0, {}
local t_rpm_ok, t_vib_hi, t_report = 0, nil, 0
local t_vlow = nil
local volt_now, temp_now, cur_now = 0, 0, 0
local mn = {n = 0, rs = 0, rmin = 1e9, rmax = 0, vmin = 1e9, tmax = 0}   -- per-minute stats
local REASON = {none = 0, user = 1, rpm = 2, vib = 3, telem = 4, overspeed = 5, battery = 6, temp = 7, time = 8}
local function event(code, rpm, rbase)
  -- one row per run end: why, when, life
  logger:write('FTGE', 'why,mins,krev,rpm,rbase,volt,temp,rotor', 'Bfffffff', code,
               (millis():tofloat() / 1000 - (t_arm or 0)) / 60, revs / 1000, rpm or 0, rbase or 0,
               volt_now, temp_now, Parameter('RLIF_ID'):get())
end

local function median(t)
  local c = {}
  for i = 1, #t do c[i] = t[i] end
  table.sort(c)
  return c[math.floor((#c + 1) / 2)]
end

local function trip(why, code)
  state = 'tripped'
  event(code or 0, rbuf and #rbuf > 0 and rbuf[#rbuf] or 0, rpm_base)
  SRV_Channels:set_output_pwm_chan_timeout(0, 1000, 500)
  arming:disarm()
  local mins = (millis():tofloat() / 1000 - t_arm) / 60
  gcs:send_text(2, string.format('fatigue: STOP %s after %.1f min, %.1f krev', why, mins, revs / 1000))
  notify:play_tune('MFT100L4CCC')
end

local function vib_now()
  local v = ahrs:get_vibration()
  return math.max(v:x(), v:y(), v:z())
end

function update()
  if not arming:is_armed() then
    if state == 'run' or state == 'learn' then
      gcs:send_text(5, string.format('fatigue: disarmed by hand, %.1f krev', revs / 1000))
      event(REASON.user, 0, rpm_base)
    end
    state, t_arm, rbuf, rpm_base, vib_base = 'idle', nil, {}, nil, nil
    return update, 100
  end
  if state == 'tripped' or state == 'done' then
    SRV_Channels:set_output_pwm_chan_timeout(0, 1000, 500)
    return update, 100
  end

  local now = millis():tofloat() / 1000
  if t_arm == nil then
    t_arm, t_last, t_state, t_rpm_ok, t_report = now, now, now, now, now
    state, revs, vib_n, vib_sum, t_vib_hi, t_vlow = 'ramp', 0, 0, 0, nil, nil
    gcs:send_text(6, string.format('fatigue: start, %d us', math.floor(P.PWM:get())))
  end
  local dt = now - t_last
  t_last = now

  local raw = esc_telem:get_rpm(0)
  if raw then
    t_rpm_ok = now
    rbuf[#rbuf + 1] = raw
    if #rbuf > WIN then table.remove(rbuf, 1) end
    revs = revs + raw / 60 * dt
  end
  local rpm = #rbuf > 0 and median(rbuf) or 0
  local vib = vib_now()
  local target = P.PWM:get()
  local pwm = target

  if state == 'ramp' then
    local f = (now - t_state) / math.max(P.RAMP:get(), 1)
    pwm = 1100 + (target - 1100) * math.min(f, 1)
    if f >= 1 + SETTLE / math.max(P.RAMP:get(), 1) then
      state, t_state = 'learn', now
    end
  elseif state == 'learn' then
    vib_n, vib_sum = vib_n + 1, vib_sum + vib
    rpm_hist[#rpm_hist + 1] = rpm
    if now - t_state >= P.BASE:get() then
      rpm_base, vib_base = median(rpm_hist), vib_sum / vib_n
      rpm_hist = {}
      state, t_state = 'run', now
      gcs:send_text(6, string.format('fatigue: normal %.0f rpm, vibe %.1f; watching', rpm_base, vib_base))
    end
  end

  -- checks that apply all the time
  if now - t_rpm_ok > 1 and now - t_arm > 2 then trip('no rpm telemetry', REASON.telem); return update, 100 end
  if rpm > P.RMAX:get() then trip(string.format('overspeed %.0f rpm', rpm), REASON.overspeed); return update, 100 end
  -- battery: ESC voltage (the FC's own battery sensor is not wired). 2 s low = stop, so a load dip
  -- doesn't trip it. Ends the run as 'done' (battery), not as a rotor failure.
  local ok, volt = pcall(function() return esc_telem:get_voltage(0) end)
  if ok and volt then volt_now = volt end
  local okt, temp = pcall(function() return esc_telem:get_temperature(0) end)
  if okt and temp then temp_now = temp end
  local okc, cur = pcall(function() return esc_telem:get_current(0) end)
  if okc and cur then cur_now = cur end
  if temp_now > P.TMAX:get() then trip(string.format('ESC hot %d C', temp_now), REASON.temp); return update, 100 end
  if ok and volt and volt > 1 then
    if volt < P.VLOW:get() then
      t_vlow = t_vlow or now
      if now - t_vlow > 2 then
        state = 'done'
        SRV_Channels:set_output_pwm_chan_timeout(0, 1000, 500)
        arming:disarm()
        gcs:send_text(4, string.format('fatigue: battery low %.2f V, stopped, rotor OK, %.1f krev', volt, revs / 1000))
        event(REASON.battery, rpm, rpm_base)
        return update, 100
      end
    else
      t_vlow = nil
    end
  end

  if state == 'run' then
    local d = (rpm - rpm_base) / rpm_base * 100
    if math.abs(d) > P.DRPM:get() then
      trip(string.format('rpm %+.0f%% (%.0f vs %.0f): blade lost or rubbing', d, rpm, rpm_base), REASON.rpm)
      return update, 100
    end
    local vlim = math.max(P.VIBX:get() * vib_base, P.VMIN:get())
    if vib > vlim then
      t_vib_hi = t_vib_hi or now
      if now - t_vib_hi > 0.3 then
        trip(string.format('vibration %.1f > %.1f: out of balance', vib, vlim), REASON.vib)
        return update, 100
      end
    else
      t_vib_hi = nil
    end
    -- follow slow drift (battery sag, warm-up); sudden changes still trip
    local a = dt / 60
    rpm_base = rpm_base + a * (rpm - rpm_base)
    vib_base = vib_base + a * (vib - vib_base)
    mn.n, mn.rs = mn.n + 1, mn.rs + rpm
    mn.rmin, mn.rmax = math.min(mn.rmin, rpm), math.max(mn.rmax, rpm)
    if volt_now > 1 then mn.vmin = math.min(mn.vmin, volt_now) end
    mn.tmax = math.max(mn.tmax, temp_now)
    if now - t_report > 60 then
      t_report = now
      local mean = mn.rs / math.max(mn.n, 1)
      gcs:send_text(6, string.format('fatigue: %.1f min %.1f krev | rpm %.0f (%.0f-%.0f) | %.2f V | %d C',
                                     (now - t_arm) / 60, revs / 1000, mean, mn.rmin, mn.rmax, mn.vmin, mn.tmax))
      logger:write('FTGM', 'mins,krev,rmean,rmin,rmax,rbase,vmin,tmax', 'ffffffff', (now - t_arm) / 60,
                   revs / 1000, mean, mn.rmin, mn.rmax, rpm_base, mn.vmin, mn.tmax)
      mn = {n = 0, rs = 0, rmin = 1e9, rmax = 0, vmin = 1e9, tmax = 0}
    end
    if P.MINS:get() > 0 and now - t_arm > P.MINS:get() * 60 then
      state = 'done'
      SRV_Channels:set_output_pwm_chan_timeout(0, 1000, 500)
      arming:disarm()
      gcs:send_text(5, string.format('fatigue: time up, no failure, %.1f krev', revs / 1000))
      event(REASON.time, rpm, rpm_base)
      return update, 100
    end
  end

  SRV_Channels:set_output_pwm_chan_timeout(0, math.floor(pwm), 50)
  logger:write('FTG', 'pwm,rpm,raw,rbase,vib,vbase,krev,volt,temp,cur,st', 'Hfffffffhfb', math.floor(pwm), rpm,
               raw or -1, rpm_base or 0, vib, vib_base or 0, revs / 1000, volt_now, temp_now, cur_now,
               ({ramp = 1, learn = 2, run = 3})[state] or 0)
  return update, 20
end

gcs:send_text(6, 'fatigue loaded: arm to start')
return update, 1000
