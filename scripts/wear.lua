-- Rotor wear test ("wear"), PROP ON, output 1. Closed loop on RPM, so the supply voltage
-- does not matter. Optional: the TEMPORARY ESC build FOXEER_F421_WEAR adds the 1P pulse.
--
-- Arm: waits for ESC telemetry (it drops ~3 s on every arm), then flies a repeating ~40 s
-- "flight" as rpm targets around WRR_RHOV (hover rpm), until the rotor breaks, WRR_CYC
-- cycles are done, or you disarm:
--   spin-up, hover, slow gusts (2 Hz), fast gusts (6 Hz), manoeuvre buzz (12 Hz),
--   punch-outs, hover, descent, bounce, settle.
-- WRR_SCL scales every excursion from hover. Fast segments are tracked as well as the
-- motor can; that is fine, it is the load history that matters.
--
-- Break detection: with rpm held, a lost blade (or chunk) shows up as a sudden DROP in
-- the throttle needed (less drag); rubbing as a RISE. Cycle 1 warms up, cycle 2 learns
-- the throttle at every 0.2 s slot of the flight, then every slot is compared with it.
-- The median of the last 5 slots (1 s) beyond WRR_DTHR % of the learnt throttle above idle
-- trips. Learnt values follow slow drift by 10 % per cycle.
-- Duty cycle: WRR_ON minutes flying (12 = one flight), then WRR_OFF minutes resting with the
-- motor stopped (still armed), repeat. It also rests early when the ESC reaches WRR_TMAX - 15 C,
-- and only flies again once the ESC is back under WRR_TMAX - 40 C (and WRR_OFF has passed).
-- The first cycle after a rest is not checked (motor and blades warming up).
-- Also stops on: no rpm telemetry 1 s, overspeed, cannot hold rpm (2 s below half the
-- target), battery low (ESC voltage), ESC hot.
-- On any stop: throttle zero, DISARM, message + tune, cycles and revolutions reported.
-- Only ONE bench script (wear / fatigue / spin1 / bench*) may be on the FC.
-- Guard round the rotor, stand out of its plane, eye protection.

local KEY = 75
assert(param:add_table(KEY, 'WRR_', 16), 'wear: param table')
assert(param:add_param(KEY, 1, 'RHOV', 12750))  -- hover rpm
assert(param:add_param(KEY, 2, 'SCL', 1.0))     -- scale on every excursion from hover
assert(param:add_param(KEY, 3, 'CYC', 0))       -- stop after this many cycles (0 = until break)
assert(param:add_param(KEY, 4, 'DTHR', 15))     -- throttle change vs learnt that trips (%)
assert(param:add_param(KEY, 5, 'RMAX', 21000))  -- overspeed (rpm)
assert(param:add_param(KEY, 6, 'VLOW', 21.0))   -- battery low from ESC voltage (V, 2 s); 6S 3.5 V/cell
assert(param:add_param(KEY, 7, 'TMAX', 90))     -- ESC too hot (deg C)
assert(param:add_param(KEY, 8, 'PMAX', 1900))   -- never command more than this (us)
assert(param:add_param(KEY, 9, 'KP', 0.006))    -- rpm loop: us per rpm of error
assert(param:add_param(KEY, 10, 'KI', 0.020))   -- rpm loop: us per rpm per second
assert(param:add_param(KEY, 11, 'ON', 12))      -- fly this many minutes (one flight) ...
assert(param:add_param(KEY, 12, 'OFF', 10))     -- ... then rest this many minutes, motor stopped
assert(param:add_param(KEY, 13, 'SLEW', 20000)) -- rpm target never changes faster than this (rpm/s)
assert(param:add_param(KEY, 14, 'TRK', 30))     -- stop if rpm is off target by more than this % for 1 s
assert(param:add_param(KEY, 15, 'LOCK', 0))     -- set to 1 by a rotor-type stop: no run until you inspect and set 0

local P = {}
for _, n in ipairs({'RHOV', 'SCL', 'CYC', 'DTHR', 'RMAX', 'VLOW', 'TMAX', 'PMAX', 'KP', 'KI', 'ON', 'OFF', 'SLEW', 'TRK', 'LOCK'}) do
  P[n] = Parameter('WRR_' .. n)
end

-- ---------------------------------------------------------------------------------
-- the flight: {name, seconds, function(t) -> rpm as a fraction of hover, before WRR_SCL}
local function hold(f) return function() return f end end
local function ramp(a, b, T) return function(t) return a + (b - a) * t / T end end
local function sine(amp, f) return function(t) return 1 + amp * math.sin(2 * math.pi * f * t) end end
local EDGE = 0.08                                    -- s, raised-cosine edge of every step
local function square(hi, lo, t_hi, t_lo)
  return function(t)
    local p = t % (t_hi + t_lo)
    local function s(x) x = math.max(0, math.min(1, x / EDGE)); return 0.5 - 0.5 * math.cos(math.pi * x) end
    if p < t_hi then return lo + (hi - lo) * s(p) end
    return hi + (lo - hi) * s(p - t_hi)
  end
end
local PLAN = {
  {'spinup',  3.0, ramp(0.4, 1.0, 3.0)},
  {'hover',   6.0, hold(1.0)},
  {'gust2',   4.0, sine(0.10, 2)},
  {'gust6',   4.0, sine(0.10, 6)},
  {'buzz12',  3.0, sine(0.06, 12)},
  {'punch',   4.8, square(1.30, 0.90, 0.4, 0.8)},   -- 4 punch-outs to 130 % hover rpm
  {'hover',   4.0, hold(1.0)},
  {'descend', 3.0, hold(0.80)},
  {'bounce',  4.0, square(1.05, 0.80, 0.5, 0.5)},   -- flare / bounce, 1 Hz
  {'settle',  3.0, ramp(0.80, 1.0, 3.0)},
}
local CYCLE = 0
for _, s in ipairs(PLAN) do CYCLE = CYCLE + s[2] end
local SLOT = 0.2                                     -- template slot (s)
local NSLOT = math.floor(CYCLE / SLOT + 0.5)

-- ---------------------------------------------------------------------------------
local REASON = {user = 1, thr = 2, telem = 4, overspeed = 5, battery = 6, temp = 7, cycles = 8, stall = 9, track = 10}
local ROTOR = {[REASON.thr] = true, [REASON.stall] = true, [REASON.track] = true}   -- stops that latch WRR_LOCK
local state = 'idle'                                 -- idle / wait / run / stopped
local t_wait, t_seen = nil, nil
local t_arm, t_last, t_rpm_ok, t_vlow, t_stall = nil, 0, 0, nil, nil
local revs, cyc = 0, 0
local tmpl = {}                                      -- learnt throttle above idle per slot
local slot_i, slot_buf = -1, {}
local ratios = {}
local ratio_now = 1
local volt, temp, cur = 0, 0, 0
local cs = nil
local g = 0.02                                       -- feed-forward: us above 1000 per rpm (starts low: undershoot, the loop pulls it up)
local integ = 0                                      -- PI integrator (us)
local rpm_f = 0                                      -- filtered rpm for control
local tgt_s = nil                                    -- slew-limited rpm target
local t_trk = nil
local phase, t_phase, t_fly = 'fly', 0, 0            -- fly / rest; flight-time clock
local check_from = 2                                 -- first cycle whose slots are checked
local flights = 0

local function median(t)
  local c = {}
  for i = 1, #t do c[i] = t[i] end
  table.sort(c)
  return c[math.floor((#c + 1) / 2)]
end

local function new_stats() return {rmin = 9, rmax = 0, rsum = 0, n = 0, vmin = 99, tmax = -99} end

local function event(code, rpm)
  logger:write('WRE', 'why,mins,cyc,krev,rpm,ratio,volt,temp,rotor', 'Bffffffff', code,
               t_arm and (millis():tofloat() / 1000 - t_arm) / 60 or 0, cyc, revs / 1000,
               rpm or 0, ratio_now, volt, temp, Parameter('RLIF_ID'):get())
end

local function stop(level, why, code, rpm)
  state = 'stopped'
  SRV_Channels:set_output_pwm_chan_timeout(0, 1000, 500)
  arming:disarm()
  event(code, rpm)
  if ROTOR[code] then
    P.LOCK:set_and_save(1)
    gcs:send_text(2, 'wear: LOCKED - inspect the rotor, then set WRR_LOCK 0 to run again')
  end
  gcs:send_text(level, string.format('wear: STOP %s | cycle %d, %.1f min, %.1f krev', why, cyc,
                                     (millis():tofloat() / 1000 - (t_arm or 0)) / 60, revs / 1000))
  notify:play_tune(code == REASON.thr and 'MFT100L4CCC' or 'MFT200L8C')
end

local function plan_at(tc)
  for _, s in ipairs(PLAN) do
    if tc < s[2] then return s[3](tc), s[1] end
    tc = tc - s[2]
  end
  return 1, 'hover'
end

local function seg_index(name)
  for i, s in ipairs(PLAN) do if s[1] == name then return i end end
  return 0
end

function update()
  if not arming:is_armed() then
    if state == 'run' then
      event(REASON.user, 0)
      gcs:send_text(5, string.format('wear: disarmed by hand | cycle %d, %.1f krev', cyc, revs / 1000))
    end
    state, t_arm, t_wait = 'idle', nil, nil
    return update, 100
  end
  if state == 'stopped' then
    SRV_Channels:set_output_pwm_chan_timeout(0, 1000, 500)
    return update, 100
  end
  if P.LOCK:get() > 0 then
    SRV_Channels:set_output_pwm_chan_timeout(0, 1000, 500)
    arming:disarm()
    gcs:send_text(2, 'wear: LOCKED after a rotor stop - inspect the rotor, set WRR_LOCK 0, arm again')
    state = 'idle'
    return update, 500
  end

  local now = millis():tofloat() / 1000
  -- after arming the ESC telemetry goes quiet for ~3 s: hold zero throttle until rpm has
  -- been coming in for 0.5 s (max 8 s), then start the flight
  if t_arm == nil then
    if state ~= 'wait' then
      state, t_wait, t_seen = 'wait', now, nil
      gcs:send_text(6, 'wear: armed, waiting for ESC telemetry')
    end
    -- get_rpm keeps returning the last value for up to 1 s after the data stops, so ignore
    -- the first 1.2 s: anything valid after that is fresh
    if now - t_wait > 1.2 and esc_telem:get_rpm(0) then t_seen = t_seen or now else t_seen = nil end
    if t_seen == nil or now - t_seen < 0.5 then
      if now - t_wait > 8 then
        state, t_arm = 'stopped', now
        SRV_Channels:set_output_pwm_chan_timeout(0, 1000, 500)
        arming:disarm()
        gcs:send_text(2, 'wear: STOP no ESC telemetry 8 s after arming')
        return update, 100
      end
      SRV_Channels:set_output_pwm_chan_timeout(0, 1000, 50)
      return update, 10
    end
    t_arm, t_last, t_rpm_ok, t_vlow, t_stall = now, now, now, nil, nil
    state, revs, cyc, tmpl, slot_i, slot_buf, ratios, ratio_now = 'run', 0, 0, {}, -1, {}, {}, 1
    integ, rpm_f, tgt_s, t_trk = 0, 0, nil, nil
    phase, t_phase, t_fly, check_from, flights = 'fly', now, 0, 2, 1
    cs = new_stats()
    gcs:send_text(6, string.format('wear: start, hover %d rpm, %.0f s cycle, learning on cycle 2',
                                   math.floor(P.RHOV:get()), CYCLE))
  end
  local dt = math.max(now - t_last, 0.001)
  t_last = now

  -- telemetry
  local raw = esc_telem:get_rpm(0)
  if raw then t_rpm_ok = now; revs = revs + raw / 60 * dt end
  local ok, v = pcall(function() return esc_telem:get_voltage(0) end)
  if ok and v then volt = v end
  local okt, tc_ = pcall(function() return esc_telem:get_temperature(0) end)
  if okt and tc_ then temp = math.floor(tc_ / 100) end   -- centidegrees -> deg C
  local okc, c = pcall(function() return esc_telem:get_current(0) end)
  if okc and c then cur = c end

  -- duty cycle: rest with the motor stopped, then fly again from the start of a cycle
  local thot = P.TMAX:get() - 15
  if phase == 'rest' then
    SRV_Channels:set_output_pwm_chan_timeout(0, 1000, 50)
    integ, rpm_f, t_stall, tgt_s, t_trk = 0, 0, nil, nil, nil
    if now - t_rpm_ok > 3 then stop(2, 'no rpm telemetry while resting', REASON.telem, 0); return update, 100 end
    if temp > P.TMAX:get() then stop(2, string.format('ESC hot %d C', temp), REASON.temp, raw); return update, 100 end
    if now - t_phase >= P.OFF:get() * 60 and temp <= P.TMAX:get() - 40 then
      phase, t_phase, flights = 'fly', now, flights + 1
      t_fly = math.ceil(t_fly / CYCLE) * CYCLE           -- restart at a cycle boundary (spin-up)
      check_from = math.floor(t_fly / CYCLE + 0.5) + 1   -- skip the warm-up cycle
      slot_i, slot_buf, ratios, ratio_now = -1, {}, {}, 1
      gcs:send_text(6, string.format('wear: flight %d start | ESC %d C | %.1f krev so far', flights, temp, revs / 1000))
    end
    logger:write('WEAR', 'pwm,rpm,tgt,ff,integ,ratio,krev,volt,temp,cyc,seg', 'Hffffffffhb', 1000,
                 raw or -1, 0, 0, 0, ratio_now, revs / 1000, volt, temp, cyc, 0)
    return update, 50
  end
  t_fly = t_fly + dt
  if now - t_phase >= P.ON:get() * 60 or temp >= thot then
    phase, t_phase = 'rest', now
    SRV_Channels:set_output_pwm_chan_timeout(0, 1000, 50)
    gcs:send_text(6, string.format('wear: flight %d done (%s) | resting %.0f min | %.1f krev | ESC %d C',
                                   flights, temp >= thot and 'ESC warm' or 'time', P.OFF:get(), revs / 1000, temp))
    return update, 50
  end

  -- where in the flight
  local t = t_fly
  local new_cyc = math.floor(t / CYCLE)
  local tc = t - new_cyc * CYCLE
  if new_cyc ~= cyc then
    local mean = cs.n > 0 and cs.rsum / cs.n or 1
    if cyc >= check_from then
      gcs:send_text(6, string.format('wear: cycle %d ok | thr/learnt %.3f (%.3f-%.3f) | %.1f krev | %.1f V | %d C',
                                     cyc + 1, mean, cs.rmin, cs.rmax, revs / 1000, cs.vmin, cs.tmax))
    elseif cyc == 1 then
      gcs:send_text(6, string.format('wear: learnt %d slots, watching from cycle 3', NSLOT))
    else
      gcs:send_text(6, string.format('wear: cycle 1 done, hover needs %.0f us', 1000 + g * P.RHOV:get()))
    end
    logger:write('WRC', 'cyc,krev,tmean,tmin,tmax,vmin,tmpmax', 'Hffffff', cyc + 1, revs / 1000, mean,
                 cs.rmin, cs.rmax, cs.vmin, cs.tmax)
    cyc = new_cyc
    cs = new_stats()
    if P.CYC:get() > 0 and cyc >= P.CYC:get() then
      stop(5, string.format('%d cycles done, no failure', cyc), REASON.cycles, raw)
      return update, 100
    end
  end
  local frac, seg = plan_at(tc)
  local target = P.RHOV:get() * (1 + P.SCL:get() * (frac - 1))
  target = math.min(target, P.RMAX:get() * 0.9)
  if tgt_s == nil then tgt_s = target end
  local step = P.SLEW:get() * dt
  tgt_s = tgt_s + math.max(-step, math.min(step, target - tgt_s))
  target = tgt_s

  -- rpm loop: feed-forward (learnt us/rpm) + PI on the error
  if raw then rpm_f = rpm_f + math.min(1, dt / 0.02) * (raw - rpm_f) end
  local err = target - rpm_f
  local ff = g * target
  local pwm_u = 1000 + ff + P.KP:get() * err + integ
  local pmax = P.PMAX:get()
  local pwm = math.max(1050, math.min(pwm_u, pmax))
  if pwm == pwm_u or (pwm_u > pmax and err < 0) or (pwm_u < 1050 and err > 0) then
    integ = integ + P.KI:get() * err * dt               -- anti-windup: only when not pushing a limit
  end
  integ = math.max(-150, math.min(integ, 150))
  -- move the integrator into the feed-forward slowly, so g learns the motor/supply
  if rpm_f > 3000 then
    local shift = integ * math.min(1, dt / 3)
    g = math.max(0.005, math.min(g + shift / math.max(target, 1000), 0.08))
    integ = integ - shift
  end

  -- slot bookkeeping: throttle above idle needed at each point of the flight
  local si = math.floor(tc / SLOT)
  if si ~= slot_i then
    if slot_i >= 0 and #slot_buf > 0 then
      local m = median(slot_buf)
      local k = slot_i
      if cyc == 1 then
        tmpl[k] = m
      elseif cyc >= check_from and tmpl[k] and tmpl[k] > 50 then
        local r = m / tmpl[k]
        ratios[#ratios + 1] = r
        if #ratios > 5 then table.remove(ratios, 1) end
        ratio_now = median(ratios)
        cs.rmin, cs.rmax = math.min(cs.rmin, r), math.max(cs.rmax, r)
        cs.rsum, cs.n = cs.rsum + r, cs.n + 1
        tmpl[k] = tmpl[k] + 0.1 * (m - tmpl[k])
        if #ratios >= 5 and math.abs(ratio_now - 1) * 100 > P.DTHR:get() then
          stop(2, string.format('throttle %+.0f%% vs learnt in %s: %s', (ratio_now - 1) * 100, seg,
                                ratio_now < 1 and 'blade lost?' or 'rubbing / drag?'), REASON.thr, raw)
          return update, 100
        end
      end
    end
    slot_i, slot_buf = si, {}
  end
  slot_buf[#slot_buf + 1] = pwm - 1000

  -- hard limits
  if now - t_rpm_ok > 1 then stop(2, 'no rpm telemetry', REASON.telem, 0); return update, 100 end
  if raw and raw > P.RMAX:get() then stop(2, string.format('overspeed %.0f rpm', raw), REASON.overspeed, raw); return update, 100 end
  if t - (check_from - 1) * CYCLE > 3 and rpm_f < 0.5 * target and tc > 3 then
    t_stall = t_stall or now
    if now - t_stall > 2 then stop(2, string.format('cannot hold rpm (%.0f vs %.0f)', rpm_f, target), REASON.stall, raw); return update, 100 end
  else
    t_stall = nil
  end
  if tc > 3 and math.abs(rpm_f - target) > P.TRK:get() / 100 * target then
    t_trk = t_trk or now
    if now - t_trk > 1 then
      stop(2, string.format('rpm off target %.0f vs %.0f for 1 s (desync / damage?)', rpm_f, target), REASON.track, raw)
      return update, 100
    end
  else
    t_trk = nil
  end
  if temp > P.TMAX:get() then stop(2, string.format('ESC hot %d C', temp), REASON.temp, raw); return update, 100 end
  if volt > 1 and volt < P.VLOW:get() then
    t_vlow = t_vlow or now
    if now - t_vlow > 2 then
      stop(4, string.format('battery low %.2f V (rotor OK)', volt), REASON.battery, raw)
      return update, 100
    end
  else
    t_vlow = nil
  end
  if volt > 1 then cs.vmin = math.min(cs.vmin, volt) end
  cs.tmax = math.max(cs.tmax, temp)

  SRV_Channels:set_output_pwm_chan_timeout(0, math.floor(pwm), 50)
  logger:write('WEAR', 'pwm,rpm,tgt,ff,integ,ratio,krev,volt,temp,cyc,seg', 'Hffffffffhb', math.floor(pwm),
               raw or -1, target, ff, integ, ratio_now, revs / 1000, volt, temp, cyc, seg_index(seg))
  return update, 10
end

gcs:send_text(6, string.format('wear loaded (rpm loop, %d on / %d off min): %.0f s cycle, arm to start', math.floor(P.ON:get()), math.floor(P.OFF:get()), CYCLE))
return update, 1000
