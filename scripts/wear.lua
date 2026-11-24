-- Rotor wear test ("wear"), PROP ON, output 1. Pair with the TEMPORARY ESC build
-- FOXEER_F421_WEAR, which adds the once-per-rev (1P) torque pulse on top of whatever
-- throttle this script sends, as cyclic does in flight.
--
-- Arm and it flies a repeating ~40 s "flight" on the throttle, until the rotor breaks,
-- WR_CYC cycles are done, or you disarm:
--   spin-up, hover, slow gusts (2 Hz), fast gusts (6 Hz), manoeuvre buzz (12 Hz),
--   punch-outs, hover, descent, bounce, settle.
-- Levels are relative to WR_HOV (hover throttle); WR_SCL scales every excursion.
--
-- Break detection, rpm only (the FC does not need to be on the stand):
-- cycle 1 is a warm-up, cycle 2 learns the rpm at every 0.2 s slot of the flight, and
-- from cycle 3 every slot is compared with what it was. A blade or chunk lost makes the
-- motor run faster at the same throttle; something rubbing makes it slower. The median
-- of the last 5 slot ratios (1 s) beyond WR_DRPM % trips. Each slot's learnt rpm follows
-- slow drift (battery sag) by 10 % per cycle.
-- Also stops on: no rpm telemetry 1 s, overspeed, battery low (ESC voltage), ESC hot.
-- On any stop: throttle zero, DISARM, message + tune, cycles and revolutions reported.
-- Only ONE bench script (wear / fatigue / spin1 / bench*) may be on the FC.
-- Guard round the rotor, stand out of its plane, eye protection.

local KEY = 74
assert(param:add_table(KEY, 'WR_', 10), 'wear: param table')
assert(param:add_param(KEY, 1, 'HOV', 1500))    -- hover throttle (us)
assert(param:add_param(KEY, 2, 'SCL', 1.0))     -- scale on every excursion from hover
assert(param:add_param(KEY, 3, 'CYC', 0))       -- stop after this many cycles (0 = until break)
assert(param:add_param(KEY, 4, 'DRPM', 8))      -- rpm change vs learnt that trips (%)
assert(param:add_param(KEY, 5, 'RMAX', 21000))  -- overspeed (rpm)
assert(param:add_param(KEY, 6, 'VLOW', 14.0))   -- battery low from ESC voltage (V, 2 s)
assert(param:add_param(KEY, 7, 'TMAX', 90))     -- ESC too hot (deg C)
assert(param:add_param(KEY, 8, 'PMAX', 1850))   -- never command more than this (us)

local P = {}
for _, n in ipairs({'HOV', 'SCL', 'CYC', 'DRPM', 'RMAX', 'VLOW', 'TMAX', 'PMAX'}) do
  P[n] = Parameter('WR_' .. n)
end

-- ---------------------------------------------------------------------------------
-- the flight: {name, seconds, function(t) -> offset from hover in us (before WR_SCL)}
local function hold(d) return function() return d end end
local function ramp(a, b, T) return function(t) return a + (b - a) * t / T end end
local function sine(amp, f) return function(t) return amp * math.sin(2 * math.pi * f * t) end end
local function square(hi, lo, t_hi, t_lo)
  return function(t) if t % (t_hi + t_lo) < t_hi then return hi end return lo end
end
local PLAN = {
  {'spinup',  2.0, ramp(-350, 0, 2.0)},
  {'hover',   6.0, hold(0)},
  {'gust2',   4.0, sine(120, 2)},
  {'gust6',   4.0, sine(120, 6)},
  {'buzz12',  3.0, sine(80, 12)},
  {'punch',   4.8, square(250, -100, 0.4, 0.8)},   -- 4 punch-outs
  {'hover',   4.0, hold(0)},
  {'descend', 3.0, hold(-200)},
  {'bounce',  4.0, square(50, -200, 0.5, 0.5)},    -- flare / bounce, 1 Hz
  {'settle',  3.0, ramp(-200, 0, 3.0)},
}
local CYCLE = 0
for _, s in ipairs(PLAN) do CYCLE = CYCLE + s[2] end
local SLOT = 0.2                                     -- template slot (s)
local NSLOT = math.floor(CYCLE / SLOT + 0.5)

-- ---------------------------------------------------------------------------------
local REASON = {user = 1, rpm = 2, telem = 4, overspeed = 5, battery = 6, temp = 7, cycles = 8}
local state = 'idle'                                 -- idle / wait / run / stopped
local t_wait, t_seen = nil, nil                      -- telemetry wait after arming
local t_arm, t_last, t_rpm_ok, t_vlow = nil, 0, 0, nil
local revs, cyc = 0, 0
local tmpl = {}                                      -- learnt rpm per slot
local slot_i, slot_buf = -1, {}
local ratios = {}                                    -- last 5 slot ratios
local ratio_now = 1
local volt, temp, cur = 0, 0, 0
local cs = nil                                       -- per-cycle stats

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
  gcs:send_text(level, string.format('wear: STOP %s | cycle %d, %.1f min, %.1f krev', why, cyc,
                                     (millis():tofloat() / 1000 - t_arm) / 60, revs / 1000))
  notify:play_tune(code == REASON.rpm and 'MFT100L4CCC' or 'MFT200L8C')
end

local function plan_at(tc)                           -- tc: time within the cycle
  for _, s in ipairs(PLAN) do
    if tc < s[2] then return s[3](tc), s[1] end
    tc = tc - s[2]
  end
  return 0, 'hover'
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

  local now = millis():tofloat() / 1000
  -- after arming the ESC telemetry goes quiet for ~3 s: hold zero throttle until
  -- rpm has been coming in for 0.5 s (max 8 s), then start the flight
  if t_arm == nil then
    if state ~= 'wait' then
      state, t_wait, t_seen = 'wait', now, nil
      gcs:send_text(6, 'wear: armed, waiting for ESC telemetry')
    end
    if esc_telem:get_rpm(0) then
      t_seen = t_seen or now
    else
      t_seen = nil
    end
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
  end
  if t_arm == nil then
    t_arm, t_last, t_rpm_ok, t_vlow = now, now, now, nil
    state, revs, cyc, tmpl, slot_i, slot_buf, ratios, ratio_now = 'run', 0, 0, {}, -1, {}, {}, 1
    cs = new_stats()
    gcs:send_text(6, string.format('wear: start, hover %d us, %.0f s cycle, learning on cycle 2',
                                   math.floor(P.HOV:get()), CYCLE))
  end
  local dt = now - t_last
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

  -- where in the flight
  local t = now - t_arm
  local new_cyc = math.floor(t / CYCLE)
  local tc = t - new_cyc * CYCLE
  if new_cyc ~= cyc then
    -- cycle finished: report + log
    local mean = cs.n > 0 and cs.rsum / cs.n or 1
    if cyc >= 2 then
      gcs:send_text(6, string.format('wear: cycle %d ok | rpm/learnt %.3f (%.3f-%.3f) | %.1f krev | %.2f V | %d C',
                                     cyc + 1, mean, cs.rmin, cs.rmax, revs / 1000, cs.vmin, cs.tmax))
    elseif cyc == 1 then
      gcs:send_text(6, string.format('wear: learnt %d slots, watching from cycle 3', NSLOT))
    end
    logger:write('WRC', 'cyc,krev,rmean,rmin,rmax,vmin,tmax', 'Hffffff', cyc + 1, revs / 1000, mean,
                 cs.rmin, cs.rmax, cs.vmin, cs.tmax)
    cyc = new_cyc
    cs = new_stats()
    if P.CYC:get() > 0 and cyc >= P.CYC:get() then
      stop(5, string.format('%d cycles done, no failure', cyc), REASON.cycles, raw)
      return update, 100
    end
  end
  local off, seg = plan_at(tc)
  local pwm = P.HOV:get() + P.SCL:get() * off
  pwm = math.max(1050, math.min(pwm, P.PMAX:get()))

  -- slot bookkeeping: close a slot when time moves into the next one
  local si = math.floor(tc / SLOT)
  if si ~= slot_i then
    if slot_i >= 0 and #slot_buf > 0 then
      local m = median(slot_buf)
      local k = slot_i
      if cyc == 1 or (cyc == 2 and si == 0 and tmpl[k] == nil) then
        tmpl[k] = m                                  -- learning cycle
      elseif tmpl[k] and tmpl[k] > 300 then
        local r = m / tmpl[k]
        ratios[#ratios + 1] = r
        if #ratios > 5 then table.remove(ratios, 1) end
        ratio_now = median(ratios)
        cs.rmin, cs.rmax = math.min(cs.rmin, r), math.max(cs.rmax, r)
        cs.rsum, cs.n = cs.rsum + r, cs.n + 1
        tmpl[k] = tmpl[k] + 0.1 * (m - tmpl[k])      -- follow slow drift
        if #ratios >= 5 and math.abs(ratio_now - 1) * 100 > P.DRPM:get() then
          stop(2, string.format('rpm %+.0f%% vs learnt in %s: blade lost or rubbing',
                                (ratio_now - 1) * 100, seg), REASON.rpm, m)
          return update, 100
        end
      end
    end
    slot_i, slot_buf = si, {}
  end
  if raw then slot_buf[#slot_buf + 1] = raw end

  -- hard limits
  if now - t_rpm_ok > 1 and t > 2 then stop(2, 'no rpm telemetry', REASON.telem, 0); return update, 100 end
  if raw and raw > P.RMAX:get() then stop(2, string.format('overspeed %.0f rpm', raw), REASON.overspeed, raw); return update, 100 end
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
  logger:write('WEAR', 'pwm,rpm,tmpl,ratio,krev,volt,temp,cur,cyc,seg', 'Hfffffhfhb', math.floor(pwm),
               raw or -1, tmpl[si] or 0, ratio_now, revs / 1000, volt, temp, cur, cyc, seg_index(seg))
  return update, 10
end

gcs:send_text(6, string.format('wear loaded: %.0f s cycle, arm to start', CYCLE))
return update, 1000
