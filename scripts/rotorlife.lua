-- Rotor life counter ("rotorlife"): counts the revolutions the fitted rotor has done, keeps the
-- count across reboots (a saved parameter), warns past RLIF_WARN and refuses to ARM past
-- RLIF_LIMIT until the rotor is replaced and the counter reset. Never cuts the motor in
-- flight (that is a crash); with RLIF_CUT 1 it also cuts throttle on the bench (disarmed
-- by the script) once the limit is passed.
--
-- Counts from ESC rpm telemetry (output RLIF_CHAN), integrated every 50 ms.
-- Parameters (krev = 1000 revolutions; a 10 min hover at 12 750 rpm is ~128 krev):
--   RLIF_KREV   revolutions on the fitted rotor so far (krev). Saved every 30 s while
--               spinning and at disarm.
--   RLIF_WARN   warn above this (krev)
--   RLIF_LIMIT  block arming above this (krev)
--   RLIF_RESET  set to 1 after fitting a new rotor: zeroes RLIF_KREV, then returns to 0
--   RLIF_ID     the rotor's stamped version x10 (v3.1 -> 31), logged with the count
--   RLIF_CUT    1: on the bench, disarm when the limit is passed while spinning
--   RLIF_CHAN   ESC telemetry index (0-based) of the rotor motor
-- WARN/LIMIT defaults are placeholders until spin tests give a real life figure.

local KEY = 72
assert(param:add_table(KEY, 'RLIF_', 8), 'rotorlife: param table')
assert(param:add_param(KEY, 1, 'KREV', 0))
assert(param:add_param(KEY, 2, 'WARN', 100))
assert(param:add_param(KEY, 3, 'LIMIT', 150))
assert(param:add_param(KEY, 4, 'RESET', 0))
assert(param:add_param(KEY, 5, 'ID', 0))
assert(param:add_param(KEY, 6, 'CUT', 0))
assert(param:add_param(KEY, 7, 'CHAN', 0))

local P = {}
for _, n in ipairs({'KREV', 'WARN', 'LIMIT', 'RESET', 'ID', 'CUT', 'CHAN'}) do
  P[n] = Parameter('RLIF_' .. n)
end

local auth = arming:get_aux_auth_id()
local krev = P.KREV:get()
local t_last = millis():tofloat() / 1000
local t_save = t_last
local t_msg = 0
local dirty = false
local was_armed = false

local function save()
  if dirty then
    P.KREV:set_and_save(krev)
    dirty = false
  end
end

local function check_arming()
  if not auth then return end
  if krev >= P.LIMIT:get() then
    arming:set_aux_auth_failed(auth, string.format('rotor life %.0f/%.0f krev: replace rotor, set RLIF_RESET 1',
                                                   krev, P.LIMIT:get()))
  else
    arming:set_aux_auth_passed(auth)
  end
end

function update()
  local now = millis():tofloat() / 1000
  local dt = now - t_last
  t_last = now

  if P.RESET:get() > 0 then
    krev, dirty = 0, true
    save()
    P.RESET:set_and_save(0)
    gcs:send_text(5, string.format('rotorlife: reset, rotor id %d', math.floor(P.ID:get())))
  end

  local rpm = esc_telem:get_rpm(math.floor(P.CHAN:get())) or 0
  if rpm > 100 then
    krev = krev + rpm / 60 * dt / 1000
    dirty = true
  end

  local armed = arming:is_armed()
  if now - t_save > 30 or (was_armed and not armed) then
    save()
    t_save = now
  end
  was_armed = armed

  local warn, limit = P.WARN:get(), P.LIMIT:get()
  if armed and krev >= warn and now - t_msg > 10 then
    t_msg = now
    gcs:send_text(krev >= limit and 2 or 4,
                  string.format('rotorlife: %.0f/%.0f krev%s', krev, limit, krev >= limit and ' LIMIT, land' or ''))
    notify:play_tune('MFT200L8C')
  end
  if armed and krev >= limit and P.CUT:get() > 0 and not vehicle:get_likely_flying() then
    arming:disarm()
    gcs:send_text(2, 'rotorlife: limit passed, disarmed (RLIF_CUT 1)')
  end

  check_arming()
  logger:write('RLIF', 'krev,rpm,id', 'ffB', krev, rpm, math.floor(P.ID:get()))
  return update, 50
end

check_arming()
gcs:send_text(6, string.format('rotorlife: %.1f krev on rotor %d (warn %.0f, limit %.0f)',
                               krev, math.floor(P.ID:get()), P.WARN:get(), P.LIMIT:get()))
return update, 1000
