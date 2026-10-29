--[[
   Runs motorid.lua against the binding mock (mock_ap.lua) and checks it.
   One scenario per interpreter run (the script adds a parameter table,
   which can only happen once):

     runlua test_motorid_lua.lua SCENARIO [csv_out]

   scenarios: default, short, sines, noprop_set, not_motor, disarm, notelem
   `default` can also write the logged MIDT records to a CSV (used by the
   Python end-to-end test to make a synthetic dataflash log).
--]]

local here = (arg and arg[0] or ''):match('(.*/)') or './'
-- the ArduPilot Lua build has no dofile/loadfile/require: read + load
local function loadpath(path)
   local f = assert(io.open(path, 'r'))
   local src = f:read('a')
   f:close()
   return assert(load(src, '@' .. path))
end
local M = loadpath(here .. 'mock_ap.lua')()
M.loadpath = loadpath
local script = here .. '../motorid.lua'
local scenario = arg[1] or 'default'
local fails = 0

local function check(cond, what)
   if cond then
      print('  ok   ' .. what)
   else
      print('  FAIL ' .. what)
      fails = fails + 1
   end
end

local function texts_matching(pat)
   local n = 0
   for _, t in ipairs(M.texts) do if t.text:find(pat) then n = n + 1 end end
   return n
end

-- durations of each contiguous block of one test id
local function blocks()
   local out = {}
   local cur
   for _, r in ipairs(M.messages('MIDT')) do
      local id = r.fields.test
      if not cur or cur.id ~= id then
         cur = {id = id, t0 = r.t, t1 = r.t, n = 0, fmin = 1e9, fmax = -1, pmin = 1e9, pmax = -1}
         out[#out + 1] = cur
      end
      cur.t1 = r.t
      cur.n = cur.n + 1
      cur.fmin = math.min(cur.fmin, r.fields.f); cur.fmax = math.max(cur.fmax, r.fields.f)
      cur.pmin = math.min(cur.pmin, r.fields.pwm); cur.pmax = math.max(cur.pmax, r.fields.pwm)
   end
   return out
end

print('scenario ' .. scenario)
M.verbose = (arg[3] == 'verbose')

if scenario == 'default' then
   M.preset = {MID_ENABLE = 1, MID_PROP = 0}
   M.load(script)
   M.run(400, {{2.0, function() M.armed = true end}, {390, function() M.armed = false end}})
   local c = M.messages('MIDC')
   check(#c == 1, 'one MIDC context record')
   if #c == 1 then
      local f = c[1].fields
      check(f.prop == 0 and f.chan == 1 and f.func == 33 and f.poles == 14 and f.mask == 15 and f.ver == 1,
            'MIDC fields prop/chan/func/poles/mask/ver')
      check(math.abs(f.dur - 191) < 0.01, string.format("plan length %.1f s (expect 191)", f.dur))
   end
   check(#M.messages('MIDP') == 1 and #M.messages('MIDS') == 1, 'plan records MIDP and MIDS')
   local b = blocks()
   local ids = {}
   for _, x in ipairs(b) do ids[#ids + 1] = x.id end
   -- coast-downs alternate CST_PWM / idle inside test 2, so test 2 is one block
   check(table.concat(ids, ',') == '0,1,0,2,0,3,0,4,0', 'test order ' .. table.concat(ids, ','))
   local want = {[1] = 24, [2] = 24, [3] = 60, [4] = 60}
   for _, x in ipairs(b) do
      if want[x.id] then
         check(math.abs((x.t1 - x.t0) - want[x.id]) < 0.05, string.format('test %d lasts %.2f s', x.id, x.t1 - x.t0))
      end
      if x.id == 1 then check(x.pmin == 1100 and x.pmax == 1800, 'steps 1100..1800') end
      if x.id == 2 then check(x.pmin == 1000 and x.pmax == 1500, 'coast-downs 1500 / zero') end
      if x.id == 3 then check(x.pmin >= 1100 and x.pmax <= 1500 and x.fmin >= 1 and x.fmax <= 50.01 and x.fmax > 49,
                              string.format('small sweep pwm %d..%d f %.2f..%.2f', x.pmin, x.pmax, x.fmin, x.fmax)) end
      if x.id == 4 then check(x.pmin >= 1100 and x.pmax <= 1900, string.format('big sweep pwm %d..%d', x.pmin, x.pmax)) end
   end
   local n = #M.messages('MIDT')
   local rate = n / 191
   check(rate > 400 and rate < 460, string.format('MIDT rate %.0f Hz', rate))
   check(texts_matching('run 1:') == 1 and texts_matching('done') == 1, 'GCS start and done messages')
   local last_out = M.outputs[#M.outputs].t
   check(last_out < 2.0 + 191 + 0.1, string.format('output handed back after the plan (last override %.1f s)', last_out))
   if arg[2] then
      local f = assert(io.open(arg[2], 'w'))
      f:write('t,test,pwm,f\n')
      for _, r in ipairs(M.messages('MIDT')) do
         f:write(string.format('%.6f,%d,%d,%.5f\n', r.t, r.fields.test, r.fields.pwm, r.fields.f))
      end
      f:close()
      print('  wrote ' .. arg[2])
   end

elseif scenario == 'short' then
   -- a short plan (71 s) for the Python end-to-end test; writes the CSV
   M.preset = {MID_ENABLE = 1, MID_PROP = 0, MID_GAP = 2, MID_STP_TIME = 1.5, MID_CST_NUM = 2,
               MID_CST_TIME = 2, MID_SW_TIME = 20}
   M.load(script)
   M.run(80, {{1.0, function() M.armed = true end}})
   local c = M.messages('MIDC')
   check(#c == 1 and math.abs(c[1].fields.dur - 71) < 0.01, 'short plan is 71 s')
   if arg[2] then
      local f = assert(io.open(arg[2], 'w'))
      f:write('t,test,pwm,f\n')
      for _, r in ipairs(M.messages('MIDT')) do
         f:write(string.format('%.6f,%d,%d,%.5f\n', r.t, r.fields.test, r.fields.pwm, r.fields.f))
      end
      f:close()
   end

elseif scenario == 'sines' then
   M.preset = {MID_ENABLE = 1, MID_PROP = 1, MID_TESTS = 16, MID_SIN_NUM = 4}
   M.load(script)
   M.run(60, {{1.0, function() M.armed = true end}})
   local freqs = {}
   for _, r in ipairs(M.messages('MIDT')) do
      if r.fields.test == 5 then freqs[string.format('%.2f', r.fields.f)] = true end
   end
   local fl = {}
   for k in pairs(freqs) do fl[#fl + 1] = k end
   table.sort(fl, function(a, b) return tonumber(a) < tonumber(b) end)
   check(table.concat(fl, ' ') == '1.00 3.68 13.57 50.00', 'sine frequencies ' .. table.concat(fl, ' '))
   check(M.messages('MIDC')[1].fields.prop == 1, 'prop 1 logged')

elseif scenario == 'noprop_set' then
   M.preset = {MID_ENABLE = 1}           -- MID_PROP left at -1
   M.load(script)
   M.run(10, {{1.0, function() M.armed = true end}})
   check(#M.messages('MIDT') == 0 and #M.outputs == 0, 'refuses to run with MID_PROP = -1')
   check(texts_matching('set MID_PROP') == 1, 'tells the user to set MID_PROP (once)')

elseif scenario == 'not_motor' then
   M.preset = {MID_ENABLE = 1, MID_PROP = 0}
   M.params.SERVO1_FUNCTION = 1          -- RCPassThru, not a motor
   M.load(script)
   M.run(10, {{1.0, function() M.armed = true end}})
   check(#M.outputs == 0, 'refuses to drive a non-motor output')
   check(texts_matching('not a motor') == 1, 'says why')

elseif scenario == 'disarm' then
   M.preset = {MID_ENABLE = 1, MID_PROP = 0}
   M.load(script)
   M.run(40, {{1.0, function() M.armed = true end}, {20.0, function() M.armed = false end},
              {25.0, function() M.armed = true end}})
   local t_stop = nil
   for _, o in ipairs(M.outputs) do if o.t > 20.01 and o.t < 25 then t_stop = o.t end end
   check(t_stop == nil, 'no output override while disarmed')
   check(texts_matching('disarmed') == 1, 'reports the stop')
   local c = M.messages('MIDC')
   check(#c == 2 and c[2].fields.run == 2, 'rearming starts run 2 from the beginning')

elseif scenario == 'notelem' then
   M.preset = {MID_ENABLE = 1, MID_PROP = 0}
   M.telemetry = false
   M.load(script)
   M.run(20, {{1.0, function() M.armed = true end}})
   local all_neg = true
   for _, r in ipairs(M.messages('MIDT')) do if r.fields.rpm ~= -1 then all_neg = false end end
   check(all_neg, 'logs rpm = -1 without telemetry')
   check(texts_matching('no ESC rpm') == 1, 'warns once about missing telemetry')
else
   error('unknown scenario ' .. scenario)
end

print(fails == 0 and 'PASS' or ('FAILED ' .. fails))
if fails > 0 then error('test failures') end
