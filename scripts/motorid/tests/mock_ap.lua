--[[
   Minimal mock of the ArduPilot scripting bindings used by motorid.lua,
   so the script can be run and checked on a PC with a plain Lua 5.3.

   It is NOT a simulator: the "motor" is a first-order lag from pwm to
   rpm. It checks the things that would make the real script fail on the
   flight controller: parameter names/indices, logger:write name, label
   and format lengths, argument counts and integer types, GCS text length.
--]]

local M = {}

M.now_us = 0                 -- simulated time
M.armed = false
M.telemetry = true           -- false: esc_telem:get_rpm returns nil
M.params = {SERVO1_FUNCTION = 33, MOT_PWM_MIN = 1000, SERVO_BLH_POLES = 14}
M.preset = {}                -- values a parameter has "saved" before the script adds it
M.log = {}                   -- {name=, t=, fields={...}}
M.texts = {}                 -- GCS messages
M.outputs = {}               -- {t=, chan=, pwm=, timeout=}
M.rpm = 0.0
M.last_pwm = 1000
M.last_out_us = -1e12

-- --------------------------------------------------------------- uint32
local U32 = {}
U32.__index = U32
local function u32(v) return setmetatable({v = math.floor(v) % 4294967296}, U32) end
local function val(x) if type(x) == 'table' then return x.v end return x end
U32.__sub = function(a, b) return u32(val(a) - val(b)) end
U32.__add = function(a, b) return u32(val(a) + val(b)) end
U32.__lt = function(a, b) return val(a) < val(b) end
U32.__le = function(a, b) return val(a) <= val(b) end
function U32:tofloat() return self.v + 0.0 end
function U32:toint() return self.v end

function micros() return u32(M.now_us) end
function millis() return u32(M.now_us / 1000) end

-- ------------------------------------------------------------ parameters
local tables = {}
param = {}
function param:add_table(key, prefix, n)
   assert(math.type(key) == 'integer' and key >= 0 and key <= 200, 'bad table key')
   assert(n >= 1 and n <= 63, 'bad table size')
   assert(#prefix <= 15, 'prefix too long')
   for k, t in pairs(tables) do
      assert(t.prefix ~= prefix, 'prefix already used')
   end
   assert(tables[key] == nil, 'table key already used')
   tables[key] = {prefix = prefix, n = n, idx = {}}
   return true
end
function param:add_param(key, idx, name, default)
   local t = assert(tables[key], 'add_param before add_table')
   assert(idx >= 1 and idx <= t.n, 'param index out of range: ' .. name)
   assert(not t.idx[idx], 'param index used twice: ' .. name)
   local full = t.prefix .. name
   assert(#full <= 16, 'param name longer than 16 chars: ' .. full)
   t.idx[idx] = full
   if M.preset[full] ~= nil then M.params[full] = M.preset[full] else M.params[full] = default end
   return true
end
function param:get(name) return M.params[name] end
function param:set(name, v) if M.params[name] == nil then return false end M.params[name] = v return true end

local P = {}
P.__index = P
function Parameter(name)
   assert(name == nil or M.params[name] ~= nil, 'Parameter(): unknown ' .. tostring(name))
   return setmetatable({name = name}, P)
end
function P:get() return M.params[self.name] end
function P:set(v) M.params[self.name] = v return true end
function P:set_and_save(v) M.params[self.name] = v return true end
function P:init(name) self.name = name return M.params[name] ~= nil end

-- -------------------------------------------------------------- vehicle
arming = {}
function arming:is_armed() return M.armed end

SRV_Channels = {}
function SRV_Channels:set_output_pwm_chan_timeout(chan, pwm, timeout)
   assert(math.type(chan) == 'integer' and chan >= 0 and chan < 32, 'bad channel')
   assert(math.type(pwm) == 'integer' or pwm == math.floor(pwm), 'pwm must be an integer')
   M.outputs[#M.outputs + 1] = {t = M.now_us * 1e-6, chan = chan, pwm = pwm, timeout = timeout}
   M.last_pwm = pwm
   M.last_out_us = M.now_us
end
function SRV_Channels:find_channel(func)
   for i = 1, 32 do if M.params['SERVO' .. i .. '_FUNCTION'] == func then return i - 1 end end
   return nil
end

-- toy motor: first-order lag (30 ms) towards 30 rpm per us above 1000
function M.step_motor(dt)
   local pwm = 1000
   if M.now_us - M.last_out_us < 50000 then pwm = M.last_pwm end
   local target = math.max(0, 30 * (pwm - 1000))
   M.rpm = M.rpm + (target - M.rpm) * math.min(dt / 0.03, 1)
end

esc_telem = {}
function esc_telem:get_rpm(i)
   assert(math.type(i) == 'integer', 'esc instance must be an integer')
   if not M.telemetry then return nil end
   return M.rpm
end

-- --------------------------------------------------------------- logger
local INT_RANGE = {b = {-128, 127}, B = {0, 255}, h = {-32768, 32767}, H = {0, 65535},
                   i = {-2147483648, 2147483647}, I = {0, 4294967295}}
logger = {}
function logger:write(name, labels, fmt, ...)
   assert(#name <= 4, 'log name longer than 4 chars: ' .. name)
   assert(#labels < 58, 'labels must be less than 58 chars: ' .. name)
   assert(#fmt < 15, 'format must be less than 15 chars: ' .. name)
   local _, commas = labels:gsub(',', ',')
   local nl = commas + 1
   assert(nl == #fmt, 'label does not match format: ' .. name)
   local args = table.pack(...)
   assert(args.n == #fmt, 'format does not match number of arguments: ' .. name)
   local fields = {}
   local i = 1
   for lab in labels:gmatch('[^,]+') do
      local v = args[i]
      local c = fmt:sub(i, i)
      if INT_RANGE[c] then
         assert(type(v) == 'number' and v == math.floor(v), string.format('%s.%s must be an integer, got %s', name, lab, tostring(v)))
         assert(v >= INT_RANGE[c][1] and v <= INT_RANGE[c][2], string.format('%s.%s out of range: %s', name, lab, tostring(v)))
      elseif c == 'f' or c == 'd' then
         assert(type(v) == 'number', string.format('%s.%s must be a number', name, lab))
      else
         error('format char not handled by the mock: ' .. c)
      end
      fields[lab] = v
      i = i + 1
   end
   M.log[#M.log + 1] = {name = name, t = M.now_us * 1e-6, fields = fields}
end

-- ------------------------------------------------------------------ gcs
gcs = {}
function gcs:send_text(sev, text)
   assert(#text <= 50, 'GCS text longer than 50 chars: ' .. text)
   M.texts[#M.texts + 1] = {t = M.now_us * 1e-6, sev = sev, text = text}
   if M.verbose then print(string.format('%8.3f  GCS: %s', M.now_us * 1e-6, text)) end
end

-- ------------------------------------------------------------ scheduler
-- load the script and run it until time t_end (s); `events` is a list of
-- {t, function} called once when the simulated time passes t
function M.load(path)
   local chunk = M.loadpath(path)
   local fn, delay = chunk()
   M.fn, M.delay = fn, delay
end

function M.run(t_end, events, jitter_ms)
   jitter_ms = jitter_ms or 0.33   -- 2 ms requested -> ~430 Hz like on the H743
   events = events or {}
   local ei = 1
   table.sort(events, function(a, b) return a[1] < b[1] end)
   while M.now_us * 1e-6 < t_end do
      local dt = M.delay + jitter_ms
      M.now_us = M.now_us + dt * 1000
      M.step_motor(dt / 1000)
      while ei <= #events and events[ei][1] <= M.now_us * 1e-6 do
         events[ei][2]()
         ei = ei + 1
      end
      local fn, delay = M.fn()
      assert(type(fn) == 'function' and type(delay) == 'number', 'update must return function, delay')
      M.fn, M.delay = fn, delay
   end
end

function M.messages(name)
   local out = {}
   for _, r in ipairs(M.log) do if r.name == name then out[#out + 1] = r end end
   return out
end

return M
