--[[
   motorid.lua: bench excitation for motor + ESC + prop identification

   Drives ONE motor output through a configurable test plan (throttle
   steps, coast-downs, a small and a big log sweep, fixed-frequency
   sines) while logging the throttle and ESC-telemetry rpm at the script
   rate (~430 Hz). The log is analysed offline with motorid.py.

   Arm to start (MID_ENABLE = 1), disarm to stop at any time. When the
   plan is finished the script stops overriding the output, so it goes
   back to whatever the vehicle commands. See motorid.md.

   Log messages:
     MIDC  run context, once at the start of each run (prop state, output,
           motor poles, test mask, plan version ...)
     MIDP  plan settings: steps, coast-downs, sweeps
     MIDS  plan settings: fixed sines, pwm limit
     MIDT  excitation + response at the script rate:
           test (id below), pwm (us), f (Hz, 0 = none), rpm (-1 = no telemetry)
           (same fields as the older bench.lua BNCH message)

   Test ids: 0 gap (zero throttle), 1 steps, 2 coast-downs, 3 small sweep,
             4 big sweep, 5 fixed sines
--]]

local PLAN_VERSION = 1      -- bump when the meaning of the plan changes

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local PARAM_TABLE_KEY = 113  -- must be unique among the scripts on this flight controller
local PARAM_TABLE_PREFIX = "MID_"

-- add a parameter and bind it to a variable
local function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
   return Parameter(PARAM_TABLE_PREFIX .. name)
end

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 24), 'could not add param table')

--[[
  // @Param: MID_ENABLE
  // @DisplayName: Motor ID enable
  // @Description: Enable the motor identification test. When enabled the test plan runs once each time the vehicle is armed.
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local MID_ENABLE = bind_add_param('ENABLE', 1, 0)

--[[
  // @Param: MID_TESTS
  // @DisplayName: Motor ID test selection
  // @Description: Tests to run, in this order. Each test is preceded by MID_GAP seconds at zero throttle.
  // @Bitmask: 0:Throttle steps,1:Coast-downs,2:Small sweep,3:Big sweep,4:Fixed sines
  // @User: Standard
--]]
local MID_TESTS = bind_add_param('TESTS', 2, 15)

--[[
  // @Param: MID_CHAN
  // @DisplayName: Motor ID output channel
  // @Description: 1-based servo output channel of the motor under test. Its SERVOn_FUNCTION must be a motor (33-40 or 82-85), otherwise the script refuses to run. ESC telemetry is read for the same channel.
  // @Range: 1 32
  // @User: Standard
--]]
local MID_CHAN = bind_add_param('CHAN', 3, 1)

--[[
  // @Param: MID_PROP
  // @DisplayName: Motor ID prop state
  // @Description: What is fitted to the motor; written to the log so the analysis knows. -1 means not set: the script refuses to run until it is set. 2 and above are user-defined prop type numbers.
  // @Values: -1:Not set,0:No prop,1:Prop fitted
  // @Range: -1 100
  // @User: Standard
--]]
local MID_PROP = bind_add_param('PROP', 4, -1)

--[[
  // @Param: MID_GAP
  // @DisplayName: Motor ID gap
  // @Description: Time at zero throttle before each test (the motor coasts or brakes to a stop).
  // @Units: s
  // @Range: 1 30
  // @User: Advanced
--]]
local MID_GAP = bind_add_param('GAP', 5, 5)

--[[
  // @Param: MID_STP_MIN
  // @DisplayName: Motor ID first step
  // @Description: Throttle of the first step.
  // @Units: PWM
  // @Range: 1000 2000
  // @User: Advanced
--]]
local MID_STP_MIN = bind_add_param('STP_MIN', 6, 1100)

--[[
  // @Param: MID_STP_MAX
  // @DisplayName: Motor ID last step
  // @Description: Throttle of the last step (steps go from MID_STP_MIN up to this in MID_STP_INC increments).
  // @Units: PWM
  // @Range: 1000 2000
  // @User: Advanced
--]]
local MID_STP_MAX = bind_add_param('STP_MAX', 7, 1800)

--[[
  // @Param: MID_STP_INC
  // @DisplayName: Motor ID step increment
  // @Description: Throttle increase from one step to the next.
  // @Units: PWM
  // @Range: 10 500
  // @User: Advanced
--]]
local MID_STP_INC = bind_add_param('STP_INC', 8, 100)

--[[
  // @Param: MID_STP_TIME
  // @DisplayName: Motor ID step length
  // @Description: Time at each throttle step.
  // @Units: s
  // @Range: 1 30
  // @User: Advanced
--]]
local MID_STP_TIME = bind_add_param('STP_TIME', 9, 3)

--[[
  // @Param: MID_CST_PWM
  // @DisplayName: Motor ID coast-down throttle
  // @Description: Throttle held before each cut to zero in the coast-down test.
  // @Units: PWM
  // @Range: 1000 2000
  // @User: Advanced
--]]
local MID_CST_PWM = bind_add_param('CST_PWM', 10, 1500)

--[[
  // @Param: MID_CST_TIME
  // @DisplayName: Motor ID coast-down time
  // @Description: Time at MID_CST_PWM, and then at zero throttle, in each coast-down cycle.
  // @Units: s
  // @Range: 1 30
  // @User: Advanced
--]]
local MID_CST_TIME = bind_add_param('CST_TIME', 11, 4)

--[[
  // @Param: MID_CST_NUM
  // @DisplayName: Motor ID coast-down count
  // @Description: Number of coast-down cycles.
  // @Range: 1 10
  // @User: Advanced
--]]
local MID_CST_NUM = bind_add_param('CST_NUM', 12, 3)

--[[
  // @Param: MID_SS_BASE
  // @DisplayName: Motor ID small sweep centre
  // @Description: Centre throttle of the small sweep.
  // @Units: PWM
  // @Range: 1000 2000
  // @User: Advanced
--]]
local MID_SS_BASE = bind_add_param('SS_BASE', 13, 1300)

--[[
  // @Param: MID_SS_AMP
  // @DisplayName: Motor ID small sweep amplitude
  // @Description: Throttle swing either side of MID_SS_BASE in the small sweep.
  // @Units: PWM
  // @Range: 0 500
  // @User: Advanced
--]]
local MID_SS_AMP = bind_add_param('SS_AMP', 14, 200)

--[[
  // @Param: MID_BS_BASE
  // @DisplayName: Motor ID big sweep centre
  // @Description: Centre throttle of the big sweep.
  // @Units: PWM
  // @Range: 1000 2000
  // @User: Advanced
--]]
local MID_BS_BASE = bind_add_param('BS_BASE', 15, 1500)

--[[
  // @Param: MID_BS_AMP
  // @DisplayName: Motor ID big sweep amplitude
  // @Description: Throttle swing either side of MID_BS_BASE in the big sweep.
  // @Units: PWM
  // @Range: 0 500
  // @User: Advanced
--]]
local MID_BS_AMP = bind_add_param('BS_AMP', 16, 400)

--[[
  // @Param: MID_F_MIN
  // @DisplayName: Motor ID lowest frequency
  // @Description: Start frequency of the sweeps, and lowest of the fixed sines.
  // @Units: Hz
  // @Range: 0.1 100
  // @User: Advanced
--]]
local MID_F_MIN = bind_add_param('F_MIN', 17, 1)

--[[
  // @Param: MID_F_MAX
  // @DisplayName: Motor ID highest frequency
  // @Description: End frequency of the sweeps, and highest of the fixed sines. Keep it well below the script update rate (about 430 Hz): 50 Hz gives about 9 updates per cycle.
  // @Units: Hz
  // @Range: 0.1 100
  // @User: Advanced
--]]
local MID_F_MAX = bind_add_param('F_MAX', 18, 50)

--[[
  // @Param: MID_SW_TIME
  // @DisplayName: Motor ID sweep length
  // @Description: Length of each sweep. The frequency rises exponentially from MID_F_MIN to MID_F_MAX.
  // @Units: s
  // @Range: 5 300
  // @User: Advanced
--]]
local MID_SW_TIME = bind_add_param('SW_TIME', 19, 60)

--[[
  // @Param: MID_SIN_BASE
  // @DisplayName: Motor ID sines centre
  // @Description: Centre throttle of the fixed-frequency sines.
  // @Units: PWM
  // @Range: 1000 2000
  // @User: Advanced
--]]
local MID_SIN_BASE = bind_add_param('SIN_BASE', 20, 1600)

--[[
  // @Param: MID_SIN_AMP
  // @DisplayName: Motor ID sines amplitude
  // @Description: Throttle swing either side of MID_SIN_BASE for the fixed-frequency sines.
  // @Units: PWM
  // @Range: 0 500
  // @User: Advanced
--]]
local MID_SIN_AMP = bind_add_param('SIN_AMP', 21, 250)

--[[
  // @Param: MID_SIN_NUM
  // @DisplayName: Motor ID number of sines
  // @Description: Number of fixed-frequency sines, spaced evenly on a log scale from MID_F_MIN to MID_F_MAX.
  // @Range: 1 20
  // @User: Advanced
--]]
local MID_SIN_NUM = bind_add_param('SIN_NUM', 22, 4)

--[[
  // @Param: MID_SIN_TIME
  // @DisplayName: Motor ID sine length
  // @Description: Time at each fixed frequency.
  // @Units: s
  // @Range: 1 60
  // @User: Advanced
--]]
local MID_SIN_TIME = bind_add_param('SIN_TIME', 23, 8)

--[[
  // @Param: MID_PWM_MAX
  // @DisplayName: Motor ID throttle limit
  // @Description: The script never commands more than this, whatever the plan says.
  // @Units: PWM
  // @Range: 1000 2000
  // @User: Standard
--]]
local MID_PWM_MAX = bind_add_param('PWM_MAX', 24, 1900)

-- test ids (also written to the log)
local TEST_GAP, TEST_STEPS, TEST_COAST, TEST_SMALL_SWEEP, TEST_BIG_SWEEP, TEST_SINES = 0, 1, 2, 3, 4, 5

local UPDATE_MS = 2         -- reschedule interval; the scheduler gives ~430 Hz
local OVERRIDE_MS = 50      -- output override timeout: if the script stops, the output reverts
local TAIL_S = 3            -- zero throttle logged after the last test (coast-down)

-- -------------------------------------------------------------------------
-- building the test plan
--
-- The plan is a list of segments {id, t0, dur, fn}: fn(t) returns the
-- throttle (us) and the excitation frequency (Hz) at time t (s) since the
-- start of that segment. It is built once when a run starts, from the
-- parameters at that moment.

local idle_pwm = 1000       -- zero throttle (MOT_PWM_MIN), set at run start

local function hold(pwm)
   return function(t) return pwm, 0 end
end

-- exponential sweep from f0 to f1 over T seconds. The phase is the exact
-- integral of the frequency, so update jitter does not distort the sweep.
local function sweep(base, amp, f0, f1, T)
   local k = math.log(f1 / f0)
   return function(t)
      local f, phase
      if math.abs(k) < 1e-6 then
         f = f0
         phase = 2 * math.pi * f0 * t
      else
         f = f0 * math.exp(k * t / T)
         phase = 2 * math.pi * f0 * T / k * (math.exp(k * t / T) - 1)
      end
      return math.floor(base + amp * math.sin(phase)), f
   end
end

local function sine(base, amp, f)
   return function(t)
      return math.floor(base + amp * math.sin(2 * math.pi * f * t)), f
   end
end

local function build_plan()
   local plan = {}
   local t = 0
   local function add(id, dur, fn)
      if dur > 0 then
         plan[#plan + 1] = {id = id, t0 = t, dur = dur, fn = fn}
         t = t + dur
      end
   end
   local mask = math.floor(MID_TESTS:get())
   local gap = MID_GAP:get()
   local f0, f1 = MID_F_MIN:get(), MID_F_MAX:get()

   if (mask & 1) ~= 0 then  -- throttle steps
      add(TEST_GAP, gap, hold(idle_pwm))
      local p = MID_STP_MIN:get()
      while p <= MID_STP_MAX:get() do
         add(TEST_STEPS, MID_STP_TIME:get(), hold(math.floor(p)))
         p = p + math.max(MID_STP_INC:get(), 1)
      end
   end
   if (mask & 2) ~= 0 then  -- coast-downs: run at CST_PWM, then cut to zero
      add(TEST_GAP, gap, hold(idle_pwm))
      for _ = 1, math.floor(MID_CST_NUM:get()) do
         add(TEST_COAST, MID_CST_TIME:get(), hold(math.floor(MID_CST_PWM:get())))
         add(TEST_COAST, MID_CST_TIME:get(), hold(idle_pwm))
      end
   end
   if (mask & 4) ~= 0 then  -- small sweep
      add(TEST_GAP, gap, hold(idle_pwm))
      add(TEST_SMALL_SWEEP, MID_SW_TIME:get(), sweep(MID_SS_BASE:get(), MID_SS_AMP:get(), f0, f1, MID_SW_TIME:get()))
   end
   if (mask & 8) ~= 0 then  -- big sweep
      add(TEST_GAP, gap, hold(idle_pwm))
      add(TEST_BIG_SWEEP, MID_SW_TIME:get(), sweep(MID_BS_BASE:get(), MID_BS_AMP:get(), f0, f1, MID_SW_TIME:get()))
   end
   if (mask & 16) ~= 0 then  -- fixed sines, log-spaced from F_MIN to F_MAX
      add(TEST_GAP, gap, hold(idle_pwm))
      local n = math.max(math.floor(MID_SIN_NUM:get()), 1)
      for i = 0, n - 1 do
         local f = f0
         if n > 1 then f = f0 * (f1 / f0) ^ (i / (n - 1)) end
         add(TEST_SINES, MID_SIN_TIME:get(), sine(MID_SIN_BASE:get(), MID_SIN_AMP:get(), f))
      end
   end
   if #plan > 0 then
      add(TEST_GAP, TAIL_S, hold(idle_pwm))  -- log the final coast-down
   end
   return plan, t
end

-- -------------------------------------------------------------------------
-- checks before a run

local function is_motor_function(func)
   return (func >= 33 and func <= 40) or (func >= 82 and func <= 85)
end

-- returns the servo function of the channel, or nil and a reason
local function check_output(chan)
   if chan < 1 or chan > 32 then
      return nil, string.format('MID_CHAN %d invalid', chan)
   end
   local func = param:get(string.format('SERVO%d_FUNCTION', chan))
   if func == nil then
      return nil, string.format('no SERVO%d_FUNCTION', chan)
   end
   func = math.floor(func)
   if not is_motor_function(func) then
      return nil, string.format('SERVO%d_FUNCTION %d is not a motor', chan, func)
   end
   return func
end

-- -------------------------------------------------------------------------
-- the run state machine: IDLE -> RUNNING -> DONE (until disarmed)

local IDLE, RUNNING, DONE = 0, 1, 2
local state = IDLE
local plan, total = nil, 0
local seg_index = 1
local t_start_us = nil
local chan = 1
local run_count = 0
local no_rpm_since = nil    -- time (s) since telemetry went missing, for a warning
local warned_rpm = false

-- GCS message; keep text short: MAVLink status text is 50 characters
local function say(severity, text)
   gcs:send_text(severity, 'MotorID: ' .. text)
end

local function start_run()
   chan = math.floor(MID_CHAN:get())
   local func, why = check_output(chan)
   if not func then
      say(MAV_SEVERITY.ERROR, why)
      return false
   end
   local prop = math.floor(MID_PROP:get())
   if prop < 0 then
      say(MAV_SEVERITY.ERROR, 'set MID_PROP first (0 off, 1 on)')
      return false
   end
   idle_pwm = math.floor(param:get('MOT_PWM_MIN') or 1000)
   if idle_pwm <= 0 then idle_pwm = 1000 end
   plan, total = build_plan()
   if #plan == 0 then
      say(MAV_SEVERITY.ERROR, 'MID_TESTS selects no tests')
      return false
   end
   run_count = run_count + 1
   local poles = math.floor(param:get('SERVO_BLH_POLES') or 0)
   -- context record: everything the analysis needs to interpret the run
   logger:write('MIDC', 'ver,run,prop,chan,func,poles,mask,pwmin,pwmax,gap,dur', 'BHhBHBHHHff',
                PLAN_VERSION, run_count, prop, chan, func, poles, math.floor(MID_TESTS:get()),
                idle_pwm, math.floor(MID_PWM_MAX:get()), MID_GAP:get(), total)
   logger:write('MIDP', 'st0,st1,sti,stt,cpwm,ct,cn,ssb,ssa,bsb,bsa,f0,f1,swt', 'HHHfHfBHHHHfff',
                math.floor(MID_STP_MIN:get()), math.floor(MID_STP_MAX:get()), math.floor(MID_STP_INC:get()),
                MID_STP_TIME:get(), math.floor(MID_CST_PWM:get()), MID_CST_TIME:get(), math.floor(MID_CST_NUM:get()),
                math.floor(MID_SS_BASE:get()), math.floor(MID_SS_AMP:get()),
                math.floor(MID_BS_BASE:get()), math.floor(MID_BS_AMP:get()),
                MID_F_MIN:get(), MID_F_MAX:get(), MID_SW_TIME:get())
   logger:write('MIDS', 'sinb,sina,sinn,sint', 'HHBf',
                math.floor(MID_SIN_BASE:get()), math.floor(MID_SIN_AMP:get()), math.floor(MID_SIN_NUM:get()),
                MID_SIN_TIME:get())
   seg_index = 1
   t_start_us = micros()
   no_rpm_since, warned_rpm = nil, false
   say(MAV_SEVERITY.INFO, string.format('run %d: %.0f s, out %d, prop %d', run_count, total, chan, prop))
   return true
end

-- one excitation step while running; returns false when the plan is finished
local function run_step()
   -- seconds since the start (wrap-safe: unsigned 32-bit difference)
   local t = (micros() - t_start_us):tofloat() * 1.0e-6
   if t >= total then
      return false
   end
   -- advance to the segment containing t
   while seg_index < #plan and t >= plan[seg_index].t0 + plan[seg_index].dur do
      seg_index = seg_index + 1
   end
   local seg = plan[seg_index]
   local pwm, f = seg.fn(t - seg.t0)
   -- safety clamp: never above MID_PWM_MAX, never below zero throttle
   pwm = math.floor(math.max(idle_pwm, math.min(pwm, MID_PWM_MAX:get())))
   SRV_Channels:set_output_pwm_chan_timeout(chan - 1, pwm, OVERRIDE_MS)

   local rpm = esc_telem:get_rpm(chan - 1)
   if rpm == nil then
      rpm = -1
      if no_rpm_since == nil then no_rpm_since = t end
      if not warned_rpm and pwm > idle_pwm and t - no_rpm_since > 2 then
         say(MAV_SEVERITY.WARNING, 'no ESC rpm telemetry, out ' .. chan)
         warned_rpm = true
      end
   else
      no_rpm_since = nil
   end
   logger:write('MIDT', 'test,pwm,f,rpm', 'BHff', seg.id, pwm, f, rpm)
   return true
end

local function update()
   local armed = arming:is_armed()
   local enabled = MID_ENABLE:get() == 1

   if state == RUNNING then
      if not armed then
         say(MAV_SEVERITY.WARNING, 'disarmed, run stopped')
         state = IDLE
      elseif not enabled then
         say(MAV_SEVERITY.WARNING, 'MID_ENABLE cleared, run stopped')
         state = DONE   -- stop overriding the output; wait for disarm
      elseif not run_step() then
         say(MAV_SEVERITY.INFO, string.format('run %d done, disarm now', run_count))
         state = DONE
      end
      return update, UPDATE_MS
   end

   if state == DONE then
      if not armed then state = IDLE end
      return update, 100
   end

   -- IDLE: start on arming
   if armed and enabled then
      if start_run() then
         state = RUNNING
         return update, UPDATE_MS
      end
      state = DONE   -- refused: do not retry until disarmed
   end
   return update, 100
end

gcs:send_text(MAV_SEVERITY.INFO, 'MotorID loaded')

return update, 1000
