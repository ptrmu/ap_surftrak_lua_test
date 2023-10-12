-- range_hold_test.lua
-- Test the RangeHold mode in ArduSub.
-- Peter Mullen October 2023
--
-- This file uses EmmyLua Annotattions
-- https://luals.github.io/wiki/annotations/
-- Too many annotations can make the code unreadable. They are used where they help with
-- code completion, not necessarily with type safety.
--
-- This test uses a state machine to drive a sub over an undulating bottom in RangeHold mode. 
-- The sub is configured to use a LUA Range Finder driver that is also implemented in this file.
-- The test can compare the actual height of the sub above the bottom with the expected height and
-- determine if the RangeHold mode is working.
--
-- For this test, the bottom pattern is radial around the sub starting point. It doesn't matter which 
-- direction the sub moves. The following Parameters influence how the test is run:
--  SCR_USER1 is a code for which synthetic bottom to generate.
--  SCR_USER2 is the average bottom depth in meters
--  SCR_USER3 is the max distance in meters the sub can get from the desired depth and pass the test.


local TEST_NAME = "RNGHLD"
local UPDATE_PERIOD_MS = 20


math.randomseed(micros():toint())


-------------------------------------------------------------------------------

--#region GCS messaging

---@param str1 string
local function gcs_send(str1)
    gcs:send_text(6, string.format("%s: %s", TEST_NAME, str1))
end

local gcs_send_trim = (function()
    local gcs_send_times = {}
    local gcs_eaten_count = {}
    local GCS_SEND_PERIOD_S = 1.0

    ---@param str1 string
    ---@param str2 string
    local function func(str1, str2)
        if not str1 then return end

        local time_curr_s = millis():tofloat() / 1000.0
        local time_first_s = gcs_send_times[str1]
        if (time_first_s) then
            local dur_since_first = time_curr_s - time_first_s
            if dur_since_first < GCS_SEND_PERIOD_S then
                if not gcs_eaten_count[str1] then
                    gcs_eaten_count[str1] = 0
                end
                gcs_eaten_count[str1] = gcs_eaten_count[str1] + 1
                return
            end
        end

        local send_str = nil
        local eaten_count = gcs_eaten_count[str1]
        if eaten_count then
            gcs_eaten_count[str1] = nil
            if str2 then
                send_str = string.format("%s %s (+%i)", str1, str2, eaten_count)
            else
                send_str = string.format("%s (+%i)", str1, eaten_count)
            end
        else
            if str2 then
                send_str = string.format("%s %s", str1, str2)
            else
                send_str = string.format("%s", str1)
            end
        end

        gcs_send(send_str)
        gcs_send_times[str1] = time_curr_s
    end

    return func
end)()


--#endregion
-------------------------------------------------------------------------------

--region Classes

-- An instance of the Context Class is used to communicate between different parts
-- of this test. In particular, it carries data between the virtual range finder
-- and the state machine that is running the test.
---@class Context
---@field dur_script_s number
---@field dur_state_s number
---@field pos_curr Location_ud|nil
---@field sub_z_m number
---@field bottom_z_m number
---@field true_rngfnd_m number
---@field rngfnd_m number
---@field synsig_id number
---@field bottom_depth_m number
---@field match_tolerance_m number
---@field signal_period_m number

---@class SyntheticSignal
---@field series_factory fun(amplitude_m: number, elements: table[]): fun(x: number): number,number
---@field add_noise_factory fun(mean: number, std_dev: number): fun(x: number): number
---@field add_outlier_factory fun(rate_ops: number, callback_period_ms: number, mean: number, std_dev: number): fun(x: number): number
---@field add_delay_factory fun(delay_s: number, callback_interval_ms: number): fun(x: number): number
---@field TOTAL_PERIOD_S number
---@field CHIRP_G number


---@class Synsig
---@field depth_variation_func fun(x: number): number,number
---@field add_noise_func fun(x: number): number
---@field add_outlier_func fun(x: number): number
---@field add_delay_func fun(x: number): number


--#endregion
-------------------------------------------------------------------------------

--region Sub control

-- NOTE: QGC sends continuous RC_override mavlink messages. These overwhelm
-- the override commands from this script. Disable the joystick in QGC while
-- running this script

local APPROX_SPEED_UPDOWN_MPS = 0.5
local APPROX_SPEED_FORWARD_MPS = 1.0

local set_velocity = (function()
    local joystick_channels = {
        pitch = 1,
        roll = 2,
        throttle = 3,
        yaw = 4,
        forward = 5,
        lateral = 6,
    }

    local function set_vel(chan, pwm)
        rc:get_channel(chan):set_override(pwm)
    end

    return {
        up = function() set_vel(joystick_channels.throttle, 1700) end,
        down = function() set_vel(joystick_channels.throttle, 1300) end,
        forward = function() set_vel(joystick_channels.forward, 1700) end,
        none = function()
            set_vel(joystick_channels.throttle, 1500)
            set_vel(joystick_channels.forward, 1500)
        end,
    }
end)()


--#endregion
-------------------------------------------------------------------------------

--region Virtual range finder driver

-- The sub must be configured to use this driver with the parameter:
--  RNGFND1 = 36
-- Once this driver is initialized it is called every time in the update loop.
-- This driver is initialized in the first state of the test state machine.
-- The synthetic_signal routines generate a synthetic bottom which is combined
-- with the sub depth to get a virtual range measurement. This measument is 
-- corrupted with noise and outliers and delayed before it is sent to ArduPilot.

---@param ctx Context
local function get_synthetic_signal(ctx)

    ---@type SyntheticSignal
    local ss = require("synthetic_signal")

    -- query SCR_USERx for parameters
    -- SCR_USER1 is a code for which synthetic bottom to generate
    local synsig_id = param:get('SCR_USER1')
    if not synsig_id or synsig_id <= 0.0 then
        synsig_id = 1
    end
    -- SCR_USER2 is the bottom depth
    local bottom_depth_m = param:get('SCR_USER2')
    if not bottom_depth_m or math.abs(bottom_depth_m) < 0.1 then
        bottom_depth_m = 22.0
    end
    -- SCR_USER3 is the sub depth tolerance for a successful test.
    local match_tolerance_m = param:get('SCR_USER3')
    if not match_tolerance_m or math.abs(match_tolerance_m) < 0.0001 then
        match_tolerance_m = 2.0
    end


    local SERIES_RAMP_M = 10.0
    local series_elements_ramp = {
        { 0.5 * SERIES_RAMP_M, "mid" },
        { 0.5 * SERIES_RAMP_M, "saw", 0.0,  0.25 },
        { 1.0 * SERIES_RAMP_M, "max" },
        { 1.0 * SERIES_RAMP_M, "saw", 0.25, 0.75 },
        { 1.0 * SERIES_RAMP_M, "min" },
        { 0.5 * SERIES_RAMP_M, "saw", 0.75, 1.0 },
        { 0.5 * SERIES_RAMP_M, "mid" },
    }


    ---@type Synsig
    local synsig

    if synsig_id == 2 then
        synsig = {
            depth_variation_func = ss.series_factory(4.0, series_elements_ramp),
            add_noise_func = ss.add_noise_factory(0.0, 0.25),
            add_outlier_func = ss.add_outlier_factory(0.5, UPDATE_PERIOD_MS, 8.0, 2.0),
            add_delay_func = ss.add_delay_factory(0.1, UPDATE_PERIOD_MS),
        }
    else
        synsig = {
            depth_variation_func = ss.series_factory(4.0, series_elements_ramp),
            add_noise_func = ss.add_noise_factory(0.0, 0.0),
            add_outlier_func = ss.add_outlier_factory(0.0, UPDATE_PERIOD_MS, 10.0, 4.0),
            add_delay_func = ss.add_delay_factory(0.0, UPDATE_PERIOD_MS),
        }
    end

    -- Save parameters to the context
    ctx.synsig_id = synsig_id
    ctx.bottom_depth_m = bottom_depth_m
    ctx.match_tolerance_m = match_tolerance_m
    ctx.signal_period_m = ss.TOTAL_PERIOD_S

    return synsig
end


-- Need to specify which ArduPilot Range finder to use for this virtual test
local rndfnd_sensor_number = 1
local LUA_DRIVER_CODE = 36 -- parameter number for lua rangefinder


---@param lua_driver_backend AP_RangeFinder_Backend_ud
---@param pos_origin Location_ud
---@param synsig Synsig
---@return function(ctx: Context)
local function rngfnd_func_factory(lua_driver_backend, pos_origin, synsig)
    return function(ctx)
        -- Note if we were not able to get a position this time and don't create a measurement
        if not ctx.pos_curr then
            gcs_send_trim("rngfnd_func could not generate a reading.", "No position was available.")
            return
        end

        -- Generate a seafloor depth
        local dist_traveled_m = pos_origin:get_distance(ctx.pos_curr)
        local x_series, delta_m = synsig.depth_variation_func(dist_traveled_m)
        local bottom_z_m = delta_m - ctx.bottom_depth_m

        -- Generate the true rangefinder measurement
        local sub_z_m = ctx.pos_curr:alt() / 100.0
        local rngfnd_true_m = sub_z_m - bottom_z_m

        -- Corrupt the measurment
        local rngfnd_noise_m = synsig.add_noise_func(rngfnd_true_m)
        local rngfnd_outlier_m = synsig.add_outlier_func(rngfnd_noise_m)
        local rngfnd_m = synsig.add_delay_func(rngfnd_outlier_m)

        -- Return a measurement to ArduPilot
        if rngfnd_m > 0 then
            local sent_successfully = lua_driver_backend:handle_script_msg(rngfnd_m)
            if not sent_successfully then
                -- This should never happen as we already checked for a valid configured lua backend above
                gcs_send_trim("LUA Range Finder Driver error", "handle_script_msg() was not sent")
            end

            -- Fill in range finder values so the test can verify them
            ctx.sub_z_m = sub_z_m
            ctx.bottom_z_m = bottom_z_m
            ctx.true_rngfnd_m = rngfnd_true_m
            ctx.rngfnd_m = rngfnd_m
        end

        -- gcs_send(string.format("%.2f, %.2f, %.2f, %.2f", sub_z_m, bottom_z_m, rngfnd_true_m, rngfnd_m))
        logger:write('RNFN', 'sub_z,bottom_z,true_rngfnd,rngfnd', 'ffff', 'mmmm', '----', sub_z_m, bottom_z_m,
            rngfnd_true_m, rngfnd_m)
    end
end


---@type function(ctx: Context)
local rngfnd_func = nil

---@param ctx Context
local function rngfnd_init(ctx)
    -- Get a backend driver
    local lua_driver_backend = nil

    local sensor_count = rangefinder:num_sensors() -- number of sensors connected
    for j = 0, sensor_count - 1 do
        if j + 1 == rndfnd_sensor_number then
            local device = rangefinder:get_backend(j)
            if (device and (device:type() == LUA_DRIVER_CODE)) then
                -- this is a lua driver for a back end
                lua_driver_backend = device
                break
            end
        end
    end

    if not lua_driver_backend then
        gcs_send_trim("No LUA range finder driver configured.", "For esample: set param RNGFND1_TYPE to 36")
        return
    end

    -- Get the test start location
    if not ctx.pos_curr then
        gcs_send_trim("Could not get a position from the AHRS.", "Will keep trying")
        return
    end

    -- Get the object that generates the synthetic signal
    local synsig = get_synthetic_signal(ctx)

    rngfnd_func = rngfnd_func_factory(lua_driver_backend, ctx.pos_curr:copy(), synsig)

    gcs_send("Range Finder Initialized.")
end


--#endregion
-------------------------------------------------------------------------------

--#region Test state machine

-- This is a linear state machine. The states are listed in a table and are executed
-- in the order in that table. The success and failure termination states are not in
-- the table and are entered based on the value returned from the state funcs. The
-- SRET_XXX variables define these possible return values.
--
-- Each state is implemented as a factory which return a function that is repeatedly
-- executed. Initialization for the state happens in the factory code and clean up
-- for the state happens when the repeat function decides to exit the state.
--

local SRET_SAME = 1         -- Execute this state again.
local SRET_NEXT = 2         -- Execute the next state.
local SRET_DONE_FAILURE = 3 -- An error occured, go to the failure termination state.


local TEST_RANGE_1 = 8.0
local TEST_RANGE_2 = 14.0


---@param ctx_setup Context
local function adjust_depth_factory(ctx_setup, range_idx)
    local ctx = ctx_setup
    local DEPTH_MATCH_TOL_M = 0.25

    local test_range = TEST_RANGE_1
    if range_idx > 1 then
        test_range = TEST_RANGE_2
    end

    local sub_depth_target_m = ctx_setup.bottom_depth_m - test_range
    local sub_depth_curr_m = -ctx_setup.pos_curr:alt() / 100.0
    local DEPTH_ADJUST_TIMEOUT_S = 1.5 * math.abs(sub_depth_target_m - sub_depth_curr_m) / APPROX_SPEED_UPDOWN_MPS

    local function clean_up()
        set_velocity.none()
    end

    return function()
        -- Determine if we need to move up or down.
        local delta = sub_depth_target_m + ctx.pos_curr:alt() / 100.0
        if delta < 0 then
            set_velocity.up()
        else
            set_velocity.down()
        end

        if math.abs(delta) < DEPTH_MATCH_TOL_M then
            gcs_send(string.format("Goto Depth Done = %.2fm", sub_depth_target_m))
            clean_up()
            return SRET_NEXT
        end

        if ctx.dur_state_s > DEPTH_ADJUST_TIMEOUT_S then
            gcs_send(string.format("Goto Depth timout. Did not achieve %.2fm. ?Is QGC Joystick disabled?", sub_depth_target_m))
            clean_up()
            return SRET_DONE_FAILURE
        end

        gcs_send_trim(string.format("Goto Depth %.2fm,", sub_depth_target_m),
            string.format("%.2fm, %.2fs", delta, ctx.dur_state_s))
        return SRET_SAME
    end
end


local function follow_bottom_factory(ctx_setup, target_distance_m)
    local ctx = ctx_setup
    local FOLLOW_DISTANCE_TIMEOUT_S = 1.5 * target_distance_m / APPROX_SPEED_FORWARD_MPS
    local FOLLOW_BOTTOM_MATCH_TOL_M = ctx.match_tolerance_m
    local pos_start = ctx.pos_curr
    local rngfnd_start_m = ctx.true_rngfnd_m
    local range_delta_max = 0.0

    local function clean_up()
        set_velocity.none()
    end

    return function()
        -- Drive forward
        set_velocity.forward()

        -- Calculate how far we have traveled
        local distance_traveled_m = pos_start:get_distance(ctx.pos_curr)
        local delta = target_distance_m - distance_traveled_m

        -- Calculate how far the sub is from maintaining a constant distance from the bottom.
        local range_delta = math.abs(ctx.true_rngfnd_m - rngfnd_start_m)
        if range_delta > range_delta_max then
            range_delta_max = range_delta
        end

        if delta <= 0.0 then
            gcs_send(string.format("Follow bottom complete. Distance traveled = %.2fm, Max range delta = %.2f",
                distance_traveled_m, range_delta_max))
            clean_up()
            return SRET_NEXT
        end

        if ctx.dur_state_s > FOLLOW_DISTANCE_TIMEOUT_S then
            gcs_send(string.format("Follow bottom timout. Did not achieve distance %.2fm.", target_distance_m))
            clean_up()
            return SRET_DONE_FAILURE
        end

        if math.abs(ctx.true_rngfnd_m - rngfnd_start_m) > FOLLOW_BOTTOM_MATCH_TOL_M then
            gcs_send(string.format("RangeHold mode failure. Did not follow the bottom by %.2fm.",
                ctx.true_rngfnd_m - rngfnd_start_m))
            clean_up()
            return SRET_DONE_FAILURE
        end

        gcs_send_trim(string.format("FB %.2fm,", target_distance_m),
            string.format("%.2fm, %.2fs, %.2f", delta, ctx.dur_state_s, range_delta))
        return SRET_SAME
    end
end


local function cleanup_test()
    set_velocity.none()
    vehicle:set_mode(19)
    arming:disarm()
end


local function state_pause_factory_factory(dur_pause_s)
    return function(ctx_setup)
        local ctx = ctx_setup
        return function()
            gcs_send_trim(string.format("Pausing for %.2fs,", dur_pause_s),
                string.format("%.2fs to go", dur_pause_s - ctx.dur_state_s))
            if ctx.dur_state_s < dur_pause_s then
                return SRET_SAME
            end
            return SRET_NEXT
        end
    end
end


local function state_init_rngfnd_factory(ctx_setup)
    ---@type Context
    local ctx = ctx_setup
    return function()
        -- Try to initialize the virtual range finder
        rngfnd_init(ctx)

        -- The non-nil func variable indicates that initialization was succsessful.
        if rngfnd_func then
            gcs_send(string.format("Starting test. synsig_id:%.0f, bottom_depth:%.2fm, tolerance:%.2fm, bottom pattern length:%.2fm",
                ctx.synsig_id, ctx.bottom_depth_m, ctx.match_tolerance_m, ctx.signal_period_m))
            return SRET_NEXT
        end

        -- Timeout and return error if initialization doesn't happen.
        if ctx.dur_state_s > 10.0 then
            return SRET_DONE_FAILURE
        end

        -- Try again the next time.
        return SRET_SAME
    end
end


local function state_arm_factory(ctx_setup)
    -- Try to arm
    arming:arm()

    return function()
        if arming:is_armed() then
            gcs_send("Arm success.")
            return SRET_NEXT
        end

        gcs_send("Arm failure.")
        return SRET_DONE_FAILURE
    end
end


local function state_descend_to_test_depth_factory(ctx_setup)
    return adjust_depth_factory(ctx_setup, 1)
end


local function state_range_hold_mode_factory(ctx_setup)
    vehicle:set_mode(21)
    return function()
        if vehicle:get_mode() == 21 then
            gcs_send("RANGEHOLD mode enabled")
            return SRET_NEXT
        end
        gcs_send("RangeHold mode was not enabled")
        return SRET_DONE_FAILURE
    end
end


local function state_follow_bottom_1_factory(ctx_setup)
    return follow_bottom_factory(ctx_setup, ctx_setup.signal_period_m)
end


local function state_change_depth_factory(ctx_setup)
    return adjust_depth_factory(ctx_setup, 2)
end


local function state_follow_bottom_2_factory(ctx_setup)
    return follow_bottom_factory(ctx_setup, ctx_setup.signal_period_m)
end


local function state_manual_mode_factory(ctx_setup)
    vehicle:set_mode(19)
    return function()
        return SRET_NEXT
    end
end


local function state_disarm_factory(ctx_setup)
    arming:disarm()
    return function()
        return SRET_NEXT
    end
end


local function state_done_success_factory(ctx_setup)
    gcs_send("** Complete ** SUCCESS!!")
    return function()
        return SRET_SAME
    end
end

local function state_done_failure_factory(ctx_setup)
    cleanup_test()
    gcs_send("** Complete ** FAILURE!!")
    return function()
        return SRET_SAME
    end
end


local state_factories = {
    state_init_rngfnd_factory,
    state_arm_factory,
    state_descend_to_test_depth_factory,
    state_pause_factory_factory(5.),
    state_range_hold_mode_factory,
    state_pause_factory_factory(5.),
    state_follow_bottom_1_factory,
    state_change_depth_factory,
    state_pause_factory_factory(5.),
    state_follow_bottom_2_factory,
    state_pause_factory_factory(5.),
    state_manual_mode_factory,
    state_disarm_factory,
}


local time_script_s = millis():tofloat() / 1000.0
local time_state_s = time_script_s
local state_curr_idx = 1
local init_state = true


---@type Context
local context = {
    dur_script_s = 0.0,
    dur_state_s = 0.0,
    pos_curr = nil,
    sub_z_m = 0.0,
    bottom_z_m = 0.0,
    true_rngfnd_m = 0.0,
    rngfnd_m = 0.0,
    synsig_id = 0,
    bottom_depth_m = 0.0,
    match_tolerance_m = 0.0,
    signal_period_m = 0.0
}

---@type function()
local state_func = nil

local function update()
    -- Set the current time
    local time_curr_s = millis():tofloat() / 1000.0
    context.dur_script_s = time_curr_s - time_script_s
    context.pos_curr = ahrs:get_position()

    -- Do the virtual Range Finder driver function always.
    if rngfnd_func then
        rngfnd_func(context)
    end

    -- Prepare for executing the state.
    --  step_curr_idx < 0 => in a done state, Don't move from this state.
    --  Success happens automatically when the last step returns SRET_NEXT
    --  Initialize the state if init_state is true
    if state_curr_idx > 0 then
        if state_curr_idx > #state_factories then
            state_func = state_done_success_factory(context)
            state_curr_idx = -1
        elseif init_state then
            state_func = state_factories[state_curr_idx](context)
            init_state = false
            time_state_s = time_curr_s
        end
    end

    -- Set the state duration
    context.dur_state_s = time_curr_s - time_state_s

    -- Execute the current state.
    local sret = state_func()

    -- Move to the next state
    if sret == SRET_NEXT then
        state_curr_idx = state_curr_idx + 1
        init_state = true

        -- Anything besides SRET_SAME -> An error occured transition to the last state.
    elseif sret == SRET_DONE_FAILURE then
        state_func = state_done_failure_factory(context)
        state_curr_idx = -1
    end

    return update, UPDATE_PERIOD_MS
end

--#endregion

gcs:send_text(0, string.format("Loaded range_hold_test.lua"))

return update()
