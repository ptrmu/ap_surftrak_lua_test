-- synthetic_signal.lua
-- Test the RangeHold mode in ArduSub.
-- Peter Mullen October 2023
--
-- This file uses EmmyLua Annotattions
-- https://luals.github.io/wiki/annotations/
-- Too many annotations can make the code unreadable. They are used where they help with
-- code completion, not necessarily with type safety.
--
-- This file is meant to be loaded with the "require" statement. The require statement
-- will return a table with the following elements
--  series_factory(amplitude_m, elements)
--  add_noise_factory(mean, std_dev)
--  add_outlier_factory(rate_ops, callback_interval_ms, mean, std_dev)
--  add_delay_factory(delay_s, callback_interval_ms)
--  CHIRP_G
--  TOTAL_PERIOD_S
--
-- Each of the factory methods will return another function that can be called
-- to generate a synthetic measurement. The series_factory() function takes a
-- list of elements that are used to generate a synthetic series of data points.
-- Here are examples that could be arguments to series_factory():
--
-- local TSERIES_ALL_S = 4.0
-- local tseries_elements = {
--     { TSERIES_ALL_S, "mid" },
--     { TSERIES_ALL_S, "max" },
--     { TSERIES_ALL_S, "mid" },
--     { TSERIES_ALL_S, "saw" },
--     { TSERIES_ALL_S, "square" },
--     { TSERIES_ALL_S, "sin" },
--     { TSERIES_ALL_S, "chirp" }
-- }

-- local TSERIES_RAMP_S = 8.0
-- local tseries_elements_ramp = {
--     { 0.5 * TSERIES_RAMP_S, "saw", 0.0,  0.25 },
--     { 1.0 * TSERIES_RAMP_S, "max" },
--     { 1.0 * TSERIES_RAMP_S, "saw", 0.25, 0.75 },
--     { 1.0 * TSERIES_RAMP_S, "min" },
--     { 0.5 * TSERIES_RAMP_S, "saw", 0.75, 1.0 }
-- }

-- local TSERIES_SQUARE_S = 30.0
-- local tseries_elements_square = {
--     { TSERIES_SQUARE_S, "square" }
-- }

-- local TSERIES_FLAT_S = 40.0
-- local tseries_elements_flat = {
--     { TSERIES_FLAT_S, "mid" }
-- }

-- local TSERIES_CHIRP_S = 20.0
-- local tseries_elements_chirp = {
--     { TSERIES_CHIRP_S, "mid" },
--     { TSERIES_CHIRP_S / (ss.CHIRP_G ^ 0), "chirp" },
--     { TSERIES_CHIRP_S / (ss.CHIRP_G ^ 1), "chirp" },
--     { TSERIES_CHIRP_S / (ss.CHIRP_G ^ 2), "chirp" },
--     { TSERIES_CHIRP_S / (ss.CHIRP_G ^ 3), "chirp" },
--     { TSERIES_CHIRP_S / (ss.CHIRP_G ^ 4), "chirp" },
--     { TSERIES_CHIRP_S / (ss.CHIRP_G ^ 5), "chirp" },
--     { TSERIES_CHIRP_S / (ss.CHIRP_G ^ 6), "chirp" }
-- }

local ss = {}

local CHIRP_G = 1.5     -- in a single chirp period the frequency increases from f0 to f0 * CHIRP_G
local CHIRP_F0 = 2 / (CHIRP_G + 1)
local CHIRP_C =  CHIRP_F0 * (CHIRP_G - 1) / 2.0

ss.CHIRP_G = CHIRP_G
ss.TOTAL_PERIOD_S = 0.0


-- return a function of t
--  domain 0. .. 1.
--  range -1. .. 1.
local function series_func(series_type)
    if series_type == "max" then
        return function(t)
            return 1.0
        end
    end

    if series_type == "min" then
        return function(t)
            return -1.0
        end
    end

    if series_type == "sin" then
        return function(t)
            return math.sin(t * 2.0 * math.pi)
        end
    end

    -- square wave
    if series_type == "square" then
        return function(t)
            if t < 0.5 then
                return 1.0
            end
            return -1.0
        end
    end

    if series_type == "saw" then
        return function(t)
            if t < 0.25 then
                return 4.0 * t
            end
            if t < 0.75 then
                return -4.0 * t + 2.0
            end
            return 4.0 * t - 4.0
        end
    end

    if series_type == 'chirp' then
        return function(t)
            return math.sin(2.0 * math.pi * (CHIRP_C * t * t + CHIRP_F0 * t))
        end
    end

    return function(t)
        return 0.0
    end
end


-- each element of the elements table
--  time in seconds
--  function 'min', 'max', 'sin', 'square', 'saw', 'chirp'
--  begin phase 0.0 - 1.0
--  end phase 0.0 - 1.0
--
-- Return a function of t described by the elements table
--  amplitude
--  elements
-- NOTE: time or t is the independent variable to these functions. Any type of independent
--   variable also works. For example: it would be useful to use distance the t arguemnt.
function ss.series_factory(amplitude_m, elements)
    -- indicies into an element
    local IDX_DUR = 1
    local IDX_FUNC = 2
    local IDX_PBEG = 3
    local IDX_PEND = 4
    local IDX_TBEG = 5
    local IDX_TEND = 6
    local IDX_PSCALE = 7

    -- fill in default values for missing arguments
    -- Find total duration
    local dur_total_s = 0.0

    for i, e in ipairs(elements) do
        e[IDX_FUNC] = series_func(e[IDX_FUNC])

        if not e[IDX_DUR] or e[IDX_DUR] <= 0 then
            e[IDX_DUR] = 1.0
        end
        if not e[IDX_FUNC] then
            e[IDX_FUNC] = series_func("")
        end
        if not e[IDX_PBEG] then
            e[IDX_PBEG] = 0.0
        end
        if not e[IDX_PEND] then
            e[IDX_PEND] = 1.0
        end

        -- insure the phase beg/end is valid
        if e[IDX_PBEG] < 0 or e[IDX_PEND] - e[IDX_PBEG] < 1.0e-6 or e[IDX_PEND] > 1.0 then
            e[IDX_PBEG] = 0.0
            e[IDX_PEND] = 1.0
        end

        -- pre-calc the phase scale
        e[IDX_PSCALE] = (e[IDX_PEND] - e[IDX_PBEG]) / e[IDX_DUR]

        -- accumulate total time
        e[IDX_TBEG] = dur_total_s
        dur_total_s = dur_total_s + e[IDX_DUR]
        e[IDX_TEND] = dur_total_s
    end

    ss.TOTAL_PERIOD_S = dur_total_s

    local idx_curr = 1
    local t_base_s = -1

    return function(t_now_s)
        -- Initialize on the first invocation
        if t_base_s < 0 then
            t_base_s = t_now_s
        end

        -- determine the time in this cycle
        local t_cycle_s = math.fmod(t_now_s - t_base_s, dur_total_s)

        -- search for the element responsible for this time in the cycle.
        if t_cycle_s < elements[idx_curr][IDX_TBEG] then
            idx_curr = 1
        end
        while not (t_cycle_s < elements[idx_curr][IDX_TEND]) do
            idx_curr = idx_curr + 1
            assert(idx_curr <= #elements, "elements index out of range")
        end

        -- use this element for evaluating the function
        local e = elements[idx_curr]
        local t_element_s = t_cycle_s - e[IDX_TBEG]

        -- adjust the phase
        local t_phase = t_element_s * e[IDX_PSCALE] + e[IDX_PBEG]

        -- evaluate the function
        local value = e[IDX_FUNC](t_phase) * amplitude_m

        return t_cycle_s, value
    end
end


local function identity_func()
    return function(m) return m end
end


function ss.add_noise_factory(mean, std_dev)
    if mean == 0.0 and std_dev == 0.0 then
        return identity_func()
    end

    if std_dev == 0.0 then
        return function(m) return m + mean end
    end

    -- Use the Box-Muller algorithm to generate normally distributed error that is added to the sample.
    return function(m)
        return m + mean + std_dev * math.sqrt(-2 * math.log(math.random())) * math.cos(2 * math.pi * math.random())
    end
end


function ss.add_outlier_factory(rate_ops, callback_interval_ms, mean, std_dev)
    if rate_ops == 0.0 then
        return identity_func()
    end

    -- Create a function to generate outliers
    local outlier_func = ss.add_noise_factory(mean, std_dev)

    -- Rate of outlier events in a callback interval
    local rate_opi = rate_ops * callback_interval_ms / 1000.0

    -- Poisson probability of zero events in an interval - Poisson formula is just exp in this case
    local poisson_prob_zero = math.exp(-rate_opi)

    return function(m)
        -- Poisson probability of 1 or more events in this interval is 1-P(0)
        if math.random() > poisson_prob_zero then
            return outlier_func(m)
        end
        return m
    end
end


function ss.add_delay_factory(delay_s, callback_interval_ms)
 
    if delay_s == 0.0 or callback_interval_ms == 0 then
        return identity_func()
    end

    local delay_line = {}
    local delay_count = math.ceil(delay_s / callback_interval_ms * 1000.0)
    if delay_count <= 0 then
        return identity_func()
    end

    local next_idx = -1
    return function(m)
        if next_idx < 1 then
            for i = 1, delay_count do
                delay_line[i] = m
            end
            next_idx = 1
        end

        local m_delay = delay_line[next_idx]
        delay_line[next_idx] = m
        next_idx = next_idx + 1
        if next_idx > #delay_line then
            next_idx = 1
        end
        return m_delay
    end
end


return ss