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
--  const_funcfact
--  scale_funcfact
--  square_funcfact
--  saw_funcfact
--  sin_funcfact
--  chirp_funcfact
--  CHIRP_G number
--  series_funcfact
--  add_noise_funcfact
--  add_delay_funcfac
--
-- Each of the funcfact methods will return another function. The series_funcfact() 
-- takes a list of elements that are used to generate a synthetic series of data points.
-- Here are examples that could be arguments to series_funcfact():
--
-- local TSERIES_RAMP_S = 8.0
-- local tseries_elements_ramp = {
--     { 0.5 * TSERIES_RAMP_S, ss.saw_funcfact(), 0.0,  0.25 },
--     { 1.0 * TSERIES_RAMP_S, ss.const_funcfact(1.0) },
--     { 1.0 * TSERIES_RAMP_S, ss.saw_funcfact(), 0.25, 0.75 },
--     { 1.0 * TSERIES_RAMP_S, ss.const_funcfact(-1.0) },
--     { 0.5 * TSERIES_RAMP_S, ss.saw_funcfact(), 0.75, 1.0 }
-- }

-- local TSERIES_SQUARE_S = 30.0
-- local tseries_elements_square = {
--     { TSERIES_SQUARE_S, ss.square_funcfact() }
-- }

-- local TSERIES_MINUS_SIN_S = 30.0
-- local tseries_elements_minus_sin = {
--     { TSERIES_MINUS_SIN_S, ss.scale_funcfact(-1.0, ss.sin_funcfact()) }
-- }

-- local TSERIES_COS_S = 15.0
-- local tseries_elements_cos = {
--     { TSERIES_COS_S, ss.const_funcfact(1.0) },
--     { TSERIES_COS_S, ss.sin_funcfact(), 0.25 }
-- }

-- local TSERIES_FLAT_S = 40.0
-- local tseries_elements_flat = {
--     { TSERIES_FLAT_S, ss.const_funcfact() }
-- }

-- local TSERIES_CHIRP_S = 20.0
-- local tseries_elements_chirp = {
--     { TSERIES_CHIRP_S, ss.const_funcfact() },
--     { TSERIES_CHIRP_S / (ss.CHIRP_G ^ 0), ss.chirp_funcfact() },
--     { TSERIES_CHIRP_S / (ss.CHIRP_G ^ 1), ss.chirp_funcfact() },
--     { TSERIES_CHIRP_S / (ss.CHIRP_G ^ 2), ss.chirp_funcfact() },
--     { TSERIES_CHIRP_S / (ss.CHIRP_G ^ 3), ss.chirp_funcfact() },
--     { TSERIES_CHIRP_S / (ss.CHIRP_G ^ 4), ss.chirp_funcfact() },
--     { TSERIES_CHIRP_S / (ss.CHIRP_G ^ 5), ss.chirp_funcfact() },
--     { TSERIES_CHIRP_S / (ss.CHIRP_G ^ 6), ss.chirp_funcfact() }
-- }
--



-- return a function of t
--  domain 0. .. 1.
--  range -1. .. 1.
local function const_funcfact(value)
    if value == nil then
        value = 0.0
    end
    return function(t)
        return value
    end
end

local function scale_funcfact(factor, pre_func)
    if not pre_func then
        return const_funcfact()
    end
    if factor == 1.0 then
        return pre_func
    end
    return function(t) return factor * pre_func(t) end
end

local function square_funcfact()
    return function(t)
        if t < 0.5 then
            return 1.0
        end
        return -1.0
    end
end

local function saw_funcfact()
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

local function sin_funcfact()
    return function(t)
        return math.sin(t * 2.0 * math.pi)
    end
end

local CHIRP_G = 1.5     -- default value for chirp_g

---@param chirp_g number  -- in a single chirp period the frequency increases from f0 to f0 * chirp_g
local function chirp_funcfact(chirp_g)
    if chirp_g == nil then
        chirp_g = CHIRP_G
    end
    local chirp_f0 = 2 / (chirp_g + 1)
    local chirp_c =  chirp_f0 * (chirp_g - 1) / 2.0
    return function(t)
        return math.sin(2.0 * math.pi * (chirp_c * t * t + chirp_f0 * t))
    end
end


-- each element of the elements table
--  time in seconds
--  function f(t) => domain 0.0 .. 1.0, range -1.0 .. 1.0
--  begin phase 0.0 - 1.0
--  end phase 0.0 - 1.0+ if nil: begin phase + 1.0
--
-- Return a function of t described by the elements table
--  amplitude
--  elements
-- NOTE: time or t is the independent variable to these functions. Any type of independent
--   variable also works. For example: it would be useful to use distance in the t arguemnt.
local function series_funcfact(amplitude_m, elements)
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

        if not e[IDX_DUR] or e[IDX_DUR] <= 0.0 then
            e[IDX_DUR] = 1.0
        end
        if not e[IDX_FUNC] then
            e[IDX_FUNC] = const_funcfact(0.0)
        end
        if e[IDX_PBEG] == nil then
            e[IDX_PBEG] = 0.0
        end
        if e[IDX_PEND] == nil then
            e[IDX_PEND] = e[IDX_PBEG] + 1.0
        end

        -- insure the phase beg/end is valid
        if e[IDX_PEND] - e[IDX_PBEG] < 1.0e-6 then
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

    local idx_curr = 1
    local t_base_s = -1

    local function func(t_now_s)
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
        local t_phase = math.fmod(t_element_s * e[IDX_PSCALE] + e[IDX_PBEG], 1.0)

        -- evaluate the function
        local value = e[IDX_FUNC](t_phase) * amplitude_m

        return t_cycle_s, value
    end

    return func, dur_total_s
end


local function identity_funcfact(pre_func)
    if pre_func then
        return pre_func
    end
    return function(m) return m end
end


local function add_noise_funcfact(mean, std_dev, rate_ops, callback_interval_ms, pre_func)

    -- Use the Box-Muller algorithm to generate normally distributed error that is added to the sample.
    local function box_muller_func(m)
        return m + mean + std_dev * math.sqrt(-2 * math.log(math.random())) * math.cos(2 * math.pi * math.random())
    end

    local function noise_funcfact()
        if mean == 0.0 and std_dev == 0.0 then
            return identity_funcfact(pre_func)
        end

        if std_dev == 0.0 and pre_func then
            return function(m) return pre_func(m) + mean end
        end

        if std_dev == 0.0 then
            return function(m) return m + mean end
        end

        if pre_func then
            return function(m) return box_muller_func(pre_func(m)) end
        end

        return box_muller_func
    end

    -- Just simple normally distributed noise
    if rate_ops == nil then
        return noise_funcfact()
    end

    -- Use poisson distribution to generate outliers
    if rate_ops == 0.0 then
        return identity_funcfact(pre_func)
    end

    -- Create a function to generate outliers
    local outlier_func = noise_funcfact()

    -- Rate of outlier events in a callback interval
    local rate_opi = rate_ops * callback_interval_ms / 1000.0

    -- Poisson probability of zero events in an interval - Poisson formula is just exp in this case
    local poisson_prob_zero = math.exp(-rate_opi)

    return function(m)
        -- Poisson probability of 1 or more events in this interval is 1-P(0)
        if math.random() > poisson_prob_zero then
            return outlier_func(m)
        end
        if pre_func then
            return pre_func(m)
        end
        return m
    end
end


local function add_delay_funcfac(delay_s, callback_interval_ms, pre_func)

    if delay_s == 0.0 or callback_interval_ms == 0 then
        return identity_funcfact(pre_func)
    end

    local delay_line = {}
    local delay_count = math.ceil(delay_s / callback_interval_ms * 1000.0)
    if delay_count <= 0 then
        return identity_funcfact(pre_func)
    end

    local next_idx = -1
    local function add_delay_func(m)
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

    if pre_func then
        return function(m) return add_delay_func(pre_func(m)) end
    end

    return add_delay_func
end


---@class SyntheticSignal
---@field const_funcfact fun(value: number|nil): fun(t: number): number
---@field scale_funcfact fun(factor: number, pre_func: fun(t: number): number): fun(t: number): number
---@field square_funcfact fun(): fun(t: number): number
---@field saw_funcfact fun(): fun(t: number): number
---@field sin_funcfact fun(): fun(t: number): number
---@field chirp_funcfact fun(chirp_g: number|nil): fun(t: number): number
---@field CHIRP_G number
---@field series_funcfact fun(amplitude_m: number, elements: table): fun(x: number): number, number
---@field add_noise_funcfact fun(mean: number, std_dev: number, rate_ops: number|nil, callback_period_ms: number|nil, pre_func: nil|fun(x:number): number): fun(x: number): number
---@field add_delay_funcfac fun(delay_s: number, callback_interval_ms: number, pre_func: nil|fun(x:number): number): fun(x: number): number

---@type SyntheticSignal
local ss = {
    const_funcfact = const_funcfact,
    scale_funcfact = scale_funcfact,
    square_funcfact = square_funcfact,
    saw_funcfact = saw_funcfact,
    sin_funcfact = sin_funcfact,
    chirp_funcfact = chirp_funcfact,
    CHIRP_G = CHIRP_G,
    series_funcfact = series_funcfact,
    add_noise_funcfact = add_noise_funcfact,
    add_delay_funcfac = add_delay_funcfac,
}

return ss
