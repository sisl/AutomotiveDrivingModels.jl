function _mod2pi2(x::Float64)
    val = mod2pi(x)
    if val > pi
        val -= 2pi
    end
    return val
end

#######################

immutable LaneBoundary
    style::Symbol # ∈ :solid, :broken, :double
    color::Symbol # ∈ :yellow, white
end
const NULL_BOUNDARY = LaneBoundary(:unknown, :unknown)

#######################

immutable SpeedLimit
    lo::Float64 # [m/s]
    hi::Float64 # [m/s]
end
const DEFAULT_SPEED_LIMIT = SpeedLimit(-Inf, Inf)

#######################