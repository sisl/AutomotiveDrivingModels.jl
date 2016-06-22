function _mod2pi2(x::Float64)
    val = mod2pi(x)
    if val > pi
        val -= 2pi
    end
    return val
end