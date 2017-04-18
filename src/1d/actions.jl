"""
    LaneFollowingAccel
Longitudinal acceleration
"""
immutable LaneFollowingAccel
    a::Float64
end
Base.show(io::IO, a::LaneFollowingAccel) = @printf(io, "LaneFollowingAccel(%6.3f)", a.a)
Base.length(::Type{LaneFollowingAccel}) = 1
Base.convert(::Type{LaneFollowingAccel}, v::Vector{Float64}) = LaneFollowingAccel(v[1])
function Base.copy!(v::Vector{Float64}, a::LaneFollowingAccel)
    v[1] = a.a
    v
end

function propagate(veh::Vehicle1D, action::LaneFollowingAccel, roadway::StraightRoadway, Δt::Float64)

    a = action.a
    s, v = veh.state.s, veh.state.v

    s′ = s + v*Δt + a*Δt*Δt/2
    v′ = v + a*Δt

    s′ = mod_position_to_roadway(s′, roadway)

    return State1D(s′, v′)
end