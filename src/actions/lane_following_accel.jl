"""
    LaneFollowingAccel
Longitudinal acceleration

# Fields
- `a::Float64` longitudinal acceleration [m/s^2]
"""
struct LaneFollowingAccel
    a::Float64
end

function propagate(veh::Vehicle1D, action::LaneFollowingAccel, roadway::StraightRoadway, Δt::Float64)

    a = action.a
    s, v = veh.state.s, veh.state.v

    s′ = s + v*Δt + a*Δt*Δt/2
    v′ = v + a*Δt

    s′ = mod_position_to_roadway(s′, roadway)

    return State1D(s′, v′)
end

function propagate(veh::Entity{VehicleState,D,I}, action::LaneFollowingAccel, roadway::Roadway, ΔT::Float64) where {D,I}

    a_lon = action.a

    ds = veh.state.v

    ΔT² = ΔT*ΔT
    Δs = ds*ΔT + 0.5*a_lon*ΔT²

    v₂ = ds + a_lon*ΔT

    roadind = move_along(veh.state.posF.roadind, roadway, Δs)
    posG = roadway[roadind].pos
    posF = Frenet(roadind, roadway, t=veh.state.posF.t, ϕ=veh.state.posF.ϕ)
    VehicleState(posG, posF, v₂)
end


#XXX these should probably be removed
# Base.show(io::IO, a::LaneFollowingAccel) = @printf(io, "LaneFollowingAccel(%6.3f)", a.a)
# Base.length(::Type{LaneFollowingAccel}) = 1
# Base.convert(::Type{LaneFollowingAccel}, v::Vector{Float64}) = LaneFollowingAccel(v[1])

# function Base.copyto!(v::Vector{Float64}, a::LaneFollowingAccel)
#     v[1] = a.a
#     v
# end
