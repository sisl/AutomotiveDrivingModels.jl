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

    ds = vel(veh.state)

    ΔT² = ΔT*ΔT
    Δs = ds*ΔT + 0.5*a_lon*ΔT²

    v₂ = ds + a_lon*ΔT

    roadind = move_along(posf(veh.state).roadind, roadway, Δs)
    posG = roadway[roadind].pos
    posF = Frenet(roadind, roadway, t=posf(veh.state).t, ϕ=posf(veh.state).ϕ)
    VehicleState(posG, posF, v₂)
end
