"""
    LaneFollowingAccel
Longitudinal acceleration.
The resulting vehicle velocity is capped below at 0 (i.e. standstill). Negative velocities are not allowed.

# Fields
- `a::Float64` longitudinal acceleration [m/s^2]
"""
struct LaneFollowingAccel
    a::Float64
end

function propagate(veh::Entity{VehicleState,D,I}, action::LaneFollowingAccel, roadway::Roadway, ΔT::Float64) where {D,I}

    a_lon = action.a

    ds = vel(veh.state)

    ΔT² = ΔT*ΔT
    Δs = ds*ΔT + 0.5*a_lon*ΔT²

    v₂ = max(ds + a_lon*ΔT, 0.)  # no negative velocities

    roadind = move_along(posf(veh.state).roadind, roadway, Δs)
    posG = roadway[roadind].pos
    posF = Frenet(roadind, roadway, t=posf(veh.state).t, ϕ=posf(veh.state).ϕ)
    VehicleState(posG, posF, v₂)
end
