"""
    LatLonAccel
Acceleration in the frenet frame

# Fields
- `a_lat::Float64` Lateral acceleration [m/s^2]
- `a_lon::Float64` Longitudinal acceleration [m/s^2]
"""
struct LatLonAccel
    a_lat::Float64
    a_lon::Float64
end
LatLonAccel(alat::Float64, alon::LaneFollowingAccel) = LatLonAccel(alat, alon.a)

Base.show(io::IO, a::LatLonAccel) = @printf(io, "LatLonAccel(%6.3f, %6.3f)", a.a_lat, a.a_lon)
Base.length(::Type{LatLonAccel}) = 2
Base.convert(::Type{LatLonAccel}, v::Vector{Float64}) = LatLonAccel(v[1], v[2])
function Base.copyto!(v::Vector{Float64}, a::LatLonAccel)
    v[1] = a.a_lat
    v[2] = a.a_lon
    v
end
function propagate(veh::Entity{VehicleState, D, I}, action::LatLonAccel, roadway::Roadway, ΔT::Float64) where {D, I}

    a_lat = action.a_lat
    a_lon = action.a_lon

     v = vel(veh.state)
     ϕ = posf(veh.state).ϕ
    ds = v*cos(ϕ)
     t = posf(veh.state).t
    dt = v*sin(ϕ)

    ΔT² = ΔT*ΔT
    Δs = ds*ΔT + 0.5*a_lon*ΔT²
    Δt = dt*ΔT + 0.5*a_lat*ΔT²

    ds₂ = ds + a_lon*ΔT
    dt₂ = dt + a_lat*ΔT
    speed₂ = sqrt(dt₂*dt₂ + ds₂*ds₂)
    v₂ = sqrt(dt₂*dt₂ + ds₂*ds₂) # v is the magnitude of the velocity vector
    ϕ₂ = atan(dt₂, ds₂)

    roadind = move_along(posf(veh.state).roadind, roadway, Δs)
    footpoint = roadway[roadind]
    posG = VecE2{Float64}(footpoint.pos.x,footpoint.pos.y) + polar(t + Δt, footpoint.pos.θ + π/2)

    posG = VecSE2{Float64}(posG.x, posG.y, footpoint.pos.θ + ϕ₂)

    veh = VehicleState(posG, roadway, v₂)
    return veh
end

