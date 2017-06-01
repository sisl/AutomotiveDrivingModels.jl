"""
    LatLonAccel
Acceleration in the frenet frame
"""
immutable LatLonAccel
    a_lat::Float64
    a_lon::Float64
end
Base.show(io::IO, a::LatLonAccel) = @printf(io, "LatLonAccel(%6.3f, %6.3f)", a.a_lat, a.a_lon)
Base.length(::Type{LatLonAccel}) = 2
Base.convert(::Type{LatLonAccel}, v::Vector{Float64}) = LatLonAccel(v[1], v[2])
function Base.copy!(v::Vector{Float64}, a::LatLonAccel)
    v[1] = a.a_lat
    v[2] = a.a_lon
    v
end
function propagate{D<:Union{VehicleDef, BicycleModel}}(veh::Entity{VehicleState, D, Int}, action::LatLonAccel, roadway::Roadway, ΔT::Float64)

    a_lat = action.a_lat
    a_lon = action.a_lon

     v = veh.state.v
     ϕ = veh.state.posF.ϕ
    ds = v*cos(ϕ)
     t = veh.state.posF.t
    dt = v*sin(ϕ)

    ΔT² = ΔT*ΔT
    Δs = ds*ΔT + 0.5*a_lon*ΔT²
    Δt = dt*ΔT + 0.5*a_lat*ΔT²

    ds₂ = ds + a_lon*ΔT
    dt₂ = dt + a_lat*ΔT
    speed₂ = sqrt(dt₂*dt₂ + ds₂*ds₂)
    v₂ = sqrt(dt₂*dt₂ + ds₂*ds₂) # v is the magnitude of the velocity vector
    ϕ₂ = atan2(dt₂, ds₂) 

    roadind = move_along(veh.state.posF.roadind, roadway, Δs)
    footpoint = roadway[roadind]

    posG = convert(VecE2, footpoint.pos) + polar(t + Δt, footpoint.pos.θ + π/2)

    posG = VecSE2(posG.x, posG.y, footpoint.pos.θ + ϕ₂)

    return VehicleState(posG, roadway, v₂)
end
function Base.get(::Type{LatLonAccel}, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0)
    accel_lat = get(ACCFT, rec, roadway, vehicle_index, pastframe)
    accel_lon = get(ACCFS, rec, roadway, vehicle_index, pastframe)
    LatLonAccel(accel_lat, accel_lon)
end
