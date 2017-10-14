function propagate(veh::Vehicle, action::AccelDesang, roadway::Roadway, Δt::Float64; n_integration_steps::Int=4)

    a = action.a # accel
    ϕdes = action.ϕdes # desired heading angle

    x = veh.state.posG.x
    y = veh.state.posG.y
    θ = veh.state.posG.θ
    v = veh.state.v

    δt = Δt/n_integration_steps

    for i in 1 : n_integration_steps

        posF = Frenet(VecSE2(x, y, θ), roadway)
        ω = ϕdes - posF.ϕ

        x += v*cos(θ)*δt
        y += v*sin(θ)*δt
        θ += ω*δt
        v += a*δt
    end

    posG = VecSE2(x, y, θ)
    VehicleState(posG, roadway, v)
end

function propagate(veh::Vehicle, action::AccelTurnrate, roadway::Roadway, Δt::Float64; n_integration_steps::Int=4)

    a = action.a # accel
    ω = action.ω # turnrate

    x = veh.state.posG.x
    y = veh.state.posG.y
    θ = veh.state.posG.θ
    v = veh.state.v

    δt = Δt / n_integration_steps

    for i in 1 : n_integration_steps
        x += v*cos(θ)*δt
        y += v*sin(θ)*δt
        θ += ω*δt
        v += a*δt
    end

    posG = VecSE2(x, y, θ)
    VehicleState(posG, roadway, v)
end
function Base.get(::Type{AccelTurnrate}, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0)
    accel = get(ACC, rec, roadway, vehicle_index, pastframe)
    turnrate = get(TURNRATEG, rec, roadway, vehicle_index, pastframe)
    AccelTurnrate(accel, turnrate)
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

# TODO: move this to a folder roadwaystate_bicyclemodel_int
function propagate(veh::Entity{RoadwayState, BicycleModel, Int}, action::AccelSteeringAngle, roadway::Roadway, Δt::Float64)

    L = veh.def.a + veh.def.b
    l = -veh.def.b

    a = action.a # accel [m/s²]
    δ = action.δ # steering wheel angle [rad]

    x = veh.state.posG.x
    y = veh.state.posG.y
    θ = veh.state.posG.θ
    v = veh.state.v

    s = v*Δt + a*Δt*Δt/2 # distance covered
    v′ = v + a*Δt

    if abs(δ) < 0.01 # just drive straight
        posG = veh.state.posG + polar(s, θ)
    else # drive in circle

        R = L/tan(δ) # turn radius

        β = s/R
        xc = x - R*sin(θ) + l*cos(θ)
        yc = y + R*cos(θ) + l*sin(θ)

        θ′ = mod(θ+β, 2π)
        x′ = xc + R*sin(θ+β) - l*cos(θ′)
        y′ = yc - R*cos(θ+β) - l*sin(θ′)

        posG = VecSE2(x′, y′, θ′)
    end

    VehicleState(posG, roadway, v′)
end


"""
Perfect integration of accel over Δt
"""
# function propagate(veh::Vehicle, action::Accel, roadway::Roadway, Δt::Float64)

#     a = action.a

#     ds = veh.state.v

#     ΔT² = ΔT*ΔT
#     Δs = ds*ΔT + 0.5*a_lon*ΔT²

#     v₂ = ds + a_lon*ΔT

#     roadind = move_along(veh.state.posF.roadind, roadway, Δs)
#     posG = roadway[roadind].pos
#     VehicleState(posG, roadway, v₂)
# end