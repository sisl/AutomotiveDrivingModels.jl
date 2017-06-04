immutable AccelSteeringAngle
    a::Float64 # accel [m/s²]
    δ::Float64 # steering angle [rad]
end
Base.show(io::IO, a::AccelSteeringAngle) = @printf(io, "AccelSteeringAngle(%6.3f,%6.3f)", a.a, a.δ)
Base.length(::Type{AccelSteeringAngle}) = 2
Base.convert(::Type{AccelSteeringAngle}, v::Vector{Float64}) = AccelSteeringAngle(v[1], v[2])
function Base.copy!(v::Vector{Float64}, a::AccelSteeringAngle)
    v[1] = a.a
    v[2] = a.δ
    v
end
function propagate(veh::Entity{VehicleState, BicycleModel, Int}, action::AccelSteeringAngle, roadway::Roadway, Δt::Float64)

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