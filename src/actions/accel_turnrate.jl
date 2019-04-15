"""
    AccelTurnrate
An action type with a longitudinal acceleration and a turn rate

# Fields
- `a::Float64` longitudinal acceleration [m/s^2]
- `ω::Float64` desired turn rate [rad/sec]
"""
struct AccelTurnrate
    a::Float64
    ω::Float64
end
Base.show(io::IO, a::AccelTurnrate) = @printf(io, "AccelTurnrate(%6.3f,%6.3f)", a.a, a.ω)
Base.length(::Type{AccelTurnrate}) = 2

"""
Convert an input vector containing desired acceleration and turn rate
into the AccelTurnrate type
"""
Base.convert(::Type{AccelTurnrate}, v::Vector{Float64}) = AccelTurnrate(v[1], v[2])

"""
Return a vector containing the desired acceleration and turn rate
"""
function Base.copyto!(v::Vector{Float64}, a::AccelTurnrate)
    v[1] = a.a
    v[2] = a.ω
    v
end

"""
Propagate vehicle forward in time using a desired acceleration and
turn rate
"""
function propagate(veh::Entity{VehicleState,D,I}, action::AccelTurnrate, roadway::Roadway, Δt::Float64; n_integration_steps::Int=4) where {D,I}

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
