"""
    AccelDesang
An action type with a longitudinal acceleration and a desired heading angle

# Fields
- `a::Float64` longitudinal acceleration [m/s^2]
- `ϕdes::Float64` desired heading angle
"""  
struct AccelDesang
    a::Float64 # accel [m/s²]
    ϕdes::Float64 # Desired heading angle
end
Base.show(io::IO, a::AccelDesang) = @printf(io, "AccelDesang(%6.3f,%6.3f)", a.a, a.ϕdes)
Base.length(::Type{AccelDesang}) = 2

"""
    Base.convert(::Type{AccelDesang}, v::Vector{Float64})
Convert a vector containing the acceleration and desired heading angle
into AccelDesang type
"""
Base.convert(::Type{AccelDesang}, v::Vector{Float64}) = AccelDesang(v[1], v[2])

"""
    Base.copyto!(v::Vector{Float64}, a::AccelDesang)
Extract the AccelDesang components into a vector and return the vector
"""
function Base.copyto!(v::Vector{Float64}, a::AccelDesang)
    v[1] = a.a
    v[2] = a.ϕdes
    v
end

"""
    propagate(veh::Entity{VehicleState,D,I}, action::AccelDesang, roadway::Roadway, Δt::Float64; n_integration_steps::Int=4) where {D,I}
Propagate vehicle forward in time using a desired acceleration
and heading angle
"""
function propagate(veh::Entity{VehicleState,D,I}, action::AccelDesang, roadway::Roadway, Δt::Float64; n_integration_steps::Int=4) where {D,I}

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
