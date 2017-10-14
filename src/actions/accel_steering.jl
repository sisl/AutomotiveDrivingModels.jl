struct AccelSteeringAngle
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
