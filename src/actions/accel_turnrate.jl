struct AccelTurnrate
    a::Float64 # accel [m/s²]
    ω::Float64 # turnrate [rad/s]
end
Base.show(io::IO, a::AccelTurnrate) = @printf(io, "AccelTurnrate(%6.3f,%6.3f)", a.a, a.ω)
Base.length(::Type{AccelTurnrate}) = 2
Base.convert(::Type{AccelTurnrate}, v::Vector{Float64}) = AccelTurnrate(v[1], v[2])
function Base.copy!(v::Vector{Float64}, a::AccelTurnrate)
    v[1] = a.a
    v[2] = a.ω
    v
end
