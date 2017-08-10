"""
    StoppingAccel

A longitudinal acceleration which does not decrease the speed past 0.
"""
struct StoppingAccel
    a::Float64 # [m/s²]
end
Base.show(io::IO, a::StoppingAccel) = @printf(io, "StoppingAccel(%6.3f)", a.a)
Base.length(::Type{StoppingAccel}) = 1
Base.convert(::Type{StoppingAccel}, v::Vector{Float64}) = StoppingAccel(v[1])
function Base.copy!(v::Vector{Float64}, a::StoppingAccel)
    v[1] = a.a
    v
end
Base.convert(::Type{Accel}, a::StoppingAccel) = Accel(a.a)
Base.convert(::Type{StoppingAccel}, a::Accel) = StoppingAccel(a.a)