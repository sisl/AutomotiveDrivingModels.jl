"""
    Accel

A longitudinal acceleration
"""
struct Accel
    a::Float64 # [m/s²]
end
Base.show(io::IO, a::Accel) = @printf(io, "Accel(%6.3f)", a.a)
Base.length(::Type{Accel}) = 1
Base.convert(::Type{Accel}, v::Vector{Float64}) = Accel(v[1])
function Base.copy!(v::Vector{Float64}, a::Accel)
    v[1] = a.a
    v
end