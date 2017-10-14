"""
    LatLonAccel
Acceleration in the frenet frame
"""
struct LatLonAccel
    a_lat::Float64 # [m/s²]
    a_lon::Float64 # [m/s²]
end
Base.show(io::IO, a::LatLonAccel) = @printf(io, "LatLonAccel(%6.3f, %6.3f)", a.a_lat, a.a_lon)
Base.length(::Type{LatLonAccel}) = 2
Base.convert(::Type{LatLonAccel}, v::Vector{Float64}) = LatLonAccel(v[1], v[2])
function Base.copy!(v::Vector{Float64}, a::LatLonAccel)
    v[1] = a.a_lat
    v[2] = a.a_lon
    v
end

