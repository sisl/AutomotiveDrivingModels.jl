"""

Original location: 1d/states.jl
"""
struct State1D
    s::Float64 # position
    v::Float64 # speed [m/s]
end
Base.write(io::IO, ::MIME"text/plain", s::State1D) = @printf(io, "%.16e %.16e", s.s, s.v)
function Base.read(io::IO, ::MIME"text/plain", ::Type{State1D})
    i = 0
    tokens = split(strip(readline(io)), ' ')
    s = parse(Float64, tokens[i+=1])
    v = parse(Float64, tokens[i+=1])
    return State1D(s,v)
end

"""
Original location: 1d/vehicles.jl
"""
const Vehicle1D = Entity{State1D, VehicleDef, Int64}
const Scene1D = Frame{Vehicle1D}
Scene1D(n::Int=100) = Frame(Vehicle1D, n)
Scene1D(arr::Vector{Vehicle1D}) = Frame{Vehicle1D}(arr, length(arr))
