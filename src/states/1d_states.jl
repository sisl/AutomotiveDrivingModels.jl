"""
    State1D

A data type to represent one dimensional states

# Fields 
    - `s::Float64` position 
    - `v::Float64` speed [m/s]
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
    Vehicle1D
A specific instance of the Entity type defined in Records.jl to represent vehicles in 1d environments.
"""
const Vehicle1D = Entity{State1D, VehicleDef, Int64}

"""
    Scene1D

A specific instance of the Frame type defined in Records.jl to represent a list of vehicles in 1d environments.

# constructors
    Scene1D(n::Int=100)
    Scene1D(arr::Vector{Vehicle1D})
"""
const Scene1D = Frame{Vehicle1D}
Scene1D(n::Int=100) = Frame(Vehicle1D, n)
Scene1D(arr::Vector{Vehicle1D}) = Frame{Vehicle1D}(arr, length(arr))

get_center(veh::Vehicle1D) = veh.state.s
get_footpoint(veh::Vehicle1D) = veh.state.s
get_front(veh::Vehicle1D) = veh.state.s + veh.def.length/2
get_rear(veh::Vehicle1D) = veh.state.s - veh.def.length/2