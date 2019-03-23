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
const Vehicle1D = Entity{State1D, VehicleDef, Int}
const Scene1D = Frame{Vehicle1D}
Scene1D(n::Int=100) = Frame(Vehicle1D, n)
Scene1D(arr::Vector{Vehicle1D}) = Frame{Vehicle1D}(arr, length(arr))


"""
original location: 2d/vehicles/vehicles.jl
"""
struct VehicleState
    posG::VecSE2{Float64} # global
    posF::Frenet # lane-relative frame
    v::Float64
end

VehicleState() = VehicleState(VecSE2(), NULL_FRENET, NaN)
VehicleState(posG::VecSE2{Float64}, v::Float64) = VehicleState(posG, NULL_FRENET, v)
VehicleState(posG::VecSE2{Float64}, roadway::Roadway, v::Float64) = VehicleState(posG, Frenet(posG, roadway), v)
VehicleState(posG::VecSE2{Float64}, lane::Lane, roadway::Roadway, v::Float64) = VehicleState(posG, Frenet(posG, lane, roadway), v)
VehicleState(posF::Frenet, roadway::Roadway, v::Float64) = VehicleState(get_posG(posF, roadway), posF, v)

Base.show(io::IO, s::VehicleState) = print(io, "VehicleState(", s.posG, ", ", s.posF, ", ", @sprintf("%.3f", s.v), ")")

function Base.write(io::IO, ::MIME"text/plain", s::VehicleState)
    @printf(io, "%.16e %.16e %.16e", s.posG.x, s.posG.y, s.posG.θ)
    @printf(io, " %d %.16e %d %d", s.posF.roadind.ind.i, s.posF.roadind.ind.t, s.posF.roadind.tag.segment, s.posF.roadind.tag.lane)
    @printf(io, " %.16e %.16e %.16e", s.posF.s, s.posF.t, s.posF.ϕ)
    @printf(io, " %.16e", s.v)
end
function Base.read(io::IO, ::MIME"text/plain", ::Type{VehicleState})
    tokens = split(strip(readline(io)), ' ')
    i = 0
    posG = VecSE2(parse(Float64, tokens[i+=1]), parse(Float64, tokens[i+=1]), parse(Float64, tokens[i+=1]))
    roadind = RoadIndex(CurveIndex(parse(Int, tokens[i+=1]), parse(Float64, tokens[i+=1])),
                        LaneTag(parse(Int, tokens[i+=1]), parse(Int, tokens[i+=1])))
    posF = Frenet(roadind, parse(Float64, tokens[i+=1]), parse(Float64, tokens[i+=1]), parse(Float64, tokens[i+=1]))
    v = parse(Float64, tokens[i+=1])
    return VehicleState(posG, posF, v)
end
