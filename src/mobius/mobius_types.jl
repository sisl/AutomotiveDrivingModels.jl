baremodule AgentClass
    const MOTORCYCLE = 1
    const CAR        = 2
    const TRUCK      = 3
end

immutable VehicleDef
    class::Int # ∈ AgentClass
    length::Float64
    width::Float64
end
function VehicleDef(;
    class::Int=AgentClass.CAR,
    length::Float64=4.0,
    width::Float64=1.8,
    )

    VehicleDef(class, length, width)
end

const NULL_VEHICLEDEF = VehicleDef(AgentClass.CAR, NaN, NaN)

Base.show(io::IO, d::VehicleDef) = @printf(io, "VehicleDef(%s, %.3f, %.3f)", d.class == AgentClass.CAR ? "CAR" : d.class == AgentClass.MOTORCYCLE ? "MOTORCYCLE" : "TRUCK", d.length, d.width)
Base.write(io::IO, ::MIME"text/plain", def::VehicleDef) = @printf(io, "%d %.16e %.16e", def.class, def.length, def.width)
function Base.read(io::IO, ::MIME"text/plain", ::Type{VehicleDef})
    tokens = split(strip(readline(io)), ' ')
    class = parse(Int, tokens[1])
    length = parse(Float64, tokens[2])
    width = parse(Float64, tokens[3])
    return VehicleDef(class, length, width)
end

immutable State1D
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
    LaneFollowingAccel
Longitudinal acceleration
"""
immutable LaneFollowingAccel
    a::Float64
end
Base.show(io::IO, a::LaneFollowingAccel) = @printf(io, "LaneFollowingAccel(%6.3f)", a.a)
Base.length(::Type{LaneFollowingAccel}) = 1
Base.convert(::Type{LaneFollowingAccel}, v::Vector{Float64}) = LaneFollowingAccel(v[1])
function Base.copy!(v::Vector{Float64}, a::LaneFollowingAccel)
    v[1] = a.a
    v
end

typealias MobiusVehicle Entity{State1D, VehicleDef, Int}
typealias MobiusScene Frame{MobiusVehicle}
MobiusScene(n::Int=100) = Frame(MobiusVehicle, n)

function propagate(veh::MobiusVehicle, action::LaneFollowingAccel, Δt::Float64, roadway::StraightRoadway)

    a = action.a
    s, v = veh.state.s, veh.state.v

    s′ = s + v*Δt + a*Δt*Δt/2
    v′ = v + a*Δt

    s′ = mod_position_to_roadway(s′, roadway)

    return State1D(s′, v′)
end

function get_neighbor_fore(scene::MobiusScene, vehicle_index::Int, roadway::StraightRoadway)
    s_ego_fore = scene[vehicle_index].state.s + scene[vehicle_index].def.length/2
    best_ind = 0
    best_gap = Inf
    for (i,veh) in enumerate(scene)
        if i != vehicle_index
            s_oth_rear = veh.state.s - veh.def.length/2 # back point of car
            while s_oth_rear < s_ego_fore
                s_oth_rear += roadway.length
            end
            Δs = s_oth_rear - s_ego_fore
            if Δs < best_gap
                best_gap, best_ind = Δs, i
            end
        end
    end
    return NeighborLongitudinalResult(best_ind, best_gap)
end

# function render!(
#     rendermodel::RenderModel,
#     veh::MobiusVehicle,
#     color::Colorant=RGB(rand(), rand(), rand())
#     )

#     s = veh.state.s
#     add_instruction!(rendermodel, render_vehicle, (s, 0.0, 0.0, veh.def.length, veh.def.width, color))
#     return rendermodel
# end