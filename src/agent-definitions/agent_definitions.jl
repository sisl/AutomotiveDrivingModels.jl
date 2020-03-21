"""
    AbstractAgentDefinition
An Agent definition represents static parameters characterizing an agent, 
such as its physical dimensions.
"""
abstract type AbstractAgentDefinition end

"""
    length(def::AbstractAgentDefinition)
return the length of the vehicle 
"""
Base.length(def::AbstractAgentDefinition) = error("length not implemented for agent definition of type $(typeof(def))")

"""
    width(def::AbstractAgentDefinition)
return the width of the vehicle 
"""
width(def::AbstractAgentDefinition) = error("width not implemented for agent definition of type $(typeof(def))")

"""
    class(def::AbstractAgentDefinition)
return the class of the vehicle 
"""
class(def::AbstractAgentDefinition) = error("class not implemented for agent definition of type $(typeof(def))")

"""
A module to represent the different class of agents:
- Motorcycle
- Car
- Truck
- Pedestrian
"""
module AgentClass
    const MOTORCYCLE = 1
    const CAR        = 2
    const TRUCK      = 3
    const PEDESTRIAN = 4
end


"""
    VehicleDef(;class::Float64, length::Float64, width::Float64)
Vehicle definition which contains a class and a bounding box.
"""
struct VehicleDef <: AbstractAgentDefinition
    class::Int64 # ∈ AgentClass
    length::Float64
    width::Float64
end
function VehicleDef(;class::Int64 = AgentClass.CAR,
                   length::Float64 = 4.0,
                    width::Float64 = 1.8)
    return VehicleDef(class, length, width)
end

Base.length(d::VehicleDef) = d.length
width(d::VehicleDef) = d.width
class(d::VehicleDef) = d.class

const NULL_VEHICLEDEF = VehicleDef(AgentClass.CAR, NaN, NaN)

function Base.show(io::IO, d::VehicleDef)
    class = d.class == AgentClass.CAR ? "CAR" :
          d.class == AgentClass.MOTORCYCLE ? "MOTORCYCLE" :
          d.class == AgentClass.TRUCK ? "TRUCK" :
          d.class == AgentClass.PEDESTRIAN ? "PEDESTRIAN" :
          "UNKNOWN"
    @printf(io, "VehicleDef(%s, %.3f, %.3f)", class, d.length, d.width)
end
Base.write(io::IO, def::VehicleDef) = @printf(io, "%d %.16e %.16e", def.class, def.length, def.width)
function Base.read(io::IO, ::Type{VehicleDef})
    tokens = split(strip(readline(io)), ' ')
    class = parse(Int, tokens[1])
    length = parse(Float64, tokens[2])
    width = parse(Float64, tokens[3])
    return VehicleDef(class, length, width)
end

"""
    BicycleModel
    BicycleModel(def::VehicleDef; a::Float64 = 1.5, b::Float64 = 1.5)
Vehicle definition representing the bicycle model

# Fields 
- `def::VehicleDef`
- `a::Float64` distance between cg and front axle [m]
- `b::Float64` distance between cg and rear axle [m]
"""
struct BicycleModel <: AbstractAgentDefinition
    def::VehicleDef
    a::Float64 # distance between cg and front axle [m]
    b::Float64 # distance between cg and rear axle [m]
end
function BicycleModel(def::VehicleDef;
    a::Float64 = 1.5,
    b::Float64 = 1.5,
    )

    return BicycleModel(def, a, b)
end

Base.length(d::BicycleModel) = d.def.length
width(d::BicycleModel) = d.def.width
class(d::BicycleModel) = d.def.class
