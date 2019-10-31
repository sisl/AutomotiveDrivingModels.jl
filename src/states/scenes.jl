"""
    Scene 

A Scene is a specific instance of the Frame type defined in Records. It represents a collection of vehicles at a given time.

# Constructors 
    - `Scene(n::Int=100)`
    - `Scene(arr::Vector{Vehicle})`
"""
const Scene = Frame{Vehicle}
Scene(n::Int=100) = Frame(Vehicle, n)
Scene(arr::Vector{Vehicle}) = Frame{Vehicle}(arr, length(arr))

Base.show(io::IO, scene::Scene) = print(io, "Scene(with $(length(scene)) cars)")

"""
    Base.convert(::Type{Vehicle}, veh::Entity{VehicleState, D, Int64}) where D<:AbstractAgentDefinition

Converts an entity in Vehicle (it is converting the agent definition only)
"""
function Base.convert(::Type{Entity{VehicleState, VehicleDef, I}}, veh::Entity{VehicleState, D, I}) where {D<:Union{VehicleDef, BicycleModel}, I}
    vehdef = VehicleDef(class(veh.def), length(veh.def), width(veh.def))
    return Vehicle(veh.state, vehdef, veh.id)
end

"""
    SceneRecord
A SceneRecord is a specific instance of the QueueRecord type defined in Records.jl. It represents a collection of Scenes.

# constructor
    SceneRecord(capacity::Int, timestep::Float64, frame_capacity::Int=100)
"""
const SceneRecord = QueueRecord{Vehicle}
SceneRecord(capacity::Int, timestep::Float64, frame_capacity::Int=100) = QueueRecord(Vehicle, capacity, timestep, frame_capacity)
Base.show(io::IO, rec::SceneRecord) = print(io, "SceneRecord(nscenes=", nframes(rec), ")")
