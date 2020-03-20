"""
    Scene 

A Scene is a specific instance of the Frame type defined in Records. It represents a collection of vehicles at a given time.

# Constructors 
    - `Scene(n::Int=100)`
    - `Scene(arr::Vector{Vehicle})`
"""
const Scene = Frame{Entity{VehicleState, VehicleDef, Int64}}
Scene(n::Int=100) = Frame(Entity{VehicleState, VehicleDef, Int64}, n)
Scene(arr::Vector{Entity{VehicleState, VehicleDef, Int64}}) = Frame{Entity{VehicleState, VehicleDef, Int64}}(arr, length(arr))

Base.show(io::IO, scene::Scene) = print(io, "Scene(with $(length(scene)) cars)")

"""
    Base.convert(::Type{Entity{S, VehicleDef, I}}, veh::Entity{S, D, I}) where {S,D<:AbstractAgentDefinition,I}

Converts the definition of an entity
"""
function Base.convert(::Type{Entity{S, VehicleDef, I}}, veh::Entity{S, D, I}) where {S,D<:AbstractAgentDefinition,I}
    vehdef = VehicleDef(class(veh.def), length(veh.def), width(veh.def))
    return Entity{S, VehicleDef, I}(veh.state, vehdef, veh.id)
end

"""
    SceneRecord
A SceneRecord is a specific instance of the QueueRecord type defined in Records.jl. It represents a collection of Scenes.

# constructor
    SceneRecord(capacity::Int, timestep::Float64, frame_capacity::Int=100)
"""
const SceneRecord = QueueRecord{Entity{VehicleState, VehicleDef, Int64}}
SceneRecord(capacity::Int, timestep::Float64, frame_capacity::Int=100) = QueueRecord(Entity{VehicleState, VehicleDef, Int64}, capacity, timestep, frame_capacity)
Base.show(io::IO, rec::SceneRecord) = print(io, "SceneRecord(nscenes=", nframes(rec), ")")
