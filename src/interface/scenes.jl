"""
    Data structure to represent an agent 

- state : for example position and velocity
- def : definition e.g. size of the vehicle
- id : e.g. a string or an int to identify the entity
""" 
struct Entity{S,D,I}
    state::S
    def::D
    id::I
end

Base.:(==)(a::Entity{S,D,I}, b::Entity{S,D,I}) where {S,D,I} = a.state == b.state && a.def == b.def && a.id == b.id


"""
    a Scene is made of a list of entity
"""
const Scene{S,D,I} = Vector{Entity{S,D,I}}



# convenience functions

Base.in(id::I, scene::Scene{S,D,I}) where {S,D,I} = findfirst(id, scene) != 0

function Base.findfirst(id::I, scene::Scene{S,D,I}) where {S,D,I}
    for (i,e) in enumerate(scene)
        if e.id == id
            return i
        end
    end
    return nothing
end

"""
    id2index(scene::Scene{S,D,I}, id::I) where {S,D,I}
returns the index of the entity identified by id in the scene
"""
function id2index(scene::Scene{S,D,I}, id::I) where {S,D,I}
    entity_index = findfirst(id, scene)
    if entity_index == nothing
        throw(BoundsError(scene, id))
    end
    return entity_index
end

"""
    get_by_id(scene::Scene{S,D,I}, id::I) where {S,D,I}
returns the entity identified by id in the scene
"""
get_by_id(scene::Scene{S,D,I}, id::I) where {S,D,I} = scene[id2index(scene, id)] 

"""
    get_first_available_id(scene::Scene{S,D,I}) where {S,D,I}
returns the first available id in the current scene 
It increments the id by `one(I)`. If `+` and `one` are not defined for objects of type I
this will break.
"""
function get_first_available_id(scene::Scene{S,D,I}) where {S,D,I}
    ids = Set{I}(entity.id for entity in scene)
    id_one = one(I)
    id = id_one
    while id ∈ ids
        id += id_one
    end
    return id
end