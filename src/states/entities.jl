"""
    Entity{S,D,I}
Immutable data structure to represent entities (vehicle, pedestrian, ...). 
Entities are defined by a state, a definition, and an id. 
The state of an entity usually models changing values while the definition and the id should not change. 

# Constructor 

`Entity(state, definition, id)`

Copy constructor that keeps the definition and id but changes the state (a new object is still created):

`Entity(entity::Entity{S,D,I}, s::S)`

# Fields 

- `state::S`
- `def::D`
- `id::I`
"""
struct Entity{S,D,I} # state, definition, identification
    state::S
    def::D
    id::I
end
Entity(entity::Entity{S,D,I}, s::S) where {S,D,I} = Entity(s, entity.def, entity.id)
