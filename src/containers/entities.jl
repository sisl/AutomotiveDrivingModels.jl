struct Entity{S,D,I} # state, definition, identification
    state::S
    def::D
    id::I
end
Entity(entity::Entity{S,D,I}, s::S) where {S,D,I} = Entity(s, entity.def, entity.id)
