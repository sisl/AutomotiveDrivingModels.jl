struct Entity{S,D,I} # state, definition, identification
    state::S
    def::D
    id::I
end
Entity(entity::Entity{S,D,I}, s::S) where {S,D,I} = Entity(s, entity.def, entity.id)

Base.:(==)(a::Entity{S,D,I}, b::Entity{S,D,I}) where {S,D,I} = a.state == b.state && a.def == b.def && a.id == b.id