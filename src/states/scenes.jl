"""
    Scene{E}

Container to store a list of entities. 
The main difference from a regular array is that its size is defined at construction and is fixed.
(`push!` is O(1))

# Constructors 

- `Scene(arr::AbstractVector; capacity::Int=length(arr))`
- `Scene(::Type{E}, capacity::Int=100) where {E}`


# Fields 

To interact with `Scene` object it is preferable to use functions rather than accessing the fields directly.

- `entities::Vector{E}`
- `n::Int` current number of entities in the scene
"""
mutable struct Scene{E}
    entities::Vector{E} # NOTE: I tried StaticArrays; was not faster
    n::Int
end
function Scene(arr::AbstractVector{E}; capacity::Int=length(arr)) where {E}
    capacity ≥ length(arr) || error("capacity cannot be less than entitiy count! (N ≥ length(arr))")
    entities = Array{E}(undef, capacity)
    copyto!(entities, arr)
    return Scene{E}(entities, length(arr))
end
function Scene(::Type{E}, capacity::Int=100) where {E}
    entities = Array{E}(undef, capacity)
    return Scene{E}(entities, 0)
end

Base.show(io::IO, scene::Scene{E}) where {E}= @printf(io, "Scene{%s}(%d entities)", string(E), length(scene))

"""
    capacity(scene::Scene)
returns the maximum number of entities that can be put in the scene. 
To get the current number of entities use `length` instead.
"""
capacity(scene::Scene) = length(scene.entities)

Base.length(scene::Scene) = scene.n
Base.getindex(scene::Scene, i::Int) = scene.entities[i]
Base.eltype(scene::Scene{E}) where {E} = E

Base.lastindex(scene::Scene) = scene.n
function Base.setindex!(scene::Scene{E}, entity::E, i::Int) where {E}
    scene.entities[i] = entity
    return scene
end
function Base.empty!(scene::Scene)
    scene.n = 0
    return scene
end
function Base.deleteat!(scene::Scene, entity_index::Int)
    for i in entity_index : scene.n - 1
        scene.entities[i] = scene.entities[i+1]
    end
    scene.n -= 1
    scene
end

function Base.iterate(scene::Scene{E}, i::Int=1) where {E}
    if i > length(scene)
        return nothing
    end
    return (scene.entities[i], i+1)
end

function Base.copyto!(dest::Scene{E}, src::Scene{E}) where {E}
    for i in 1 : src.n
        dest.entities[i] = src.entities[i]
    end
    dest.n = src.n
    return dest
end
Base.copy(scene::Scene{E}) where {E} = copyto!(Scene(E, capacity(scene)), scene)

function Base.push!(scene::Scene{E}, entity::E) where {E}
    scene.n += 1
    scene.entities[scene.n] = entity
    return scene
end


####
"""
    EntityScene{S,D,I} = Scene{Entity{S,D,I}}
Alias for `Scene` when the entities in the scene are of type `Entity`

# Constructors
- `EntityScene(::Type{S},::Type{D},::Type{I}) where {S,D,I}`
- `EntityScene(::Type{S},::Type{D},::Type{I},capacity::Int)` 

"""
const EntityScene{S,D,I} = Scene{Entity{S,D,I}}
EntityScene(::Type{S},::Type{D},::Type{I}) where {S,D,I} = Scene(Entity{S,D,I})
EntityScene(::Type{S},::Type{D},::Type{I},capacity::Int) where {S,D,I} = Scene(Entity{S,D,I}, capacity)

Base.in(id::I, scene::EntityScene{S,D,I}) where {S,D,I} = findfirst(id, scene) !== nothing

function Base.findfirst(id::I, scene::EntityScene{S,D,I}) where {S,D,I}
    for entity_index in 1 : scene.n
        entity = scene.entities[entity_index]
        if entity.id == id
            return entity_index
        end
    end
    return nothing
end

function id2index(scene::EntityScene{S,D,I}, id::I) where {S,D,I}
    entity_index = findfirst(id, scene)
    if entity_index === nothing
        throw(BoundsError(scene, [id]))
    end
    return entity_index
end

"""
    get_by_id(scene::EntityScene{S,D,I}, id::I) where {S,D,I}
Retrieve the entity by its `id`. This function uses `findfirst` which is O(n).
"""
get_by_id(scene::EntityScene{S,D,I}, id::I) where {S,D,I} = scene[id2index(scene, id)] 

function get_first_available_id(scene::EntityScene{S,D,I}) where {S,D,I}
    ids = Set{I}(entity.id for entity in scene)
    id_one = one(I)
    id = id_one
    while id ∈ ids
        id += id_one
    end
    return id
end
function Base.push!(scene::EntityScene{S,D,I}, s::S) where {S,D,I}
    id = get_first_available_id(scene)
    entity = Entity{S,D,I}(s, D(), id)
    push!(scene, entity)
end

Base.delete!(scene::EntityScene{S,D,I}, entity::Entity{S,D,I}) where {S,D,I} = deleteat!(scene, findfirst(entity.id, scene))
function Base.delete!(scene::EntityScene{S,D,I}, id::I) where {S,D,I}
    entity_index = findfirst(id, scene)
    if entity_index != nothing
        deleteat!(scene, entity_index)
    end
    return scene
end

###

function Base.write(io::IO, frames::Vector{EntityScene{S,D,I}}) where {S,D,I}
    println(io, length(frames))
    for scene in frames
        println(io, length(scene))
        for entity in scene
            write(io, entity.state)
            print(io, "\n")
            write(io, entity.def)
            print(io, "\n")
            write(io, string(entity.id))
            print(io, "\n")
        end
    end
end
function Base.read(io::IO, ::Type{Vector{EntityScene{S,D,I}}}) where {S,D,I}

    n = parse(Int, readline(io))
    frames = Array{EntityScene{S,D,I}}(undef, n)

    for i in 1 : n
        m = parse(Int, readline(io))
        scene = Scene(Entity{S,D,I}, m)
        for j in 1 : m
            state = read(io, S)
            def = read(io, D)
            strid = readline(io)
            id = try 
                parse(I, strid)
            catch e
                if e isa MethodError
                    id = try
                        convert(I, strid)
                    catch e 
                        error("Cannot parse $strid as $I")
                    end
                else 
                    error("Cannot parse $strid as $I")
                end
            end
            push!(scene, Entity(state,def,id))
        end
        frames[i] = scene
    end

    return frames
end
