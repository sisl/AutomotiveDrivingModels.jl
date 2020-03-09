mutable struct Frame{E}
    entities::Vector{E} # NOTE: I tried StaticArrays; was not faster
    n::Int
end
function Frame(arr::AbstractVector{E}; capacity::Int=length(arr)) where {E}
    capacity ≥ length(arr) || error("capacity cannot be less than entitiy count! (N ≥ length(arr))")
    entities = Array{E}(undef, capacity)
    copyto!(entities, arr)
    return Frame{E}(entities, length(arr))
end
function Frame(::Type{E}, capacity::Int=100) where {E}
    entities = Array{E}(undef, capacity)
    return Frame{E}(entities, 0)
end

Base.show(io::IO, frame::Frame{E}) where {E}= @printf(io, "Frame{%s}(%d entities)", string(E), length(frame))

capacity(frame::Frame) = length(frame.entities)
Base.length(frame::Frame) = frame.n
Base.getindex(frame::Frame, i::Int) = frame.entities[i]
Base.eltype(frame::Frame{E}) where {E} = E

Base.lastindex(frame::Frame) = frame.n
function Base.setindex!(frame::Frame{E}, entity::E, i::Int) where {E}
    frame.entities[i] = entity
    return frame
end
function Base.empty!(frame::Frame)
    frame.n = 0
    return frame
end
function Base.deleteat!(frame::Frame, entity_index::Int)
    for i in entity_index : frame.n - 1
        frame.entities[i] = frame.entities[i+1]
    end
    frame.n -= 1
    frame
end

function Base.iterate(frame::Frame{E}, i::Int=1) where {E}
    if i > length(frame)
        return nothing
    end
    return (frame.entities[i], i+1)
end

function Base.copyto!(dest::Frame{E}, src::Frame{E}) where {E}
    for i in 1 : src.n
        dest.entities[i] = src.entities[i]
    end
    dest.n = src.n
    return dest
end
Base.copy(frame::Frame{E}) where {E} = copyto!(Frame(E, capacity(frame)), frame)

function Base.push!(frame::Frame{E}, entity::E) where {E}
    frame.n += 1
    frame.entities[frame.n] = entity
    return frame
end


####

const EntityFrame{S,D,I} = Frame{Entity{S,D,I}}
EntityFrame(::Type{S},::Type{D},::Type{I}) where {S,D,I} = Frame(Entity{S,D,I})
EntityFrame(::Type{S},::Type{D},::Type{I},N::Int) where {S,D,I} = Frame(Entity{S,D,I}, N)

Base.in(id::I, frame::EntityFrame{S,D,I}) where {S,D,I} = findfirst(id, frame) != nothing
function Base.findfirst(id::I, frame::EntityFrame{S,D,I}) where {S,D,I}
    for entity_index in 1 : frame.n
        entity = frame.entities[entity_index]
        if entity.id == id
            return entity_index
        end
    end
    return nothing
end
function id2index(frame::EntityFrame{S,D,I}, id::I) where {S,D,I}
    entity_index = findfirst(id, frame)
    if entity_index === nothing
        throw(BoundsError(frame, [id]))
    end
    return entity_index
end
get_by_id(frame::EntityFrame{S,D,I}, id::I) where {S,D,I} = frame[id2index(frame, id)] 
function get_first_available_id(frame::EntityFrame{S,D,I}) where {S,D,I}
    ids = Set{I}(entity.id for entity in frame)
    id_one = one(I)
    id = id_one
    while id ∈ ids
        id += id_one
    end
    return id
end
function Base.push!(frame::EntityFrame{S,D,I}, s::S) where {S,D,I}
    id = get_first_available_id(frame)
    entity = Entity{S,D,I}(s, D(), id)
    push!(frame, entity)
end

Base.delete!(frame::EntityFrame{S,D,I}, entity::Entity{S,D,I}) where {S,D,I} = deleteat!(frame, findfirst(entity.id, frame))
function Base.delete!(frame::EntityFrame{S,D,I}, id::I) where {S,D,I}
    entity_index = findfirst(id, frame)
    if entity_index != nothing
        deleteat!(frame, entity_index)
    end
    return frame
end

###

function Base.write(io::IO, mime::MIME"text/plain", frames::Vector{EntityFrame{S,D,I}}) where {S,D,I}
    println(io, length(frames))
    for frame in frames
        println(io, length(frame))
        for entity in frame
            write(io, mime, entity.state)
            print(io, "\n")
            write(io, mime, entity.def)
            print(io, "\n")
            write(io, mime, entity.id)
            print(io, "\n")
        end
    end
end
function Base.read(io::IO, mime::MIME"text/plain", ::Type{Vector{EntityFrame{S,D,I}}}) where {S,D,I}

    n = parse(Int, readline(io))
    frames = Array{EntityFrame{S,D,I}}(undef, n)

    for i in 1 : n
        m = parse(Int, readline(io))
        frame = Frame(Entity{S,D,I}, m)
        for j in 1 : m
            state = read(io, mime, S)
            def = read(io, mime, D)
            id = read(io, mime, I)
            push!(frame, Entity(state,def,id))
        end
        frames[i] = frame
    end

    return frames
end
