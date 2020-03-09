struct RecordFrame
    lo::Int
    hi::Int
end
Base.length(recframe::RecordFrame) = recframe.hi - recframe.lo + 1 # number of objects in the frame
Base.write(io::IO, ::MIME"text/plain", recframe::RecordFrame) = @printf(io, "%d %d", recframe.lo, recframe.hi)
function Base.read(io::IO, ::MIME"text/plain", ::Type{RecordFrame})
    tokens = split(strip(readline(io)), ' ')
    lo = parse(Int, tokens[1])
    hi = parse(Int, tokens[2])
    return RecordFrame(lo, hi)
end

struct RecordState{S,I}
    state::S
    id::I
end

mutable struct ListRecord{S,D,I} # State, Definition, Identification
    timestep::Float64
    frames::Vector{RecordFrame}
    states::Vector{RecordState{S,I}}
    defs::Dict{I, D}
end
ListRecord(timestep::Float64, ::Type{S}, ::Type{D}, ::Type{I}=Int) where {S,D,I} = ListRecord{S,D,I}(timestep, RecordFrame[], RecordState{S}[], Dict{I,D}())

Base.show(io::IO, rec::ListRecord{S,D,I}) where {S,D,I} = @printf(io, "ListRecord{%s, %s, %s}(%d frames)", string(S), string(D), string(I), nframes(rec))
function Base.write(io::IO, mime::MIME"text/plain", rec::ListRecord)

    show(io, rec)
    print(io, "\n")
    @printf(io, "%.16e\n", rec.timestep)

    # defs
    println(io, length(rec.defs))
    for (id,def) in rec.defs
        write(io, mime, id)
        print(io, "\n")
        write(io, mime, def)
        print(io, "\n")
    end

    # ids & states
    println(io, length(rec.states))
    for recstate in rec.states
        write(io, mime, recstate.id)
        print(io, "\n")
        write(io, mime, recstate.state)
        print(io, "\n")
    end

    # frames
    println(io, nframes(rec))
    for recframe in rec.frames
        write(io, mime, recframe)
        print(io, "\n")
    end
end
function Base.read(io::IO, mime::MIME"text/plain", ::Type{ListRecord{S,D,I}}) where {S,D,I}
    readline(io) # skip first line

    timestep = parse(Float64, readline(io))

    n = parse(Int, readline(io))
    defs = Dict{I,D}()
    for i in 1 : n
        id = read(io, mime, I)
        defs[id] = read(io, mime, D)
    end

    n = parse(Int, readline(io))
    states = Array{RecordState{S,I}}(undef, n)
    for i in 1 : n
        id = read(io, mime, I)
        state = read(io, mime, S)
        states[i] = RecordState{S,I}(state, id)
    end

    n = parse(Int, readline(io))
    frames = Array{RecordFrame}(undef, n)
    for i in 1 : n
        frames[i] = read(io, mime, RecordFrame)
    end

    return ListRecord{S,D,I}(timestep, frames, states, defs)
end

function Base.push!(rec::ListRecord{S,D,I}, frame::EntityFrame{S,D,I}) where {S,D,I}
    states = RecordState{S,I}[RecordState(e.state, e.id) for e in frame]
    push!(rec.frames, RecordFrame(length(rec.states)+1, length(rec.states)+length(states)))
    append!(rec.states, states)
    for e in frame
        if !haskey(rec.defs, e.id)
            rec.defs[e.id] = e.def
        else
            @assert rec.defs[e.id] == e.def # not sure whether to keep this check
        end
    end
    return rec
end

get_statetype(rec::ListRecord{S,D,I}) where {S,D,I} = S 
get_deftype(rec::ListRecord{S,D,I}) where {S,D,I} = D
get_idtype(rec::ListRecord{S,D,I}) where {S,D,I} = I

nframes(rec::ListRecord) = length(rec.frames)
nstates(rec::ListRecord) = length(rec.states)
nids(rec::ListRecord) = length(keys(rec.defs))

frame_inbounds(rec::ListRecord, frame_index::Int) = 1 ≤ frame_index ≤ nframes(rec)
n_objects_in_frame(rec::ListRecord, frame_index::Int) = length(rec.frames[frame_index])

get_ids(rec::ListRecord) = collect(keys(rec.defs))
nth_id(rec::ListRecord, frame_index::Int, n::Int=1) = rec.states[rec.frames[frame_index].lo + n-1].id

get_time(rec::ListRecord, frame_index::Int) = rec.timestep * (frame_index-1)
get_timestep(rec::ListRecord) = rec.timestep
get_elapsed_time(rec::ListRecord, frame_lo::Int, frame_hi::Int) = rec.timestep * (frame_hi - frame_lo)

function findfirst_stateindex_with_id(rec::ListRecord{S,D,I}, id::I, frame_index::Int) where {S,D,I}
    recframe = rec.frames[frame_index]
    for i in recframe.lo : recframe.hi
        if rec.states[i].id == id
            return i
        end
    end
    return nothing
end
function findfirst_frame_with_id(rec::ListRecord{S,D,I}, id::I) where {S,D,I}
    for frame in 1:length(rec.frames)
        if findfirst_stateindex_with_id(rec, id, frame) != nothing
            return frame
        end
    end
    return nothing
end
function findlast_frame_with_id(rec::ListRecord{S,D,I}, id::Int) where {S,D,I}
    for frame in reverse(1:length(rec.frames))
        if findfirst_stateindex_with_id(rec, id, frame) != nothing
            return frame
        end
    end
    return nothing
end

Base.in(id::I, rec::ListRecord{S,D,I}, frame_index::Int) where {S,D,I} = findfirst_stateindex_with_id(rec, id, frame_index) != nothing
get_state(rec::ListRecord{S,D,I}, id::I, frame_index::Int) where {S,D,I} = rec.states[findfirst_stateindex_with_id(rec, id, frame_index)].state
get_def(rec::ListRecord{S,D,I}, id::I) where {S,D,I} = rec.defs[id]
Base.get(rec::ListRecord{S,D,I}, id::I, frame_index::Int) where {S,D,I} = Entity(get_state(rec, id, frame_index), get_def(rec,id), id)
function Base.get(rec::ListRecord, stateindex::Int)
    recstate = rec.states[stateindex]
    return Entity(recstate.state, get_def(rec, recstate.id), recstate.id)
end

function get_subinterval(rec::ListRecord{S,D,I}, frame_index_lo::Int, frame_index_hi::Int) where {S,D,I}
    frame_index_hi ≥ frame_index_lo || throw(DomainError())

    frame_indexes = frame_index_lo : frame_index_hi
    frames = Array{RecordFrame}(undef, length(frame_indexes))
    states = Array{RecordState{S,I}}(undef, rec.frames[frame_index_hi].hi - rec.frames[frame_index_lo].lo + 1)
    defs = Dict{I, D}()

    hi = 1
    for (i,frame_index) in enumerate(frame_indexes)
        frame = rec.frames[frame_index]
        n = length(frame)
        lo = hi
        hi += n-1
        copyto!(states, lo, rec.states, frame.lo, n)
        frames[i] = RecordFrame(lo, hi)
        hi += 1
    end

    for state in states
        defs[state.id] = get_def(rec, state.id)
    end

    return ListRecord{S,D,I}(rec.timestep, frames, states, defs)
end
get_subinterval(rec::ListRecord, range::UnitRange{Int64}) = get_subinterval(rec, a.start, a.stop)

#################################

EntityFrame(rec::ListRecord{S,D,I}, N::Int=100) where {S,D,I} = Frame(Entity{S,D,I}, N)
function allocate_frame(rec::ListRecord{S,D,I}) where {S,D,I}
    max_n_objects = maximum(n_objects_in_frame(rec,i) for i in 1 : nframes(rec))
    return Frame(Entity{S,D,I}, max_n_objects)
end
function Base.get!(frame::EntityFrame{S,D,I}, rec::ListRecord{S,D,I}, frame_index::Int) where {S,D,I}

    empty!(frame)

    if frame_inbounds(rec, frame_index)
        recframe = rec.frames[frame_index]
        for stateindex in recframe.lo : recframe.hi
            push!(frame, get(rec, stateindex))
        end
    end

    return frame
end


#################################

"""
An iterator for looping over all states for a particular entity id.
Each element is a Tuple{Int,S} containing the frame index and the state.
"""
struct ListRecordStateByIdIterator{S,D,I}
    rec::ListRecord{S,D,I}
    id::I
end

function Base.iterate(iter::ListRecordStateByIdIterator, frame_index::Int=1)
    while frame_index ≤ nframes(iter.rec) && !in(iter.id, iter.rec, frame_index)
        frame_index += 1
    end
    if frame_index > nframes(iter.rec)
        return nothing 
    end
    item = (frame_index, get_state(iter.rec, iter.id, frame_index))
    return (item, frame_index+1)
end

Base.length(iter::ListRecordStateByIdIterator) = sum(in(iter.id, iter.rec, i) for i in 1:nframes(iter.rec))
Base.eltype(iter::ListRecordStateByIdIterator{S,D,I}) where {S,D,I} = Tuple{Int, S}

################################

"""
An iterator for looping over all scenes.
Each element is an EntityFrame{S,D,I}.
The same frame is continuously overwritten.
As such, one should not call collect() on a frame iterator.
"""
struct ListRecordFrameIterator{S,D,I}
    rec::ListRecord{S,D,I}
    scene::EntityFrame{S,D,I}
end
ListRecordFrameIterator(rec::ListRecord{S,D,I}) where {S,D,I} = ListRecordFrameIterator(rec, allocate_frame(rec))

function Base.iterate(iter::ListRecordFrameIterator, frame_index::Int=1)
    if frame_index > nframes(iter.rec)
        return nothing
    end
    get!(iter.scene, iter.rec, frame_index)
    return (iter.scene, frame_index+1)
end

Base.length(iter::ListRecordFrameIterator) = nframes(iter.rec)
Base.eltype(iter::ListRecordFrameIterator{S,D,I}) where {S,D,I} = EntityFrame{S,D,I}