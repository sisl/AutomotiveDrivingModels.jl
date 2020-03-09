"""
    convert(ListRecord, qrec::QueueRecord{E})

Converts a QueueRecord into the corresponding ListRecord.
"""
function Base.convert(::Type{ListRecord{S,D,I}}, qrec::QueueRecord{Entity{S,D,I}}) where {S,D,I}

    frames = Array{RecordFrame}(undef, nframes(qrec))
    states = Array{RecordState{S,I}}(undef, nstates(qrec))
    defs = Dict{I, D}()

    lo = 1
    for (i,pastframe) in enumerate(1-nframes(qrec) : 0)
        frame = qrec[pastframe]

        hi = lo-1
        for entity in frame
            hi += 1
            defs[entity.id] = entity.def
            states[hi] = RecordState{S,I}(entity.state, entity.id)
        end

        frames[i] = RecordFrame(lo, hi)
        lo = hi + 1
    end

    return ListRecord{S,D,I}(get_timestep(qrec), frames, states, defs)
end
Base.convert(::Type{ListRecord}, qrec::QueueRecord{Entity{S,D,I}}) where {S,D,I} = convert(ListRecord{S,D,I}, qrec)

"""
    convert(QueueRecord, lrec::ListRecord)

Converts a ListRecord into the corresponding QueueRecord{Entity{S,D,I}}.
Note that the timesteps for a ListRecord are not necessarily constant timesteps.
"""
function Base.convert(::Type{QueueRecord{Entity{S,D,I}}}, lrec::ListRecord{S,D,I}) where {S,D,I}

    N = nframes(lrec)
    M = maximum(n_objects_in_frame(lrec, i) for i in 1 : N)
    retval = QueueRecord(Entity{S,D,I}, N, get_timestep(lrec), M)

    frame = Frame(Entity{S,D,I}, M)
    for i in 1 : N
        get!(frame, lrec, i)
        update!(retval, frame)
    end

    return retval
end


"""
    get_sparse_lookup(lrec::ListRecord{S,D,I})

Converts a ListRecord into the corresponding sparse matrix containing the states, SparseMatrixCSC{S,Int}.
This requires I <: Integer.

In the sparse array, each column is the state of an entity, and reach row is a frame entry.

A tuple containing the sparse matrix and a dictionary mapping from ids to the column number are returned.
"""
function get_sparse_lookup(rec::ListRecord{S,D,I}) where {S,D,I<:Integer}

    # ids are by time of entry
    # translate id to index on range of 1:n
    id_lookup = Dict{I,Int}(id => index for (index, id) in
        (rec.defs |> keys |> collect |> sort |> enumerate))

    m = nframes(rec) # num rows
    n = nids(rec) # num cols
    n_states = length(rec.states)

    # the row/frame of each state
    Is = Vector{Int}(undef, n_states)
    # the column/car index (not the id, see id_lookup)
    Js = similar(Is)
    # the states themselves
    Vs = similar(Is, S)

    # the index into I, J, and V (on [1:n_states])
    idx = 1
    for (fid, frame) in enumerate(rec.frames)
        for stateid in frame.lo : frame.hi
            recstate = rec.states[stateid]
            Is[idx] = fid
            Js[idx] = id_lookup[recstate.id]
            Vs[idx] = recstate.state

            idx += 1
        end
    end

    sparsemat = sparse(Is, Js, Vs, m, n)
    return (sparsemat, id_lookup)
end