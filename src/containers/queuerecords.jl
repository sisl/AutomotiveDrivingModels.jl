mutable struct QueueRecord{E}
    frames::Vector{Frame{E}}
    timestep::Float64
    nframes::Int # number of active Frames
end
function QueueRecord(::Type{E}, capacity::Int, timestep::Float64, frame_capacity::Int=100) where {E}
    frames = Array{Frame{E}}(undef, capacity)
    for i in 1 : length(frames)
        frames[i] = Frame(E, frame_capacity)
    end
    QueueRecord{E}(frames, timestep, 0)
end

Base.show(io::IO, rec::QueueRecord) = print(io, "QueueRecord(nframes=", rec.nframes, ")")

capacity(rec::QueueRecord) = length(rec.frames)
nframes(rec::QueueRecord) = rec.nframes
function nstates(rec::QueueRecord)
    retval = 0
    for frame_index in 1 : nframes(rec)
        retval += length(rec.frames[frame_index])
    end
    return retval
end

function Base.deepcopy(rec::QueueRecord)
    retval = QueueRecord(capacity(rec), rec.timestep, capacity(rec.frames[1]))
    for i in 1 : rec.nframes
        copyto!(retval.frames[i], rec.frames[i])
    end
    retval
end

pastframe_inbounds(rec::QueueRecord, pastframe::Int) = 1 ≤ 1-pastframe ≤ rec.nframes

"""
Indexed by pastframe, so pastframe == 0 is the current scene, -1 is the previous frame, etc.
"""
Base.getindex(rec::QueueRecord, pastframe::Int) = rec.frames[1 - pastframe]

get_time(rec::QueueRecord, pastframe::Int) = -get_elapsed_time(rec, pastframe)
get_timestep(rec::QueueRecord) = rec.timestep
get_elapsed_time(rec::QueueRecord, pastframe::Int) = -pastframe*rec.timestep
function get_elapsed_time(
    rec::QueueRecord,
    pastframe_farthest_back::Int,
    pastframe_most_recent::Int,
    )

    (pastframe_most_recent - pastframe_farthest_back)*rec.timestep
end

function Base.empty!(rec::QueueRecord)
    rec.nframes = 0
    return rec
end

function push_back_records!(rec::QueueRecord)
    for i in min(rec.nframes+1, capacity(rec)) : -1 : 2
        copyto!(rec.frames[i], rec.frames[i-1])
    end
    return rec
end
function Base.insert!(rec::QueueRecord{E}, frame::Frame{E}, pastframe::Int=0) where {E}
    copyto!(rec[pastframe], frame)
    return rec
end
function Base.get!(frame::Frame{E}, rec::QueueRecord{E}, pastframe::Int=0) where {E}
    copyto!(frame, rec[pastframe])
    frame
end
function update!(rec::QueueRecord{E}, frame::Frame{E}) where {E}
    push_back_records!(rec)
    insert!(rec, frame, 0)
    rec.nframes = min(rec.nframes+1, capacity(rec))
    return rec
end

function allocate_frame(rec::QueueRecord{E}) where {E}
    max_n_objects = maximum(length(rec[j]) for j in 0 : 1-length(rec))
    return Frame(E, max_n_objects)
end

const EntityQueueRecord{S,D,I} = QueueRecord{Entity{S,D,I}}

Base.length(record::QueueRecord) = nframes(record)
function Base.iterate(record::QueueRecord, state::Int64=(-nframes(record)+1))
    return state<=0 ? (record[state], state+1) : nothing
end
