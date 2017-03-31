export
    TrajdataSegment,

    nsteps,
    pull_record,
    pull_continuous_segments,

    sample_random_subinterval

immutable TrajdataSegment
    trajdata_index :: Int # index within EvaluationData.trajdatas of the relevant trajdata
    egoid          :: Int # the active vehicle
    frame_lo       :: Int # the starting frame in trajdata (does not count any sort of history)
    frame_hi       :: Int # the ending frame in trajdata, simulation is from frame_lo and propagates until frame_hi
end
function Base.:(==)(a::TrajdataSegment, b::TrajdataSegment)
    a.trajdata_index   == b.trajdata_index &&
    a.egoid            == b.egoid &&
    a.frame_lo         == b.frame_lo &&
    a.frame_hi         == b.frame_hi
end
function Base.show(io::IO, seg::TrajdataSegment)
    println(io, "TrajdataSegment")
    @printf(io, "\ttrajdata_index:  %d\n", seg.trajdata_index)
    @printf(io, "\tegoid:           %d\n", seg.egoid)
    @printf(io, "\tframe_lo:        %d\n", seg.frame_lo)
    @printf(io, "\tframe_hi:        %d\n", seg.frame_hi)
end

nsteps(seg::TrajdataSegment) = seg.frame_hi - seg.frame_lo # total number of sim steps
Records.nframes(seg::TrajdataSegment) = nsteps(seg) + 1 # total number of frames spanned by trajdata segment

function pull_record(seg::TrajdataSegment, trajdata::Trajdata, prime_history::Int=0)
    rec = SceneRecord(nframes(seg)+prime_history, get_timestep(trajdata))
    scene = Scene()

    # prime_history
    for i in prime_history : -1 : 1
        frame = seg.frame_lo - i
        get!(scene, trajdata, frame)
        update!(rec, scene)
    end

    for frame in seg.frame_lo : seg.frame_hi
        get!(scene, trajdata, frame)
        update!(rec, scene)
    end
    rec
end

"""
    sample_random_subinterval(seg::TrajdataSegment, nframes::Int)
Returns a new TrajdataSegment containing a uniformly random subinterval of length nframes
from the given segment.
Throws an error if the TrajdataSegment is too short
"""
function sample_random_subinterval(seg::TrajdataSegment, nframes::Int)
    len = AutoCore.nframes(seg)

    dom_hi = len - nframes + 1
    domain = 1 : dom_hi

    frame_lo = rand(domain) + seg.frame_lo - 1
    frame_hi = frame_lo + nframes - 1

    TrajdataSegment(seg.trajdata_index, seg.egoid, frame_lo, frame_hi)
end

"""
    pull_continuous_segments(trajdata::Trajdata, trajdata_index::Int)
Run through the Trajdata and pull TrajdataSegments for each id for
continuous frames in which they appear
"""
type SegmentInProgress
    id::Int
    frame_lo::Int
    updated::Bool
end
function pull_continuous_segments(trajdata::Trajdata, trajdata_index::Int)

    segments = TrajdataSegment[]
    segments_in_progress = Dict{Int, SegmentInProgress}()

    for (frameindex, tdframe) in enumerate(trajdata.frames)

        for iip in values(segments_in_progress)
            iip.updated = false
        end

        for frame in tdframe.lo : tdframe.hi
            state = trajdata.states[frame]

            if haskey(segments_in_progress, state.id)
                segments_in_progress[state.id].updated = true
            else
                segments_in_progress[state.id] = SegmentInProgress(state.id, frameindex, true)
            end
        end

        for iip in values(segments_in_progress)
            if !iip.updated
                push!(segments, TrajdataSegment(trajdata_index, iip.id, iip.frame_lo, frameindex-1))
            end
        end
        filter!((id,iip)->iip.updated, segments_in_progress)
    end

    for iip in values(segments_in_progress)
        push!(segments, TrajdataSegment(trajdata_index, iip.id, iip.frame_lo, length(trajdata.frames)))
    end

    segments
end