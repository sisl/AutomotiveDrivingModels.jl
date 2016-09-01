export
    TrajdataSegment,

    nsteps,
    pull_record,
    pull_continuous_segments

immutable TrajdataSegment
    trajdata_index :: Int # index within EvaluationData.trajdatas of the relevant trajdata
    egoid          :: Int # the active vehicle
    frame_lo       :: Int # the starting frame in trajdata (does not count any sort of history)
    frame_hi       :: Int # the ending frame in trajdata, simulation is from frame_lo and propagates until frame_hi
end
function Base.(:(==))(a::TrajdataSegment, b::TrajdataSegment)
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
AutoCore.nframes(seg::TrajdataSegment) = nsteps(seg) + 1 # total number of frames spanned by trajdata segment

function pull_record(seg::TrajdataSegment, trajdata::Trajdata)
    rec = SceneRecord(nframes(seg), get_mean_timestep(trajdata))
    scene = Scene()
    for frame in seg.frame_lo : seg.frame_hi
        get!(scene, trajdata, frame)
        update!(rec, scene)
    end
    rec
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