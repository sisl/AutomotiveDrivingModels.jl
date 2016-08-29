export
    EvaluationData,
    get_segment_copies

function get_segment_copies(trajdatas::Vector{Trajdata}, segments::Vector{TrajdataSegment})
    segment_copies = Array(Trajdata, length(segments))
    for (i, seg) in enumerate(segments)
        segment_copies[i] = Trajdata(trajdatas[seg.trajdata_index], seg.frame_lo, seg.frame_hi)
    end
    segment_copies
end

type EvaluationData
    trajdatas::Vector{Trajdata}
    segments::Vector{TrajdataSegment}
    segment_copies::Vector{Trajdata} # one copy of each segment, segment[i] has copy segment_copies[i]

    function EvaluationData(
        trajdatas::Vector{Trajdata},
        segments::Vector{TrajdataSegment},
        segment_copies::Vector{Trajdata} = get_segment_copies(trajdatas, segments),
        )

        retval = new()
        retval.trajdatas = trajdatas
        retval.segments = segments
        retval.segment_copies = segment_copies
        retval
    end
end