export
    TrajdataSegment,
    nsteps

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