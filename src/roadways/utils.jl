immutable LaneBoundary
    style::Symbol # ∈ :solid, :broken, :double
    color::Symbol # ∈ :yellow, white
end
const NULL_BOUNDARY = LaneBoundary(:unknown, :unknown)

#######################

immutable SpeedLimit
    lo::Float64 # [m/s]
    hi::Float64 # [m/s]
end
const DEFAULT_SPEED_LIMIT = SpeedLimit(-Inf, Inf)

#######################

immutable LaneTag
    segment :: Int # segment id
    lane    :: Int # index in segment.lanes of this lane
end
Base.show(io::IO, tag::LaneTag) = @printf(io, "LaneTag(%d, %d)", tag.segment, tag.lane)
hash(id::LaneTag, h::UInt=zero(UInt)) = hash(id.segment, hash(id.lane, h))
Base.:(==)(a::LaneTag, b::LaneTag) = a.segment == b.segment && a.lane == b.lane
const NULL_LANETAG = LaneTag(0,0)