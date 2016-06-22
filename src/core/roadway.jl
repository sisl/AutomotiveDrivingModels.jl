#######################################

immutable LaneTag
    segment :: Int # segment id
    lane    :: Int # index in segment.lanes of this lane
end
const NULL_LANETAG = LaneTag(0,0)

#######################################

immutable LaneBoundary
    style::Symbol # ∈ :solid, :broken, :double
    color::Symbol # ∈ :yellow, white
end
const NULL_BOUNDARY = LaneBoundary(:unknown, :unknown)

#######################################

type Lane
    tag            :: LaneTag
    width          :: Float64 # [m]
    curve          :: Curve
    boundary_left  :: LaneBoundary
    boundary_right :: LaneBoundary
    prev           :: LaneTag
    next           :: LaneTag
end

has_next(lane::Lane) = lane.next != NULL_LANETAG
has_prev(lane::Lane) = lane.prev != NULL_LANETAG

# TODO: lane accessor with CurveIndex that handles prev and next

#######################################

type RoadSegment
    # a list of lanes forming a single road with a common direction
    id::Int
    lanes::Vector{Lane} # lanes are stored right to left
    StreetSegment(id::Int, lanes::Vector{Lane}=Lane[]) = new(id, lanes)
end

#######################################

type Roadway
    segments::Vector{RoadSegment}
end

function Base.getindex(roadway::Roadway, segid::Int)
    for seg in roadway.segments
        if seg.id == segid
            return seg
        end
    end
    error("Could not find segid $segid in roadway")
end
function Base.getindex(roadway::Roadway, tag::LaneTag)
    seg = roadway[tag.segment]
    seg.lanes[tag.lane]
end

Base.next(roadway::Roadway, lane::Lane) = roadway[lane.next]
     prev(roadway::Roadway, lane::Lane) = roadway[lane.prev]

function Vec.proj(posG::VecSE2, lane::Lane)
    # TODO: also handle the hanging bits from prev and next
    proj(posG, lane.curve)
end

immutable RoadProjection
    curveproj::CurveProjection
    dist2::Float64 # squared distance
    tag::LaneTag
end

function project_to_closest_lane(posG::VecSE2, seg::RoadSegment)

    best_dist2 = Inf
    best_lanetag = NULL_LANETAG
    best_proj = CurveProjection(CurveIndex(-1,-1), NaN, NaN)

    for lane in seg.lanes
        curveproj = proj(posG, lane.curve)
        dist2 = abs2(lane.curve[curveproj.ind])
        if dist2 < best_dist2
            best_dist2 = dist2
            best_lanetag = lane.tag
            best_proj = curveproj
        end
    end

    RoadProjection(best_curveproj, best_dist2, best_lanetag)
end
function project_to_closest_lane(posG::VecSE2, roadway::Roadway)

    best_proj = RoadProjection(CurveProjection(CurveIndex(-1,-1), NaN, NaN), Inf, NULL_LANETAG)

    for seg in roadway.segments
        proj = project_to_closest_lane(posG, seg)
        if proj.dist2 < best_proj.dist2
            best_proj = proj
        end
    end

    proj
end
