#######################################

immutable LaneTag
    segment :: Int # segment id
    lane    :: Int # index in segment.lanes of this lane
end
hash(id::LaneTag, h::UInt=zero(UInt)) = hash(id.segment, hash(id.lane, h))
const NULL_LANETAG = LaneTag(0,0)

#######################################

immutable RoadIndex
    ind::CurveIndex
    tag::LaneTag
end
const NULL_ROADINDEX = RoadIndex(CurveIndex(-1,NaN), LaneTag(-1,-1))

#######################################

immutable LaneBoundary
    style::Symbol # ∈ :solid, :broken, :double
    color::Symbol # ∈ :yellow, white
end
const NULL_BOUNDARY = LaneBoundary(:unknown, :unknown)

#######################################

const DEFAULT_LANE_WIDTH = 3.0 # [m]

type Lane
    tag            :: LaneTag
    curve          :: Curve
    width          :: Float64 # [m]
    boundary_left  :: LaneBoundary
    boundary_right :: LaneBoundary
    prev           :: RoadIndex # connects some RoadIndex to the first point of this lane
    next           :: RoadIndex # connects some the last point of this lane to some RoadIndex

    function Lane(
        tag::LaneTag,
        curve::Curve;
        width::Float64 = DEFAULT_LANE_WIDTH,
        boundary_left::LaneBoundary = NULL_BOUNDARY,
        boundary_right::LaneBoundary = NULL_BOUNDARY,
        prev::RoadIndex = NULL_ROADINDEX,
        next::RoadIndex = NULL_ROADINDEX,
        )

        retval = new()
        retval.tag = tag
        retval.curve = curve
        retval.width = width
        retval.boundary_left = boundary_left
        retval.boundary_right = boundary_right
        retval.prev = prev
        retval.next = next
        retval
    end
end

has_next(lane::Lane) = lane.next != NULL_ROADINDEX
has_prev(lane::Lane) = lane.prev != NULL_ROADINDEX

function connect!(prev::Lane, next::Lane)
    prev.next = RoadIndex(CurveIndex(1,0.0), next.tag)
    next.prev = RoadIndex(CurveIndex(length(prev.curve)-1,1.0), prev.tag)
    (prev, next)
end

#######################################

type RoadSegment
    # a list of lanes forming a single road with a common direction
    id::Int
    lanes::Vector{Lane} # lanes are stored right to left
    RoadSegment(id::Int, lanes::Vector{Lane}=Lane[]) = new(id, lanes)
end

#######################################

type Roadway
    segments::Vector{RoadSegment}

    Roadway(segments::Vector{RoadSegment}=RoadSegment[]) = new(segments)
end

"""
    lane[ind::CurveIndex, roadway::Roadway]
Accessor for lanes based on a CurveIndex.
Note that we extend the definition of a CurveIndex,
previously ind.i ∈ [1, length(curve)-1], to:

    ind.i ∈ [0, length(curve)]

where 1 ≤ ind.i ≤ length(curve)-1 is as before, but if the index
is on the section between two lanes, we use:

    ind.i = length(curve), ind.t ∈ [0,1] for the region between curve[end] → next
    ind.i = 0,             ind.t ∈ [0,1] for the region between prev → curve[1]
"""
function Base.getindex(lane::Lane, ind::CurveIndex, roadway::Roadway)
    if ind.i == 0
        pt_lo = prev_lane_point(lane, roadway)
        pt_hi = lane.curve[1]
        s_gap = abs(pt_hi.pos - pt_lo.pos)
        pt_lo = CurvePt(pt_lo.pos, -s_gap, pt_lo.k, pt_lo.kd)
        lerp(pt_lo, pt_hi, ind.t)
    elseif ind.i < length(lane.curve)
        lane.curve[ind]
    else
        pt_hi = next_lane_point(lane, roadway)
        pt_lo = lane.curve[end]
        s_gap = abs(pt_hi.pos - pt_lo.pos)
        pt_lo = CurvePt(pt_lo.pos, -s_gap, pt_lo.k, pt_lo.kd)
        lerp( pt_lo, pt_hi, ind.t)
    end
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

next_lane(lane::Lane, roadway::Roadway) = roadway[lane.next.tag]
prev_lane(lane::Lane, roadway::Roadway) = roadway[lane.prev.tag]
next_lane_point(lane::Lane, roadway::Roadway) = roadway[lane.next]
prev_lane_point(lane::Lane, roadway::Roadway) = roadway[lane.prev]

function has_segment(roadway::Roadway, segid::Int)
    for seg in roadway.segments
        if seg.id == segid
            return true
        end
    end
    false
end
function has_lanetag(roadway::Roadway, tag::LaneTag)
    if !has_segment(roadway, tag.segment)
        return false
    end
    seg = roadway[tag.segment]
    1 ≤ tag.lane ≤ length(seg.lanes)
end

immutable RoadProjection
    curveproj::CurveProjection
    tag::LaneTag
end

"""
    proj(posG::VecSE2, lane::Lane, roadway::Roadway)
Return the RoadProjection for projecting posG onto the lane.
This will automatically project to the next or prev curve as appropriate.
"""
function Vec.proj(posG::VecSE2, lane::Lane, roadway::Roadway)
    curveproj = proj(posG, lane.curve)
    rettag = lane.tag
    if curveproj.ind == CurveIndex(1,0.0) && has_prev(lane)
        pt_lo = prev_lane_point(lane, roadway)
        pt_hi = lane.curve[1]

        t = get_lerp_time_unclamped(pt_lo, pt_hi, posG)
        if t ≤ 0.0
            return proj(posG, prev_lane(lane, roadway), roadway)
        elseif t < 1.0 # for t == 1.0 we use the actual end of the lane
            @assert(0.0 ≤ t < 1.0)
            footpoint = lerp(pt_lo.pos, pt_hi.pos, t)
            ind = CurveIndex(0, t)
            curveproj = get_curve_projection(posG, footpoint, ind)
        end
    elseif curveproj.ind == CurveIndex(length(lane.curve)-1,1.0) && has_next(lane)
        pt_lo = lane.curve[end]
        pt_hi = next_lane_point(lane, roadway)

        t = get_lerp_time_unclamped(pt_lo, pt_hi, posG)
        if t ≥ 1.0
            return proj(posG, next_lane(lane, roadway), roadway)
        elseif t < 1.0 # for t == 1.0 we use the actual start of the lane
            @assert(0.0 ≤ t ≤ 1.0)
            footpoint = lerp(pt_lo.pos, pt_hi.pos, t)
            ind = CurveIndex(length(lane.curve), t)
            curveproj = get_curve_projection(posG, footpoint, ind)
        end
    end
    RoadProjection(curveproj, rettag)
end

"""
    proj(posG::VecSE2, seg::RoadSegment, roadway::Roadway)
Return the RoadProjection for projecting posG onto the segment.
Tries all of the lanes and gets the closest one
"""
function Vec.proj(posG::VecSE2, seg::RoadSegment, roadway::Roadway)

    best_dist2 = Inf
    best_proj = RoadProjection(CurveProjection(CurveIndex(-1,-1), NaN, NaN), NULL_LANETAG)

    for lane in seg.lanes
        roadproj = proj(posG, lane, roadway)
        footpoint = roadway[roadproj.tag][roadproj.curveproj.ind, roadway]
        dist2 = abs2(posG - footpoint.pos)
        if dist2 < best_dist2
            best_dist2 = dist2
            best_proj = roadproj
        end
    end

    best_proj
end

"""
    proj(posG::VecSE2, seg::RoadSegment, roadway::Roadway)
Return the RoadProjection for projecting posG onto the roadway.
Tries all of the lanes and gets the closest one
"""
function Vec.proj(posG::VecSE2, roadway::Roadway)

    best_dist2 = Inf
    best_proj = RoadProjection(CurveProjection(CurveIndex(-1,-1), NaN, NaN), NULL_LANETAG)

    for seg in roadway.segments
        for lane in seg.lanes
            roadproj = proj(posG, lane, roadway)
            targetlane = roadway[roadproj.tag]
            footpoint = targetlane[roadproj.curveproj.ind, roadway]
            dist2 = abs2(posG - footpoint.pos)
            if dist2 < best_dist2
                best_dist2 = dist2
                best_proj = roadproj
            end
        end
    end

    best_proj
end

############################################

RoadIndex(roadproj::RoadProjection) = RoadIndex(roadproj.curveproj.ind, roadproj.tag)

function Base.getindex(roadway::Roadway, roadind::RoadIndex)
    lane = roadway[roadind.tag]
    lane[roadind.ind, roadway]
end

"""
    move_along(roadind::RoadIndex, road::Roadway, Δs::Float64)
Return the RoadIndex at ind's s position + Δs
"""
function move_along(roadind::RoadIndex, roadway::Roadway, Δs::Float64)

    lane = roadway[roadind.tag]
    curvept = lane[roadind.ind, roadway]

    if curvept.s + Δs < 0.0
        if has_prev(lane)
            pt_lo = prev_lane_point(lane, roadway)
            pt_hi = lane.curve[1]
            s_gap = abs(pt_hi.pos - pt_lo.pos)

            if curvept.s + Δs < -s_gap
                lane_prev = prev_lane(lane, roadway)
                curveind = CurveIndex(length(lane_prev.curve)-1,1.0)
                roadind = RoadIndex(curveind, lane_prev.tag)
                return move_along(roadind, roadway, Δs + curvept.s + s_gap)
            else # in the gap between lanes
                t = (s_gap + curvept.s + Δs) / s_gap
                curveind = CurveIndex(0, t)
                RoadIndex(curveind, lane.tag)
            end

        else # no prev lane, return the beginning of this one
            curveind = CurveIndex(1, 0.0)
            return RoadIndex(curveind, roadind.tag)
        end
    elseif curvept.s + Δs > lane.curve[end].s
        if has_next(lane)
            pt_lo = lane.curve[end]
            pt_hi = next_lane_point(lane, roadway)
            s_gap = abs(pt_hi.pos - pt_lo.pos)

            if curvept.s + Δs ≥ pt_lo.s + s_gap
                curveind = CurveIndex(1,0.0)
                roadind = RoadIndex(curveind, lane.next.tag)
                return move_along(roadind, roadway, Δs - (lane.curve[end].s + s_gap - curvept.s))
            else # in the gap between lanes
                t = (Δs - (lane.curve[end].s - curvept.s)) / s_gap
                curveind = CurveIndex(0, t)
                RoadIndex(curveind, lane.next.tag)
            end
        else # no next lane, return the end of this lane
            curveind = CurveIndex(length(lane.curve)-1, 1.0)
            return RoadIndex(curveind, roadind.tag)
        end
    else
        ind = get_curve_index(roadind.ind, lane.curve, Δs)
        RoadIndex(ind, roadind.tag)
    end
end

n_lanes_right(lane::Lane, roadway::Roadway) = lane.tag.lane - 1
function n_lanes_left(lane::Lane, roadway::Roadway)
    seg = roadway[lane.tag.segment]
    length(seg.lanes) - lane.tag.lane
end