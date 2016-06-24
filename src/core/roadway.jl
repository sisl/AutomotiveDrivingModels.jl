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

const DEFAULT_LANE_WIDTH = 3.0 # [m]

type Lane
    tag            :: LaneTag
    curve          :: Curve
    width          :: Float64 # [m]
    boundary_left  :: LaneBoundary
    boundary_right :: LaneBoundary
    prev           :: LaneTag
    next           :: LaneTag

    function Lane(
        tag::LaneTag,
        curve::Curve;
        width::Float64 = DEFAULT_LANE_WIDTH,
        boundary_left::LaneBoundary = NULL_BOUNDARY,
        boundary_right::LaneBoundary = NULL_BOUNDARY,
        prev::LaneTag = NULL_LANETAG,
        next::LaneTag = NULL_LANETAG,
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

has_next(lane::Lane) = lane.next != NULL_LANETAG
has_prev(lane::Lane) = lane.prev != NULL_LANETAG

function connect!(prev::Lane, next::Lane)
    prev.next = next.tag
    next.prev = prev.tag
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

immutable RoadProjection
    curveproj::CurveProjection
    tag::LaneTag
end

"""
    proj(posG::VecSE2, lane::Lane, roadway::Roadway)
Return the RoadProjection for projecting posG onto the lane.
This will automatically project to the next or prev curve as appropriate.
If the pt is between curves the CurveIndex will have i = 0, and it will be on the
curve downstream of that point.
"""
function Vec.proj(posG::VecSE2, lane::Lane, roadway::Roadway)
    curveproj = proj(posG, lane.curve)
    if curveproj.ind == CurveIndex(1,0.0) && has_prev(lane)
        lane_prev = roadway[lane.prev]
        pt_lo = lane_prev.curve[end]
        pt_hi = lane.curve[1]

        t = get_lerp_time_unclamped(pt_lo, pt_hi, posG)
        if t < 0.0
            return proj(posG, lane_prev, roadway)
        end

        @assert(0.0 ≤ t ≤ 1.0)

        if t < 1.0
            footpoint = lerp( pt_lo.pos, pt_hi.pos, t)
            ind = CurveIndex(0.0, t)
        else
            footpoint = pt_hi.pos
            ind = CurveIndex(1.0,0.0)
        end

        curveproj = get_curve_projection(posG, footpoint, ind)

    elseif curveproj.ind == CurveIndex(length(lane.curve)-1,1.0) && has_next(lane)
        lane_next = roadway[lane.next]
        pt_lo = lane.curve[end]
        pt_hi = lane_next.curve[1]

        t = get_lerp_time_unclamped(pt_lo, pt_hi, posG)
        if t > 1.0
            return proj(posG, lane_next, roadway)
        end

        @assert(0.0 ≤ t ≤ 1.0)

        footpoint = lerp( pt_lo.pos, pt_hi.pos, t)
        ind = CurveIndex(0.0, t)
        lane = lane_next
        curveproj = get_curve_projection(posG, footpoint, ind)
    end

    RoadProjection(curveproj, lane.tag)
end

# function project_to_closest_lane(posG::VecSE2, seg::RoadSegment)

#     best_dist2 = Inf
#     best_lanetag = NULL_LANETAG
#     best_proj = CurveProjection(CurveIndex(-1,-1), NaN, NaN)

#     for lane in seg.lanes
#         curveproj = proj(posG, lane.curve)
#         dist2 = abs2(lane.curve[curveproj.ind])
#         if dist2 < best_dist2
#             best_dist2 = dist2
#             best_lanetag = lane.tag
#             best_proj = curveproj
#         end
#     end

#     RoadProjection(best_curveproj, best_dist2, best_lanetag)
# end
# function project_to_closest_lane(posG::VecSE2, roadway::Roadway)

#     best_proj = RoadProjection(CurveProjection(CurveIndex(-1,-1), NaN, NaN), Inf, NULL_LANETAG)

#     for seg in roadway.segments
#         proj = project_to_closest_lane(posG, seg)
#         if proj.dist2 < best_proj.dist2
#             best_proj = proj
#         end
#     end

#     proj
# end

############################################

immutable RoadIndex
    ind::CurveIndex
    tag::LaneTag
end

# """
#     move_along(ind::CurveIndex, curve::Curve, Δs::Float64)
# Return the CurveIndex at ind's s position + Δs
# """
# function get_curve_index(ind::CurveIndex, curve::Curve, Δs::Float64)

#     L = length(curve)
#     ind_lo, ind_hi = ind.i, ind.i+1

#     s_lo = curve[ind_lo].s
#     s_hi = curve[ind_hi].s
#     s = lerp(s_lo, s_hi, ind.t)

#     if Δs ≥ 0.0

#         if s + Δs > s_hi && ind_hi < L
#             while s + Δs > s_hi && ind_hi < L
#                 Δs -= (s_hi - s)
#                 s = s_hi
#                 ind_lo += 1
#                 ind_hi += 1
#                 s_lo = curve[ind_lo].s
#                 s_hi = curve[ind_hi].s
#             end
#         else
#             Δs = s + Δs - s_lo
#         end

#         t = Δs/(s_hi - s_lo)
#         CurveIndex(ind_lo, t)
#     else
#         if s + Δs < s_lo  && ind_lo > 1
#             while s + Δs < s_lo  && ind_lo > 1
#                 Δs += (s - s_lo)
#                 s = s_lo
#                 ind_lo -= 1
#                 ind_hi -= 1
#                 s_lo = curve[ind_lo].s
#                 s_hi = curve[ind_hi].s
#             end
#         else
#             Δs = s + Δs - s_lo
#         end

#         t = 1.0 - Δs/(s_hi - s_lo)
#         CurveIndex(ind_lo, t)
#     end
# end
