#######################################

immutable LaneTag
    segment :: Int # segment id
    lane    :: Int # index in segment.lanes of this lane
end
hash(id::LaneTag, h::UInt=zero(UInt)) = hash(id.segment, hash(id.lane, h))
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

function Base.getindex(lane::Lane, ind::CurveIndex, roadway::Roadway)
    if ind.i != 0
        lane.curve[ind]
    else
        lane_prev = roadway[lane.prev]
        pt_hi = lane.curve[1]
        pt_lo = lane_prev.curve[end]
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
        if t ≥ 1.0
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
            footpoint = roadway[roadproj.tag][roadproj.curveproj.ind, roadway]
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

immutable RoadIndex
    ind::CurveIndex
    tag::LaneTag
end
const NULL_ROADINDEX = RoadIndex(CurveIndex(-1,NaN), LaneTag(-1,-1))

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
            lane_prev = roadway[lane.prev]
            pt_lo = lane_prev.curve[end]
            pt_hi = lane.curve[1]
            s_gap = abs(pt_hi.pos - pt_lo.pos)

            if curvept.s + Δs < -s_gap
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
            lane_next = roadway[lane.next]
            pt_lo = lane.curve[end]
            pt_hi = lane_next.curve[1]
            s_gap = abs(pt_hi.pos - pt_lo.pos)

            if curvept.s + Δs ≥ pt_lo.s + s_gap
                curveind = CurveIndex(1,0.0)
                roadind = RoadIndex(curveind, lane_next.tag)
                return move_along(roadind, roadway, Δs - (lane.curve[end].s + s_gap - curvept.s))
            else # in the gap between lanes
                t = (Δs - (lane.curve[end].s - curvept.s)) / s_gap
                curveind = CurveIndex(0, t)
                RoadIndex(curveind, lane_next.tag)
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

############################################

# """
#     Base.convert(::Roadway, rndf::RNDF)
# Converts an rndf to a Roadway
# """
# function Base.convert(::Roadway, rndf::RNDF;
#     verbosity::Int=0,
#     convert_ll2utm::Bool=true,
#     CYCLE_CONNECTION_THRESHOLD::Float64=6.25 # [m] WHAT IS THIS???
#     EDGE_DIST_THRESHOLD::Float64=50.0 # [m] WHAT IS THIS???
#     )

#     verbosity < 1 || println("Convert RNDF to Roadway")
#     starttime = time()

#     roadway = Roadway()

#     node_map = Dict{WaypointID, Int}()
#     lane_ids = Dict{LaneID, Vector{Int}}()

#     # build the road network graph
#     verbosity < 1 || @printf("BUILDING GRAPH [%.2f]\n", time()-starttime)
#     for segment in values(rndf.segments)
#         for lane in values(segment.lanes)
#             @assert(lane.id.segment == segment.id)
#             ids = sort!(collect(keys(lane.waypoints))::Vector{Int})
#             for (waypoint_index,lla) in lane.waypoints
#                 lat, lon = lla.lat, lla.lon
#                 if convert_ll2utm
#                     utm = convert(UTM, LatLonAlt(deg2rad(lat), deg2rad(lon), NaN))
#                     eas, nor = utm.e, utm.n
#                 else
#                     eas, nor = lat, lon
#                 end
#                 pos = VecSE2(eas,nor,NaN) # unknown heading for now

#                 node = StreetNode(WaypointID(segment.id, lane.id.lane, waypoint_index), pos,
#                                   NaN, NaN, NaN, NaN, NaN, -999, -999, NaN, NaN)

#                 push!(sn.nodes, node)
#                 node_index = add_vertex!(G)
#                 @assert(node_index == length(sn.nodes))

#                 node_map[WaypointID(segment.id, lane.id.lane, waypoint_index)] = node_index
#             end

#             # starting with a root node, find the next closest node to either end of the chain
#             # to build the path
#             root_bot = 1 # [index in ids]
#             root_top = 1 # [index in ids]
#             ids_yet_to_take = Set(collect(2:length(ids)))
#             new_order = Int[1]
#             while !isempty(ids_yet_to_take)
#                 closest_node_to_bot = -1 # index in ids
#                 best_dist_bot = Inf
#                 closest_node_to_top = -1 # index in ids
#                 best_dist_top = Inf

#                 node_index_T = node_map[WaypointID(segment.id, lane.id.lane, ids[root_top])]
#                 node_index_B = node_map[WaypointID(segment.id, lane.id.lane, ids[root_bot])]
#                 nodeT = sn.nodes[node_index_T]
#                 nodeB = sn.nodes[node_index_B]
#                 Tn, Te = nodeT.pos.x, nodeT.pos.y
#                 Bn, Be = nodeB.pos.x, nodeB.pos.y

#                 for id in ids_yet_to_take
#                     node_index = node_map[WaypointID(segment.id, lane.id.lane, ids[id])]
#                     node = sn.nodes[node_index]

#                     index_n, index_e = node.pos.x, node.pos.y
#                     dnT, deT = Tn-index_n, Te-index_e
#                     dnB, deB = Bn-index_n, Be-index_e
#                     distT = sqrt(dnT*dnT + deT*deT)
#                     distB = sqrt(dnB*dnB + deB*deB)
#                     if distT < best_dist_top
#                         best_dist_top, closest_node_to_top = distT, id
#                     end
#                     if distB < best_dist_bot
#                         best_dist_bot, closest_node_to_bot = distB, id
#                     end
#                 end

#                 @assert(min(best_dist_top, best_dist_bot) < EDGE_DIST_THRESHOLD)
#                 if best_dist_bot < best_dist_top
#                     root_bot = closest_node_to_bot
#                     delete!(ids_yet_to_take, root_bot)
#                     unshift!(new_order, closest_node_to_bot)
#                 else
#                     root_top = closest_node_to_top
#                     delete!(ids_yet_to_take, root_top)
#                     push!(new_order, closest_node_to_top)
#                 end
#             end

#             # infer direction from whether ids are generally increasing or decreasing
#             ids_generally_increasing = sum([(ids[new_order[i]] - ids[new_order[i-1]] > 0.0) for i in 2:length(ids)]) > 0.0
#             node_index = node_map[WaypointID(segment.id, lane.id.lane, ids[new_order[1]])]

#             local i = 2
#             while i ≤ length(new_order)
#                 nextnode_index = node_map[WaypointID(segment.id, lane.id.lane, ids[new_order[i]])]

#                 ids_generally_increasing ? add_edge!(G, node_index, nextnode_index) : add_edge!(G, nextnode_index, node_index)
#                 node_index = nextnode_index
#                 i += 1
#             end

#             # if this is a cycle - connect it
#             # TODO(tim): use direction to infer whether this is correct
#             node_index_T = node_map[WaypointID(segment.id, lane.id.lane, ids[root_top])]
#             nodeT = sn.nodes[node_index_T]
#             node_index_B = node_map[WaypointID(segment.id, lane.id.lane, ids[root_bot])]
#             nodeB = sn.nodes[node_index_B]
#             if dist(nodeT, nodeB) < CYCLE_CONNECTION_THRESHOLD
#                 if ids_generally_increasing
#                     add_edge!(G, node_index_T, node_index_B)
#                 else
#                     add_edge!(G, node_index_B, node_index_T)
#                 end
#             end

#             @assert(length(new_order) == length(ids))
#             lane_ids[lane.id] = ids[new_order]
#         end
#     end

#     # add exit edges
#     verbosity < 1 || @printf("ADDING EXIT EDGES [%.2f]\n", time()-starttime)
#     for segment in values(rndf.segments)
#         for lane in values(segment.lanes)
#             for (exitnode, entry) in lane.exits
#                 exit_waypoint = WaypointID(segment.id, lane.id.lane, exitnode)
#                 if haskey(node_map, exit_waypoint)
#                     add_edge!(G, node_map[exit_waypoint], node_map[entry])
#                 end
#             end
#         end
#     end

#     # add nodes to tiles
#     verbosity < 1 || @printf("ADDING NODES TO TILES [%.2f]\n", time()-starttime)
#     for segment in values(rndf.segments)
#         for lane in values(segment.lanes)
#             # keep the curve in the tile
#             ids = lane_ids[lane.id]

#             first_node_laneindex = 1 # NOTE(tim): index in lane
#             first_node = sn.nodes[node_map[WaypointID(segment.id, lane.id.lane, ids[first_node_laneindex])]]
#             index_e, index_n = utm2tileindex(first_node.pos.x, first_node.pos.y)
#             tile = get_tile!(sn, index_e, index_n)
#             n_pts_in_lane = length(lane.waypoints)
#             @assert(n_pts_in_lane == length(ids))

#             while first_node_laneindex ≤ n_pts_in_lane
#                 # find the last node in this lane that is still in the tile
#                 final_node_laneindex = findfirst(index->begin
#                                                     node_index = node_map[WaypointID(segment.id, lane.id.lane, ids[index])]
#                                                     node = sn.nodes[node_index]
#                                                     node_e, node_n = utm2tileindex(node.pos.x, node.pos.y)
#                                                     return (index_e != node_e) || (index_n != node_n)
#                                                 end,
#                                             (first_node_laneindex+1) : n_pts_in_lane)

#                 if final_node_laneindex == 0
#                     final_node_laneindex = n_pts_in_lane
#                 else
#                     final_node_laneindex += first_node_laneindex-1
#                 end

#                 node_indeces = Array(Int, length(first_node_laneindex : final_node_laneindex))
#                 nodes = Array(StreetNode, length(node_indeces))
#                 for (k,laneindex) in enumerate(first_node_laneindex : final_node_laneindex)
#                     node_index = node_map[WaypointID(segment.id, lane.id.lane, ids[laneindex])]
#                     node_indeces[k] = node_index
#                     nodes[k] = sn.nodes[node_index]
#                 end

#                 streetsegment = get_segment!(tile, segment.id)

#                 if has_lane(streetsegment, lane.id.lane)
#                     # need to stitch lanes together
#                     streetlane = streetsegment.lanes[lane.id.lane]

#                     if dist(nodes[end], sn.nodes[streetlane.node_indeces[1]]) < dist(sn.nodes[streetlane.node_indeces[end]], nodes[1])
#                         streetlane.node_indeces = [node_indeces, streetlane.node_indeces]
#                     else
#                         streetlane.node_indeces = [streetlane.node_indeces; node_indeces]
#                     end
#                 else
#                     width = isnan(lane.width) ? DEFAULT_LANE_WIDTH : lane.width*METERS_PER_FOOT

#                     lanetag = LaneTag(tile.index_e, tile.index_n, lane.id.segment, lane.id.lane)
#                     streetlane = StreetLane(lanetag, width, lane.boundary_left, lane.boundary_right, node_indeces, Curve(), false, false)
#                     add_lane!(streetsegment, streetlane)
#                 end

#                 first_node_laneindex = final_node_laneindex+1
#                 if first_node_laneindex ≤ n_pts_in_lane
#                     node = sn.nodes[node_map[WaypointID(segment.id, lane.id.lane, ids[first_node_laneindex])]]
#                     index_e, index_n = utm2tileindex(node.pos.x, node.pos.y)
#                     tile = get_tile!(sn, index_e, index_n)
#                 end
#             end
#         end
#     end

#     # attempt to connect hanging edges
#     verbosity < 1 || @printf("CONNECTING HANGING EDGES [%.2f]\n", time()-starttime)

#     const THRESHOLD_HANGING_EDGE_LINE_DIST_TO_CONNECT = 30.0 # [m]
#     const THRESHOLD_HANGING_EDGE_ANGLE_DIFFERENCE  = deg2rad(20) # [rad]
#     const THRESHOLD_HANGING_EDGE_ANGLE_DIFFERENCE2 = deg2rad(5) # [rad]

#     for node_index in vertices(G)

#         node = sn.nodes[node_index]

#         if outdegree(G, node_index) == 0 && indegree(G, node_index) > 0
#             # NOTE(tim): this is an ending node

#             prevnode_index = prev_node_index(sn, node_index)
#             prevnode = sn.nodes[prevnode_index]

#             θ = atan2(node.pos.y - prevnode.pos.y, node.pos.x - prevnode.pos.x)
#             E = VecE2(node.pos.x, node.pos.y)
#             vE = Vec.polar(1.0, θ)

#             best_node_index = node_index
#             best_score = Inf

#             tile = get_tile(sn, node)
#             for seg in values(tile.segments)
#                 for lane in values(seg.lanes)
#                     if (seg.id == node.id.segment && lane.id.lane == node.id.lane) || length(lane.node_indeces) < 2
#                         continue
#                     end

#                     for (i,testnode_index) in enumerate(lane.node_indeces)

#                         testnode = sn.nodes[testnode_index]

#                         θ₂ = i > 1 ? atan2(testnode.pos.y - sn.nodes[lane.node_indeces[i-1]].pos.y,
#                                            testnode.pos.x - sn.nodes[lane.node_indeces[i-1]].pos.x) :
#                                      atan2(sn.nodes[lane.node_indeces[i+1]].pos.y - testnode.pos.y,
#                                            sn.nodes[lane.node_indeces[i+1]].pos.x - testnode.pos.x)

#                         vT = Vec.polar(1.0, θ₂)
#                         T = VecE2(testnode.pos.x, testnode.pos.y)
#                         A = T - E

#                         c_p = dot(A,vE)
#                         if c_p > 0.0
#                             proj = vE * c_p
#                             perp_dist = hypot(A - proj)
#                             line_dist = hypot(proj)

#                             # NOTE(tim): angle between start and end orientation
#                             Δθ = angledist(θ, θ₂)

#                             # NOTE(tim): angle between dangling edge and projection
#                             θ₃ = atan2(testnode.pos.y - node.pos.y, testnode.pos.x - node.pos.x)
#                             Δ₃ = angledist(θ,θ₃)

#                             score = Δθ + 10.0*Δ₃

#                             if line_dist < THRESHOLD_HANGING_EDGE_LINE_DIST_TO_CONNECT &&
#                                 Δθ < THRESHOLD_HANGING_EDGE_ANGLE_DIFFERENCE &&
#                                 Δ₃ < THRESHOLD_HANGING_EDGE_ANGLE_DIFFERENCE2 &&
#                                 score < best_score

#                                 best_score = score
#                                 best_node_index = testnode_index
#                             end
#                         end
#                     end
#                 end
#             end

#             if node_index != best_node_index
#                 verbosity > 1 || println("ADDED END NODE HANGING EDGE ", node.pos.x, "  ", node.pos.y)
#                 add_edge!(G, node_index, best_node_index)
#             end
#         end
#         if indegree(G, node_index) == 0 && outdegree(G, node_index) > 0
#             # NOTE(tim): this is a starting node
#             nextnode = sn.nodes[next_node_index(sn, node_index)]
#             θ = atan2(nextnode.pos.y - node.pos.y, nextnode.pos.x - node.pos.x)
#             E = VecE2(node.pos.x, node.pos.y)
#             vE = Vec.polar(-1.0, θ)

#             best_node_index = node_index
#             best_score = Inf

#             tile = get_tile(sn, node)
#             for seg in values(tile.segments)
#                 for lane in values(seg.lanes)
#                     if (seg.id == node.id.segment && lane.id.lane == node.id.lane) || length(lane.node_indeces) < 2
#                         continue
#                     end

#                     for (i,testnode_index) in enumerate(lane.node_indeces)

#                         testnode = sn.nodes[testnode_index]

#                         θ₂ = i > 1 ? atan2(testnode.pos.y - sn.nodes[lane.node_indeces[i-1]].pos.y,
#                                            testnode.pos.x - sn.nodes[lane.node_indeces[i-1]].pos.x) :
#                                      atan2(sn.nodes[lane.node_indeces[i+1]].pos.y - testnode.pos.y,
#                                            sn.nodes[lane.node_indeces[i+1]].pos.x - testnode.pos.x)



#                         vT = Vec.polar(1.0, θ₂)
#                         T = VecE2(testnode.pos.x, testnode.pos.y)
#                         A = T - E

#                         c_p = dot(A,vE)
#                         if c_p > 0.0
#                             proj = vE * c_p
#                             perp_dist = hypot(A - proj)
#                             line_dist = hypot(proj)
#                             Δθ = angledist(θ,θ₂)

#                             θ₃ = atan2(node.pos.y - testnode.pos.y, node.pos.x - testnode.pos.x)
#                             Δ₃ = angledist(θ,θ₃)

#                             score = Δθ + 10.0*Δ₃

#                             if line_dist < THRESHOLD_HANGING_EDGE_LINE_DIST_TO_CONNECT &&
#                                 Δθ < THRESHOLD_HANGING_EDGE_ANGLE_DIFFERENCE &&
#                                 Δ₃ < THRESHOLD_HANGING_EDGE_ANGLE_DIFFERENCE2 &&
#                                 score < best_score

#                                 best_score = score
#                                 best_node_index = testnode_index
#                             end
#                         end
#                     end
#                 end
#             end

#             if node_index != best_node_index
#                 verbosity > 1 || println("ADDED START NODE HANGING EDGE ", node.pos.x, "  ", node.pos.y)
#                 add_edge!(G, best_node_index, node_index)
#             end
#         end
#     end

#     # check that each node from the graph is in a matching lane in its tile
#     for node_index in vertices(G)
#         node = sn.nodes[node_index]
#         tile = get_tile(sn, node)
#         if !has_lane(tile, node.id)
#             println("MISSING ", node.id)
#         end
#     end

#     # check that each lane is non-empty
#     for tile in values(sn.tile_dict)
#         for seg in values(tile.segments)
#             for lane in values(seg.lanes)
#                 @assert(!isempty(lane.node_indeces))
#             end
#         end
#     end

#     # compute curves for segments
#     # TODO(tim): maybe do this across tile boundaries for the entire lane at a time
#     verbosity < 1 || @printf("COMPUTE CURVES [%.2f]\n", time()-starttime)
#     for tile in values(sn.tile_dict)
#         for seg in values(tile.segments)
#             for key in keys(seg.lanes)

#                 lane = seg.lanes[key]

#                 node_indeces = lane.node_indeces
#                 node_index_prev = prev_node_index(sn, node_indeces[1])
#                 node_index_next = next_node_index(sn, node_indeces[end])
#                 has_prev = node_index_prev != node_indeces[1]
#                 has_next = node_index_next != node_indeces[end]

#                 n_nodes = length(node_indeces) + has_prev + has_next
#                 if n_nodes < 2
#                     println("DELETING LANE WITH $n_nodes nodes")
#                     delete!(seg.lanes, key)
#                     continue
#                 end

#                 pts = Array(Float64, 2, n_nodes)
#                 total = 0
#                 if has_prev
#                     total += 1
#                     node_prev = sn.nodes[node_index_prev]
#                     pts[1,total] = node_prev.pos.x
#                     pts[2,total] = node_prev.pos.y
#                 end
#                 for node_index in node_indeces
#                     total += 1
#                     node = sn.nodes[node_index]
#                     pts[1,total] = node.pos.x
#                     pts[2,total] = node.pos.y
#                 end
#                 if has_next
#                     total += 1
#                     node_next = sn.nodes[node_index_next]
#                     pts[1,total] = node_next.pos.x
#                     pts[2,total] = node_next.pos.y
#                 end

#                 if total > 1
#                     lane.curve = fit_curve(pts, lane.id.lane, DESIRED_DISTANCE_BETWEEN_CURVE_SAMPLES)
#                     if has_prev
#                         # adjust s so dist to first node is 0.0
#                         c = lane.curve
#                         extind = Curves.closest_point_extind_to_curve(c, pts[1,2], pts[2,2])
#                         pt = Curves.curve_at(c, extind)
#                         lane.curve = Curve(lane.id.lane, c.s - pt.s, c.x, c.y, c.t, c.k, c.k_d)
#                     end
#                     lane.has_leading_node = has_prev
#                     lane.has_trailing_node = has_next
#                 else
#                     error("LANE WITH ONE POINT!")
#                 end
#             end
#         end
#     end

#     # fix the extind and d_along for each waypoint
#     verbosity < 1 || @printf("COMPUTE NODE EXTIND AND D_ALONG [%.2f]\n", time()-starttime)
#     for tile in values(sn.tile_dict)
#         for seg in values(tile.segments)
#             for lane in values(seg.lanes)
#                 guess = 1.0
#                 for node_index in lane.node_indeces
#                     node = sn.nodes[node_index]
#                     @assert(length(lane.curve) > 1)
#                     extind = closest_point_extind_to_curve_guess(lane.curve, node.pos.x, node.pos.y, guess)
#                     footpoint = curve_at(lane.curve, extind)
#                     d_along = footpoint.s

#                     @assert(!isnan(d_along))

#                     sn.nodes[node_index] = StreetNode(node.id,
#                                                       VecSE2(node.pos.x,node.pos.y,footpoint.θ),
#                                                       extind,
#                                                       d_along,
#                                                       node.d_merge,
#                                                       node.d_split,
#                                                       node.d_end,
#                                                       node.n_lanes_left,
#                                                       node.n_lanes_right,
#                                                       node.marker_dist_left,
#                                                       node.marker_dist_right
#                                                       )
#                 end
#             end
#         end
#     end

#     # compute remaining values for each waypoint
#     verbosity < 1 || @printf("COMPUTE NODE D_ VALUES [%.2f]\n", time()-starttime)
#     for tile in values(sn.tile_dict)
#         for seg in values(tile.segments)
#             for lane in values(seg.lanes)
#                 for node_index in lane.node_indeces

#                     node = sn.nodes[node_index]

#                     d_end   = _distance_to_lane_end(sn, seg, lane.id.lane, node.extind)
#                     d_split = _distance_to_lane_split(sn, seg, lane.id.lane, node.extind)
#                     d_merge = _distance_to_lane_merge(sn, seg, lane.id.lane, node.extind)
#                     n_lanes_left, marker_dist_left, n_lanes_right, marker_dist_right = _calc_num_lanes_on_side(sn, tile, node_index)

#                     @assert(n_lanes_left ≥ 0)
#                     @assert(n_lanes_left ≥ 0)
#                     @assert(marker_dist_left > 0.0)
#                     @assert(marker_dist_right > 0.0)

#                     sn.nodes[node_index] = StreetNode(node.id,
#                                                       node.pos,
#                                                       node.extind,
#                                                       node.d_along,
#                                                       d_merge,
#                                                       d_split,
#                                                       d_end,
#                                                       n_lanes_left,
#                                                       n_lanes_right,
#                                                       marker_dist_left,
#                                                       marker_dist_right
#                                                       )
#                 end
#             end
#         end
#     end

#     verbosity < 1 || @printf("DONE [%.2f]\n", time()-starttime)
#     sn
# end
