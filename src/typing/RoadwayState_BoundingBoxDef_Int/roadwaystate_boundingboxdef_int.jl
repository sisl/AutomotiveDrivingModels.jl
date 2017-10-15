const Vehicle = Entity{RoadwayState,VehicleDef,Int}
const Scene = Frame{Vehicle}
const SceneRecord = QueueRecord{Vehicle}
const Trajdata = ListRecord{RoadwayState,VehicleDef,Int}

Scene(n::Int=100) = Frame(Vehicle, n)

get_center(veh::Vehicle) = veh.state.posG
get_footpoint(veh::Vehicle) = veh.state.posG + polar(veh.state.posF.t, veh.state.posG.θ-veh.state.posF.ϕ-π/2)
get_front(veh::Vehicle) = veh.state.posG + polar(veh.def.len/2, veh.state.posG.θ)
get_rear(veh::Vehicle) = veh.state.posG - polar(veh.def.len/2, veh.state.posG.θ)

function get_lane_width(veh::Vehicle, roadway::Roadway)

    lane = roadway[veh.state.tag]

    if n_lanes_left(lane, roadway) > 0
        footpoint = get_footpoint(veh)
        lane_left = roadway[LaneTag(lane.tag.segment, lane.tag.lane + 1)]
        return -proj(footpoint, lane_left, roadway).curveproj.t
    else
        return lane.width
    end
end
function get_markerdist_left(veh::Vehicle, roadway::Roadway)
    t = veh.state.posF.t
    lane_width = get_lane_width(veh, roadway)
    return lane_width/2 - t
end
function get_markerdist_right(veh::Vehicle, roadway::Roadway)
    t = veh.state.posF.t
    lane_width = get_lane_width(veh, roadway)
    return lane_width/2 + t
end

function _mod2pi2(x::Float64)
    val = mod2pi(x)
    if val > pi
        val -= 2pi
    end
    return val
end

Frenet(posG::VecSE2, roadway::Roadway) = Frenet(proj(posG, roadway), roadway)
function Frenet(roadind::RoadIndex, roadway::Roadway; t::Float64=0.0, ϕ::Float64=0.0)
    s = roadway[roadind].s
    ϕ = _mod2pi2(ϕ)
    Frenet(s, t, ϕ)
end
function Frenet(roadproj::RoadProjection, roadway::Roadway)
    roadind = RoadIndex(roadproj.curveproj.ind, roadproj.tag)
    s = roadway[roadind].s
    t = roadproj.curveproj.t
    ϕ = _mod2pi2(roadproj.curveproj.ϕ)
    Frenet(s, t, ϕ)
end


function RoadwayState(posG::VecSE2, roadway::Roadway, v::Float64)
    roadproj = proj(posG, roadway)
    posF = Frenet(roadproj, roadway)
    RoadwayState(posG, posF, roadproj.tag, v)
end
function RoadwayState(posF::Frenet, tag::LaneTag, roadway::Roadway, v::Float64)
    posG = get_global_position(posF, roadway[tag].curve)
    return RoadwayState(posG, posF, tag, v)
end


abstract type VehicleTargetPoint end
immutable VehicleTargetPointFront <: VehicleTargetPoint end
get_targetpoint_delta(::VehicleTargetPointFront, veh::Vehicle) = veh.def.len/2*cos(veh.state.posF.ϕ)
immutable VehicleTargetPointCenter <: VehicleTargetPoint end
get_targetpoint_delta(::VehicleTargetPointCenter, veh::Vehicle) = 0.0
immutable VehicleTargetPointRear <: VehicleTargetPoint end
get_targetpoint_delta(::VehicleTargetPointRear, veh::Vehicle) = -veh.def.len/2*cos(veh.state.posF.ϕ)

const VEHICLE_TARGET_POINT_CENTER = VehicleTargetPointCenter()

function get_neighbor_fore_along_lane(
    scene::Scene,
    roadway::Roadway,
    tag_start::LaneTag,
    s_base::Float64,
    targetpoint_primary::VehicleTargetPoint, # the reference point whose distance we want to minimize
    targetpoint_valid::VehicleTargetPoint; # the reference point, which if distance to is positive, we include the vehicle
    max_distance_fore::Float64 = 250.0, # max distance to search forward [m]
    index_to_ignore::Int=-1,
    )

    best_ind = 0
    best_dist = max_distance_fore
    tag_target = tag_start

    dist_searched = 0.0
    while dist_searched < max_distance_fore

        lane = roadway[tag_target]

        for (i,veh) in enumerate(scene)
            if i != index_to_ignore

                s_adjust = NaN

                roadind = RoadIndex(lane, veh.state.posF.s, roadway)

                if veh.state.tag == tag_target
                    s_adjust = 0.0

                elseif is_between_segments_hi(roadind.ind, lane.curve) &&
                       is_in_entrances(roadway[tag_target], veh.state.tag)

                    distance_between_lanes = abs(roadway[tag_target].curve[1].pos - roadway[veh.state.tag].curve[end].pos)
                    s_adjust = -(roadway[veh.state.tag].curve[end].s + distance_between_lanes)

                elseif is_between_segments_lo(roadind.ind) &&
                       is_in_exits(roadway[tag_target], veh.state.tag)

                    distance_between_lanes = abs(roadway[tag_target].curve[end].pos - roadway[veh.state.tag].curve[1].pos)
                    s_adjust = roadway[tag_target].curve[end].s + distance_between_lanes
                end

                if !isnan(s_adjust)
                    s_valid = veh.state.posF.s + get_targetpoint_delta(targetpoint_valid, veh) + s_adjust
                    dist_valid = s_valid - s_base + dist_searched
                    if dist_valid ≥ 0.0
                        s_primary = veh.state.posF.s + get_targetpoint_delta(targetpoint_primary, veh) + s_adjust
                        dist = s_primary - s_base + dist_searched
                        if dist < best_dist
                            best_dist = dist
                            best_ind = i
                        end
                    end
                end
            end
        end

        if best_ind != 0
            break
        end

        if !has_next(lane) ||
           (tag_target == tag_start && dist_searched != 0.0) # exit after visiting this lane a 2nd time
            break
        end

        dist_searched += (lane.curve[end].s - s_base)
        s_base = -abs(lane.curve[end].pos - next_lane_point(lane, roadway).pos) # negative distance between lanes
        tag_target = next_lane(lane, roadway).tag
    end

    NeighborLongitudinalResult(best_ind, best_dist)
end
function get_neighbor_fore_along_lane(
    scene::Scene,
    vehicle_index::Int,
    roadway::Roadway,
    targetpoint_ego::VehicleTargetPoint,
    targetpoint_primary::VehicleTargetPoint, # the reference point whose distance we want to minimize
    targetpoint_valid::VehicleTargetPoint; # the reference point, which if distance to is positive, we include the vehicle
    max_distance_fore::Float64 = 250.0 # max distance to search forward [m]
    )

    veh_ego = scene[vehicle_index]
    tag_start = veh_ego.state.tag
    s_base = veh_ego.state.posF.s + get_targetpoint_delta(targetpoint_ego, veh_ego)

    get_neighbor_fore_along_lane(scene, roadway, tag_start, s_base,
        targetpoint_primary, targetpoint_valid,
        max_distance_fore=max_distance_fore, index_to_ignore=vehicle_index)
end
function get_neighbor_fore_along_left_lane(
    scene::Scene,
    vehicle_index::Int,
    roadway::Roadway,
    targetpoint_ego::VehicleTargetPoint,
    targetpoint_primary::VehicleTargetPoint, # the reference point whose distance we want to minimize
    targetpoint_valid::VehicleTargetPoint; # the reference point, which if distance to is positive, we include the vehicle
    max_distance_fore::Float64 = 250.0 # max distance to search forward [m]
    )

    retval = NeighborLongitudinalResult(0, max_distance_fore)

    veh_ego = scene[vehicle_index]
    lane = roadway[veh_ego.state.tag]
    if n_lanes_left(lane, roadway) > 0
        lane_left = roadway[LaneTag(lane.tag.segment, lane.tag.lane + 1)]
        roadproj = proj(veh_ego.state.posG, lane_left, roadway)
        tag_start = roadproj.tag
        s_base = lane_left[roadproj.curveproj.ind, roadway].s + get_targetpoint_delta(targetpoint_ego, veh_ego)

        retval = get_neighbor_fore_along_lane(scene, roadway, tag_start, s_base,
                                              targetpoint_primary, targetpoint_valid,
                                              index_to_ignore=vehicle_index,
                                              max_distance_fore=max_distance_fore)
    end

    retval
end
function get_neighbor_fore_along_right_lane(
    scene::Scene,
    vehicle_index::Int,
    roadway::Roadway,
    targetpoint_ego::VehicleTargetPoint,
    targetpoint_primary::VehicleTargetPoint, # the reference point whose distance we want to minimize
    targetpoint_valid::VehicleTargetPoint; # the reference point, which if distance to is positive, we include the vehicle
    max_distance_fore::Float64 = 250.0 # max distance to search forward [m]
    )

    retval = NeighborLongitudinalResult(0, max_distance_fore)

    veh_ego = scene[vehicle_index]
    lane = roadway[veh_ego.state.tag]
    if n_lanes_right(lane, roadway) > 0
        lane_right = roadway[LaneTag(lane.tag.segment, lane.tag.lane - 1)]
        roadproj = proj(veh_ego.state.posG, lane_right, roadway)
        tag_start = roadproj.tag
        s_base = lane_right[roadproj.curveproj.ind, roadway].s + get_targetpoint_delta(targetpoint_ego, veh_ego)

        retval = get_neighbor_fore_along_lane(scene, roadway, tag_start, s_base,
                                              targetpoint_primary, targetpoint_valid,
                                              index_to_ignore=vehicle_index,
                                              max_distance_fore=max_distance_fore)
    end

    retval
end

function get_neighbor_fore_along_lane(
    scene::Scene,
    roadway::Roadway,
    tag_start::LaneTag,
    s_base::Float64;
    max_distance_fore::Float64 = 250.0, # max distance to search forward [m]
    index_to_ignore::Int=-1,
    )

    get_neighbor_fore_along_lane(scene, roadway, tag_start, s_base,
        VEHICLE_TARGET_POINT_CENTER, VEHICLE_TARGET_POINT_CENTER,
        max_distance_fore=max_distance_fore,
        index_to_ignore=index_to_ignore)
end
function get_neighbor_fore_along_lane(scene::Scene, vehicle_index::Int, roadway::Roadway;
    max_distance_fore::Float64 = 250.0 # max distance to search forward [m]
    )

    get_neighbor_fore_along_lane(scene, vehicle_index, roadway,
        VEHICLE_TARGET_POINT_CENTER, VEHICLE_TARGET_POINT_CENTER,
        VEHICLE_TARGET_POINT_CENTER, max_distance_fore=max_distance_fore)
end
function get_neighbor_fore_along_left_lane(scene::Scene, vehicle_index::Int, roadway::Roadway;
    max_distance_fore::Float64 = 250.0 # max distance to search forward [m]
    )

    get_neighbor_fore_along_left_lane(scene, vehicle_index, roadway,
        VEHICLE_TARGET_POINT_CENTER, VEHICLE_TARGET_POINT_CENTER,
        VEHICLE_TARGET_POINT_CENTER, max_distance_fore=max_distance_fore)
end
function get_neighbor_fore_along_right_lane(scene::Scene, vehicle_index::Int, roadway::Roadway;
    max_distance_fore::Float64 = 250.0 # max distance to search forward [m]
    )

    get_neighbor_fore_along_right_lane(scene, vehicle_index, roadway,
        VEHICLE_TARGET_POINT_CENTER, VEHICLE_TARGET_POINT_CENTER,
        VEHICLE_TARGET_POINT_CENTER, max_distance_fore=max_distance_fore)
end

function get_neighbor_rear_along_lane(
    scene::Scene,
    roadway::Roadway,
    tag_start::LaneTag,
    s_base::Float64,
    targetpoint_primary::VehicleTargetPoint, # the reference point whose distance we want to minimize
    targetpoint_valid::VehicleTargetPoint; # the reference point, which if distance to is positive, we include the vehicle
    max_distance_rear::Float64 = 250.0, # max distance to search rearward [m]
    index_to_ignore::Int=-1,
    )

    best_ind = 0
    best_dist = max_distance_rear
    tag_target = tag_start

    ignore = Set{Int}()

    dist_searched = 0.0
    while dist_searched < max_distance_rear

        lane = roadway[tag_target]

        for (i,veh) in enumerate(scene)
            if i != index_to_ignore && !in(veh.id, ignore)

                s_adjust = NaN

                roadind = RoadIndex(lane, veh.state.posF.s, roadway)

                if veh.state.tag == tag_target
                    s_adjust = 0.0

                elseif is_between_segments_hi(roadind.ind, lane.curve) &&
                       is_in_entrances(roadway[tag_target], veh.state.tag)

                    distance_between_lanes = abs(roadway[tag_target].curve[1].pos - roadway[veh.state.tag].curve[end].pos)
                    s_adjust = -(roadway[veh.state.tag].curve[end].s + distance_between_lanes)

                elseif is_between_segments_lo(roadind.ind) &&
                       is_in_exits(roadway[tag_target], veh.state.tag)

                    distance_between_lanes = abs(roadway[tag_target].curve[end].pos - roadway[veh.state.tag].curve[1].pos)
                    s_adjust = roadway[tag_target].curve[end].s + distance_between_lanes
                end

                if !isnan(s_adjust)
                    s_valid = veh.state.posF.s + get_targetpoint_delta(targetpoint_valid, veh) + s_adjust
                    dist_valid = s_base - s_valid + dist_searched
                    if dist_valid ≥ 0.0
                        s_primary = veh.state.posF.s + get_targetpoint_delta(targetpoint_primary, veh) + s_adjust
                        dist = s_base - s_primary + dist_searched
                        if dist < best_dist
                            best_dist = dist
                            best_ind = i
                        end
                    else
                        push!(ignore, veh.id)
                    end
                end
            end
        end

        if best_ind != 0
            break
        end

        if !has_prev(lane) ||
           (tag_target == tag_start && dist_searched != 0.0) # exit after visiting this lane a 2nd time
            break
        end

        dist_searched += s_base
        s_base = lane.curve[end].s + abs(lane.curve[end].pos - prev_lane_point(lane, roadway).pos) # end of prev lane plus crossover
        tag_target = prev_lane(lane, roadway).tag
    end

    NeighborLongitudinalResult(best_ind, best_dist)
end
function get_neighbor_rear_along_lane(
    scene::Scene,
    vehicle_index::Int,
    roadway::Roadway,
    targetpoint_ego::VehicleTargetPoint,
    targetpoint_primary::VehicleTargetPoint, # the reference point whose distance we want to minimize
    targetpoint_valid::VehicleTargetPoint; # the reference point, which if distance to is positive, we include the vehicle
    max_distance_rear::Float64 = 250.0 # max distance to search rearward [m]
    )

    veh_ego = scene[vehicle_index]
    tag_start = veh_ego.state.tag
    s_base = veh_ego.state.posF.s + get_targetpoint_delta(targetpoint_ego, veh_ego)

    get_neighbor_rear_along_lane(scene, roadway, tag_start, s_base,
        targetpoint_primary, targetpoint_valid,
        max_distance_rear=max_distance_rear, index_to_ignore=vehicle_index)
end
function get_neighbor_rear_along_left_lane(
    scene::Scene,
    vehicle_index::Int,
    roadway::Roadway,
    targetpoint_ego::VehicleTargetPoint,
    targetpoint_primary::VehicleTargetPoint, # the reference point whose distance we want to minimize
    targetpoint_valid::VehicleTargetPoint; # the reference point, which if distance to is positive, we include the vehicle
    max_distance_rear::Float64 = 250.0 # max distance to search forward [m]
    )

    retval = NeighborLongitudinalResult(0, max_distance_rear)

    veh_ego = scene[vehicle_index]
    lane = roadway[veh_ego.state.tag]
    if n_lanes_left(lane, roadway) > 0
        lane_left = roadway[LaneTag(lane.tag.segment, lane.tag.lane + 1)]
        roadproj = proj(veh_ego.state.posG, lane_left, roadway)
        tag_start = roadproj.tag
        s_base = lane_left[roadproj.curveproj.ind, roadway].s + get_targetpoint_delta(targetpoint_ego, veh_ego)

        retval = get_neighbor_rear_along_lane(scene, roadway, tag_start, s_base,
                                              targetpoint_primary, targetpoint_valid,
                                              index_to_ignore=vehicle_index,
                                              max_distance_rear=max_distance_rear)
    end

    retval
end
function get_neighbor_rear_along_right_lane(
    scene::Scene,
    vehicle_index::Int,
    roadway::Roadway,
    targetpoint_ego::VehicleTargetPoint,
    targetpoint_primary::VehicleTargetPoint, # the reference point whose distance we want to minimize
    targetpoint_valid::VehicleTargetPoint; # the reference point, which if distance to is positive, we include the vehicle
    max_distance_rear::Float64 = 250.0 # max distance to search forward [m]
    )

    retval = NeighborLongitudinalResult(0, max_distance_rear)

    veh_ego = scene[vehicle_index]
    lane = roadway[veh_ego.state.tag]
    if n_lanes_right(lane, roadway) > 0
        lane_right = roadway[LaneTag(lane.tag.segment, lane.tag.lane - 1)]
        roadproj = proj(veh_ego.state.posG, lane_right, roadway)
        tag_start = roadproj.tag
        s_base = lane_right[roadproj.curveproj.ind, roadway].s + get_targetpoint_delta(targetpoint_ego, veh_ego)

        retval = get_neighbor_rear_along_lane(scene, roadway, tag_start, s_base,
                                              targetpoint_primary, targetpoint_valid,
                                              index_to_ignore=vehicle_index,
                                              max_distance_rear=max_distance_rear)
    end

    retval
end

function get_neighbor_rear_along_lane(
    scene::Scene,
    roadway::Roadway,
    tag_start::LaneTag,
    s_base::Float64;
    max_distance_rear::Float64 = 250.0, # max distance to search rearward [m]
    index_to_ignore::Int=-1,
    )

    get_neighbor_rear_along_lane(scene, roadway, tag_start, s_base,
        VEHICLE_TARGET_POINT_CENTER, VEHICLE_TARGET_POINT_CENTER,
        max_distance_rear=max_distance_rear,
        index_to_ignore=index_to_ignore)
end
function get_neighbor_rear_along_lane(scene::Scene, vehicle_index::Int, roadway::Roadway;
    max_distance_rear::Float64 = 250.0 # max distance to search forward [m]
    )

    get_neighbor_rear_along_lane(scene, vehicle_index, roadway,
        VEHICLE_TARGET_POINT_CENTER, VEHICLE_TARGET_POINT_CENTER,
        VEHICLE_TARGET_POINT_CENTER, max_distance_rear=max_distance_rear)
end
function get_neighbor_rear_along_left_lane(
    scene::Scene,
    vehicle_index::Int,
    roadway::Roadway;
    max_distance_rear::Float64 = 250.0 # max distance to search forward [m]
    )

    get_neighbor_rear_along_left_lane(scene, vehicle_index, roadway,
        VEHICLE_TARGET_POINT_CENTER, VEHICLE_TARGET_POINT_CENTER,
        VEHICLE_TARGET_POINT_CENTER, max_distance_rear=max_distance_rear)
end
function get_neighbor_rear_along_right_lane(
    scene::Scene,
    vehicle_index::Int,
    roadway::Roadway;
    max_distance_rear::Float64 = 250.0 # max distance to search forward [m]
    )

    get_neighbor_rear_along_right_lane(scene, vehicle_index, roadway,
        VEHICLE_TARGET_POINT_CENTER, VEHICLE_TARGET_POINT_CENTER,
        VEHICLE_TARGET_POINT_CENTER, max_distance_rear=max_distance_rear)
end

"""
    Project the given point to the same lane as the given RoadIndex.
This will return the projection of the point, along with the Δs along the lane from the RoadIndex.
"""
immutable FrenetRelativePosition
    origin::RoadIndex
    target::RoadIndex
    Δs::Float64
    t::Float64
    ϕ::Float64
end
function get_frenet_relative_position(posG::VecSE2, roadind::RoadIndex, roadway::Roadway;
    max_distance_fore::Float64 = 250.0, # max distance to search forward [m]
    max_distance_rear::Float64 = 250.0, # max distance to search backward [m]
    improvement_threshold::Float64 = 1e-4,
    )

    # project to current lane first
    tag_start = roadind.tag
    lane_start = roadway[tag_start]
    curveproj_start = proj(posG, lane_start, roadway, move_along_curves=false).curveproj
    curvept_start = lane_start[curveproj_start.ind, roadway]
    s_base = lane_start[roadind.ind, roadway].s
    s_proj = curvept_start.s
    Δs = s_proj - s_base
    sq_dist_to_curve = abs2(posG - curvept_start.pos)
    retval = FrenetRelativePosition(roadind,
                RoadIndex(curveproj_start.ind, tag_start), Δs, curveproj_start.t, curveproj_start.ϕ)

    # search downstream
    if has_next(lane_start)
        dist_searched = lane_start.curve[end].s - s_base
        s_base = -abs(lane_start.curve[end].pos - next_lane_point(lane_start, roadway).pos) # negative distance between lanes
        tag_target = next_lane(lane_start, roadway).tag
        while dist_searched < max_distance_fore

            lane = roadway[tag_target]
            curveproj = proj(posG, lane, roadway, move_along_curves=false).curveproj
            sq_dist_to_curve2 = abs2(posG - lane[curveproj.ind, roadway].pos)

            if sq_dist_to_curve2 < sq_dist_to_curve - improvement_threshold

                sq_dist_to_curve = sq_dist_to_curve2
                s_proj = lane[curveproj.ind, roadway].s
                Δs = s_proj - s_base + dist_searched
                retval = FrenetRelativePosition(
                            roadind, RoadIndex(curveproj.ind, tag_target),
                            Δs, curveproj.t, curveproj.ϕ)
            end

            if !has_next(lane)
                break
            end

            dist_searched += (lane.curve[end].s - s_base)
            s_base = -abs(lane.curve[end].pos - next_lane_point(lane, roadway).pos) # negative distance between lanes
            tag_target = next_lane(lane, roadway).tag

            if tag_target == tag_start
                break
            end
        end
    end

    # search upstream
    if has_prev(lane_start)
        dist_searched = s_base
        tag_target = prev_lane(lane_start, roadway).tag
        s_base = roadway[tag_target].curve[end].s + abs(lane_start.curve[1].pos - prev_lane_point(lane_start, roadway).pos) # end of the lane

        while dist_searched < max_distance_fore

            lane = roadway[tag_target]
            curveproj = proj(posG, lane, roadway, move_along_curves=false).curveproj
            sq_dist_to_curve2 = abs2(posG - lane[curveproj.ind, roadway].pos)

            if sq_dist_to_curve2 < sq_dist_to_curve - improvement_threshold

                sq_dist_to_curve = sq_dist_to_curve2
                s_proj = lane[curveproj.ind, roadway].s
                dist = s_base - s_proj  + dist_searched
                Δs = -dist
                retval = FrenetRelativePosition(
                            roadind, RoadIndex(curveproj.ind, tag_target),
                            Δs, curveproj.t, curveproj.ϕ)
            end

            if !has_prev(lane)
                break
            end

            dist_searched += s_base
            s_base = lane.curve[end].s + abs(lane.curve[1].pos - prev_lane_point(lane, roadway).pos) # length of this lane plus crossover
            tag_target = prev_lane(lane, roadway).tag

            if tag_target == tag_start
                break
            end
        end
    end

    retval
end
get_frenet_relative_position(veh_fore::Vehicle, veh_rear::Vehicle, roadway::Roadway) = get_frenet_relative_position(veh_fore.state.posG, veh_rear.state.posF.roadind, roadway)