type Scene
    vehicles::Vector{Vehicle} # this is a pre-allocated array that is at least as large as the maximum number of vehicles in a Trajdata frame
    n_vehicles::Int

    function Scene(n_vehicles::Int=500)
        vehicles = Array(Vehicle, n_vehicles)
        for i in 1 : length(vehicles)
            vehicles[i] = Vehicle()
        end
        new(vehicles, 0)
    end
    function Scene(
        vehicles::Vector{Vehicle},
        n_vehicles::Int=length(vehicles);
        )

        new(vehicles, n_vehicles)
    end
end

# iteration
Base.start(scene::Scene) = 1
Base.done(scene::Scene, i::Int) = i > length(scene)
Base.next(scene::Scene, i::Int) = (scene.vehicles[i], i+1)

# copying
function Base.copy!(dest::Scene, src::Scene)
    for i in 1 : src.n_vehicles
        copy!(dest.vehicles[i], src.vehicles[i])
    end
    dest.n_vehicles = src.n_vehicles
    dest
end

function Base.push!(scene::Scene, veh::Vehicle)
    scene.n_vehicles += 1
    scene.vehicles[scene.n_vehicles] = veh
    scene
end

function get_first_available_id(scene::Scene)
    ids = Set{Int}([veh.def.id for veh in scene])
    id = 1
    while in(id, ids)
        id += 1
    end
    return id
end
Base.push!(scene::Scene, state::VehicleState) = push!(scene, Vehicle(state, VehicleDef(get_first_available_id(scene))))

Base.length(scene::Scene) = scene.n_vehicles
Base.getindex(scene::Scene, i::Int) = scene.vehicles[i]
Base.endof(scene::Scene) = scene.n_vehicles
function Base.setindex!(scene::Scene, veh::Vehicle, i::Int)
    scene.vehicles[i] = veh
    scene
end
function Base.empty!(scene::Scene)
    scene.n_vehicles = 0
    scene
end
function Base.get!(scene::Scene, trajdata::Trajdata, frame::Int)

    scene.n_vehicles = 0

    if frame_inbounds(trajdata, frame)
        tdframe = trajdata.frames[frame]
        for i in tdframe.lo : tdframe.hi
            scene.n_vehicles += 1
            veh = scene.vehicles[scene.n_vehicles]
            s = trajdata.states[i]
            veh.state = s.state
            veh.def = get_vehicledef(trajdata, s.id)
        end
    end

    scene
end
function Base.get!(scene::Scene, trajdata::Trajdata, frame_lo::Int, frame_hi::Int, γ::Float64)
    # add all vehicles that exist in both the lower and upper frames

    if frame_hi != frame_lo+1 && frame_hi != frame_lo
        # NOTE: frame_hi should pretty much always be frame_lo + 1
        #       the most frequent exception would be frame_lo = frame_hi
        #       you otherwise pretty much never want another case
        warn("get!(scene, trajdata, frame_lo, frame_hi, γ) used with frame_hi = frame_lo + $(frame_hi - frame_lo)!")
    end

    scene.n_vehicles = 0

    tdframe_lo = trajdata.frames[frame_lo]
    tdframe_hi = trajdata.frames[frame_hi]

    for i in tdframe_lo.lo : tdframe_lo.hi
        s = trajdata.states[i]
        for j in tdframe_hi.lo : tdframe_hi.hi
            s2 = trajdata.states[j]
            if s2.id == s.id # we are good
                # add it
                scene.n_vehicles += 1

                state_lo = s.state
                state_hi = s2.state

                veh = scene.vehicles[scene.n_vehicles]
                veh.state = lerp(state_lo, state_hi, γ, trajdata.roadway)
                veh.def = get_vehicledef(trajdata, s.id)
                break
            end
        end
    end

    scene
end
function Base.get!(scene::Scene, trajdata::Trajdata, time::Float64)

    # performs linear interpolation between frames

    i = searchsortedfirst(trajdata.frames, time, by=frame_or_time->isa(frame_or_time, TrajdataFrame) ? frame_or_time.t : frame_or_time) # index of first frame ≥ t_current

    if i == 1
        lo, hi = 1, 2
    elseif i ≤ nframes(trajdata)
        lo, hi = i-1, i
    else # t_current > all times in trajdata.frames
        lo, hi = i-2, i-1
    end

    t_lo = get_time(trajdata, lo)
    t_hi = get_time(trajdata, hi)
    γ = (time - t_lo) / (t_hi - t_lo)

    if isapprox(γ, 0.0)
        get!(scene, trajdata, lo)
    elseif isapprox(γ, 1.0)
        get!(scene, trajdata, hi)
    else
        get!(scene, trajdata, lo, hi, γ)
    end

    scene
end
function Base.deleteat!(scene::Scene, vehicle_index::Int)
    for i in vehicle_index : scene.n_vehicles - 1
        copy!(scene.vehicles[i], scene.vehicles[i+1])
    end
    scene.n_vehicles -= 1
    scene
end
Base.delete!(scene::Scene, veh::Vehicle) = deleteat!(scene, get_index_of_first_vehicle_with_id(scene, veh.def.id))

function get_by_id(scene::Scene, id::Int)
    for i in 1 : scene.n_vehicles
        if scene.vehicles[i].def.id == id
            return scene.vehicles[i]
        end
    end
    error("vehicle with id $id not found in scene")
end
function get_index_of_first_vehicle_with_id(scene::Scene, id::Int)
    retval = 0
    for i in 1 : scene.n_vehicles
        if scene.vehicles[i].def.id == id
            retval = i
            break
        end
    end
    retval
end
iscarinframe(scene::Scene, id::Int) = get_index_of_first_vehicle_with_id(scene, id) != 0

function get_vehiclestate(scene::Scene, id::Int)
    for i in 1 : scene.n_vehicles
        if scene.vehicles[i].def.id == id
            return scene.vehicles[i].state
        end
    end
    error("get_vehiclestate: vehicle with id $id not found")
end
function get_vehicle!(veh::Vehicle, scene::Scene, id::Int)
    for i in 1 : scene.n_vehicles
        if scene.vehicles[i].def.id == id
            copy!(veh, scene.vehicles[i])
            return veh
        end
    end
    error("get_vehicle!: vehicle with id $id not found")
end
get_vehicle(scene::Scene, id::Int) = get_vehicle!(Vehicle(), scene, id)

########################

"""
    get_neighbor_index_fore(scene::Scene, vehicle_index::Int, roadway::Roadway)
Return the index of the vehicle that is in the same lane as scene[vehicle_index] and
in front of it with the smallest distance along the lane

    The method will search on the current lane first, and if no vehicle is found it
    will continue to travel along the lane following next_lane(lane, roadway).
    If no vehicle is found within `max_distance_fore,` a value of 0 is returned instead.
"""
immutable NeighborLongitudinalResult
    ind::Int # index in scene of the neighbor
    Δs::Float64 # positive distance along lane between vehicles' positions
end

export
    VehicleTargetPoint,
    VehicleTargetPointFront,
    VehicleTargetPointCenter,
    VehicleTargetPointRear,
    get_targetpoint_delta

abstract VehicleTargetPoint
immutable VehicleTargetPointFront <: VehicleTargetPoint end
get_targetpoint_delta(::VehicleTargetPointFront, veh::Vehicle) = veh.def.length/2*cos(veh.state.posF.ϕ)
immutable VehicleTargetPointCenter <: VehicleTargetPoint end
get_targetpoint_delta(::VehicleTargetPointCenter, veh::Vehicle) = 0.0
immutable VehicleTargetPointRear <: VehicleTargetPoint end
get_targetpoint_delta(::VehicleTargetPointRear, veh::Vehicle) = -veh.def.length/2*cos(veh.state.posF.ϕ)

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
                if veh.state.posF.roadind.tag == tag_target
                    s_adjust = 0.0

                elseif is_between_segments_hi(veh.state.posF.roadind.ind, lane.curve) &&
                       is_in_entrances(roadway[tag_target], veh.state.posF.roadind.tag)

                    distance_between_lanes = abs(roadway[tag_target].curve[1].pos - roadway[veh.state.posF.roadind.tag].curve[end].pos)
                    s_adjust = -(roadway[veh.state.posF.roadind.tag].curve[end].s + distance_between_lanes)

                elseif is_between_segments_lo(veh.state.posF.roadind.ind) &&
                       is_in_exits(roadway[tag_target], veh.state.posF.roadind.tag)

                    distance_between_lanes = abs(roadway[tag_target].curve[end].pos - roadway[veh.state.posF.roadind.tag].curve[1].pos)
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
    tag_start = veh_ego.state.posF.roadind.tag
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
    lane = roadway[veh_ego.state.posF.roadind.tag]
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
    lane = roadway[veh_ego.state.posF.roadind.tag]
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
            if i != index_to_ignore && !in(veh.def.id, ignore)

                s_adjust = NaN

                if veh.state.posF.roadind.tag == tag_target
                    s_adjust = 0.0

                elseif is_between_segments_hi(veh.state.posF.roadind.ind, lane.curve) &&
                       is_in_entrances(roadway[tag_target], veh.state.posF.roadind.tag)

                    distance_between_lanes = abs(roadway[tag_target].curve[1].pos - roadway[veh.state.posF.roadind.tag].curve[end].pos)
                    s_adjust = -(roadway[veh.state.posF.roadind.tag].curve[end].s + distance_between_lanes)

                elseif is_between_segments_lo(veh.state.posF.roadind.ind) &&
                       is_in_exits(roadway[tag_target], veh.state.posF.roadind.tag)

                    distance_between_lanes = abs(roadway[tag_target].curve[end].pos - roadway[veh.state.posF.roadind.tag].curve[1].pos)
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
                        push!(ignore, veh.def.id)
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
    tag_start = veh_ego.state.posF.roadind.tag
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
    lane = roadway[veh_ego.state.posF.roadind.tag]
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
    lane = roadway[veh_ego.state.posF.roadind.tag]
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
