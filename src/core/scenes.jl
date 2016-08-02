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

Base.length(scene::Scene) = scene.n_vehicles
Base.getindex(scene::Scene, i::Int) = scene.vehicles[i]
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
        frame = trajdata.frames[frame]
        for i in frame.lo : frame.hi
            scene.n_vehicles += 1
            veh = scene.vehicles[scene.n_vehicles]
            s = trajdata.states[i]
            veh.state = s.state
            veh.def = get_vehicledef(trajdata, s.id)
        end
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
immutable NeighborForeResult
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
get_targetpoint_delta(::VehicleTargetPointFront, veh::Vehicle) = 0.0
immutable VehicleTargetPointRear <: VehicleTargetPoint end
get_targetpoint_delta(::VehicleTargetPointRear, veh::Vehicle) = -veh.def.length/2*cos(veh.state.posF.ϕ)

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

        for (i,veh) in enumerate(scene)
            if i != index_to_ignore && veh.state.posF.roadind.tag == tag_target
                s_valid = veh.state.posF.s + get_targetpoint_delta(targetpoint_valid, veh)
                dist_valid = s_valid - s_base + dist_searched
                if dist_valid ≥ 0.0
                    s_primary = veh.state.posF.s + get_targetpoint_delta(targetpoint_primary, veh)
                    dist = s_primary - s_base + dist_searched
                    if 0.0 ≤ dist < best_dist
                        best_dist = dist
                        best_ind = i
                    end
                end
            end
        end

        if best_ind != 0
            break
        end

        lane = roadway[tag_target]
        if !has_next(lane) ||
           (tag_target == tag_start && dist_searched != 0.0) # exit after visiting this lane a 2nd time
            break
        end

        dist_searched += (lane.curve[end].s - s_base)
        s_base = -abs(lane.curve[end].pos - next_lane_point(lane, roadway).pos) # negative distance between lanes
        tag_target = next_lane(lane, roadway).tag
    end

    NeighborForeResult(best_ind, best_dist)
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

    retval = NeighborForeResult(0, max_distance_fore)

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

    retval = NeighborForeResult(0, max_distance_fore)

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

    best_ind = 0
    best_dist = max_distance_fore
    tag_target = tag_start

    dist_searched = 0.0
    while dist_searched < max_distance_fore

        for (i,veh) in enumerate(scene)
            if i != index_to_ignore && veh.state.posF.roadind.tag == tag_target
                s_target = veh.state.posF.s
                dist = s_target - s_base + dist_searched
                if 0.0 ≤ dist < best_dist
                    best_dist = dist
                    best_ind = i
                end
            end
        end

        if best_ind != 0
            break
        end

        lane = roadway[tag_target]
        if !has_next(lane) ||
           (tag_target == tag_start && dist_searched != 0.0) # exit after visiting this lane a 2nd time
            break
        end

        dist_searched += (lane.curve[end].s - s_base)
        s_base = -abs(lane.curve[end].pos - next_lane_point(lane, roadway).pos) # negative distance between lanes
        tag_target = next_lane(lane, roadway).tag
    end

    NeighborForeResult(best_ind, best_dist)
end
function get_neighbor_fore_along_lane(scene::Scene, vehicle_index::Int, roadway::Roadway;
    max_distance_fore::Float64 = 250.0 # max distance to search forward [m]
    )

    veh_target = scene[vehicle_index]
    tag_start = veh_target.state.posF.roadind.tag
    s_base = veh_target.state.posF.s

    get_neighbor_fore_along_lane(scene, roadway, tag_start, s_base,
        max_distance_fore=max_distance_fore, index_to_ignore=vehicle_index)
end
function get_neighbor_fore_along_left_lane(scene::Scene, vehicle_index::Int, roadway::Roadway;
    max_distance_fore::Float64 = 250.0 # max distance to search forward [m]
    )

    retval = NeighborForeResult(0, max_distance_fore)

    veh_target = scene[vehicle_index]
    lane = roadway[veh_target.state.posF.roadind.tag]
    if n_lanes_left(lane, roadway) > 0
        lane_left = roadway[LaneTag(lane.tag.segment, lane.tag.lane + 1)]
        roadproj = proj(veh_target.state.posG, lane_left, roadway)
        tag_start = roadproj.tag
        s_base = lane_left[roadproj.curveproj.ind, roadway].s

        retval = get_neighbor_fore_along_lane(scene, roadway, tag_start, s_base,
                                              index_to_ignore=vehicle_index,
                                              max_distance_fore=max_distance_fore)
    end

    retval
end
function get_neighbor_fore_along_right_lane(scene::Scene, vehicle_index::Int, roadway::Roadway;
    max_distance_fore::Float64 = 250.0 # max distance to search forward [m]
    )

    retval = NeighborForeResult(0, max_distance_fore)

    veh_target = scene[vehicle_index]
    lane = roadway[veh_target.state.posF.roadind.tag]
    if n_lanes_right(lane, roadway) > 0
        lane_right = roadway[LaneTag(lane.tag.segment, lane.tag.lane - 1)]
        roadproj = proj(veh_target.state.posG, lane_right, roadway)
        tag_start = roadproj.tag
        s_base = lane_right[roadproj.curveproj.ind, roadway].s

        retval = get_neighbor_fore_along_lane(scene, roadway, tag_start, s_base,
                                                    index_to_ignore=vehicle_index,
                                                    max_distance_fore=max_distance_fore)
    end

    retval
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

    dist_searched = 0.0
    while dist_searched < max_distance_rear

        for (i,veh) in enumerate(scene)
            if i != index_to_ignore && veh.state.posF.roadind.tag == tag_target
                s_valid = veh.state.posF.s + get_targetpoint_delta(targetpoint_valid, veh)
                dist_valid = s_base - s_valid + dist_searched
                if dist_valid ≥ 0.0
                    s_primary = veh.state.posF.s + get_targetpoint_delta(targetpoint_primary, veh)
                    dist = s_base - s_primary + dist_searched
                    if 0.0 ≤ dist < best_dist
                        best_dist = dist
                        best_ind = i
                    end
                end
            end
        end

        if best_ind != 0
            break
        end

        lane = roadway[tag_target]
        if !has_prev(lane) ||
           (tag_target == tag_start && dist_searched != 0.0) # exit after visiting this lane a 2nd time
            break
        end

        dist_searched += s_base
        s_base = lane.curve[end].s + abs(lane.curve[end].pos - prev_lane_point(lane, roadway).pos) # end of prev lane plus crossover
        tag_target = prev_lane(lane, roadway).tag
    end

    NeighborForeResult(best_ind, best_dist)
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

    retval = NeighborForeResult(0, max_distance_rear)

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

    retval = NeighborForeResult(0, max_distance_rear)

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
