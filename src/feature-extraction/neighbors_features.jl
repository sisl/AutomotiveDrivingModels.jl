"""
    NeighborLongitudinalResult

A structure to retrieve information about a neihbor in the longitudinal direction i.e. rear and front neighbors on the same lane.
If the neighbor index is equal to `nothing` it means there is no neighbor.

# Fields
- `ind::Union{Nothing, Int64}` index of the neighbor in the scene
- `Δs::Float64` positive distance along the lane between vehicles positions
"""
struct NeighborLongitudinalResult
    ind::Union{Nothing, Int64} # index in scene of the neighbor
    Δs::Float64 # positive distance along lane between vehicles' positions
end

"""
    VehicleTargetPoint 

Defines a point on an entity that is used to measure distances. 
The following target points are supported and are subtypes of VehicleTargetPoint: 
- `VehicleTargetPointFront`
- `VehicleTargetPointCenter`
- `VehicleTargetPointRear`

The method `targetpoint_delta(::VehicleTargetPoint, ::Entity)` can be used to 
compute the delta in the longitudinal direction to add when considering a specific target point.
"""
abstract type VehicleTargetPoint end

"""
    VehicleTargetPointFront <: VehicleTargetPoint
Set the target point at the front (and center) of the entity.
"""
struct VehicleTargetPointFront <: VehicleTargetPoint end

targetpoint_delta(::VehicleTargetPointFront, veh::Entity{S, D, I}) where {S,D<:AbstractAgentDefinition, I} = length(veh.def)/2*cos(posf(veh.state).ϕ)

"""
    VehicleTargetPointCenter <: VehicleTargetPoint
Set the target point at the center of the entity.
"""
struct VehicleTargetPointCenter <: VehicleTargetPoint end

targetpoint_delta(::VehicleTargetPointCenter, veh::Entity{S, D, I}) where {S,D<:AbstractAgentDefinition, I} = 0.0

"""
    VehicleTargetPointFront <: VehicleTargetPoint
Set the target point at the front (and center) of the entity.
"""
struct VehicleTargetPointRear <: VehicleTargetPoint end

targetpoint_delta(::VehicleTargetPointRear, veh::Entity{S, D, I}) where {S,D<:AbstractAgentDefinition, I} = -length(veh.def)/2*cos(posf(veh.state).ϕ)

const VEHICLE_TARGET_POINT_CENTER = VehicleTargetPointCenter()

"""
    findneighbor(scene::Frame, roadway::Roawday, ego::Entity; kwargs...)

Search through lanes and road segments to find a neighbor of `ego` in the `scene`. 

# Arguments

- `scene::Frame` the scene containing all the entities
- `roadway::Roadway` the road topology on which entities are driving 
- `ego::Entity` the entity that we want to compute the neighbor of.

# Keyword arguments

- `rear::Bool = false` set to true to search for rear neighbor, search forward neighbor by default 
- `max_distance::Float64 = 250.0` stop searching after this distance is reached, if the neighbor is further than max_distance, returns nothing
- `targetpoint_ego::VehicleTargetPoint` the point on the ego vehicle used for distance calculation, see `VehicleTargetPoint` for more info
- `targetpoint_neighbor::VehicleTargetPoint` the point on the neighbor vehicle used for distance calculation, see `VehicleTargetPoint` for more info
- `ids_to_ignore::Union{Nothing, Set{I}} = nothing` a list of entity ids to ignore for the search, ego is always ignored.
"""
function findneighbor(scene::Frame, roadway::Roadway, ego::Entity{S,D,I};
                       lane::Lane = get_lane(roadway, ego),
                       rear::Bool=false, 
                       max_distance::Float64=250.0, 
                       targetpoint_ego::VehicleTargetPoint = VehicleTargetPointCenter(),
                       targetpoint_neighbor::VehicleTargetPoint = VehicleTargetPointCenter(),
                       ids_to_ignore::Union{Nothing, Set{I}} = nothing) where {S,D,I}
    
    # project ego on desired lane
    if get_lane(roadway, ego).tag == lane.tag
        tag_start = lane.tag 
    else
        roadproj = proj(posg(ego.state), lane, roadway)
        tag_start = roadproj.tag 
    end
    s_base = posf(ego.state).s + targetpoint_delta(targetpoint_ego, ego)
    
    tag_target = tag_start
    best_ind = nothing 
    best_dist = max_distance
    
    dist_searched = 0.0
    while dist_searched < max_distance 

        curr_lane = roadway[tag_target]

        for (i, veh) in enumerate(scene)
            
            if veh.id == ego.id 
                continue
            end

            if ids_to_ignore !== nothing && veh.id ∈ ids_to_ignore
                continue
            end

            # check if veh is on thislane  
            s_adjust = NaN 

            if get_lane(roadway, veh).tag == curr_lane.tag
                s_adjust = 0.0
            
            elseif is_between_segments_hi(posf(veh.state).roadind.ind, curr_lane.curve) &&
                   is_in_entrances(roadway[tag_target], posf(veh.state).roadind.tag)
            
                distance_between_lanes = norm(VecE2(roadway[tag_target].curve[1].pos - roadway[posf(veh.state).roadind.tag].curve[end].pos))
                s_adjust = -(roadway[posf(veh.state).roadind.tag].curve[end].s + distance_between_lanes)
            
            elseif is_between_segments_lo(posf(veh.state).roadind.ind) &&
                    is_in_exits(roadway[tag_target], posf(veh.state).roadind.tag)
            
                distance_between_lanes = norm(VecE2(roadway[tag_target].curve[end].pos - roadway[posf(veh.state).roadind.tag].curve[1].pos))
                s_adjust = roadway[tag_target].curve[end].s + distance_between_lanes
            end

            if !isnan(s_adjust)
                s_valid = posf(veh.state).s + targetpoint_delta(targetpoint_neighbor, veh) + s_adjust
                if rear
                    dist_valid = s_base - s_valid + dist_searched 
                else
                    dist_valid = s_valid - s_base + dist_searched
                end
                
                if dist_valid ≥ 0.0
                    s_primary = posf(veh.state).s + targetpoint_delta(targetpoint_neighbor, veh) + s_adjust
                    if rear 
                        dist=  s_base - s_primary + dist_searched
                    else
                        dist = s_primary - s_base + dist_searched
                    end
                    if dist < best_dist
                        best_dist = dist
                        best_ind = i
                    end
                end
            end
        end

        # neighbor has been found exit
        if best_ind != nothing
            break
        end

        # no next lane and no neighbor found exit
        if !has_next(curr_lane) ||
            (tag_target == tag_start && dist_searched != 0.0) # exit after visiting this lane a 2nd time
            break
        end

        # go to the connected lane.
        if rear
            dist_searched += s_base
            s_base = curr_lane.curve[end].s + norm(VecE2(lane.cure[end].pos - prev_lane_point(curr_lane, roadway).pos))
            tag_target = prev_lane(lane, roadway).tag
        else
            dist_searched += (curr_lane.curve[end].s - s_base)
            s_base = -norm(VecE2(curr_lane.curve[end].pos - next_lane_point(curr_lane, roadway).pos)) # negative distance between lanes
            tag_target = next_lane(curr_lane, roadway).tag
        end
    end

    return NeighborLongitudinalResult(best_ind, best_dist)
end

"""
    get_neighbor_fore_along_lane(scene::EntityFrame, vehicle_index::Int, roadway::Roadway; max_distance_fore::Float64 = 250.0) 
    get_neighbor_fore_along_lane(scene::EntityFrame{S,D,I}, roadway::Roadway, tag_start::LaneTag, s_base::Float64, targetpoint_primary::VehicleTargetPoint, targetpoint_valid::VehicleTargetPoint;
                                 max_distance_fore::Float64 = 250.0, index_to_ignore::Int=-1) where {S,D<:AbstractAgentDefinition,I}
    
                                 get_neighbor_fore_along_lane(scene::EntityFrame{S,D,I}, vehicle_index::Int, roadway::Roadway, targetpoint_ego::VehicleTargetPoint, targetpoint_primary::VehicleTargetPoint, # the reference point whose distance we want to minimize
    targetpoint_valid::VehicleTargetPoint; # the reference point, which if distance to is positive, we include the vehicle
    max_distance_fore::Float64 = 250.0 # max distance to search forward [m]
    ) where {S,D<:AbstractAgentDefinition,I}

Return the index and the longitudinal distance of the vehicle that is in the same lane as scene[vehicle_index] and
in front of it with the smallest distance along the lane. The result is returned as a `NeighborLongitudinalResult` object.
The method will search on the current lane first, and if no vehicle is found it will continue to travel along the lane following next_lane(lane, roadway).
If no vehicle is found within `max_distance_fore,` the index takes a value of `nothing`.

# Notes on the optional arguments:
- `targetpoint_primary::VehicleTargetPoint` the reference point whose distance we want to minimize
- `targetpoint_valid::VehicleTargetPoint` the reference point, which if distance to is positive, we include the vehicle

"""
function get_neighbor_fore_along_lane(
    scene::EntityFrame{S,D,I},
    roadway::Roadway,
    tag_start::LaneTag,
    s_base::Float64,
    targetpoint_primary::VehicleTargetPoint, # the reference point whose distance we want to minimize
    targetpoint_valid::VehicleTargetPoint; # the reference point, which if distance to is positive, we include the vehicle
    max_distance_fore::Float64 = 250.0, # max distance to search forward [m]
    index_to_ignore::Int=-1,
    ) where {S,D<:AbstractAgentDefinition,I}
    best_ind = nothing
    best_dist = max_distance_fore
    tag_target = tag_start

    dist_searched = 0.0
    while dist_searched < max_distance_fore

        lane = roadway[tag_target]

        for (i,veh) in enumerate(scene)
            if i != index_to_ignore

                s_adjust = NaN
                if posf(veh.state).roadind.tag == tag_target
                    s_adjust = 0.0
                elseif is_between_segments_hi(posf(veh.state).roadind.ind, lane.curve) &&
                       is_in_entrances(roadway[tag_target], posf(veh.state).roadind.tag)
                    distance_between_lanes = norm(VecE2(roadway[tag_target].curve[1].pos - roadway[posf(veh.state).roadind.tag].curve[end].pos))
                    s_adjust = -(roadway[posf(veh.state).roadind.tag].curve[end].s + distance_between_lanes)

                elseif is_between_segments_lo(posf(veh.state).roadind.ind) &&
                       is_in_exits(roadway[tag_target], posf(veh.state).roadind.tag)
                    distance_between_lanes = norm(VecE2(roadway[tag_target].curve[end].pos - roadway[posf(veh.state).roadind.tag].curve[1].pos))
                    s_adjust = roadway[tag_target].curve[end].s + distance_between_lanes
                end

                if !isnan(s_adjust)
                    s_valid = posf(veh.state).s + targetpoint_delta(targetpoint_valid, veh) + s_adjust
                    dist_valid = s_valid - s_base + dist_searched
                    if dist_valid ≥ 0.0
                        s_primary = posf(veh.state).s + targetpoint_delta(targetpoint_primary, veh) + s_adjust
                        dist = s_primary - s_base + dist_searched
                        if dist < best_dist
                            best_dist = dist
                            best_ind = i
                        end
                    end
                end
            end
        end

        if best_ind != nothing
            break
        end

        if !has_next(lane) ||
           (tag_target == tag_start && dist_searched != 0.0) # exit after visiting this lane a 2nd time
            break
        end

        dist_searched += (lane.curve[end].s - s_base)
        s_base = -norm(VecE2(lane.curve[end].pos - next_lane_point(lane, roadway).pos)) # negative distance between lanes
        tag_target = next_lane(lane, roadway).tag
    end

    NeighborLongitudinalResult(best_ind, best_dist)
end
function get_neighbor_fore_along_lane(
    scene::EntityFrame{S,D,I},
    vehicle_index::Int,
    roadway::Roadway,
    targetpoint_ego::VehicleTargetPoint,
    targetpoint_primary::VehicleTargetPoint, # the reference point whose distance we want to minimize
    targetpoint_valid::VehicleTargetPoint; # the reference point, which if distance to is positive, we include the vehicle
    max_distance_fore::Float64 = 250.0 # max distance to search forward [m]
    ) where {S,D<:AbstractAgentDefinition,I}

    veh_ego = scene[vehicle_index]
    tag_start = posf(veh_ego.state).roadind.tag
    s_base = posf(veh_ego.state).s + targetpoint_delta(targetpoint_ego, veh_ego)

    get_neighbor_fore_along_lane(scene, roadway, tag_start, s_base,
        targetpoint_primary, targetpoint_valid,
        max_distance_fore=max_distance_fore, index_to_ignore=vehicle_index)
end
function get_neighbor_fore_along_lane(
    scene::EntityFrame{S,D,I},
    roadway::Roadway,
    tag_start::LaneTag,
    s_base::Float64;
    max_distance_fore::Float64 = 250.0, # max distance to search forward [m]
    index_to_ignore::Int=-1,
    ) where {S,D<:AbstractAgentDefinition,I}

    get_neighbor_fore_along_lane(scene, roadway, tag_start, s_base,
        VEHICLE_TARGET_POINT_CENTER, VEHICLE_TARGET_POINT_CENTER,
        max_distance_fore=max_distance_fore,
        index_to_ignore=index_to_ignore)
end
function get_neighbor_fore_along_lane(scene::EntityFrame, vehicle_index::Int, roadway::Roadway;
    max_distance_fore::Float64 = 250.0 # max distance to search forward [m]
    ) 

    get_neighbor_fore_along_lane(scene, vehicle_index, roadway,
        VEHICLE_TARGET_POINT_CENTER, VEHICLE_TARGET_POINT_CENTER,
        VEHICLE_TARGET_POINT_CENTER, max_distance_fore=max_distance_fore)
end

"""
    get_neighbor_fore_along_left_lane(scene::EntityFrame, vehicle_index::Int, roadway::Roadway; max_distance_fore::Float64 = 250.0 )
    get_neighbor_fore_along_left_lane(
    scene::EntityFrame{S,D,I},
    vehicle_index::Int,
    roadway::Roadway,
    targetpoint_ego::VehicleTargetPoint,
    targetpoint_primary::VehicleTargetPoint,
    targetpoint_valid::VehicleTargetPoint;
    max_distance_fore::Float64 = 250.0
    ) where {S,D<:AbstractAgentDefinition,I}

Returns the information about the front neighbor of a vehicle on its left lane. 

# Arguments

- `scene` the scene containing all vehicles information 
- `vehicle_index` the index of the vehicle in the scene for which we want to find the neighbor (ego vehicle)
- `roadway` the roadway layout 
- `targetpoint_ego` a target point on the ego vehicle that will be used to calculate the distance 
- `targetpoint_primary` the reference point whose distance we want to minimize
- `targetpoint_valid` the reference point, which if distance to is positive, we include the vehicle
- `max_distance_fore` max distance to search forward [m]
"""
function get_neighbor_fore_along_left_lane(
    scene::EntityFrame{S,D,I},
    vehicle_index::Int,
    roadway::Roadway,
    targetpoint_ego::VehicleTargetPoint,
    targetpoint_primary::VehicleTargetPoint, # the reference point whose distance we want to minimize
    targetpoint_valid::VehicleTargetPoint; # the reference point, which if distance to is positive, we include the vehicle
    max_distance_fore::Float64 = 250.0 # max distance to search forward [m]
    ) where {S,D<:AbstractAgentDefinition,I}

    retval = NeighborLongitudinalResult(nothing, max_distance_fore)

    veh_ego = scene[vehicle_index]
    lane = get_lane(roadway, veh_ego)
    if n_lanes_left(roadway, lane) > 0
        lane_left = roadway[LaneTag(lane.tag.segment, lane.tag.lane + 1)]
        roadproj = proj(posg(veh_ego.state), lane_left, roadway)
        tag_start = roadproj.tag
        s_base = lane_left[roadproj.curveproj.ind, roadway].s + targetpoint_delta(targetpoint_ego, veh_ego)

        retval = get_neighbor_fore_along_lane(scene, roadway, tag_start, s_base,
                                              targetpoint_primary, targetpoint_valid,
                                              index_to_ignore=vehicle_index,
                                              max_distance_fore=max_distance_fore)
    end

    retval
end
function get_neighbor_fore_along_left_lane(scene::EntityFrame, vehicle_index::Int, roadway::Roadway;
    max_distance_fore::Float64 = 250.0 # max distance to search forward [m]
    )

    get_neighbor_fore_along_left_lane(scene, vehicle_index, roadway,
        VEHICLE_TARGET_POINT_CENTER, VEHICLE_TARGET_POINT_CENTER,
        VEHICLE_TARGET_POINT_CENTER, max_distance_fore=max_distance_fore)
end

"""
    get_neighbor_fore_along_right_lane(scene::EntityFrame, vehicle_index::Int, roadway::Roadway; max_distance_fore::Float64 = 250.0 )
    get_neighbor_fore_along_right_lane(
    scene::EntityFrame{S,D,I},
    vehicle_index::Int,
    roadway::Roadway,
    targetpoint_ego::VehicleTargetPoint,
    targetpoint_primary::VehicleTargetPoint,
    targetpoint_valid::VehicleTargetPoint;
    max_distance_fore::Float64 = 250.0
    ) where {S,D<:AbstractAgentDefinition,I}

Returns the information about the front neighbor of a vehicle on its right lane. 

# Arguments

- `scene` the scene containing all vehicles information 
- `vehicle_index` the index of the vehicle in the scene for which we want to find the neighbor (ego vehicle)
- `roadway` the roadway layout 
- `targetpoint_ego` a target point on the ego vehicle that will be used to calculate the distance 
- `targetpoint_primary` the reference point whose distance we want to minimize
- `targetpoint_valid` the reference point, which if distance to is positive, we include the vehicle
- `max_distance_fore` max distance to search forward [m]
"""
function get_neighbor_fore_along_right_lane(
    scene::EntityFrame{S,D,I},
    vehicle_index::Int,
    roadway::Roadway,
    targetpoint_ego::VehicleTargetPoint,
    targetpoint_primary::VehicleTargetPoint, # the reference point whose distance we want to minimize
    targetpoint_valid::VehicleTargetPoint; # the reference point, which if distance to is positive, we include the vehicle
    max_distance_fore::Float64 = 250.0 # max distance to search forward [m]
    ) where {S,D<:AbstractAgentDefinition,I}

    retval = NeighborLongitudinalResult(nothing, max_distance_fore)

    veh_ego = scene[vehicle_index]
    lane = get_lane(roadway, veh_ego)
    if n_lanes_right(roadway, lane) > 0
        lane_right = roadway[LaneTag(lane.tag.segment, lane.tag.lane - 1)]
        roadproj = proj(posg(veh_ego.state), lane_right, roadway)
        tag_start = roadproj.tag
        s_base = lane_right[roadproj.curveproj.ind, roadway].s + targetpoint_delta(targetpoint_ego, veh_ego)

        retval = get_neighbor_fore_along_lane(scene, roadway, tag_start, s_base,
                                              targetpoint_primary, targetpoint_valid,
                                              index_to_ignore=vehicle_index,
                                              max_distance_fore=max_distance_fore)
    end

    retval
end
function get_neighbor_fore_along_right_lane(scene::EntityFrame, vehicle_index::Int, roadway::Roadway;
    max_distance_fore::Float64 = 250.0 # max distance to search forward [m]
    )

    get_neighbor_fore_along_right_lane(scene, vehicle_index, roadway,
        VEHICLE_TARGET_POINT_CENTER, VEHICLE_TARGET_POINT_CENTER,
        VEHICLE_TARGET_POINT_CENTER, max_distance_fore=max_distance_fore)
end

"""
    get_neighbor_fore_along_rear_lane(scene::EntityFrame, vehicle_index::Int, roadway::Roadway; max_distance_fore::Float64 = 250.0 )
    get_neighbor_fore_along_rear_lane(
    scene::EntityFrame{S,D,I},
    vehicle_index::Int,
    roadway::Roadway,
    targetpoint_ego::VehicleTargetPoint,
    targetpoint_primary::VehicleTargetPoint,
    targetpoint_valid::VehicleTargetPoint;
    max_distance_fore::Float64 = 250.0
    ) where {S,D<:AbstractAgentDefinition,I}

Returns the information about the rear neighbor of a vehicle on its own lane. 

# Arguments

- `scene` the scene containing all vehicles information 
- `vehicle_index` the index of the vehicle in the scene for which we want to find the neighbor (ego vehicle)
- `roadway` the roadway layout 
- `targetpoint_ego` a target point on the ego vehicle that will be used to calculate the distance 
- `targetpoint_primary` the reference point whose distance we want to minimize
- `targetpoint_valid` the reference point, which if distance to is positive, we include the vehicle
- `max_distance_fore` max distance to search forward [m]
"""
function get_neighbor_rear_along_lane(
    scene::EntityFrame{S,D,I},
    roadway::Roadway,
    tag_start::LaneTag,
    s_base::Float64,
    targetpoint_primary::VehicleTargetPoint, # the reference point whose distance we want to minimize
    targetpoint_valid::VehicleTargetPoint; # the reference point, which if distance to is positive, we include the vehicle
    max_distance_rear::Float64 = 250.0, # max distance to search rearward [m]
    index_to_ignore::Int=-1,
    ) where {S,D<:AbstractAgentDefinition,I}

    best_ind = nothing
    best_dist = max_distance_rear
    tag_target = tag_start

    ignore = Set{I}()

    dist_searched = 0.0
    while dist_searched < max_distance_rear

        lane = roadway[tag_target]

        for (i,veh) in enumerate(scene)
            if i != index_to_ignore && !in(veh.id, ignore)

                s_adjust = NaN

                if posf(veh.state).roadind.tag == tag_target
                    s_adjust = 0.0

                elseif is_between_segments_hi(posf(veh.state).roadind.ind, lane.curve) &&
                       is_in_entrances(roadway[tag_target], posf(veh.state).roadind.tag)

                    distance_between_lanes = norm(VecE2(roadway[tag_target].curve[1].pos - roadway[posf(veh.state).roadind.tag].curve[end].pos))
                    s_adjust = -(roadway[posf(veh.state).roadind.tag].curve[end].s + distance_between_lanes)

                elseif is_between_segments_lo(posf(veh.state).roadind.ind) &&
                       is_in_exits(roadway[tag_target], posf(veh.state).roadind.tag)

                    distance_between_lanes = norm(VecE2(roadway[tag_target].curve[end].pos - roadway[posf(veh.state).roadind.tag].curve[1].pos))
                    s_adjust = roadway[tag_target].curve[end].s + distance_between_lanes
                end

                if !isnan(s_adjust)
                    s_valid = posf(veh.state).s + targetpoint_delta(targetpoint_valid, veh) + s_adjust
                    dist_valid = s_base - s_valid + dist_searched
                    if dist_valid ≥ 0.0
                        s_primary = posf(veh.state).s + targetpoint_delta(targetpoint_primary, veh) + s_adjust
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

        if best_ind != nothing
            break
        end

        if !has_prev(lane) ||
           (tag_target == tag_start && dist_searched != 0.0) # exit after visiting this lane a 2nd time
            break
        end

        dist_searched += s_base
        s_base = lane.curve[end].s + norm(VecE2(lane.curve[end].pos - prev_lane_point(lane, roadway).pos)) # end of prev lane plus crossover
        tag_target = prev_lane(lane, roadway).tag
    end

    NeighborLongitudinalResult(best_ind, best_dist)
end
function get_neighbor_rear_along_lane(
    scene::EntityFrame{S,D,I},
    vehicle_index::Int,
    roadway::Roadway,
    targetpoint_ego::VehicleTargetPoint,
    targetpoint_primary::VehicleTargetPoint, # the reference point whose distance we want to minimize
    targetpoint_valid::VehicleTargetPoint; # the reference point, which if distance to is positive, we include the vehicle
    max_distance_rear::Float64 = 250.0 # max distance to search rearward [m]
    ) where {S,D<:AbstractAgentDefinition,I}

    veh_ego = scene[vehicle_index]
    tag_start = posf(veh_ego.state).roadind.tag
    s_base = posf(veh_ego.state).s + targetpoint_delta(targetpoint_ego, veh_ego)

    get_neighbor_rear_along_lane(scene, roadway, tag_start, s_base,
        targetpoint_primary, targetpoint_valid,
        max_distance_rear=max_distance_rear, index_to_ignore=vehicle_index)
end

function get_neighbor_rear_along_lane(
    scene::EntityFrame{S,D,I},
    roadway::Roadway,
    tag_start::LaneTag,
    s_base::Float64;
    max_distance_rear::Float64 = 250.0, # max distance to search rearward [m]
    index_to_ignore::Int=-1,
    ) where {S,D<:AbstractAgentDefinition,I}

    get_neighbor_rear_along_lane(scene, roadway, tag_start, s_base,
        VEHICLE_TARGET_POINT_CENTER, VEHICLE_TARGET_POINT_CENTER,
        max_distance_rear=max_distance_rear,
        index_to_ignore=index_to_ignore)
end
function get_neighbor_rear_along_lane(scene::EntityFrame{S,D,I}, vehicle_index::Int, roadway::Roadway;
    max_distance_rear::Float64 = 250.0 # max distance to search forward [m]
    ) where {S,D<:AbstractAgentDefinition,I}

    get_neighbor_rear_along_lane(scene, vehicle_index, roadway,
        VEHICLE_TARGET_POINT_CENTER, VEHICLE_TARGET_POINT_CENTER,
        VEHICLE_TARGET_POINT_CENTER, max_distance_rear=max_distance_rear)
end

function get_neighbor_rear_along_left_lane(
    scene::EntityFrame{S,D,I},
    vehicle_index::Int,
    roadway::Roadway,
    targetpoint_ego::VehicleTargetPoint,
    targetpoint_primary::VehicleTargetPoint, # the reference point whose distance we want to minimize
    targetpoint_valid::VehicleTargetPoint; # the reference point, which if distance to is positive, we include the vehicle
    max_distance_rear::Float64 = 250.0 # max distance to search forward [m]
    ) where {S,D<:AbstractAgentDefinition,I}

    retval = NeighborLongitudinalResult(nothing, max_distance_rear)

    veh_ego = scene[vehicle_index]
    lane = get_lane(roadway, veh_ego)
    if n_lanes_left(roadway, lane) > 0
        lane_left = roadway[LaneTag(lane.tag.segment, lane.tag.lane + 1)]
        roadproj = proj(posg(veh_ego.state), lane_left, roadway)
        tag_start = roadproj.tag
        s_base = lane_left[roadproj.curveproj.ind, roadway].s + targetpoint_delta(targetpoint_ego, veh_ego)

        retval = get_neighbor_rear_along_lane(scene, roadway, tag_start, s_base,
                                              targetpoint_primary, targetpoint_valid,
                                              index_to_ignore=vehicle_index,
                                              max_distance_rear=max_distance_rear)
    end

    retval
end
function get_neighbor_rear_along_left_lane(
    scene::EntityFrame{S,D,I},
    vehicle_index::Int,
    roadway::Roadway;
    max_distance_rear::Float64 = 250.0 # max distance to search forward [m]
    ) where {S,D<:AbstractAgentDefinition,I}

    get_neighbor_rear_along_left_lane(scene, vehicle_index, roadway,
        VEHICLE_TARGET_POINT_CENTER, VEHICLE_TARGET_POINT_CENTER,
        VEHICLE_TARGET_POINT_CENTER, max_distance_rear=max_distance_rear)
end


function get_neighbor_rear_along_right_lane(
    scene::EntityFrame{S,D,I},
    vehicle_index::Int,
    roadway::Roadway,
    targetpoint_ego::VehicleTargetPoint,
    targetpoint_primary::VehicleTargetPoint, # the reference point whose distance we want to minimize
    targetpoint_valid::VehicleTargetPoint; # the reference point, which if distance to is positive, we include the vehicle
    max_distance_rear::Float64 = 250.0 # max distance to search forward [m]
    ) where {S,D<:AbstractAgentDefinition,I}

    retval = NeighborLongitudinalResult(nothing, max_distance_rear)

    veh_ego = scene[vehicle_index]
    lane = get_lane(roadway, veh_ego)
    if n_lanes_right(roadway, lane) > 0
        lane_right = roadway[LaneTag(lane.tag.segment, lane.tag.lane - 1)]
        roadproj = proj(posg(veh_ego.state), lane_right, roadway)
        tag_start = roadproj.tag
        s_base = lane_right[roadproj.curveproj.ind, roadway].s + targetpoint_delta(targetpoint_ego, veh_ego)

        retval = get_neighbor_rear_along_lane(scene, roadway, tag_start, s_base,
                                              targetpoint_primary, targetpoint_valid,
                                              index_to_ignore=vehicle_index,
                                              max_distance_rear=max_distance_rear)
    end

    retval
end
function get_neighbor_rear_along_right_lane(
    scene::EntityFrame{S,D,I},
    vehicle_index::Int,
    roadway::Roadway;
    max_distance_rear::Float64 = 250.0 # max distance to search forward [m]
    ) where {S,D<:AbstractAgentDefinition,I}

    get_neighbor_rear_along_right_lane(scene, vehicle_index, roadway,
        VEHICLE_TARGET_POINT_CENTER, VEHICLE_TARGET_POINT_CENTER,
        VEHICLE_TARGET_POINT_CENTER, max_distance_rear=max_distance_rear)
end

"""
    FrenetRelativePosition

Contains information about the projection of a point on a lane. See `get_frenet_relative_position`.

# Fields 
- `origin::RoadIndex` original roadindex used for the projection, contains the target lane ID.
- `target::RoadIndex` roadindex reached after projection
- `Δs::Float64` longitudinal distance to the original roadindex 
- `t::Float64` lateral distance to the original roadindex in the frame of the target lane
- `ϕ::Float64` angle with the original roadindex in the frame of the target lane
"""
struct FrenetRelativePosition
    origin::RoadIndex
    target::RoadIndex
    Δs::Float64
    t::Float64
    ϕ::Float64
end

"""
    get_frenet_relative_position(posG::VecSE2{Float64}, roadind::RoadIndex, roadway::Roadway;
        max_distance_fore::Float64 = 250.0, # max distance to search forward [m]
        max_distance_rear::Float64 = 250.0, # max distance to search backward [m]
        improvement_threshold::Float64 = 1e-4,
    )

Project the given point to the same lane as the given RoadIndex.
This will return the projection of the point, along with the Δs along the lane from the RoadIndex.
The returned type is a `FrenetRelativePosition` object.
"""
function get_frenet_relative_position(posG::VecSE2{Float64}, roadind::RoadIndex, roadway::Roadway;
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
    sq_dist_to_curve = normsquared(VecE2(posG - curvept_start.pos))
    retval = FrenetRelativePosition(roadind,
                RoadIndex(curveproj_start.ind, tag_start), Δs, curveproj_start.t, curveproj_start.ϕ)

    # search downstream
    if has_next(lane_start)
        dist_searched = lane_start.curve[end].s - s_base
        s_base = -norm(VecE2(lane_start.curve[end].pos - next_lane_point(lane_start, roadway).pos)) # negative distance between lanes
        tag_target = next_lane(lane_start, roadway).tag
        while dist_searched < max_distance_fore

            lane = roadway[tag_target]
            curveproj = proj(posG, lane, roadway, move_along_curves=false).curveproj
            sq_dist_to_curve2 = normsquared(VecE2(posG - lane[curveproj.ind, roadway].pos))

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
            s_base = -norm(VecE2(lane.curve[end].pos - next_lane_point(lane, roadway).pos)) # negative distance between lanes
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
        s_base = roadway[tag_target].curve[end].s + norm(VecE2(lane_start.curve[1].pos - prev_lane_point(lane_start, roadway).pos)) # end of the lane

        while dist_searched < max_distance_fore

            lane = roadway[tag_target]
            curveproj = proj(posG, lane, roadway, move_along_curves=false).curveproj
            sq_dist_to_curve2 = normsquared(VecE2(posG - lane[curveproj.ind, roadway].pos))

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
            s_base = lane.curve[end].s + norm(VecE2(lane.curve[1].pos - prev_lane_point(lane, roadway).pos)) # length of this lane plus crossover
            tag_target = prev_lane(lane, roadway).tag

            if tag_target == tag_start
                break
            end
        end
    end

    retval
end
get_frenet_relative_position(veh_fore::Entity{S, D, I}, veh_rear::Entity{S, D, I}, roadway::Roadway) where {S,D, I} = get_frenet_relative_position(posg(veh_fore.state), posf(veh_rear.state).roadind, roadway)
