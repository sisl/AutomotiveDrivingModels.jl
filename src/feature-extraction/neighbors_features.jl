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
    find_neighbor(scene::Frame, roadway::Roawday, ego::Entity; kwargs...)

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
function find_neighbor(scene::Frame, roadway::Roadway, ego::Entity{S,D,I};
                       lane::Union{Nothing, Lane} = get_lane(roadway, ego),
                       rear::Bool=false, 
                       max_distance::Float64=250.0, 
                       targetpoint_ego::VehicleTargetPoint = VehicleTargetPointCenter(),
                       targetpoint_neighbor::VehicleTargetPoint = VehicleTargetPointCenter(),
                       ids_to_ignore::Union{Nothing, Set{I}} = nothing) where {S,D,I}
    
   
    if lane == nothing 
        return NeighborLongitudinalResult(nothing, max_distance)
    elseif get_lane(roadway, ego).tag == lane.tag
        tag_start = lane.tag 
    else  # project ego on desired lane
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
            s_base = curr_lane.curve[end].s + norm(VecE2(lane.curve[end].pos - prev_lane_point(curr_lane, roadway).pos))
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
    get_frenet_relative_position(veh_fore::Entity, veh_rear::Entity, roadway::Roadway)
return the Frenet relative position between the two vehicles. It projects the position of the first vehicle onto the lane of the second vehicle.
The result is stored as a `FrenetRelativePosition`.

Lower level:
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

"""
    dist_to_front_neighbor(roadway::Roadway, scene::Frame, veh::Entity)
Feature function to extract the longitudinal distance to the front neighbor (in the Frenet frame).
Returns `missing` if there are no front neighbor.
"""
function dist_to_front_neighbor(roadway::Roadway, scene::Frame, veh::Entity)
    neighbor = find_neighbor(scene, roadway, veh)
    if neighbor.ind === nothing 
        return missing 
    else
        return neighbor.Δs
    end
end

"""
    front_neighbor_speed(roadway::Roadway, scene::Frame, veh::Entity)
Feature function to extract the velocity of the front neighbor.
Returns `missing` if there are no front neighbor.
"""
function front_neighbor_speed(roadway::Roadway, scene::Frame, veh::Entity)
    neighbor = find_neighbor(scene, roadway, veh)
    if neighbor.ind === nothing 
        return missing 
    else
        return vel(scene[neighbor.ind])
    end
end

"""
    time_to_collision(roadway::Roadway, scene::Frame, veh::Entity)
Feature function to extract the time to collision with the front neighbor.
Returns `missing` if there are no front neighbor.
"""
function time_to_collision(roadway::Roadway, scene::Frame, veh::Entity)
    neighbor = find_neighbor(scene, roadway, veh)
    if neighbor.ind === nothing 
        return missing
    else
        len_ego = length(veh.def)
        len_oth = length(scene[neighbor.ind].def)
        Δs = neighbor.Δs - len_ego/2 - len_oth/2
        Δv = vel(scene[neighbor.ind]) - vel(veh)
        return -Δs / Δv
    end
end
