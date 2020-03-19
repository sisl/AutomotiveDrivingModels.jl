
"""
    leftlane(roadway::Roadway, veh::Entity)
returns the lane to the left of the lane veh is currently in, returns nothing if it does not exists
"""
function leftlane(roadway::Roadway, veh::Entity)
    return leftlane(roadway, get_lane(roadway, veh))
end

"""
    rightlane(roadway::Roadway, veh::Entity)
returns the lane to the right of the lane veh is currently in, returns nothing if it does not exists
"""
function rightlane(roadway::Roadway, veh::Entity)
    return rightlane(roadway, get_lane(roadway, veh))
end

n_lanes_left(roadway::Roadway, veh::Entity) = n_lanes_left(roadway, get_lane(roadway, veh))

n_lanes_right(roadway::Roadway, veh::Entity) = n_lanes_right(roadway, get_lane(roadway, veh))


"""
    lane_width(roadway::Roadway, veh::Entity)
Returns the width of the lane where `veh` is.
"""
function lane_width(roadway::Roadway, veh::Entity)

    lane = get_lane(roadway, veh.state)

    if n_lanes_left(roadway, veh) > 0
        footpoint = get_footpoint(veh)
        lane_left = roadway[LaneTag(lane.tag.segment, lane.tag.lane + 1)]
        return -proj(footpoint, lane_left, roadway).curveproj.t
    else
        return lane.width
    end
end

"""
    markerdist_left(roadway::Roadway, veh::Entity)
distance of `veh` to the left marker of the lane
"""
function markerdist_left(roadway::Roadway, veh::Entity)
    t = posf(veh.state).t
    lw = lane_width(roadway, veh)
    return lw/2 - t
end

"""
    markerdist_left(roadway::Roadway, veh::Entity)
distance of `veh` to the right marker of the lane
"""
function markerdist_right(roadway::Roadway, veh::Entity)
    t = posf(veh.state).t
    lw = lane_width(roadway, veh)
    return lw/2 + t
end

"""
    road_edge_dist_left(roadway::Roadway, veh::Entity)
feature function, extract the lateral distance to the left edge of the road
"""
function road_edge_dist_left(roadway::Roadway, veh::Entity)
    offset = posf(veh).t
    footpoint = get_footpoint(veh)
    lane = get_lane(roadway, veh)
    roadproj = proj(footpoint, lane, roadway)
    curvept = roadway[RoadIndex(roadproj)]
    lane = roadway[roadproj.tag]
    return lane.width/2 + norm(VecE2(curvept.pos - footpoint)) - offset
end

"""
    road_edge_dist_right(roadway::Roadway, veh::Entity)
feature function, extract the lateral distance to the right edge of the road
"""
function road_edge_dist_right(roadway::Roadway, veh::Entity)
    offset = posf(veh).t
    footpoint = get_footpoint(veh)
    lane = get_lane(roadway, veh)
    roadproj = proj(footpoint, lane, roadway)
    curvept = roadway[RoadIndex(roadproj)]
    lane = roadway[roadproj.tag]
    return lane.width/2 + norm(VecE2(curvept.pos - footpoint)) + offset
end

function lane_offset_left(roadway::Roadway, veh::Entity)
    t = posf(veh).t
    lane = get_lane(roadway, veh)
    if n_lanes_left(roadway, lane) > 0
        lane_offset = t - lane.width/2 - leftlane(roadway, veh).width/2
        return lane_offset
    else
        missing
    end
end

function lane_offset_right(roadway::Roadway, veh::Entity)
    t = posf(veh).t
    lane = get_lane(roadway, veh)
    if n_lanes_right(roadway, lane) > 0
        lane_offset = t + lane.width/2 + rightlane(roadway, veh).width/2
        return lane_offset
    else
        missing
    end
end

"""
    has_lane_right(roadway::Roadway, veh::Entity)
Return true if `veh` has a lane on its right.
"""
function has_lane_right(roadway::Roadway, veh::Entity)
    return n_lanes_right(roadway, veh) > 0
end

"""
    has_lane_left(roadway::Roadway, veh::Entity)
Return true if `veh` has a lane on its left.
"""
function has_lane_left(roadway::Roadway, veh::Entity)
    return n_lanes_left(roadway, veh) > 0
end

"""
    lane_curvature(roadway::Roadway, veh::Entity)
Return the curvature of the lane at `veh`'s position.
Return missing if the curvature is `NaN`
"""
function lane_curvature(roadway::Roadway, veh::Entity)
    curvept = roadway[posf(veh.state).roadind]
    val = curvept.k
    if isnan(val)
        return missing 
    end
    return val
end
