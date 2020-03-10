"""
    get_lane_width(veh::Entity, roadway::Roadway)
Returns the width of the lane where `veh` is.
"""
function get_lane_width(veh::Entity, roadway::Roadway)

    lane = get_lane(roadway, veh.state)

    if n_lanes_left(roadway, lane) > 0
        footpoint = get_footpoint(veh)
        lane_left = roadway[LaneTag(lane.tag.segment, lane.tag.lane + 1)]
        return -proj(footpoint, lane_left, roadway).curveproj.t
    else
        return lane.width
    end
end

"""
    get_markerdist_left(veh::Entity, roadway::Roadway)
distance of `veh` to the left marker of the lane
"""
function get_markerdist_left(veh::Entity, roadway::Roadway)
    t = posf(veh.state).t
    lane_width = get_lane_width(veh, roadway)
    return lane_width/2 - t
end

"""
    get_markerdist_left(veh::Entity, roadway::Roadway)
distance of `veh` to the right marker of the lane
"""
function get_markerdist_right(veh::Entity, roadway::Roadway)
    t = posf(veh.state).t
    lane_width = get_lane_width(veh, roadway)
    return lane_width/2 + t
end

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
