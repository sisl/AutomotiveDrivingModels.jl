"""
    get_lane_width(veh::Vehicle, roadway::Roadway)
Returns the width of the lane where `veh` is.
"""
function get_lane_width(veh::Entity{VehicleState, D, I}, roadway::Roadway) where {D, I}

    lane = roadway[veh.state.posF.roadind.tag]

    if n_lanes_left(lane, roadway) > 0
        footpoint = get_footpoint(veh)
        lane_left = roadway[LaneTag(lane.tag.segment, lane.tag.lane + 1)]
        return -proj(footpoint, lane_left, roadway).curveproj.t
    else
        return lane.width
    end
end

"""
    get_markerdist_left(veh::Vehicle, roadway::Roadway)
distance of `veh` to the left marker of the lane
"""
function get_markerdist_left(veh::Entity{VehicleState, D, I}, roadway::Roadway) where {D, I}
    t = veh.state.posF.t
    lane_width = get_lane_width(veh, roadway)
    return lane_width/2 - t
end

"""
    get_markerdist_left(veh::Vehicle, roadway::Roadway)
distance of `veh` to the right marker of the lane
"""
function get_markerdist_right(veh::Entity{VehicleState, D, I}, roadway::Roadway) where {D, I}
    t = veh.state.posF.t
    lane_width = get_lane_width(veh, roadway)
    return lane_width/2 + t
end
