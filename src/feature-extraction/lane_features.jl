
function get_lane_width(veh::Vehicle, roadway::Roadway)

    lane = roadway[veh.state.posF.roadind.tag]

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