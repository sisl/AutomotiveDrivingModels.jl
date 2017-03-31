function track_longitudinal!(model::LaneFollowingDriver, scene::Scene, roadway::Roadway, ego_index::Int, target_index::Int)
    veh_ego = scene[ego_index]
    v_ego = veh_ego.state.v
    v_oth = NaN
    headway = NaN

    if target_index > 0 && target_index != ego_index
        veh_target = scene[target_index]
        s_gap = get_frenet_relative_position(get_rear_center(veh_target), veh_ego.state.posF.roadind, roadway).Δs
        if s_gap > 0.0
            headway = s_gap
            v_oth = scene[target_index].state.v
        end
    end

    return track_longitudinal!(model, v_ego, v_oth, headway)
end
function observe!(model::LaneFollowingDriver, scene::Scene, roadway::Roadway, egoid::Int)

    vehicle_index = findfirst(scene, egoid)
    veh_ego = scene[vehicle_index]
    fore_res = get_neighbor_fore_along_lane(scene, vehicle_index, roadway, VehicleTargetPointFront(), VehicleTargetPointRear(), VehicleTargetPointFront())
    track_longitudinal!(model, scene, roadway, vehicle_index, fore_res.ind)

    return model
end