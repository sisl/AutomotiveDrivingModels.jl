abstract LaneFollowingDriver <: DriverModel{LaneFollowingAccel}
track_longitudinal!(model::LaneFollowingDriver, v_ego::Float64, v_oth::Float64, headway::Float64) = model # do nothing by default

function observe!(model::LaneFollowingDriver, scene::Scene1D, roadway::StraightRoadway, egoid::Int)

    vehicle_index = findfirst(scene, egoid)

    fore_res = get_neighbor_fore(scene, vehicle_index, roadway)

    v_ego = scene[vehicle_index].state.v
    v_oth = scene[fore_res.ind].state.v
    headway = fore_res.Δs

    track_longitudinal!(model, v_ego, v_oth, headway)

    return model
end