function observe!{R}(
    model::LaneFollowingDriver,
    scene::EntityFrame{PosSpeed1D, BoundingBoxDef, Int},
    roadway::R,
    egoid::Int,
    )

    vehicle_index = findfirst(scene, egoid)

    fore_res = get_neighbor_fore(scene, vehicle_index, roadway)

    v_ego = scene[vehicle_index].state.v
    v_oth = scene[fore_res.ind].state.v
    headway = fore_res.Δs

    track_longitudinal!(model, v_ego, v_oth, headway)

    return model
end