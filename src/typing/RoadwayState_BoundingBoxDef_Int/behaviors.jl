
function observe!(model::ProportionalLaneTracker, scene::Scene, roadway::Roadway, egoid::Int)

    ego_index = findfirst(scene, egoid)
    veh_ego = scene[ego_index]
    t = veh_ego.state.posF.t # lane offset
    dt = veh_ego.state.v * sin(veh_ego.state.posF.ϕ) # rate of change of lane offset
    model.a = -t*model.kp - dt*model.kd

    model
end
function observe!(model::LatLonSeparableDriver, scene::Scene, roadway::Roadway, egoid::Int)
    observe!(model.mlat, scene, roadway, egoid)
    observe!(model.mlon, scene, roadway, egoid)
    model
end

function observe!(model::MOBIL, scene::Scene, roadway::Roadway, egoid::Int)

    vehicle_index = findfirst(scene, egoid)
    veh_ego = scene[vehicle_index]
    v = veh_ego.state.v
    egostate_M = veh_ego.state

    lane_ego = roadway[veh_ego.state.tag]
    left_lane_exists = n_lanes_left(lane_ego, roadway) > 0
    right_lane_exists = n_lanes_right(lane_ego, roadway) > 0

    fore_M = get_neighbor_fore_along_lane(scene, vehicle_index, roadway, VehicleTargetPointFront(), VehicleTargetPointRear(), VehicleTargetPointFront())
    rear_M = get_neighbor_rear_along_lane(scene, vehicle_index, roadway, VehicleTargetPointFront(), VehicleTargetPointFront(), VehicleTargetPointRear())

    # accel if we do not make a lane change
    accel_M_orig = rand(observe!(reset_hidden_state!(model.mlon), scene, roadway, egoid)).a
    model.dir = DIR_MIDDLE

    advantage_threshold = model.advantage_threshold
    if right_lane_exists

        rear_R = get_neighbor_rear_along_right_lane(scene, vehicle_index, roadway, VehicleTargetPointFront(), VehicleTargetPointFront(), VehicleTargetPointRear())

        # candidate position after lane change is over
        footpoint = get_footpoint(veh_ego)
        lane = roadway[veh_ego.state.tag]
        lane_R = roadway[LaneTag(lane.tag.segment, lane.tag.lane - 1)]
        roadproj = proj(footpoint, lane_R, roadway)
        frenet_R = Frenet(RoadIndex(roadproj), roadway)
        egostate_R = RoadwayState(frenet_R, roadway, veh_ego.state.v)

        Δaccel_n = 0.0
        passes_safety_criterion = true
        if rear_R.ind != 0
            id = scene[rear_R.ind].id
            accel_n_orig = rand(observe!(reset_hidden_state!(model.mlon), scene, roadway, id)).a
            veh_ego = Entity(veh_ego, egostate_R)
            accel_n_test = rand(observe!(reset_hidden_state!(model.mlon), scene, roadway, id)).a

            body = inertial2body(get_rear(scene[rear_R.ind]), get_front(veh_ego)) # project target to be relative to ego
            s_gap = body.x

            veh_ego = Entity(veh_ego, egostate_M)
            passes_safety_criterion = accel_n_test ≥ -model.safe_decel && s_gap ≥ 0
            Δaccel_n = accel_n_test - accel_n_orig
        end

        if passes_safety_criterion

            Δaccel_o = 0.0
            if rear_M.ind != 0
                id = scene[rear_M.ind].id
                accel_o_orig = rand(observe!(reset_hidden_state!(model.mlon), scene, roadway, id)).a
                veh_ego = Entity(veh_ego, egostate_R)
                accel_o_test = rand(observe!(reset_hidden_state!(model.mlon), scene, roadway, id)).a
                veh_ego = Entity(veh_ego, egostate_M)
                Δaccel_o = accel_o_test - accel_o_orig
            end

            veh_ego = Entity(veh_ego, egostate_R)
            accel_M_test = rand(observe!(reset_hidden_state!(model.mlon), scene, roadway, egoid)).a
            veh_ego = Entity(veh_ego, egostate_M)
            Δaccel_M = accel_M_test - accel_M_orig

            Δaₜₕ = Δaccel_M + model.politeness*(Δaccel_n + Δaccel_o)
            if Δaₜₕ > advantage_threshold
                model.dir = DIR_RIGHT
                advantage_threshold = Δaₜₕ
            end
        end
    end

    if left_lane_exists
        rear_L = get_neighbor_rear_along_left_lane(scene, vehicle_index, roadway, VehicleTargetPointFront(), VehicleTargetPointFront(), VehicleTargetPointRear())

        # candidate position after lane change is over
        footpoint = get_footpoint(veh_ego)
        lane = roadway[veh_ego.state.tag]
        lane_L = roadway[LaneTag(lane.tag.segment, lane.tag.lane + 1)]
        roadproj = proj(footpoint, lane_L, roadway)
        frenet_L = Frenet(RoadIndex(roadproj), roadway)
        egostate_L = RoadwayState(frenet_L, roadway, veh_ego.state.v)

        Δaccel_n = 0.0
        passes_safety_criterion = true
        if rear_L.ind != 0
            id = scene[rear_L.ind].id
            accel_n_orig = rand(observe!(reset_hidden_state!(model.mlon), scene, roadway, id)).a
            veh_ego = Entity(veh_ego, egostate_L)
            accel_n_test = rand(observe!(reset_hidden_state!(model.mlon), scene, roadway, id)).a

            body = inertial2body(get_rear(scene[rear_L.ind]), get_front(veh_ego)) # project target to be relative to ego
            s_gap = body.x

            veh_ego = Entity(veh_ego, egostate_M)
            passes_safety_criterion = accel_n_test ≥ -model.safe_decel && s_gap ≥ 0
            Δaccel_n = accel_n_test - accel_n_orig
        end

        if passes_safety_criterion


            Δaccel_o = 0.0
            if rear_M.ind != 0
                id = scene[rear_M.ind].id
                accel_o_orig = rand(observe!(reset_hidden_state!(model.mlon), scene, roadway, id)).a
                veh_ego = Entity(veh_ego, egostate_L)
                accel_o_test = rand(observe!(reset_hidden_state!(model.mlon), scene, roadway, id)).a
                veh_ego = Entity(veh_ego, egostate_M)
                Δaccel_o = accel_o_test - accel_o_orig
            end

            veh_ego = Entity(veh_ego, egostate_L)
            accel_M_test = rand(observe!(reset_hidden_state!(model.mlon), scene, roadway, egoid)).a
            veh_ego = Entity(veh_ego, egostate_M)
            Δaccel_M = accel_M_test - accel_M_orig

            Δaₜₕ = Δaccel_M + model.politeness*(Δaccel_n + Δaccel_o)
            if Δaₜₕ > advantage_threshold
                model.dir = DIR_LEFT
                advantage_threshold = Δaₜₕ
            end
        end
    end

    model
end

function observe!(model::TimLaneChanger, scene::Scene, roadway::Roadway, egoid::Int)

    vehicle_index = findfirst(scene, egoid)

    veh_ego = scene[vehicle_index]
    v = veh_ego.state.v

    lane_ego = roadway[veh_ego.state.tag]
    left_lane_exists = n_lanes_left(lane_ego, roadway) > 0
    right_lane_exists = n_lanes_right(lane_ego, roadway) > 0

    fore_M = get_neighbor_fore_along_lane(scene, vehicle_index, roadway, VehicleTargetPointFront(), VehicleTargetPointRear(), VehicleTargetPointFront(), max_distance_fore=model.threshold_fore)
    fore_L = get_neighbor_fore_along_left_lane(scene, vehicle_index, roadway, VehicleTargetPointRear(), VehicleTargetPointRear(), VehicleTargetPointFront())
    fore_R = get_neighbor_fore_along_right_lane(scene, vehicle_index, roadway, VehicleTargetPointRear(), VehicleTargetPointRear(), VehicleTargetPointFront())
    rear_L = get_neighbor_rear_along_left_lane(scene, vehicle_index, roadway, VehicleTargetPointFront(), VehicleTargetPointFront(), VehicleTargetPointRear())
    rear_R = get_neighbor_rear_along_right_lane(scene, vehicle_index, roadway, VehicleTargetPointFront(), VehicleTargetPointFront(), VehicleTargetPointRear())

    model.dir = DIR_MIDDLE
    if fore_M.Δs < model.threshold_fore # there is a lead vehicle
        veh_M = scene[fore_M.ind]
        speed_M = veh_M.state.v
        if speed_M ≤ min(model.v_des, v) # they are driving slower than we want

            speed_ahead = speed_M

            # consider changing to a different lane
            if right_lane_exists &&
               fore_R.Δs > model.threshold_lane_change_gap_rear && # there is space rear
               rear_R.Δs > model.threshold_lane_change_gap_fore && # there is space fore
               (rear_R.ind == 0 || scene[rear_R.ind].state.v ≤ v) && # we are faster than any follower
               (fore_R.ind == 0 || scene[fore_R.ind].state.v > speed_ahead) # we are faster than any leader

                speed_ahead = fore_R.ind != 0 ? scene[fore_R.ind].state.v : Inf
                model.dir = DIR_RIGHT
            end
            if left_lane_exists &&
               fore_L.Δs > model.threshold_lane_change_gap_rear && # there is space rear
               rear_L.Δs > model.threshold_lane_change_gap_fore && # there is space fore
               (rear_L.ind == 0 || scene[rear_L.ind].state.v ≤ v) && # we are faster than any follower
               (fore_L.ind == 0 || scene[fore_L.ind].state.v > speed_ahead) # we are faster than any leader

                speed_ahead = fore_L.ind != 0 ? scene[fore_L.ind].state.v : Inf
                model.dir = DIR_LEFT
            end
        end
    end

    model
end

function track_longitudinal!(driver::LaneFollowingDriver, scene::Scene, roadway::Roadway, vehicle_index::Int, fore::NeighborLongitudinalResult)
    v_ego = scene[vehicle_index].state.v
    if fore.ind != 0
        headway, v_oth = fore.Δs, scene[fore.ind].state.v
    else
        headway, v_oth = NaN, NaN
    end
    return track_longitudinal!(driver, v_ego, v_oth, headway)
end
# function observe!(driver::Tim2DDriver, scene::Scene, roadway::Roadway, egoid::Int)

#     observe!(driver.mlane, scene, roadway, egoid)

#     vehicle_index = findfirst(scene, egoid)
#     lane_change_action = rand(driver.mlane)
#     laneoffset = get_lane_offset(lane_change_action, scene, roadway, vehicle_index)
#     lateral_speed = get_vel_t(scene[vehicle_index])

#     if lane_change_action.dir == DIR_MIDDLE
#         fore = get_neighbor_fore_along_lane(scene, vehicle_index, roadway, VehicleTargetPointFront(), VehicleTargetPointRear(), VehicleTargetPointFront())
#     elseif lane_change_action.dir == DIR_LEFT
#         fore = get_neighbor_fore_along_left_lane(scene, vehicle_index, roadway, VehicleTargetPointFront(), VehicleTargetPointRear(), VehicleTargetPointFront())
#     else
#         @assert(lane_change_action.dir == DIR_RIGHT)
#         fore = get_neighbor_fore_along_right_lane(scene, vehicle_index, roadway, VehicleTargetPointFront(), VehicleTargetPointRear(), VehicleTargetPointFront())
#     end

#     track_lateral!(driver.mlat, laneoffset, lateral_speed)
#     track_longitudinal!(driver.mlon, scene, roadway, vehicle_index, fore)

#     driver
# end