scenario = let
    
    sn = generate_straight_nlane_streetmap(2)

    speed_65mph = 29.06

    lanetagL = project_point_to_streetmap(0.0,0.0,sn).lane.id
    lanetagR = project_point_to_streetmap(0.0,-5.0,sn).lane.id

    history     = 4*DEFAULT_FRAME_PER_SEC # [pdset frames]
    horizon     = 4*DEFAULT_FRAME_PER_SEC # [pdset frames]
    x₀          = 10.5 # [m]
    delta_x     = 20.0 # [m]
    delta_speed = 0.93 # [m]
    
    trajs = Array(TrajDef, 2)
    trajs[1] = TrajDef(sn, VecE2(x₀,-4.5), speed_65mph)
    push!(trajs[1], TrajDefLinkTargetSpeed(history, lanetagR, 0.0, speed_65mph))
    push!(trajs[1], TrajDefLinkTargetSpeed(horizon, lanetagR, 0.0, speed_65mph))

    trajs[2] = TrajDef(sn, VecE2(x₀+delta_x,-4.5), delta_speed*speed_65mph)
    push!(trajs[2], TrajDefLinkTargetSpeed(history, lanetagR, 0.0, delta_speed*speed_65mph))
    push!(trajs[2], TrajDefLinkTargetSpeed(horizon, lanetagR, 0.0, delta_speed*speed_65mph))

    Scenario("follow_faster", sn, history, DEFAULT_SEC_PER_FRAME, trajs)
end