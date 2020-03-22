@testset "neighbor features" begin 
    scene=Scene(Entity{VehicleState, BicycleModel, Int}, 100)
    roadway=gen_straight_roadway(3, 200.0, lane_width=3.0)
    push!(scene,Entity(VehicleState(VecSE2(30.0,3.0,0.0), roadway, 0.0), 
        BicycleModel(VehicleDef(AgentClass.CAR, 4.826, 1.81)),1))
    push!(scene,Entity(VehicleState(VecSE2(20.0,3.0,0.0), roadway, 0.0), 
        BicycleModel(VehicleDef(AgentClass.CAR, 4.826, 1.81)),2))
    push!(scene,Entity(VehicleState(VecSE2(40.0,3.0,0.0), roadway, 0.0), 
        BicycleModel(VehicleDef(AgentClass.CAR, 4.826, 1.81)),3))
    push!(scene,Entity(VehicleState(VecSE2(20.0,0.0,0.0), roadway, 0.0), 
        BicycleModel(VehicleDef(AgentClass.CAR, 4.826, 1.81)),4))
    push!(scene,Entity(VehicleState(VecSE2(40.0,0.0,0.0), roadway, 0.0), 
        BicycleModel(VehicleDef(AgentClass.CAR, 4.826, 1.81)),5))
    push!(scene,Entity(VehicleState(VecSE2(20.0,6.0,0.0), roadway, 0.0), 
        BicycleModel(VehicleDef(AgentClass.CAR, 4.826, 1.81)),6))
    push!(scene,Entity(VehicleState(VecSE2(40.0,6.0,0.0), roadway, 0.0), 
        BicycleModel(VehicleDef(AgentClass.CAR, 4.826, 1.81)),7))
    @test find_neighbor(scene, roadway, scene[1]) == NeighborLongitudinalResult(3,10.0)
    @test find_neighbor(scene, roadway, scene[1], rear=true) == NeighborLongitudinalResult(2,10.0)
    @test find_neighbor(scene, roadway, scene[1], lane=leftlane(roadway, scene[1])) == NeighborLongitudinalResult(7,10.0)
    @test find_neighbor(scene, roadway, scene[1], lane=leftlane(roadway, scene[1]), rear=true) == NeighborLongitudinalResult(6,10.0)
    @test find_neighbor(scene, roadway, scene[1], lane=rightlane(roadway, scene[1])) == NeighborLongitudinalResult(5,10.0)
    @test find_neighbor(scene, roadway, scene[1], lane=rightlane(roadway, scene[1]), rear=true) == NeighborLongitudinalResult(4,10.0)

    trajdata = get_test_trajdata(roadway)
    scene = trajdata[1]
    @test find_neighbor(scene, roadway, scene[1]) == NeighborLongitudinalResult(2, 3.0)
    @test find_neighbor(scene, roadway, scene[2]) == NeighborLongitudinalResult(nothing, 250.0)
    scene = trajdata[2]
    @test find_neighbor(scene, roadway, scene[1]) == NeighborLongitudinalResult(2, 4.0)
    @test find_neighbor(scene, roadway, scene[2]) == NeighborLongitudinalResult(nothing, 250.0)

    roadway = gen_stadium_roadway(1)
    scene = Scene(Entity{VehicleState, VehicleDef, Int64}, 2)
    scene.n = 2
    def = VehicleDef(AgentClass.CAR, 2.0, 1.0)

    function place_at!(i, s)
        roadproj = proj(VecSE2(0.0,0.0,0.0), roadway)
        roadind = RoadIndex(roadproj.curveproj.ind, roadproj.tag)
        roadind = move_along(roadind, roadway, s)
        frenet = Frenet(roadind, roadway[roadind].s, 0.0, 0.0)
        state = VehicleState(frenet, roadway, 0.0)
        scene[i] = Entity(state, def, i)
    end

    place_at!(1, 0.0)
    place_at!(2, 0.0)
    foreinfo = find_neighbor(scene, roadway, scene[1], max_distance=Inf)
    @test isapprox(foreinfo.Δs, 0.0)

    place_at!(2, 100.0)
    foreinfo = find_neighbor(scene, roadway, scene[1], max_distance=Inf)
    @test foreinfo.ind == 2
    @test isapprox(foreinfo.Δs, 100.0)

    foreinfo = find_neighbor(scene, roadway, scene[2], max_distance=Inf)
    @test foreinfo.ind == 1
    @test isapprox(foreinfo.Δs, 277.07, atol=1e-2)

    place_at!(2, 145.0)
    foreinfo = find_neighbor(scene, roadway, scene[1], max_distance=Inf)
    @test foreinfo.ind == 2
    @test isapprox(foreinfo.Δs, 145.0, atol=1e-5)

    place_at!(2, 240.0)
    foreinfo = find_neighbor(scene, roadway, scene[1], max_distance=Inf)
    @test foreinfo.ind == 2
    @test isapprox(foreinfo.Δs, 240.0, atol=1e-5)

    place_at!(1, 240.0)
    foreinfo = find_neighbor(scene, roadway, scene[1], max_distance=Inf)
    @test foreinfo.ind == 2
    @test isapprox(foreinfo.Δs, 0.0, atol=1e-5)

    ################################################
    # get_frenet_relative_position

    place_at!(1, 0.0)
    place_at!(2, 10.0)
    frp = get_frenet_relative_position(scene[2].state.posG, scene[1].state.posF.roadind, roadway, max_distance_fore=Inf)
    @test frp.origin == scene[1].state.posF.roadind
    @test frp.target.ind.i == 1
    @test isapprox(frp.target.ind.t, 0.1, atol=0.01)
    @test frp.target.tag == scene[1].state.posF.roadind.tag
    @test isapprox(frp.Δs, 10.0, atol=1e-2)
    @test isapprox(frp.t, 0.0, atol=1e-5)
    @test isapprox(frp.ϕ, 0.0, atol=1e-7)

    place_at!(1, 10.0)
    place_at!(2, 20.0)
    frp = get_frenet_relative_position(scene[2].state.posG, scene[1].state.posF.roadind, roadway, max_distance_fore=Inf)
    @test frp.origin == scene[1].state.posF.roadind
    @test frp.target.ind.i == 1
    @test isapprox(frp.target.ind.t, 0.2, atol=0.01)
    @test frp.target.tag == scene[1].state.posF.roadind.tag
    @test isapprox(frp.Δs, 10.0, atol=1e-2)
    @test isapprox(frp.t, 0.0, atol=1e-5)
    @test isapprox(frp.ϕ, 0.0, atol=1e-7)

    place_at!(1, 0.0)
    place_at!(2, 120.0)
    frp = get_frenet_relative_position(scene[2].state.posG, scene[1].state.posF.roadind, roadway, max_distance_fore=Inf)
    @test frp.target.tag == LaneTag(1, 1)
    @test scene[1].state.posF.roadind.tag == LaneTag(6, 1)
    @test isapprox(frp.Δs, 120.0, atol=1e-2)
    @test isapprox(frp.t, 0.0, atol=1e-5)
    @test isapprox(frp.ϕ, 0.0, atol=1e-7)

    place_at!(1, 0.0)
    place_at!(2, 250.0)
    frp = get_frenet_relative_position(scene[2].state.posG, scene[1].state.posF.roadind, roadway, max_distance_fore=Inf)
    @test frp.target.tag != scene[1].state.posF.roadind.tag
    @test isapprox(frp.Δs, 250.0, atol=1e-2)
    @test isapprox(frp.t, 0.0, atol=1e-5)
    @test isapprox(frp.ϕ, 0.0, atol=1e-7)
end

@testset "feature extraction" begin 
    roadway = gen_straight_roadway(4, 100.0)

    scene = Scene([Entity(VehicleState(VecSE2( 0.0,0.0,0.0), roadway, 10.0), VehicleDef(AgentClass.CAR, 5.0, 2.0), 1),
        Entity(VehicleState(VecSE2(10.0,0.0,0.0), roadway, 10.0), VehicleDef(AgentClass.CAR, 5.0, 2.0), 2),
            ])

    # test each feature individually 
    pos1 = extract_feature(featuretype(posgx), posgx, roadway, [scene], 1)
    pos2 = extract_feature(featuretype(posgx), posgx, roadway, [scene], 2)
    poss = extract_feature(featuretype(posgx), posgx, roadway, [scene], [1,2])
    @test poss[1][1] == pos1[1]
    @test poss[2][1] == pos2[2]
    @test pos1[1] == 0.0
    @test pos2[2] == 10.0

    scene = Scene([Entity(VehicleState(VecSE2(1.1,1.2,0.0), roadway, 10.0), VehicleDef(AgentClass.CAR, 5.0, 2.0), 1)])
    posy = extract_feature(featuretype(posgy), posgy, roadway, [scene], 1)
    @test posy[1] == 1.2
    posθ = extract_feature(featuretype(posgθ), posgθ, roadway, [scene], 1)
    @test posθ[1] == 0.0
    poss = extract_feature(featuretype(posfs), posfs, roadway, [scene], 1)
    @test poss[1] == 1.1
    post = extract_feature(featuretype(posft), posft, roadway, [scene], 1)
    @test post[1] == 1.2
    posϕ = extract_feature(featuretype(posfϕ), posfϕ, roadway, [scene], 1)
    @test posϕ[1] == 0.0


    scene = Scene([Entity(VehicleState(VecSE2(1.1,1.2,0.0), roadway, 10.0), VehicleDef(AgentClass.CAR, 5.0, 2.0), 1),
                Entity(VehicleState(VecSE2(1.5,1.2,0.0), roadway, 10.0), VehicleDef(AgentClass.CAR, 5.0, 2.0), 2)])
    coll = extract_feature(featuretype(iscolliding), iscolliding, roadway, [scene], 1)
    @test coll[1]

    d = extract_feature(featuretype(distance_to(1)), distance_to(1), roadway, [scene], 1)
    @test d[1] == 0.0
    d = extract_feature(featuretype(distance_to(2)), distance_to(2), roadway, [scene], 1)
    @test d[1] ≈ 0.4

    df = extract_feature(featuretype(turn_rate_g), turn_rate_g, roadway, [scene, scene], [1])
    @test df[1][1] === missing
    @test df[1][2] == 0.0

    # extract multiple features 
    roadway = gen_straight_roadway(3, 1000.0, lane_width=1.0)
    scene = Scene([
                Entity(VehicleState(VecSE2( 0.0,0.0,0.0), roadway, 10.0), VehicleDef(AgentClass.CAR, 5.0, 2.0), 1),
                Entity(VehicleState(VecSE2(10.0,0.0,0.0), roadway, 10.0), VehicleDef(AgentClass.CAR, 5.0, 2.0), 2),
            ])

    dfs = extract_features((iscolliding, markerdist_left, markerdist_right), roadway, [scene], [1,2])
    @test isapprox(dfs[1][1,3], 0.5)
    @test isapprox(dfs[2][1,3], 0.5)
    @test isapprox(dfs[1][1,2], 0.5)
    @test isapprox(dfs[2][1,2], 0.5)

    # integration testing, all the features 
    feature_list = (posgx, posgy, posgθ, posfs, posft, posfϕ, vel, velfs, velft, velgx, velgy, 
                    time_to_crossing_right, time_to_crossing_left, 
                    estimated_time_to_lane_crossing, iswaiting, acc, accfs, accft, jerk, 
                    jerkft, turn_rate_g, turn_rate_f, isbraking, isaccelerating,
                    lane_width, lane_offset_left, lane_offset_right, has_lane_left, 
                    has_lane_right, lane_curvature)
    dfs = extract_features(feature_list, roadway, [scene, scene], [1,2])
    for id=[1,2]
        @test ncol(dfs[id]) == length(feature_list)
        @test nrow(dfs[id]) == 2
    end

    scene = Scene([
            Entity(VehicleState(VecSE2( 1.0,0.0,0.0), roadway, 10.0), VehicleDef(AgentClass.CAR, 5.0, 2.0), 1),
            Entity(VehicleState(VecSE2(10.0,0.0,0.0), roadway, 10.0), VehicleDef(AgentClass.CAR, 5.0, 2.0), 2),
            Entity(VehicleState(VecSE2(12.0,1.0,0.0), roadway, 10.0), VehicleDef(AgentClass.CAR, 5.0, 2.0), 3),
            Entity(VehicleState(VecSE2( 0.0,1.0,0.0), roadway, 10.0), VehicleDef(AgentClass.CAR, 5.0, 2.0), 4),
        ])
    dfs= extract_features((dist_to_front_neighbor, front_neighbor_speed, time_to_collision), roadway, [scene], [1,2,3,4])
    @test isapprox(dfs[1][1,1], 9.0)

    # type inferrence 
    for fun in feature_list
        @inferred featuretype(fun)
    end

end # features

@testset "lidar sensor" begin
    #=
    Create a scene with 7 vehicles, an ego in the middle and 6 surrounding 
    vehicles. Use this setup to check that lidar sensors work for various 
    range and angle settings. 
    =#
    num_veh = 7
    ego_index = 1
    roadway = gen_straight_roadway(4, 400.)
    scene = Scene(Entity{VehicleState,VehicleDef,Int64}, num_veh)
    # order: ego, fore, rear, left, right, fore_fore, fore_fore_fore
    speeds = [10., 15., 15., 0., -5, 20., 20.]
    positions = [200., 220., 150., 200., 200., 240., 350.]
    lanes = [2,2,2,4,1,2,2]
    for i in 1:num_veh
        lane = roadway.segments[1].lanes[lanes[i]]
        road_idx = RoadIndex(proj(VecSE2(0.0, 0.0, 0.0), lane, roadway))
        veh_state = VehicleState(Frenet(road_idx, roadway), roadway, speeds[i])
        veh_state = move_along(veh_state, roadway, positions[i])
        veh_def = VehicleDef(AgentClass.CAR, 2., 2.)
        push!(scene, Entity(veh_state, veh_def, i))
    end

    # basic lidar with sufficient range for all vehicles
    nbeams = 4
    lidar = LidarSensor(nbeams, max_range=200., angle_offset=0.)
    observe!(lidar, scene, roadway, ego_index)
    # order: right, fore, left, back
    @test isapprox(lidar.ranges[1], 2.0, atol=4)
    @test isapprox(lidar.ranges[2], 19.0, atol=4)
    @test isapprox(lidar.ranges[3], 5.0, atol=4)
    @test isapprox(lidar.ranges[4], 49.0, atol=4)
    @test isapprox(lidar.range_rates[1], 0.0, atol=4)
    @test isapprox(lidar.range_rates[2], 5.0, atol=4)
    @test isapprox(lidar.range_rates[3], 0.0, atol=4)
    @test isapprox(lidar.range_rates[4], -5.0, atol=4)

    # angles are such that all the vehicles are missed
    nbeams = 4
    lidar = LidarSensor(nbeams, max_range=200., angle_offset=π/4)
    observe!(lidar, scene, roadway, ego_index)
    # order: right, fore, left, back
    @test isapprox(lidar.ranges[1], 200.0, atol=4)
    @test isapprox(lidar.ranges[2], 200.0, atol=4)
    @test isapprox(lidar.ranges[3], 200.0, atol=4)
    @test isapprox(lidar.ranges[4], 200.0, atol=4)
    @test isapprox(lidar.range_rates[1], 0.0, atol=4)
    @test isapprox(lidar.range_rates[2], 0.0, atol=4)
    @test isapprox(lidar.range_rates[3], 0.0, atol=4)
    @test isapprox(lidar.range_rates[4], 0.0, atol=4)

    # range short enough that it misses the rear vehicle
    nbeams = 4
    lidar = LidarSensor(nbeams, max_range=20., angle_offset=0.)
    observe!(lidar, scene, roadway, ego_index)
    # order: right, fore, left, back
    @test isapprox(lidar.ranges[1], 2.0, atol=4)
    @test isapprox(lidar.ranges[2], 19.0, atol=4)
    @test isapprox(lidar.ranges[3], 5.0, atol=4)
    @test isapprox(lidar.ranges[4], 20.0, atol=4)
    @test isapprox(lidar.range_rates[1], 0.0, atol=4)
    @test isapprox(lidar.range_rates[2], 5.0, atol=4)
    @test isapprox(lidar.range_rates[3], 0.0, atol=4)
    @test isapprox(lidar.range_rates[4], 0.0, atol=4)

end
