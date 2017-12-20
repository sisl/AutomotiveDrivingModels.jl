#=
What needs to be checked:
1. that vehicles at various angles are detected
2. that the correct distance is returned
3. that when a ray encounters multiple vehicles, the correct vehicle is identified
4. that the range rate is correctly returned
5. that the max range of the ray is respected
=#

let
    #=
    Create a scene with 7 vehicles, an ego in the middle and 6 surrounding 
    vehicles. Use this setup to check that lidar sensors work for various 
    range and angle settings. 
    =#
    num_veh = 7
    ego_index = 1
    roadway = gen_straight_roadway(4, 400.)
    scene = Scene(num_veh)
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
        push!(scene, Vehicle(veh_state, veh_def, i))
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