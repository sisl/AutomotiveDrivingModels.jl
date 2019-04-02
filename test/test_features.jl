@testset "neighbor features" begin 
    scene=Frame(Entity{VehicleState, BicycleModel, Int},100)
    roadway=gen_straight_roadway(3, 200.0, lane_width=3.0)
    push!(scene,Entity(VehicleState(VecSE2(30.0,3.0,0.0), roadway, 0.0), 
        BicycleModel(VehicleDef(AgentClass.CAR, 4.826, 1.81)),1))
    push!(scene,Entity(VehicleState(VecSE2(20.0,3.0,0.0), roadway, 0.0), 
        BicycleModel(VehicleDef(AgentClass.CAR, 4.826, 1.81)),1))
    push!(scene,Entity(VehicleState(VecSE2(40.0,3.0,0.0), roadway, 0.0), 
        BicycleModel(VehicleDef(AgentClass.CAR, 4.826, 1.81)),1))
    push!(scene,Entity(VehicleState(VecSE2(20.0,0.0,0.0), roadway, 0.0), 
        BicycleModel(VehicleDef(AgentClass.CAR, 4.826, 1.81)),1))
    push!(scene,Entity(VehicleState(VecSE2(40.0,0.0,0.0), roadway, 0.0), 
        BicycleModel(VehicleDef(AgentClass.CAR, 4.826, 1.81)),1))
    push!(scene,Entity(VehicleState(VecSE2(20.0,6.0,0.0), roadway, 0.0), 
        BicycleModel(VehicleDef(AgentClass.CAR, 4.826, 1.81)),1))
    push!(scene,Entity(VehicleState(VecSE2(40.0,6.0,0.0), roadway, 0.0), 
        BicycleModel(VehicleDef(AgentClass.CAR, 4.826, 1.81)),1))
    @test get_neighbor_fore_along_lane(scene,1,roadway) == NeighborLongitudinalResult(3,10.0)
    @test get_neighbor_rear_along_lane(scene,1,roadway) == NeighborLongitudinalResult(2,10.0)
    @test get_neighbor_fore_along_left_lane(scene,1,roadway) == NeighborLongitudinalResult(7,10.0)
    @test get_neighbor_rear_along_left_lane(scene,1,roadway) == NeighborLongitudinalResult(6,10.0)
    @test get_neighbor_fore_along_right_lane(scene,1,roadway) == NeighborLongitudinalResult(5,10.0)
    @test get_neighbor_rear_along_right_lane(scene,1,roadway) == NeighborLongitudinalResult(4,10.0)

    trajdata = get_test_trajdata(roadway)
    @test get_neighbor_fore_along_lane(get!(Scene(), trajdata, 1), 1, roadway) == NeighborLongitudinalResult(2, 3.0)
    @test get_neighbor_fore_along_lane(get!(Scene(), trajdata, 1), 2, roadway) == NeighborLongitudinalResult(nothing, 250.0)
    @test get_neighbor_fore_along_lane(get!(Scene(), trajdata, 2), 1, roadway) == NeighborLongitudinalResult(2, 4.0)
    @test get_neighbor_fore_along_lane(get!(Scene(), trajdata, 2), 2, roadway) == NeighborLongitudinalResult(nothing, 250.0)

    roadway = gen_stadium_roadway(1)
    scene = Scene(2)
    scene.n = 2
    def = VehicleDef(AgentClass.CAR, 2.0, 1.0)

    function place_at!(i, s)
        roadproj = proj(VecSE2(0.0,0.0,0.0), roadway)
        roadind = RoadIndex(roadproj.curveproj.ind, roadproj.tag)
        roadind = move_along(roadind, roadway, s)
        frenet = Frenet(roadind, roadway[roadind].s, 0.0, 0.0)
        state = VehicleState(frenet, roadway, 0.0)
        scene[i] = Vehicle(state, def, i)
    end

    place_at!(1, 0.0)
    place_at!(2, 0.0)
    foreinfo = get_neighbor_fore_along_lane(scene, 1, roadway, max_distance_fore=Inf)
    @test isapprox(foreinfo.Δs, 0.0)

    place_at!(2, 100.0)
    foreinfo = get_neighbor_fore_along_lane(scene, 1, roadway, max_distance_fore=Inf)
    @test foreinfo.ind == 2
    @test isapprox(foreinfo.Δs, 100.0)

    foreinfo = get_neighbor_fore_along_lane(scene, 2, roadway, max_distance_fore=Inf)
    @test foreinfo.ind == 1
    @test isapprox(foreinfo.Δs, 277.07, atol=1e-2)

    place_at!(2, 145.0)
    foreinfo = get_neighbor_fore_along_lane(scene, 1, roadway, max_distance_fore=Inf)
    @test foreinfo.ind == 2
    @test isapprox(foreinfo.Δs, 145.0, atol=1e-5)

    place_at!(2, 240.0)
    foreinfo = get_neighbor_fore_along_lane(scene, 1, roadway, max_distance_fore=Inf)
    @test foreinfo.ind == 2
    @test isapprox(foreinfo.Δs, 240.0, atol=1e-5)

    place_at!(1, 240.0)
    foreinfo = get_neighbor_fore_along_lane(scene, 1, roadway, max_distance_fore=Inf)
    @test foreinfo.ind == 2
    @test isapprox(foreinfo.Δs, 0.0, atol=1e-5)

    ################################################
    # get_frenet_relative_position

    place_at!(1, 0.0)
    place_at!(2, 10.0)
    frp = get_frenet_relative_position(scene[2].state.posG, scene[1].state.posF.roadind, roadway, max_distance_fore=Inf)
    @test frp.origin == scene[1].state.posF.roadind
    @test frp.target.ind.i == 0
    @test isapprox(frp.target.ind.t, 0.1, atol=0.01)
    @test frp.target.tag == scene[1].state.posF.roadind.tag
    @test isapprox(frp.Δs, 10.0, atol=1e-2)
    @test isapprox(frp.t, 0.0, atol=1e-5)
    @test isapprox(frp.ϕ, 0.0, atol=1e-7)

    place_at!(1, 10.0)
    place_at!(2, 20.0)
    frp = get_frenet_relative_position(scene[2].state.posG, scene[1].state.posF.roadind, roadway, max_distance_fore=Inf)
    @test frp.origin == scene[1].state.posF.roadind
    @test frp.target.ind.i == 0
    @test isapprox(frp.target.ind.t, 0.2, atol=0.01)
    @test frp.target.tag == scene[1].state.posF.roadind.tag
    @test isapprox(frp.Δs, 10.0, atol=1e-2)
    @test isapprox(frp.t, 0.0, atol=1e-5)
    @test isapprox(frp.ϕ, 0.0, atol=1e-7)

    place_at!(1, 0.0)
    place_at!(2, 120.0)
    frp = get_frenet_relative_position(scene[2].state.posG, scene[1].state.posF.roadind, roadway, max_distance_fore=Inf)
    @test frp.target.tag == scene[1].state.posF.roadind.tag
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

@testset begin "features interface" 
    roadway = gen_straight_roadway(3, 1000.0, lane_width=1.0)
    rec = SceneRecord(1, 0.1, 5)
    update!(rec, Scene([
            Vehicle(VehicleState(VecSE2( 0.0,0.0,0.0), roadway, 10.0), VehicleDef(AgentClass.CAR, 5.0, 2.0), 1),
            Vehicle(VehicleState(VecSE2(10.0,0.0,0.0), roadway, 10.0), VehicleDef(AgentClass.CAR, 5.0, 2.0), 2),
        ]))

    @test convert(Float64, get(IS_COLLIDING, rec, roadway, 1)) == 0.0
    @test convert(Float64, get(IS_COLLIDING, rec, roadway, 2)) == 0.0

    @test isapprox(convert(Float64, get(MARKERDIST_LEFT, rec, roadway, 1)), 0.5)
    @test isapprox(convert(Float64, get(MARKERDIST_LEFT, rec, roadway, 2)), 0.5)
    @test isapprox(convert(Float64, get(MARKERDIST_RIGHT, rec, roadway, 1)), 0.5)
    @test isapprox(convert(Float64, get(MARKERDIST_RIGHT, rec, roadway, 2)), 0.5)
    @test isapprox(convert(Float64, get(MARKERDIST_LEFT_LEFT, rec, roadway, 1)), 1.5)
    @test isapprox(convert(Float64, get(MARKERDIST_LEFT_LEFT, rec, roadway, 2)), 1.5)
    @test isnan(convert(Float64, get(MARKERDIST_RIGHT_RIGHT, rec, roadway, 1)))
    @test isnan(convert(Float64, get(MARKERDIST_RIGHT_RIGHT, rec, roadway, 2)))

    @test isapprox(convert(Float64, get(DIST_FRONT, rec, roadway, 1)), 10.0 - 5.0)
    @test convert(Float64, get(DIST_FRONT, rec, roadway, 2, censor_hi=100.0)) == 100.0

    update!(rec, Scene([
            Vehicle(VehicleState(VecSE2( 1.0,0.0,0.0), roadway, 10.0), VehicleDef(AgentClass.CAR, 5.0, 2.0), 1),
            Vehicle(VehicleState(VecSE2(10.0,0.0,0.0), roadway, 10.0), VehicleDef(AgentClass.CAR, 5.0, 2.0), 2),
            Vehicle(VehicleState(VecSE2(12.0,1.0,0.0), roadway, 10.0), VehicleDef(AgentClass.CAR, 5.0, 2.0), 3),
            Vehicle(VehicleState(VecSE2( 0.0,1.0,0.0), roadway, 10.0), VehicleDef(AgentClass.CAR, 5.0, 2.0), 4),
        ]))

    @test isapprox(convert(Float64, get(MARKERDIST_LEFT, rec, roadway, 3)), 0.5)
    @test isapprox(convert(Float64, get(MARKERDIST_RIGHT, rec, roadway, 3)), 0.5)
    @test isapprox(convert(Float64, get(MARKERDIST_LEFT_LEFT, rec, roadway, 3)), 1.5)
    @test isapprox(convert(Float64, get(MARKERDIST_RIGHT_RIGHT, rec, roadway, 3)), 1.5)

    @test isapprox(convert(Float64, get(DIST_FRONT, rec, roadway, 1)), 9.0 - 5.0)
    @test isapprox(convert(Float64, get(DIST_FRONT_LEFT, rec, roadway, 1)), 11.0 - 5.0)
    @test convert(Float64, get(DIST_FRONT_RIGHT, rec, roadway, 1)) == 100.0
    @test isapprox(convert(Float64, get(DIST_FRONT, rec, roadway, 4)), 12.0 - 5.0)
    @test convert(Float64, get(DIST_FRONT_LEFT, rec, roadway, 4)) == 100.0
    @test isapprox(convert(Float64, get(DIST_FRONT_RIGHT, rec, roadway, 4)), 1.0 - 5.0)

    @test convert(Float64, get(IS_COLLIDING, rec, roadway, 1)) == 1.0
    @test convert(Float64, get(IS_COLLIDING, rec, roadway, 2)) == 1.0
    @test convert(Float64, get(IS_COLLIDING, rec, roadway, 3)) == 1.0
    @test convert(Float64, get(IS_COLLIDING, rec, roadway, 4)) == 1.0

    update!(rec, Scene([
            Vehicle(VehicleState(VecSE2(  0.0,0.0,0.0), roadway, 10.0), VehicleDef(AgentClass.CAR, 5.0, 2.0), 1),
            Vehicle(VehicleState(VecSE2(-10.0,1.0,0.0), roadway, 10.0), VehicleDef(AgentClass.CAR, 5.0, 2.0), 2),
            Vehicle(VehicleState(VecSE2(  0.0,1.0,0.5), roadway, 10.0), VehicleDef(AgentClass.CAR, 5.0, 2.0), 3),
        ]))

    @test convert(Float64, get(IS_COLLIDING, rec, roadway, 1)) == 1.0
    @test convert(Float64, get(IS_COLLIDING, rec, roadway, 2)) == 0.0
    @test convert(Float64, get(IS_COLLIDING, rec, roadway, 3)) == 1.0

    @test get(LatLonAccel, rec, roadway, 1) == LatLonAccel(0.0, 0.0)
    @test get(AccelTurnrate, rec, roadway, 1) == AccelTurnrate(0.0, 0.0)
    # super-simple test
    for f in allfeatures()
        get(f, rec, roadway, 3)
    end
end

struct DumbFeatureExtractor <: AbstractFeatureExtractor end
@testset begin "features extractor"
    roadway = gen_straight_roadway(3, 1000.0, lane_width=1.0)
    rec = SceneRecord(1, 0.1, 5)
    update!(rec, Scene([
            Vehicle(VehicleState(VecSE2( 0.0,0.0,0.0), roadway, 10.0), VehicleDef(AgentClass.CAR, 5.0, 2.0), 1),
            Vehicle(VehicleState(VecSE2(10.0,0.0,0.0), roadway, 10.0), VehicleDef(AgentClass.CAR, 5.0, 2.0), 2),
        ]))

    ext = DumbFeatureExtractor()
    @test rec_length(ext) == 1
    @test_throws Exception length(ext)
    @test_throws Exception pull_features!(ext, Array{Float64}(undef, 0), rec, roadway, 1)

    ext = FeatureExtractor(AbstractFeature[SPEED, POSFT], 2)
    @test rec_length(ext) == 2
    @test length(ext) == 2
    @test pull_features!(ext, [NaN,NaN], rec, roadway, 1) == [10.0, 0.0]

    subset_ext = SubsetExtractor(ext, [1])
    @test rec_length(subset_ext) == 2
    @test length(subset_ext) == 1
    @test pull_features!(subset_ext, [NaN], rec, roadway, 1) == [10.0]

    subset_ext.subset[1] = 2
    @test pull_features!(subset_ext, [NaN], rec, roadway, 1) == [0.0]

    stan_ext = StandardizingExtractor(ext, [1.0, 2.0], [1.0, 4.0])
    @test rec_length(stan_ext) == 2
    @test length(stan_ext) == 2
    @test pull_features!(stan_ext, [NaN,NaN], rec, roadway, 1) == [(10.0-1.0)/1.0, (0.0-2.0)/4.0]
end

## Lidar sensor tests
#=
What needs to be checked:
1. that vehicles at various angles are detected
2. that the correct distance is returned
3. that when a ray encounters multiple vehicles, the correct vehicle is identified
4. that the range rate is correctly returned
5. that the max range of the ray is respected
=#

@testset "lidar sensor" begin
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