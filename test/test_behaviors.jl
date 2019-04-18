struct FakeDriveAction end
struct FakeDriverModel <: DriverModel{FakeDriveAction} end
@testset "driver model interface" begin

    roadway = get_test_roadway()
    trajdata = get_test_trajdata(roadway)
    veh = get(trajdata, 1, 1)

    model = FakeDriverModel()
    reset_hidden_state!(model)
    observe!(model, Scene(), roadway, 1)
    prime_with_history!(model, trajdata, roadway, 1, 2, 1)

    @test get_name(model) == "???"
    @test action_type(model) <: FakeDriveAction
    @test set_desired_speed!(model, 0.0) == FakeDriverModel()
    @test_throws ErrorException rand(model)
    @test_throws ErrorException pdf(model, FakeDriveAction())
    @test_throws ErrorException logpdf(model, FakeDriveAction())
end

@testset "IDM test" begin
    roadway = gen_straight_roadway(1, 500.0)

    models = Dict{Int, DriverModel}()
    models[1] = IntelligentDriverModel(k_spd = 1.0, v_des = 10.0)
    models[2] = IntelligentDriverModel(k_spd = 1.0)
    get_name(models[1]) == "IDM"
    set_desired_speed!(models[2], 5.0)
    @test models[2].v_des == 5.0
    veh_state = VehicleState(Frenet(roadway[LaneTag(1,1)], 0.0), roadway, 5.)
    veh1 = Vehicle(veh_state, VehicleDef(), 1)
    veh_state = VehicleState(Frenet(roadway[LaneTag(1,1)], 70.0), roadway, 5.)
    veh2 = Vehicle(veh_state, VehicleDef(), 2)

    scene = Scene()
    push!(scene, veh1)
    push!(scene, veh2)

    n_steps = 40
    dt = 0.1
    rec = SceneRecord(n_steps, dt)
    simulate!(rec, scene, roadway, models, n_steps)

    @test isapprox(get_by_id(scene, 2).state.v, models[2].v_des)

    # same with noise
    models = Dict{Int, DriverModel}()
    models[1] = IntelligentDriverModel(a = 0.0, k_spd = 1.0, v_des = 10.0, σ=1.0)
    models[2] = IntelligentDriverModel(a = 0.0, k_spd = 1.0, v_des = 5.0, σ=1.0)
    @test pdf(models[1], LaneFollowingAccel(0.0))  > 0.0
    @test logpdf(models[1], LaneFollowingAccel(0.0)) < 0.0
    n_steps = 40
    dt = 0.1
    rec = SceneRecord(n_steps, dt)
    simulate!(rec, scene, roadway, models, n_steps)

    prime_with_history!(IntelligentDriverModel(), rec, roadway, 2)

    println("There should be a warning here: ")

    # initializing vehicles too close
    veh_state = VehicleState(Frenet(roadway[LaneTag(1,1)], 0.0), roadway, 5.)
    veh1 = Vehicle(veh_state, VehicleDef(), 1)
    veh_state = VehicleState(Frenet(roadway[LaneTag(1,1)], 3.0), roadway, 5.)
    veh2 = Vehicle(veh_state, VehicleDef(), 2)

    scene = Scene()
    push!(scene, veh1)
    push!(scene, veh2)

    rec = SceneRecord(n_steps, dt)
    simulate!(rec, scene, roadway, models, 1)
end

struct FakeLaneChanger <: LaneChangeModel end
@testset "lane change interface" begin 
    l = LaneChangeChoice(DIR_RIGHT)
    io = IOBuffer()
    show(io, l)
    close(io)
    roadway = get_test_roadway()
    trajdata = get_test_trajdata(roadway)
    veh = get(trajdata, 1, 1)

    model = FakeLaneChanger()
    reset_hidden_state!(model)
    observe!(model, Scene(), roadway, 1)

    @test get_name(model) == "???"
    @test set_desired_speed!(model, 0.0) == FakeLaneChanger()
    @test_throws ErrorException rand(model)
end

@testset "MOBIL" begin
    timestep = 0.1
    lanemodel = MOBIL(timestep)

    @test get_name(lanemodel) == "MOBIL"
    set_desired_speed!(lanemodel,20.0)
    @test lanemodel.mlon.v_des == 20.0

    roadway = gen_straight_roadway(3, 1000.0)
    veh_state = VehicleState(Frenet(roadway[LaneTag(1,2)], 0.0), roadway, 10.)
    veh1 = Vehicle(veh_state, VehicleDef(), 1)
    veh_state = VehicleState(Frenet(roadway[LaneTag(1,2)], 20.0), roadway, 2.)
    veh2 = Vehicle(veh_state, VehicleDef(), 2)

    dt = 0.5
    n_steps = 10
    models = Dict{Int, DriverModel}()
    models[1] = Tim2DDriver(dt, mlane=MOBIL(dt))
    set_desired_speed!(models[1], 10.0)
    models[2] = Tim2DDriver(dt, mlane=MOBIL(dt))
    set_desired_speed!(models[2], 2.0)

    scene = Scene([veh1, veh2])

    rec = SceneRecord(n_steps, dt)
    simulate!(rec, scene, roadway, models, n_steps)

    @test scene[1].state.posF.roadind.tag == LaneTag(1, 2)
    @test scene[2].state.posF.roadind.tag == LaneTag(1, 2)

end

@testset "Tim2DDriver" begin
    timestep = 0.1
    drivermodel = Tim2DDriver(timestep)
    @test get_name(drivermodel) == "Tim2DDriver"

    set_desired_speed!(drivermodel,20.0)
    @test drivermodel.mlon.v_des == 20.0
    @test drivermodel.mlane.v_des == 20.0

    roadway = gen_straight_roadway(3, 1000.0)
    veh_state = VehicleState(Frenet(roadway[LaneTag(1,2)], 0.0), roadway, 10.)
    veh1 = Vehicle(veh_state, VehicleDef(), 1)
    veh_state = VehicleState(Frenet(roadway[LaneTag(1,2)], 10.0), roadway, 2.)
    veh2 = Vehicle(veh_state, VehicleDef(), 2)

    dt = 0.5
    n_steps = 10
    models = Dict{Int, DriverModel}()
    models[1] = Tim2DDriver(dt)
    set_desired_speed!(models[1], 10.0)
    models[2] = Tim2DDriver(dt)
    set_desired_speed!(models[2], 2.0)

    scene = Scene([veh1, veh2])

    rec = SceneRecord(n_steps, dt)
    simulate!(rec, scene, roadway, models, n_steps)

    @test scene[1].state.posF.roadind.tag == LaneTag(1, 3)
    @test scene[2].state.posF.roadind.tag == LaneTag(1, 2)
end

@testset "lane following" begin 
    roadway = gen_straight_roadway(1, 500.0)

    models = Dict{Int, DriverModel}()
    models[1] = StaticLaneFollowingDriver()
    @test get_name(models[1]) == "ProportionalSpeedTracker"
    @test pdf(models[1], LaneFollowingAccel(-1.0)) ≈ 0.0
    @test logpdf(models[1], LaneFollowingAccel(-1.0)) ≈ -Inf
    models[2] = PrincetonDriver(k = 1.0)
    @test get_name(models[2]) == "PrincetonDriver"
    @test pdf(models[2], LaneFollowingAccel(-1.0)) == Inf
    @test logpdf(models[2], LaneFollowingAccel(-1.0)) == Inf
    set_desired_speed!(models[2], 5.0)
    @test models[2].v_des == 5.0
    models[3] = ProportionalSpeedTracker(k = 1.0)
    @test get_name(models[3]) == "ProportionalSpeedTracker"
    @test pdf(models[3], LaneFollowingAccel(-1.0)) == Inf
    @test logpdf(models[3], LaneFollowingAccel(-1.0)) == Inf
    set_desired_speed!(models[3], 5.0)
    @test models[3].v_des == 5.0

    veh_state = VehicleState(Frenet(roadway[LaneTag(1,1)], 0.0), roadway, 5.)
    veh1 = Vehicle(veh_state, VehicleDef(), 1)
    veh_state = VehicleState(Frenet(roadway[LaneTag(1,1)], 70.0), roadway, 5.)
    veh2 = Vehicle(veh_state, VehicleDef(), 2)
    veh_state = VehicleState(Frenet(roadway[LaneTag(1,1)], 130.0), roadway, 5.)
    veh3 = Vehicle(veh_state, VehicleDef(), 3)

    scene = Scene()
    push!(scene, veh1)
    push!(scene, veh2)
    push!(scene, veh3)

    n_steps = 40
    dt = 0.1
    rec = SceneRecord(n_steps, dt)
    simulate!(rec, scene, roadway, models, n_steps)

    @test isapprox(get_by_id(scene, 2).state.v, models[2].v_des, atol=1e-3)
    @test isapprox(get_by_id(scene, 3).state.v, models[3].v_des)

    # same wth noise 
    models = Dict{Int, DriverModel}()
    models[1] = StaticLaneFollowingDriver()
    models[2] = PrincetonDriver(k = 1.0, v_des=5.0, σ=1.0, a=0.0)
    models[3] = ProportionalSpeedTracker(k = 1.0, v_des=5.0, σ=1.0, a=0.0)

    @test pdf(models[3], LaneFollowingAccel(0.0)) > 0.0
    @test logpdf(models[3], LaneFollowingAccel(0.0)) < 0.0
    
    @test pdf(models[2], LaneFollowingAccel(0.0)) > 0.0
    @test logpdf(models[2], LaneFollowingAccel(0.0)) < 0.0

    n_steps = 40
    dt = 0.1
    rec = SceneRecord(n_steps, dt)
    simulate!(rec, scene, roadway, models, n_steps)

    
    @test isapprox(get_by_id(scene, 2).state.v, models[2].v_des, atol=1.0)
    @test isapprox(get_by_id(scene, 3).state.v, models[3].v_des, atol=1.0)
end

function generate_sidewalk_env()
    roadway_length = 100.
    crosswalk_length = 15.
    crosswalk_width = 6.0
    crosswalk_pos = roadway_length/2
    sidewalk_width = 3.0
    sidewalk_pos = crosswalk_length/2 - sidewalk_width / 2
    # Generate straight roadway of length roadway_length with 2 lanes.
    # Returns a Roadway type (Array of segments).
    # There is already a method to generate a simple straight roadway, which we use here.
    roadway = gen_straight_roadway(2, roadway_length) 

    # Generate the crosswalk.
    # Our crosswalk does not have a predefined method for generation, so we define it with a LaneTag and a curve.
    n_samples = 2 # for curve generation
    crosswalk = Lane(LaneTag(2,1), gen_straight_curve(VecE2(crosswalk_pos, -crosswalk_length/2),
                                                    VecE2(crosswalk_pos, crosswalk_length/2),
                                                    n_samples), width = crosswalk_width)
    cw_segment = RoadSegment(2, [crosswalk])
    push!(roadway.segments, cw_segment) # Append the crosswalk to the roadway

    # Generate the sidewalk.
    top_sidewalk = Lane(LaneTag(3, 1), gen_straight_curve(VecE2(0., sidewalk_pos),
                                                        VecE2(roadway_length, sidewalk_pos),
                                                            n_samples), width = sidewalk_width)
    bottom_sidewalk = Lane(LaneTag(3, 2), gen_straight_curve(VecE2(0., -(sidewalk_pos - sidewalk_width)),
                                                            VecE2(roadway_length, -(sidewalk_pos - sidewalk_width)),
                                                                n_samples), width = sidewalk_width) 
    # Note: we subtract the sidewalk_width from the sidewalk position so that the edge is flush with the road.
    sw_segment = RoadSegment(3, [top_sidewalk, bottom_sidewalk])
    push!(roadway.segments, sw_segment)
    sidewalk = [top_sidewalk, bottom_sidewalk]
    return roadway, crosswalk, sidewalk
end


@testset "sidewalk pedestrian" begin
    roadway, crosswalk, sidewalk = generate_sidewalk_env()
    # dummy test for the constructor
    ped=SidewalkPedestrianModel(timestep=0.1,
                                crosswalk= roadway[LaneTag(1,1)],
                                sw_origin = roadway[LaneTag(1,1)],
                                sw_dest = roadway[LaneTag(1,1)]
                                )
    @test ped.ttc_threshold >= 1.0
    @test get_name(ped) == "SidewalkPedestrianModel"

    timestep = 0.1

    # Crossing pedestrian definition
    ped_init_state = VehicleState(VecSE2(49.0,-3.0,0.), sidewalk[2], roadway, 1.3)
    ped = Vehicle(ped_init_state, VehicleDef(AgentClass.PEDESTRIAN, 1.0, 1.0), 1)

    # Car definition
    car_initial_state = VehicleState(VecSE2(0.0, 0., 0.), roadway.segments[1].lanes[1],roadway, 8.0)
    car = Vehicle(car_initial_state, VehicleDef(), 2)

    scene = Scene()
    push!(scene, ped)
    push!(scene, car)

    # Define a model for each entity present in the scene
    models = Dict{Int, DriverModel}()

    ped_id = 1
    car_id = 2

    models[ped_id] = SidewalkPedestrianModel(timestep=timestep, 
                                                crosswalk= crosswalk,
                                                sw_origin = sidewalk[2],
                                                sw_dest = sidewalk[1]
                                                )

    models[car_id] = 
    LatLonSeparableDriver( # produces LatLonAccels
            ProportionalLaneTracker(), # lateral model
            IntelligentDriverModel(), # longitudinal model
    )

    nticks = 300
    rec = SceneRecord(nticks+1, timestep)
    # Execute the simulation
    simulate!(rec, scene, roadway, models, nticks)

    ped = get_by_id(rec[0], ped_id)
end