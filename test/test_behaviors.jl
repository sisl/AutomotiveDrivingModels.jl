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
    @test_throws ErrorException rand(model)
    @test_throws ErrorException pdf(model, FakeDriveAction())
    @test_throws ErrorException logpdf(model, FakeDriveAction())
end

@testset "sidewalk pedestrian" begin 
    # dummy test for the constructor
    roadway=gen_straight_roadway(1, 50.0, lane_width=3.0)
    ped=SidewalkPedestrianModel(timestep=0.1, 
                                crosswalk= roadway[LaneTag(1,1)],
                                sw_origin = roadway[LaneTag(1,1)],
                                sw_dest = roadway[LaneTag(1,1)]
                                )
    @test ped.ttc_threshold >= 1.0
end

@testset "IDM test" begin 
    roadway = gen_straight_roadway(1, 500.0)

    models = Dict{Int, DriverModel}()
    models[1] = IntelligentDriverModel(k_spd = 1.0, v_des = 10.0)
    models[2] = IntelligentDriverModel(k_spd = 1.0, v_des = 5.0)

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