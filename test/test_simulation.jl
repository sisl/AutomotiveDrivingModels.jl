struct NoCallback end
struct NoActionCallback end
AutomotiveDrivingModels.run_callback(callback::NoActionCallback, scenes::Vector{Frame{E}}, roadway::R, models::Dict{I,M}, tick::Int) where {E,R,I,M<:DriverModel} = false
struct WithActionCallback end
AutomotiveDrivingModels.run_callback(callback::WithActionCallback, scenes::Vector{Frame{E}}, actions::Union{Nothing, Vector{Frame{A}}}, roadway::R, models::Dict{I,M}, tick::Int) where {E<:Entity,A<:EntityAction,R,I,M<:DriverModel} = false

@testset "simulation" begin
    roadway = gen_straight_roadway(1, 500.0)

    models = Dict{Int, DriverModel}()
    models[1] = IntelligentDriverModel(k_spd = 1.0, v_des = 10.0)
    models[2] = IntelligentDriverModel(k_spd = 1.0, v_des = 5.0)

    veh_state = VehicleState(Frenet(roadway[LaneTag(1,1)], 0.0), roadway, 5.)
    veh1 = Entity(veh_state, VehicleDef(), 1)
    veh_state = VehicleState(Frenet(roadway[LaneTag(1,1)], 70.0), roadway, 5.)
    veh2 = Entity(veh_state, VehicleDef(), 2)

    scene = Frame([veh1, veh2])

    n_steps = 40
    dt = 0.1
    @inferred simulate(scene, roadway, models, n_steps, dt)

    reset_hidden_states!(models)

    # initializing vehicles too close
    veh_state = VehicleState(Frenet(roadway[LaneTag(1,1)], 0.0), roadway, 10.)
    veh1 = Entity(veh_state, VehicleDef(), 1)
    veh_state = VehicleState(Frenet(roadway[LaneTag(1,1)], 5.0), roadway, 5.)
    veh2 = Entity(veh_state, VehicleDef(), 2)

    scene = Frame([veh1, veh2])

    scenes = @inferred simulate(scene, roadway, models, n_steps, dt, callbacks=(CollisionCallback(),))
    @test length(scenes) < 10

    # make sure warnings, errors and deprecations in run_callback work as expected
    @test_deprecated simulate(scene, roadway, models, 10, .1, callbacks=(NoActionCallback(),))
    @test_nowarn simulate(scene, roadway, models, 10, .1, callbacks=(WithActionCallback(),))

    # collision right from start
    veh_state = VehicleState(Frenet(roadway[LaneTag(1,1)], 0.0), roadway, 10.)
    veh1 = Entity(veh_state, VehicleDef(), 1)
    veh_state = VehicleState(Frenet(roadway[LaneTag(1,1)], 1.0), roadway, 5.)
    veh2 = Entity(veh_state, VehicleDef(), 2)

    scene = Frame([veh1, veh2])

    scenes = @inferred simulate(scene, roadway, models, n_steps, dt, callbacks=(CollisionCallback(),))
    @test length(scenes) == 1
end

@testset "trajdata simulation" begin
  roadway = get_test_roadway()
  trajdata = get_test_trajdata(roadway)

  veh_state = VehicleState(Frenet(roadway[LaneTag(1,1)], 6.0), roadway, 10.)
  ego = Entity(veh_state, VehicleDef(), 2)
  model = ProportionalSpeedTracker()

  scenes = simulate_from_history(model, roadway, trajdata, ego.id, 0.1)

  @test findfirst(ego.id, scenes[end]) != nothing 
end
