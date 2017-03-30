let
    roadway = get_test_roadway()
    trajdata = get_test_trajdata(roadway)

    Δt = 0.1
    rec = SceneRecord(5, Δt)
    @test capacity(rec) == 5
    @test nframes(rec) == 0
    @test !pastframe_inbounds(rec, 0)
    @test !pastframe_inbounds(rec, -1)
    @test !pastframe_inbounds(rec, 1)

    scene = get!(Scene(), trajdata, 1)
    update!(rec, scene)
    @test nframes(rec) == 1
    @test pastframe_inbounds(rec, 0)
    @test !pastframe_inbounds(rec, -1)
    @test !pastframe_inbounds(rec, 1)
    @test isapprox(get_elapsed_time(rec, 0), 0)
    @test rec[0][1].state == get_state(trajdata, 1, 1)
    @test rec[0][1].def == get_def(trajdata, 1)
    @test rec[0][2].state == get_state(trajdata, 2, 1)
    @test rec[0][2].def == get_def(trajdata, 2)
    show(IOBuffer(), rec)


    get!(scene, trajdata, 2)
    update!(rec, scene)
    @test nframes(rec) == 2
    @test pastframe_inbounds(rec, 0)
    @test pastframe_inbounds(rec, -1)
    @test !pastframe_inbounds(rec, 1)
    @test isapprox(get_elapsed_time(rec,  0),  0)
    @test isapprox(get_elapsed_time(rec, -1), Δt)
    @test isapprox(get_elapsed_time(rec, -1, 0), Δt)
    @test rec[0][1].state == get_state(trajdata, 1, 2)
    @test rec[0][1].def == get_def(trajdata, 1)
    @test rec[0][2].state == get_state(trajdata, 2, 2)
    @test rec[0][2].def == get_def(trajdata, 2)
    @test rec[-1][1].state == get_state(trajdata, 1, 1)
    @test rec[-1][1].def == get_def(trajdata, 1)
    @test rec[-1][2].state == get_state(trajdata, 2, 1)
    @test rec[-1][2].def == get_def(trajdata, 2)

    scene2 = get!(Scene(), rec)
    @test scene2[1].state == get_state(trajdata, 1, 2)
    @test scene2[1].def == get_def(trajdata, 1)
    @test scene2[2].state == get_state(trajdata, 2, 2)
    @test scene2[2].def == get_def(trajdata, 2)

    get!(scene2, rec, -1)
    @test scene2[1].state == get_state(trajdata, 1, 1)
    @test scene2[1].def == get_def(trajdata, 1)
    @test scene2[2].state == get_state(trajdata, 2, 1)
    @test scene2[2].def == get_def(trajdata, 2)

    empty!(rec)
    @test nframes(rec) == 0

    test_veh_state = VehicleState(VecSE2(7.0,7.0,2.0), roadway, 10.0)
    test_veh_def = VehicleDef(AgentClass.CAR, 5.0, 3.0)
    test_veh = Vehicle(test_veh_state, test_veh_def, 999)
    rec[-1][1] = test_veh
    @test rec[-1][1].state == test_veh_state
end
