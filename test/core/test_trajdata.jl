let
    roadway = get_test_roadway()
    trajdata = Trajdata(roadway, 1)

    trajdata.vehdefs[1] = VehicleDef(1, AgentClass.CAR, 5.0, 3.0)
    trajdata.vehdefs[2] = VehicleDef(2, AgentClass.CAR, 5.0, 3.0)

    push!(trajdata.states, TrajdataState(1, VehicleState(VecSE2(0.0,0.0,0.0), roadway, 10.0))) # car 1, frame 1
    push!(trajdata.states, TrajdataState(2, VehicleState(VecSE2(3.0,0.0,0.0), roadway, 20.0))) # car 2, frame 1
    push!(trajdata.states, TrajdataState(1, VehicleState(VecSE2(1.0,0.0,0.0), roadway, 10.0))) # car 1, frame 2
    push!(trajdata.states, TrajdataState(2, VehicleState(VecSE2(5.0,0.0,0.0), roadway, 20.0))) # car 2, frame 2

    push!(trajdata.frames, TrajdataFrame(1,2,0.0))
    push!(trajdata.frames, TrajdataFrame(3,4,0.1))

    @test nframes(trajdata) == 2
    @test !frame_inbounds(trajdata, 0)
    @test frame_inbounds(trajdata, 1)
    @test frame_inbounds(trajdata, 2)
    @test !frame_inbounds(trajdata, 3)

    @test carsinframe(trajdata, 1) == 2
    @test carsinframe(trajdata, 2) == 2

    @test nth_carid(trajdata, 1) == 1
    @test nth_carid(trajdata, 1, 2) == 2
    @test nth_carid(trajdata, 2, 1) == 1
    @test nth_carid(trajdata, 2, 2) == 2

    @test iscarinframe(trajdata, 1, 1)
    @test iscarinframe(trajdata, 1, 2)
    @test iscarinframe(trajdata, 2, 1)
    @test iscarinframe(trajdata, 2, 2)
    @test !iscarinframe(trajdata, 3, 1)
end