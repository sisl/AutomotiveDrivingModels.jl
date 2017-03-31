function get_test_trajdata(roadway::Roadway)
    trajdata = Trajdata(0.1)

    trajdata.defs[1] = VehicleDef(AgentClass.CAR, 5.0, 3.0)
    trajdata.defs[2] = VehicleDef(AgentClass.CAR, 5.0, 3.0)

    push!(trajdata.states, RecordState{VehicleState, Int}(VehicleState(VecSE2(0.0,0.0,0.0), roadway, 10.0), 1)) # car 1, frame 1
    push!(trajdata.states, RecordState{VehicleState, Int}(VehicleState(VecSE2(3.0,0.0,0.0), roadway, 20.0), 2)) # car 2, frame 1
    push!(trajdata.states, RecordState{VehicleState, Int}(VehicleState(VecSE2(1.0,0.0,0.0), roadway, 10.0), 1)) # car 1, frame 2
    push!(trajdata.states, RecordState{VehicleState, Int}(VehicleState(VecSE2(5.0,0.0,0.0), roadway, 20.0), 2)) # car 2, frame 2

    push!(trajdata.frames, RecordFrame(1,2))
    push!(trajdata.frames, RecordFrame(3,4))

    trajdata
end

let
    roadway = get_test_roadway()
    trajdata = get_test_trajdata(roadway)

    @test nframes(trajdata) == 2
    @test !frame_inbounds(trajdata, 0)
    @test frame_inbounds(trajdata, 1)
    @test frame_inbounds(trajdata, 2)
    @test !frame_inbounds(trajdata, 3)

    @test n_objects_in_frame(trajdata, 1) == 2
    @test n_objects_in_frame(trajdata, 2) == 2

    @test nth_id(trajdata, 1) == 1
    @test nth_id(trajdata, 1, 2) == 2
    @test nth_id(trajdata, 2, 1) == 1
    @test nth_id(trajdata, 2, 2) == 2

    @test findfirst_frame_with_id(trajdata, 1) == 1
    @test findfirst_frame_with_id(trajdata, 2) == 1
    @test findfirst_frame_with_id(trajdata, -1) == 0
    @test findlast_frame_with_id(trajdata, 1) == 2
    @test findlast_frame_with_id(trajdata, 2) == 2
    @test findlast_frame_with_id(trajdata, -1) == 0

    @test sort!(get_ids(trajdata)) == [1,2]

    @test in(1, trajdata, 1)
    @test in(1, trajdata, 2)
    @test in(2, trajdata, 1)
    @test in(2, trajdata, 2)
    @test !in(3, trajdata, 1)

    @test isapprox(get_time(trajdata, 1), 0.0)
    @test isapprox(get_time(trajdata, 2), 0.1)

    @test isapprox(get_elapsed_time(trajdata, 1, 2),  0.1)
    @test isapprox(get_elapsed_time(trajdata, 2, 1), -0.1)

    @test get_timestep(trajdata) == 0.1

    veh = get(trajdata, 1, 1)
    @test veh.state == VehicleState(VecSE2(0.0,0.0,0.0), roadway, 10.0)
    @test_throws BoundsError get(trajdata, 10, 1)
    @test_throws BoundsError get(trajdata, 1, 10)

    let
        iter = ListRecordIterator(trajdata, 1)
        vehs = collect(iter)
        @test length(vehs) == 2
        @test vehs[1][1] == 1
        @test vehs[1][2].state == get_state(trajdata, 1, 1)
        @test vehs[2][1] == 2
        @test vehs[2][2].state == get_state(trajdata, 1, 2)

        iter = ListRecordIterator(trajdata, 2)
        vehs = collect(iter)
        @test length(vehs) == 2
        @test vehs[1][1] == 1
        @test vehs[1][2].state == get_state(trajdata, 2, 1)
        @test vehs[2][1] == 2
        @test vehs[2][2].state == get_state(trajdata, 2, 2)
    end

    path, io = mktemp()
    write(io, MIME"text/plain"(), trajdata)
    close(io)

    io = open(path)
    trajdata2 = read(io, MIME"text/plain"(), Trajdata)
    close(io)
    rm(path)

    @test nframes(trajdata2) == nframes(trajdata)
    for i in 1 : nframes(trajdata2)
        @test n_objects_in_frame(trajdata2, i) == n_objects_in_frame(trajdata, i)
        for j in 1 : n_objects_in_frame(trajdata, i)
            veh1 = get(trajdata, j, i)
            veh2 = get(trajdata2, j, i)
            @test veh1.id == veh2.id
            @test veh1.def.class == veh2.def.class
            @test isapprox(veh1.def.length, veh2.def.length)
            @test isapprox(veh1.def.width, veh2.def.width)

            @test isapprox(veh1.state.v, veh2.state.v)
            @test isapprox(veh1.state.posG, veh2.state.posG, atol=1e-3)
            @test isapprox(veh1.state.posF.s, veh2.state.posF.s, atol=1e-3)
            @test isapprox(veh1.state.posF.t, veh2.state.posF.t, atol=1e-3)
            @test isapprox(veh1.state.posF.ϕ, veh2.state.posF.ϕ, atol=1e-6)
            @test veh1.state.posF.roadind.tag == veh2.state.posF.roadind.tag
            @test veh1.state.posF.roadind.ind.i == veh2.state.posF.roadind.ind.i
            @test isapprox(veh1.state.posF.roadind.ind.t, veh2.state.posF.roadind.ind.t, atol=1e-3)
        end
    end

    trajdata3 = get_subinterval(trajdata2, 1, nframes(trajdata2))
    @test nframes(trajdata3) == nframes(trajdata2)
    for i in 1 : nframes(trajdata3)
        @test n_objects_in_frame(trajdata3, i) == n_objects_in_frame(trajdata2, i)
        for j in 1 : n_objects_in_frame(trajdata2, i)
            veh1 = get(trajdata2, j, i)
            veh2 = get(trajdata3, j, i)
            @test veh1.id == veh2.id
            @test veh1.def.class == veh2.def.class
            @test isapprox(veh1.def.length, veh2.def.length)
            @test isapprox(veh1.def.width, veh2.def.width)

            @test isapprox(veh1.state.v, veh2.state.v)
            @test isapprox(veh1.state.posG, veh2.state.posG, atol=1e-3)
            @test isapprox(veh1.state.posF.s, veh2.state.posF.s, atol=1e-3)
            @test isapprox(veh1.state.posF.t, veh2.state.posF.t, atol=1e-3)
            @test isapprox(veh1.state.posF.ϕ, veh2.state.posF.ϕ, atol=1e-6)
            @test veh1.state.posF.roadind.tag == veh2.state.posF.roadind.tag
            @test veh1.state.posF.roadind.ind.i == veh2.state.posF.roadind.ind.i
            @test isapprox(veh1.state.posF.roadind.ind.t, veh2.state.posF.roadind.ind.t, atol=1e-3)
        end
    end

    trajdata3 = get_subinterval(trajdata2, 1, 1)
    @test nframes(trajdata3) == 1
    let
        i = 1
        @test n_objects_in_frame(trajdata3, i) == n_objects_in_frame(trajdata2, i)
        for j in 1 : n_objects_in_frame(trajdata2, i)
            veh1 = get(trajdata2, j, i)
            veh2 = get(trajdata3, j, i)
            @test veh1.id == veh2.id
            @test veh1.def.class == veh2.def.class
            @test isapprox(veh1.def.length, veh2.def.length)
            @test isapprox(veh1.def.width, veh2.def.width)

            @test isapprox(veh1.state.v, veh2.state.v)
            @test isapprox(veh1.state.posG, veh2.state.posG, atol=1e-3)
            @test isapprox(veh1.state.posF.s, veh2.state.posF.s, atol=1e-3)
            @test isapprox(veh1.state.posF.t, veh2.state.posF.t, atol=1e-3)
            @test isapprox(veh1.state.posF.ϕ, veh2.state.posF.ϕ, atol=1e-6)
            @test veh1.state.posF.roadind.tag == veh2.state.posF.roadind.tag
            @test veh1.state.posF.roadind.ind.i == veh2.state.posF.roadind.ind.i
            @test isapprox(veh1.state.posF.roadind.ind.t, veh2.state.posF.roadind.ind.t, atol=1e-3)
        end
    end
end