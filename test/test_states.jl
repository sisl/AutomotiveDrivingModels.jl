function get_test_trajdata(roadway::Roadway)
    trajdata = Trajdata(0.1)

    trajdata.defs[1] = VehicleDef(AgentClass.CAR, 5.0, 3.0)
    trajdata.defs[2] = VehicleDef(AgentClass.CAR, 5.0, 3.0)

    push!(trajdata.states, RecordState{VehicleState, Int64}(VehicleState(VecSE2(0.0,0.0,0.0), roadway, 10.0), 1)) # car 1, frame 1
    push!(trajdata.states, RecordState{VehicleState, Int64}(VehicleState(VecSE2(3.0,0.0,0.0), roadway, 20.0), 2)) # car 2, frame 1
    push!(trajdata.states, RecordState{VehicleState, Int64}(VehicleState(VecSE2(1.0,0.0,0.0), roadway, 10.0), 1)) # car 1, frame 2
    push!(trajdata.states, RecordState{VehicleState, Int64}(VehicleState(VecSE2(5.0,0.0,0.0), roadway, 20.0), 2)) # car 2, frame 2

    push!(trajdata.frames, RecordFrame(1,2))
    push!(trajdata.frames, RecordFrame(3,4))

    trajdata
end

@testset "1d state" begin 
    s = State1D(0.0, 0.0)
    path, io = mktemp()
    write(io, MIME"text/plain"(), s)
    close(io)
    io = open(path)
    s2 = read(io, MIME"text/plain"(), State1D)
    close(io)
    @test s == s2 

    veh = Vehicle1D(s, VehicleDef(), 1)
    scene = Scene1D()
    push!(scene, veh)
    scene2 = Scene1D([veh])
    @test scene[1].state.s == 0.0
    @test first(scene2.entities) == first(scene.entities)
    @test scene2.n == scene.n == 1
    @test get_center(veh) == 0.0
    @test get_footpoint(veh) == 0.0
    @test get_front(veh) == veh.def.length/2
    @test get_rear(veh) == - veh.def.length/2

end

@testset "VehicleState" begin 
    s = VehicleState(VecSE2(0.0,0.0,0.0), Frenet(NULL_ROADINDEX, 0.0, 0.0, 0.0), 10.0)
    @test isapprox(get_vel_s(s), 10.0)
    @test isapprox(get_vel_t(s),  0.0)
    show(IOBuffer(), s.posF)
    show(IOBuffer(), s)

    s = VehicleState(VecSE2(0.0,0.0,0.0), Frenet(NULL_ROADINDEX, 0.0, 0.0, 0.1), 10.0)
    @test isapprox(get_vel_s(s), 10.0*cos(0.1))
    @test isapprox(get_vel_t(s), 10.0*sin(0.1))
    
    vehdef = VehicleDef(AgentClass.CAR, 5.0, 3.0)
    veh = Vehicle(s, vehdef, 1)
    @test isapprox(get_footpoint(veh), VecSE2(0.0,0.0,0.0))
    show(IOBuffer(), vehdef)
    show(IOBuffer(), veh)

    ri = RoadIndex(CurveIndex(1,0.1), LaneTag(1,2))
    @test isapprox(Frenet(ri, 0.0, 0.0, 0.1), Frenet(ri, 0.0, 0.0, 0.1))
    @test VehicleState(VecSE2(0.1,0.2,0.3), 1.0) == VehicleState(VecSE2(0.1,0.2,0.3), NULL_FRENET, 1.0)

    @test get_front(veh).x == vehdef.length/2
    @test get_rear(veh).x == -vehdef.length/2
end

@testset "scene" begin 
    roadway = get_test_roadway()
    trajdata = get_test_trajdata(roadway)

    s1 = VehicleState(VecSE2(0.0, 0.0, 0.0), roadway, 0.0)
    s2 = VehicleState(VecSE2(5.0, 0.0, 0.0), roadway, 0.0)
    s3 = lerp(s1, s2, 0.5, roadway)
    @test s3.posG.x == 2.5
    @test s3.posF.s == 2.5

    vehstate = VehicleState(VecSE2(0.0, 0.0, 0.0), roadway, 0.0)
    vehstate1 = VehicleState(VecSE2(0.0, 0.0, 0.0), roadway[LaneTag(1,1)], roadway, 0.0)
    @test vehstate1 == vehstate
    veh = Vehicle(vehstate, VehicleDef(), 1)
    scene1 = Scene()
    push!(scene1, veh)
    scene2 = Scene([veh])
    @test first(scene1.entities) == first(scene2.entities)
    @test scene1.n == scene2.n

    io = IOBuffer()
    show(io, scene1)
    close(io)

    veh2 = Entity(vehstate, BicycleModel(VehicleDef()), 1)
    veh3 = convert(Vehicle, veh2)
    @test veh3 == veh

    rec = SceneRecord(1, 0.5)
    rec = SceneRecord(1, 0.5, 10)

    io = IOBuffer()
    show(io, rec)
    close(io)
    scene = Scene()
    get!(scene, trajdata, 1)
    @test length(scene) == 2
    for (i,veh) in enumerate(scene)
        @test scene[i].state == get_state(trajdata, i, 1)
        @test scene[i].def == get_def(trajdata, i)
    end

    scene2 = Scene(deepcopy(scene.entities), 2)
    @test length(scene2) == 2
    for (i,veh) in enumerate(scene2)
        @test scene2[i].state == get_state(trajdata, i, 1)
        @test scene2[i].def == get_def(trajdata, i)
    end

    @test get_by_id(scene, 1) == scene[1]

    empty!(scene2)
    @test length(scene2) == 0

    copyto!(scene2, scene)
    @test length(scene2) == 2
    for (i,veh) in enumerate(scene2)
        @test scene2[i].state == get_state(trajdata, i, 1)
        @test scene2[i].def == get_def(trajdata, i)
    end

    delete!(scene2, scene2[1])
    @test length(scene2) == 1
    @test scene2[1].state == get_state(trajdata, 2, 1)
    @test scene2[1].def == get_def(trajdata, 2)
    scene2[1] = deepcopy(scene[1])
    @test scene2[1].state == get_state(trajdata, 1, 1)
    @test scene2[1].def == get_def(trajdata, 1)

    @test findfirst(1, scene) == 1
    @test findfirst(2, scene) == 2
    @test get_first_available_id(scene) == 3

    @test in(1, scene)
    @test in(2, scene)
    @test !in(3, scene)

    veh = scene[2]
    @test veh.state == get_state(trajdata, 2, 1)
    @test veh.def == get_def(trajdata, 2)

    push!(scene, get_state(trajdata, 1, 1))
end

@testset "trajdata" begin
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
    @test findfirst_frame_with_id(trajdata, -1) == nothing
    @test findlast_frame_with_id(trajdata, 1) == 2
    @test findlast_frame_with_id(trajdata, 2) == 2
    @test findlast_frame_with_id(trajdata, -1) == nothing

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
    @test_throws ArgumentError get(trajdata, 10, 1)
    @test_throws BoundsError get(trajdata, 1, 10)

    let
        iter = ListRecordStateByIdIterator(trajdata, 1)
        items = collect(iter) # list of (frame_index, state)
        @test length(items) == 2
        @test items[1][1] == 1
        @test items[1][2] == get_state(trajdata, 1, 1)
        @test items[2][1] == 2
        @test items[2][2] == get_state(trajdata, 1, 2)

        iter = ListRecordStateByIdIterator(trajdata, 2)
        items = collect(iter)
        @test length(items) == 2
        @test items[1][1] == 1
        @test items[1][2] == get_state(trajdata, 2, 1)
        @test items[2][1] == 2
        @test items[2][2] == get_state(trajdata, 2, 2)
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

@testset "SceneRecord" begin
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
