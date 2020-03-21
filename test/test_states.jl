function get_test_trajdata(roadway::Roadway)
    scene1 = Frame([
        Entity(VehicleState(VecSE2(0.0,0.0,0.0), roadway, 10.0), VehicleDef(), 1),
        Entity(VehicleState(VecSE2(3.0,0.0,0.0), roadway, 20.0), VehicleDef(), 2)
    ]
    )
    scene2 = Frame([
        Entity(VehicleState(VecSE2(1.0,0.0,0.0), roadway, 10.0), VehicleDef(), 1),
        Entity(VehicleState(VecSE2(5.0,0.0,0.0), roadway, 20.0), VehicleDef(), 2)
    ]
    )

    trajdata = [scene1, scene2]
    return trajdata
end

@testset "VehicleState" begin 
    s = VehicleState(VecSE2(0.0,0.0,0.0), Frenet(NULL_ROADINDEX, 0.0, 0.0, 0.0), 10.0)
    @test isapprox(velf(s).s, 10.0)
    @test isapprox(velf(s).t,  0.0)
    show(IOBuffer(), s.posF)
    show(IOBuffer(), s)

    s = VehicleState(VecSE2(0.0,0.0,0.0), Frenet(NULL_ROADINDEX, 0.0, 0.0, 0.1), 10.0)
    @test isapprox(velf(s).s, 10.0*cos(0.1))
    @test isapprox(velf(s).t, 10.0*sin(0.1))
    @test isapprox(velg(s).x, 10.0)
    @test isapprox(velg(s).y, 0.0)
    @test isapprox(vel(s), 10.0)
    
    vehdef = VehicleDef(AgentClass.CAR, 5.0, 3.0)
    veh = Entity(s, vehdef, 1)
    @test isapprox(get_footpoint(veh), VecSE2(0.0,0.0,0.0))
    show(IOBuffer(), vehdef)
    show(IOBuffer(), veh)

    ri = RoadIndex(CurveIndex(1,0.1), LaneTag(1,2))
    @test isapprox(Frenet(ri, 0.0, 0.0, 0.1), Frenet(ri, 0.0, 0.0, 0.1))
    @test VehicleState(VecSE2(0.1,0.2,0.3), 1.0) == VehicleState(VecSE2(0.1,0.2,0.3), NULL_FRENET, 1.0)

    @test get_front(veh).x == vehdef.length/2
    @test get_rear(veh).x == -vehdef.length/2

    roadway = gen_straight_roadway(3, 100.0)
    veh = VehicleState(VecSE2(0.0, 0.0, 0.0), roadway, 0.0)
    veh = Entity(veh, VehicleDef(), 1)
    @test get_lane(roadway, veh).tag == LaneTag(1,1)
    @test get_lane(roadway, veh).tag == get_lane(roadway, veh.state).tag
    veh = VehicleState(VecSE2(0.0, 3.0, 0.0), roadway, 0.0)
    @test get_lane(roadway, veh).tag == LaneTag(1,2)
    veh = VehicleState(VecSE2(0.0, 6.0, 0.0), roadway, 0.0)
    @test get_lane(roadway, veh).tag == LaneTag(1,3)
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
    veh = Entity(vehstate, VehicleDef(), 1)
    scene1 = Frame([veh])
    scene2 = Frame([veh])
    @test first(scene1.entities) == first(scene2.entities)
    @test scene1.n == scene2.n

    io = IOBuffer()
    show(io, scene1)
    close(io)

    veh2 = Entity(vehstate, BicycleModel(VehicleDef()), 1)
    veh3 = convert(Entity{VehicleState, VehicleDef, Int64}, veh2)
    @test veh3 == veh

    scene = Frame([veh, veh2, veh3])

    io = IOBuffer()
    show(io, scene)
    close(io)
    scene = Frame(typeof(veh))
    copyto!(scene, trajdata[1])
    @test length(scene) == 2
    for (i,veh) in enumerate(scene)
        @test scene[i].state == trajdata[1][i].state
        @test scene[i].def == trajdata[1][i].def
    end

    scene2 = Frame(deepcopy(scene.entities), 2)
    @test length(scene2) == 2
    for (i,veh) in enumerate(scene2)
        @test scene2[i].state == trajdata[1][i].state
        @test scene2[i].def == trajdata[1][i].def
    end

    @test get_by_id(scene, 1) == scene[1]

    empty!(scene2)
    @test length(scene2) == 0

    copyto!(scene2, scene)
    @test length(scene2) == 2
    for (i,veh) in enumerate(scene2)
        @test scene2[i].state == trajdata[1][i].state
        @test scene2[i].def == trajdata[1][i].def
    end

    delete!(scene2, scene2[1])
    @test length(scene2) == 1
    @test scene2[1].state == trajdata[1][2].state
    @test scene2[1].def == trajdata[1][2].def
    scene2[1] = deepcopy(scene[1])
    @test scene2[1].state == trajdata[1][1].state
    @test scene2[1].def == trajdata[1][1].def

    @test findfirst(1, scene) == 1
    @test findfirst(2, scene) == 2
    @test get_first_available_id(scene) == 3

    @test in(1, scene)
    @test in(2, scene)
    @test !in(3, scene)

    veh = scene[2]
    @test veh.state == trajdata[1][2].state
    @test veh.def == trajdata[1][2].def

    push!(scene, trajdata[1][1].state)
end
