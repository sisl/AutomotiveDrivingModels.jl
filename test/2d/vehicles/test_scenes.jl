let
    roadway = get_test_roadway()
    trajdata = get_test_trajdata(roadway)

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

    copy!(scene2, scene)
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

    @test findfirst(scene, 1) == 1
    @test findfirst(scene, 2) == 2
    @test get_first_available_id(scene) == 3

    @test in(scene, 1)
    @test in(scene, 2)
    @test !in(scene, 3)

    veh = scene[2]
    @test veh.state == get_state(trajdata, 2, 1)
    @test veh.def == get_def(trajdata, 2)

    @test get_neighbor_fore_along_lane(get!(Scene(), trajdata, 1), 1, roadway) == NeighborLongitudinalResult(2, 3.0078125)
    @test get_neighbor_fore_along_lane(get!(Scene(), trajdata, 1), 2, roadway) == NeighborLongitudinalResult(0, 250.0)
    @test get_neighbor_fore_along_lane(get!(Scene(), trajdata, 2), 1, roadway) == NeighborLongitudinalResult(2, 4.0)
    @test get_neighbor_fore_along_lane(get!(Scene(), trajdata, 2), 2, roadway) == NeighborLongitudinalResult(0, 250.0)

    push!(scene, get_state(trajdata, 1, 1))
end

let
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
