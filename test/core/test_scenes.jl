let
    trajdata = get_test_trajdata()

    scene = get!(Scene(), trajdata, 1)
    @test length(scene) == 2
    for (i,veh) in enumerate(scene)
        @test scene[i].state == get_vehiclestate(trajdata, i, 1)
        @test scene[i].def == get_vehicledef(trajdata, i)
    end

    scene2 = Scene(deepcopy(scene.vehicles), 2)
    @test length(scene2) == 2
    for (i,veh) in enumerate(scene2)
        @test scene2[i].state == get_vehiclestate(trajdata, i, 1)
        @test scene2[i].def == get_vehicledef(trajdata, i)
    end

    @test get_by_id(scene, 1) == scene[1]

    empty!(scene2)
    @test length(scene2) == 0

    copy!(scene2, scene)
    @test length(scene2) == 2
    for (i,veh) in enumerate(scene2)
        @test scene2[i].state == get_vehiclestate(trajdata, i, 1)
        @test scene2[i].def == get_vehicledef(trajdata, i)
    end

    delete!(scene2, scene2[1])
    @test length(scene2) == 1
    @test scene2[1].state == get_vehiclestate(trajdata, 2, 1)
    @test scene2[1].def == get_vehicledef(trajdata, 2)
    scene2[1] = deepcopy(scene[1])
    @test scene2[1].state == get_vehiclestate(trajdata, 1, 1)
    @test scene2[1].def == get_vehicledef(trajdata, 1)

    @test get_index_of_first_vehicle_with_id(scene, 1) == 1
    @test get_index_of_first_vehicle_with_id(scene, 2) == 2
    @test get_first_available_id(scene) == 3

    @test iscarinframe(scene, 1)
    @test iscarinframe(scene, 2)
    @test !iscarinframe(scene, 3)

    veh = get_vehicle(scene, 2)
    @test veh.state == get_vehiclestate(trajdata, 2, 1)
    @test veh.def == get_vehicledef(trajdata, 2)

    @test get_neighbor_fore_along_lane(get!(Scene(), trajdata, 1), 1, trajdata.roadway) == NeighborLongitudinalResult(2, 3.0078125)
    @test get_neighbor_fore_along_lane(get!(Scene(), trajdata, 1), 2, trajdata.roadway) == NeighborLongitudinalResult(0, 250.0)
    @test get_neighbor_fore_along_lane(get!(Scene(), trajdata, 2), 1, trajdata.roadway) == NeighborLongitudinalResult(2, 4.0)
    @test get_neighbor_fore_along_lane(get!(Scene(), trajdata, 2), 2, trajdata.roadway) == NeighborLongitudinalResult(0, 250.0)

    push!(scene, get_vehiclestate(trajdata, 1, 1))
end

let
    roadway = gen_stadium_roadway(1)
    scene = Scene(2)

    function place_at(s)
        roadproj = proj(VecSE2(0.0,0.0,0.0), roadway)
        roadind = RoadIndex(roadproj.curveproj.ind, roadproj.tag)
        roadind = move_along(roadind, roadway, s)
        frenet = Frenet(roadind, roadway[roadind].s, 0.0, 0.0)
        VehicleState(frenet, roadway, 0.0)
    end

    push!(scene, Vehicle(place_at(0.0), VehicleDef(1, AgentClass.CAR, 2.0, 1.0)))
    push!(scene, Vehicle(place_at(0.0), VehicleDef(2, AgentClass.CAR, 2.0, 1.0)))
    foreinfo = get_neighbor_fore_along_lane(scene, 1, roadway, max_distance_fore=Inf)
    @test isapprox(foreinfo.Δs, 0.0)

    scene[2].state = place_at(100.0)
    foreinfo = get_neighbor_fore_along_lane(scene, 1, roadway, max_distance_fore=Inf)
    @test foreinfo.ind == 2
    @test isapprox(foreinfo.Δs, 100.0)

    foreinfo = get_neighbor_fore_along_lane(scene, 2, roadway, max_distance_fore=Inf)
    @test foreinfo.ind == 1
    @test isapprox(foreinfo.Δs, 277.07, atol=1e-2)

    scene[2].state = place_at(145.0)
    foreinfo = get_neighbor_fore_along_lane(scene, 1, roadway, max_distance_fore=Inf)
    @test foreinfo.ind == 2
    @test isapprox(foreinfo.Δs, 145.0, atol=1e-5)

    scene[2].state = place_at(240.0)
    foreinfo = get_neighbor_fore_along_lane(scene, 1, roadway, max_distance_fore=Inf)
    @test foreinfo.ind == 2
    @test isapprox(foreinfo.Δs, 240.0, atol=1e-5)

    scene[1].state = place_at(240.0)
    foreinfo = get_neighbor_fore_along_lane(scene, 1, roadway, max_distance_fore=Inf)
    @test foreinfo.ind == 2
    @test isapprox(foreinfo.Δs, 0.0, atol=1e-5)

    ################################################
    # get_frenet_relative_position

    scene[1].state = place_at(0.0)
    scene[2].state = place_at(10.0)
    frp = get_frenet_relative_position(scene[2].state.posG, scene[1].state.posF.roadind, roadway, max_distance_fore=Inf)
    @test frp.origin == scene[1].state.posF.roadind
    @test frp.target.ind.i == 0
    @test isapprox(frp.target.ind.t, 0.1, atol=0.01)
    @test frp.target.tag == scene[1].state.posF.roadind.tag
    @test isapprox(frp.Δs, 10.0, atol=1e-2)
    @test isapprox(frp.t, 0.0, atol=1e-5)
    @test isapprox(frp.ϕ, 0.0, atol=1e-7)

    scene[1].state = place_at(10.0)
    scene[2].state = place_at(20.0)
    frp = get_frenet_relative_position(scene[2].state.posG, scene[1].state.posF.roadind, roadway, max_distance_fore=Inf)
    @test frp.origin == scene[1].state.posF.roadind
    @test frp.target.ind.i == 0
    @test isapprox(frp.target.ind.t, 0.2, atol=0.01)
    @test frp.target.tag == scene[1].state.posF.roadind.tag
    @test isapprox(frp.Δs, 10.0, atol=1e-2)
    @test isapprox(frp.t, 0.0, atol=1e-5)
    @test isapprox(frp.ϕ, 0.0, atol=1e-7)

    scene[1].state = place_at(0.0)
    scene[2].state = place_at(120.0)
    frp = get_frenet_relative_position(scene[2].state.posG, scene[1].state.posF.roadind, roadway, max_distance_fore=Inf)
    @test frp.target.tag == scene[1].state.posF.roadind.tag
    @test isapprox(frp.Δs, 120.0, atol=1e-2)
    @test isapprox(frp.t, 0.0, atol=1e-5)
    @test isapprox(frp.ϕ, 0.0, atol=1e-7)

    scene[1].state = place_at(0.0)
    scene[2].state = place_at(250.0)
    frp = get_frenet_relative_position(scene[2].state.posG, scene[1].state.posF.roadind, roadway, max_distance_fore=Inf)
    @test frp.target.tag != scene[1].state.posF.roadind.tag
    @test isapprox(frp.Δs, 250.0, atol=1e-2)
    @test isapprox(frp.t, 0.0, atol=1e-5)
    @test isapprox(frp.ϕ, 0.0, atol=1e-7)
end
