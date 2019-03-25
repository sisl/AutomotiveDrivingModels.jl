@testset "vehicle features" begin
    vehdef = VehicleDef(AgentClass.CAR, 5.0, 3.0)
    veh = Vehicle(s, vehdef, 1)
    @test isapprox(get_footpoint(veh), VecSE2(0.0,0.0,0.0))
    show(IOBuffer(), vehdef)
    show(IOBuffer(), veh)
end

@testset "neighbor features" begin 
    scene=Frame(Entity{VehicleState, BicycleModel, Int},100)
    roadway=gen_straight_roadway(3, 200.0, lane_width=3.0)
    push!(scene,Entity(VehicleState(VecSE2(30.0,3.0,0.0), roadway, 0.0), 
        BicycleModel(VehicleDef(AgentClass.CAR, 4.826, 1.81)),1))
    push!(scene,Entity(VehicleState(VecSE2(20.0,3.0,0.0), roadway, 0.0), 
        BicycleModel(VehicleDef(AgentClass.CAR, 4.826, 1.81)),1))
    push!(scene,Entity(VehicleState(VecSE2(40.0,3.0,0.0), roadway, 0.0), 
        BicycleModel(VehicleDef(AgentClass.CAR, 4.826, 1.81)),1))
    push!(scene,Entity(VehicleState(VecSE2(20.0,0.0,0.0), roadway, 0.0), 
        BicycleModel(VehicleDef(AgentClass.CAR, 4.826, 1.81)),1))
    push!(scene,Entity(VehicleState(VecSE2(40.0,0.0,0.0), roadway, 0.0), 
        BicycleModel(VehicleDef(AgentClass.CAR, 4.826, 1.81)),1))
    push!(scene,Entity(VehicleState(VecSE2(20.0,6.0,0.0), roadway, 0.0), 
        BicycleModel(VehicleDef(AgentClass.CAR, 4.826, 1.81)),1))
    push!(scene,Entity(VehicleState(VecSE2(40.0,6.0,0.0), roadway, 0.0), 
        BicycleModel(VehicleDef(AgentClass.CAR, 4.826, 1.81)),1))
    @test get_neighbor_fore_along_lane(scene,1,roadway) == NeighborLongitudinalResult(3,10.0)
    @test get_neighbor_rear_along_lane(scene,1,roadway) == NeighborLongitudinalResult(2,10.0)
    @test get_neighbor_fore_along_left_lane(scene,1,roadway) == NeighborLongitudinalResult(7,10.0)
    @test get_neighbor_rear_along_left_lane(scene,1,roadway) == NeighborLongitudinalResult(6,10.0)
    @test get_neighbor_fore_along_right_lane(scene,1,roadway) == NeighborLongitudinalResult(5,10.0)
    @test get_neighbor_rear_along_right_lane(scene,1,roadway) == NeighborLongitudinalResult(4,10.0)

    trajdata = get_test_trajdata(roadway)
    @test get_neighbor_fore_along_lane(get!(Scene(), trajdata, 1), 1, roadway) == NeighborLongitudinalResult(2, 3.0078125)
    @test get_neighbor_fore_along_lane(get!(Scene(), trajdata, 1), 2, roadway) == NeighborLongitudinalResult(nothing, 250.0)
    @test get_neighbor_fore_along_lane(get!(Scene(), trajdata, 2), 1, roadway) == NeighborLongitudinalResult(2, 4.0)
    @test get_neighbor_fore_along_lane(get!(Scene(), trajdata, 2), 2, roadway) == NeighborLongitudinalResult(nothing, 250.0)

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
