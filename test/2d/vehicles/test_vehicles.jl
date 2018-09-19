let

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
end

let
    roadway = get_test_roadway()
    roadproj = proj(VecSE2(0.0, 0.0, 0.0), roadway)
    roadind = RoadIndex(roadproj)

    @test roadind.ind.i == 1
    @test isapprox(roadind.ind.t, 0.0)
    @test roadind.tag == LaneTag(2,1)
    f = Frenet(roadind, roadway, t=0.1, ϕ=0.2)
    @test f.roadind === roadind
    @test isapprox(f.s, 0.0)
    @test f.t == 0.1
    @test f.ϕ == 0.2
end

let
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
end


let
    # dummy test for the constructor
    roadway=gen_straight_roadway(1, 50.0, lane_width=3.0)
    ped=SidewalkPedestrianModel(timestep=0.1, 
                                crosswalk= roadway[LaneTag(1,1)],
                                sw_origin = roadway[LaneTag(1,1)],
                                sw_dest = roadway[LaneTag(1,1)]
                                )
    @test ped.ttc_threshold >= 1.0
end
