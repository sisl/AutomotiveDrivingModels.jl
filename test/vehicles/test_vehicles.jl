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