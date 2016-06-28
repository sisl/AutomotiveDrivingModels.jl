let

    s = VehicleState(VecSE2(0.0,0.0,0.0), Frenet(NULL_ROADINDEX, 0.0, 0.0, 0.0), 10.0)
    @test isapprox(get_vel_s(s), 10.0)
    @test isapprox(get_vel_t(s),  0.0)

    s = VehicleState(VecSE2(0.0,0.0,0.0), Frenet(NULL_ROADINDEX, 0.0, 0.0, 0.1), 10.0)
    @test isapprox(get_vel_s(s), 10.0*cos(0.1))
    @test isapprox(get_vel_t(s), 10.0*sin(0.1))

    vehdef = VehicleDef(1, AgentClass.CAR, 5.0, 3.0)
    veh = Vehicle(s, vehdef)
    @test isapprox(get_footpoint(veh), VecSE2(0.0,0.0,0.0))

    ri = RoadIndex(CurveIndex(1,0.1), LaneTag(1,2))
    @test isapprox(Frenet(ri, 0.0, 0.0, 0.1), Frenet(ri, 0.0, 0.0, 0.1))
    @test VehicleState(VecSE2(0.1,0.2,0.3), 1.0) == VehicleState(VecSE2(0.1,0.2,0.3), NULL_FRENET, 1.0)
end