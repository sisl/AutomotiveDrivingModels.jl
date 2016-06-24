let

    s = VehicleState(VecSE2(0.0,0.0,0.0), Frenet(NULL_ROADINDEX, 0.0, 0.0, 0.0), 10.0)
    @test isapprox(get_vel_s(s), 10.0)
    @test isapprox(get_vel_t(s),  0.0)

    s = VehicleState(VecSE2(0.0,0.0,0.0), Frenet(NULL_ROADINDEX, 0.0, 0.0, 0.1), 10.0)
    @test isapprox(get_vel_s(s), 10.0*cos(0.1))
    @test isapprox(get_vel_t(s), 10.0*sin(0.1))

    # veh = Vehicle(1, AgentClass.CAR, 5.0, 3.0, )
end