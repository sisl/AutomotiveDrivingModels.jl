let

    roadway = gen_straight_roadway(3, 1000.0, lane_width=1.0)
    rec = SceneRecord(1, 0.1, 5)
    update!(rec, Scene([
            Vehicle(VehicleState(VecSE2( 0.0,0.0,0.0), roadway, 10.0), VehicleDef(AgentClass.CAR, 5.0, 2.0), 1),
            Vehicle(VehicleState(VecSE2(10.0,0.0,0.0), roadway, 10.0), VehicleDef(AgentClass.CAR, 5.0, 2.0), 2),
        ]))

    @test convert(Float64, get(IS_COLLIDING, rec, roadway, 1)) == 0.0
    @test convert(Float64, get(IS_COLLIDING, rec, roadway, 2)) == 0.0

    @test isapprox(convert(Float64, get(MARKERDIST_LEFT, rec, roadway, 1)), 0.5)
    @test isapprox(convert(Float64, get(MARKERDIST_LEFT, rec, roadway, 2)), 0.5)
    @test isapprox(convert(Float64, get(MARKERDIST_RIGHT, rec, roadway, 1)), 0.5)
    @test isapprox(convert(Float64, get(MARKERDIST_RIGHT, rec, roadway, 2)), 0.5)
    @test isapprox(convert(Float64, get(MARKERDIST_LEFT_LEFT, rec, roadway, 1)), 1.5)
    @test isapprox(convert(Float64, get(MARKERDIST_LEFT_LEFT, rec, roadway, 2)), 1.5)
    @test isnan(convert(Float64, get(MARKERDIST_RIGHT_RIGHT, rec, roadway, 1)))
    @test isnan(convert(Float64, get(MARKERDIST_RIGHT_RIGHT, rec, roadway, 2)))

    @test isapprox(convert(Float64, get(DIST_FRONT, rec, roadway, 1)), 10.0 - 5.0)
    @test convert(Float64, get(DIST_FRONT, rec, roadway, 2, censor_hi=100.0)) == 100.0

    update!(rec, Scene([
            Vehicle(VehicleState(VecSE2( 1.0,0.0,0.0), roadway, 10.0), VehicleDef(AgentClass.CAR, 5.0, 2.0), 1),
            Vehicle(VehicleState(VecSE2(10.0,0.0,0.0), roadway, 10.0), VehicleDef(AgentClass.CAR, 5.0, 2.0), 2),
            Vehicle(VehicleState(VecSE2(12.0,1.0,0.0), roadway, 10.0), VehicleDef(AgentClass.CAR, 5.0, 2.0), 3),
            Vehicle(VehicleState(VecSE2( 0.0,1.0,0.0), roadway, 10.0), VehicleDef(AgentClass.CAR, 5.0, 2.0), 4),
        ]))

    @test isapprox(convert(Float64, get(MARKERDIST_LEFT, rec, roadway, 3)), 0.5)
    @test isapprox(convert(Float64, get(MARKERDIST_RIGHT, rec, roadway, 3)), 0.5)
    @test isapprox(convert(Float64, get(MARKERDIST_LEFT_LEFT, rec, roadway, 3)), 1.5)
    @test isapprox(convert(Float64, get(MARKERDIST_RIGHT_RIGHT, rec, roadway, 3)), 1.5)

    @test isapprox(convert(Float64, get(DIST_FRONT, rec, roadway, 1)), 9.0 - 5.0)
    @test isapprox(convert(Float64, get(DIST_FRONT_LEFT, rec, roadway, 1)), 11.0 - 5.0)
    @test convert(Float64, get(DIST_FRONT_RIGHT, rec, roadway, 1)) == 100.0
    @test isapprox(convert(Float64, get(DIST_FRONT, rec, roadway, 4)), 12.0 - 5.0)
    @test convert(Float64, get(DIST_FRONT_LEFT, rec, roadway, 4)) == 100.0
    @test isapprox(convert(Float64, get(DIST_FRONT_RIGHT, rec, roadway, 4)), 1.0 - 5.0)

    @test convert(Float64, get(IS_COLLIDING, rec, roadway, 1)) == 1.0
    @test convert(Float64, get(IS_COLLIDING, rec, roadway, 2)) == 1.0
    @test convert(Float64, get(IS_COLLIDING, rec, roadway, 3)) == 1.0
    @test convert(Float64, get(IS_COLLIDING, rec, roadway, 4)) == 1.0

    update!(rec, Scene([
            Vehicle(VehicleState(VecSE2(  0.0,0.0,0.0), roadway, 10.0), VehicleDef(AgentClass.CAR, 5.0, 2.0), 1),
            Vehicle(VehicleState(VecSE2(-10.0,1.0,0.0), roadway, 10.0), VehicleDef(AgentClass.CAR, 5.0, 2.0), 2),
            Vehicle(VehicleState(VecSE2(  0.0,1.0,0.5), roadway, 10.0), VehicleDef(AgentClass.CAR, 5.0, 2.0), 3),
        ]))

    @test convert(Float64, get(IS_COLLIDING, rec, roadway, 1)) == 1.0
    @test convert(Float64, get(IS_COLLIDING, rec, roadway, 2)) == 0.0
    @test convert(Float64, get(IS_COLLIDING, rec, roadway, 3)) == 1.0

    # super-simple test
    for f in allfeatures()
        get(f, rec, roadway, 3)
    end
end