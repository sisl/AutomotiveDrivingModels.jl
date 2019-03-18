@testset "Actions" begin 
    roadway = get_test_roadway()
    trajdata = get_test_trajdata(roadway)
    veh = get(trajdata, 1, 1)

    let
        s = VehicleState()
        @test VehicleState() == propagate(veh, s, roadway, NaN)
    end

    let
        a = AccelTurnrate(0.1,0.2)
        @test a == convert(AccelTurnrate, [0.1,0.2])
        @test copyto!([NaN, NaN], AccelTurnrate(0.1,0.2)) == [0.1,0.2]

        s = propagate(veh, AccelTurnrate(0.0,0.0), roadway, 1.0)
        @test isapprox(s.posG.x, veh.state.v)
        @test isapprox(s.posG.y, 0.0)
        @test isapprox(s.posG.θ, 0.0)
    end

    let
        a = AccelDesang(0.1,0.2)
        @test a == convert(AccelDesang, [0.1,0.2])
        @test copyto!([NaN, NaN], AccelDesang(0.1,0.2)) == [0.1,0.2]

        s = propagate(veh, AccelDesang(0.0,0.0), roadway, 1.0)
        @test isapprox(s.posG.x, veh.state.v*1.0)
        @test isapprox(s.posG.y, 0.0)
        @test isapprox(s.posG.θ, 0.0)
        @test isapprox(s.v, veh.state.v)
    end

    let
        a = LatLonAccel(0.1,0.2)
        @test a == convert(LatLonAccel, [0.1,0.2])
        @test copyto!([NaN, NaN], LatLonAccel(0.1,0.2)) == [0.1,0.2]

        Δt = 0.1
        s = propagate(veh, LatLonAccel(0.0,0.0), roadway, Δt)
        @test isapprox(s.posG.x, veh.state.v*Δt)
        @test isapprox(s.posG.y, 0.0)
        @test isapprox(s.posG.θ, 0.0)
        @test isapprox(s.v, veh.state.v)
    end

    let 
        r = gen_straight_roadway(4, 100.0)
        a = LaneFollowingAccel(2.0)
        v = Vehicle(VehicleState(VecSE2(0.0, 0.0, 0.0), r, 2.0), VehicleDef(), 1)
        lanetag = v.state.posF.roadind.tag
        Δt = 0.1
        vp = propagate(v, a, r, Δt)
        @test vp.posF.roadind.tag == lanetag
    end

end