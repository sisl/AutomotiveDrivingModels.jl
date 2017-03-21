type FakeDriveAction <: DriveAction end
type FakeActionContext <: ActionContext end

let
    roadway = get_test_roadway()
    trajdata = get_test_trajdata(roadway)
    veh = get(trajdata, 1, 1)

    @test_throws ErrorException length(FakeDriveAction)
    @test_throws ErrorException convert(FakeDriveAction, Float64[])
    @test_throws ErrorException convert(Vector{Float64}, FakeDriveAction())
    @test_throws ErrorException copy!(Float64[], FakeDriveAction())
    @test_throws ErrorException propagate(veh, FakeDriveAction(), FakeActionContext(), roadway)

    let
        a = AccelTurnrate(0.1,0.2)
        @test a == convert(AccelTurnrate, [0.1,0.2])
        @test convert(Vector{Float64}, a) == [0.1,0.2]
        @test copy!([NaN, NaN], AccelTurnrate(0.1,0.2)) == [0.1,0.2]

        s = propagate(veh, AccelTurnrate(0.0,0.0), IntegratedContinuous(1.0,1), roadway)
        @test isapprox(s.posG.x, veh.state.v)
        @test isapprox(s.posG.y, 0.0)
        @test isapprox(s.posG.θ, 0.0)
    end

    let
        a = AccelDesang(0.1,0.2)
        @test a == convert(AccelDesang, [0.1,0.2])
        @test convert(Vector{Float64}, a) == [0.1,0.2]
        @test copy!([NaN, NaN], AccelDesang(0.1,0.2)) == [0.1,0.2]

        s = propagate(veh, AccelDesang(0.0,0.0), IntegratedContinuous(1.0,1), roadway)
        @test isapprox(s.posG.x, veh.state.v)
        @test isapprox(s.posG.y, 0.0)
        @test isapprox(s.posG.θ, 0.0)
    end

    let
        a = NextState(VehicleState())
        @test VehicleState() == propagate(veh, a, ContextFree(), roadway)
    end
end