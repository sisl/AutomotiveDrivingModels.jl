let
    veh = Entity(PosSpeed1D(0.0,1.0), BoundingBoxDef(), 1)

    s2 = propagate(veh, Accel(0.0), Straight1DRoadway(100.0), 1.0)
    @test s2.s ≈ 1.0
    @test s2.v ≈ 1.0
    s2 = propagate(veh, Accel(0.0), Straight1DRoadway(100.0), 0.5)
    @test s2.s ≈ 0.5
    @test s2.v ≈ 1.0
    s2 = propagate(veh, Accel(1.0), Straight1DRoadway(100.0), 1.0)
    @test s2.s ≈ 1.5
    @test s2.v ≈ 2.0
    s2 = propagate(veh, Accel(-1.0), Straight1DRoadway(100.0), 1.0)
    @test s2.s ≈ 0.5
    @test s2.v ≈ 0.0
    s2 = propagate(veh, Accel(-2.0), Straight1DRoadway(100.0), 1.0)
    @test s2.s ≈  0.0
    @test s2.v ≈ -1.0

    s2 = propagate(veh, StoppingAccel(0.0), Straight1DRoadway(100.0), 1.0)
    @test s2.s ≈ 1.0
    @test s2.v ≈ 1.0
    s2 = propagate(veh, StoppingAccel(0.0), Straight1DRoadway(100.0), 0.5)
    @test s2.s ≈ 0.5
    @test s2.v ≈ 1.0
    s2 = propagate(veh, StoppingAccel(1.0), Straight1DRoadway(100.0), 1.0)
    @test s2.s ≈ 1.5
    @test s2.v ≈ 2.0
    s2 = propagate(veh, StoppingAccel(-1.0), Straight1DRoadway(100.0), 1.0)
    @test s2.s ≈ 0.5
    @test s2.v ≈ 0.0
    s2 = propagate(veh, StoppingAccel(-2.0), Straight1DRoadway(100.0), 1.0)
    @test s2.s ≈ 0.25
    @test s2.v ≈ 0.0
end