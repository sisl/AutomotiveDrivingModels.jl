let
    roadway = gen_straight_roadway(1, 1000.0)
    @test length(roadway.segments) == 1

    seg = roadway[1]
    @test length(seg.lanes) == 1

    lane = seg.lanes[1]
    @test isapprox(lane.width, DEFAULT_LANE_WIDTH)
    @test lane.curve[1] == CurvePt(VecSE2(0.0,0.0,0.0), 0.0)
    @test lane.curve[2] == CurvePt(VecSE2(1000.0,0.0,0.0), 1000.0)
end

let
    roadway = gen_straight_roadway(3, 500.0, lane_width=1.0)
    @test length(roadway.segments) == 1

    seg = roadway[1]
    @test length(seg.lanes) == 3

    lane = seg.lanes[1]
    @test isapprox(lane.width, 1.0)
    @test lane.curve[1] == CurvePt(VecSE2(0.0,0.0,0.0), 0.0)
    @test lane.curve[2] == CurvePt(VecSE2(500.0,0.0,0.0), 500.0)

    lane = seg.lanes[2]
    @test isapprox(lane.width, 1.0)
    @test lane.curve[1] == CurvePt(VecSE2(0.0,1.0,0.0), 0.0)
    @test lane.curve[2] == CurvePt(VecSE2(500.0,1.0,0.0), 500.0)

    lane = seg.lanes[3]
    @test isapprox(lane.width, 1.0)
    @test lane.curve[1] == CurvePt(VecSE2(0.0,2.0,0.0), 0.0)
    @test lane.curve[2] == CurvePt(VecSE2(500.0,2.0,0.0), 500.0)
end

let
    roadway = gen_stadium_roadway(1, length=100.0, width=10.0,
                                  radius=25.0, ncurvepts_per_turn=2)
    @test length(roadway.segments) == 4

    seg = roadway[1]
    @test length(seg.lanes) == 1

    lane = seg.lanes[1]
    @test lane.curve[1] == CurvePt(VecSE2(100.0,0.0,0.0), 0.0)
    @test lane.curve[2] == CurvePt(VecSE2(125.0,25.0,π/2), 25*π/2)

    @test roadway[LaneTag(2,1)].curve[1] == CurvePt(VecSE2(125.0,35.0,π/2), 0.0)

    roadway = gen_stadium_roadway(4)
end