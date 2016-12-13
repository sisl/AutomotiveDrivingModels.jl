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

    origin = VecSE2(1.0,2.0,deg2rad(90))
    roadway = gen_straight_roadway(1, 100.0, lane_width=1.0, origin=origin)
    @test length(roadway.segments) == 1
    lane = roadway[1].lanes[1]
    @test lane.curve[1].pos == origin
    @test isapprox(lane.curve[2].pos.x,         1.0, atol=1e-6)
    @test isapprox(lane.curve[2].pos.y,       102.0, atol=1e-6)
    @test isapprox(lane.curve[2].pos.θ, deg2rad(90), atol=1e-6)

    roadway = gen_straight_roadway(2, 100.0, lane_widths=[1.0, 2.0])
    @test length(roadway.segments) == 1
    @test isapprox(roadway[1].lanes[1].curve[1].pos.y, 0.0, atol=1e-6)
    @test isapprox(roadway[1].lanes[2].curve[1].pos.y, 0.5 + 1.0, atol=1e-6)
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