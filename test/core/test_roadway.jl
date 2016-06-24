function get_test_roadway()
    roadway = Roadway()

    seg1 = RoadSegment(1,
        [Lane(LaneTag(1,1),
            [CurvePt(VecSE2(-2.0,0.0,0.0), 0.0),
             CurvePt(VecSE2(-1.0,0.0,0.0), 1.0)]),
         Lane(LaneTag(1,2),
            [CurvePt(VecSE2(-2.0,0.0,0.0), 0.0),
             CurvePt(VecSE2(-1.0,0.0,0.0), 1.0)]),
         Lane(LaneTag(1,3),
            [CurvePt(VecSE2(-2.0,0.0,0.0), 0.0),
             CurvePt(VecSE2(-1.0,0.0,0.0), 1.0)])
        ])

    seg2 = RoadSegment(2,
        [Lane(LaneTag(2,1),
            [CurvePt(VecSE2(0.0,0.0,0.0), 0.0),
             CurvePt(VecSE2(1.0,0.0,0.0), 1.0),
             CurvePt(VecSE2(2.0,0.0,0.0), 2.0),
             CurvePt(VecSE2(3.0,0.0,0.0), 3.0)]),
         Lane(LaneTag(2,2),
            [CurvePt(VecSE2(0.0,0.0,0.0), 0.0),
             CurvePt(VecSE2(1.0,0.0,0.0), 1.0),
             CurvePt(VecSE2(2.0,0.0,0.0), 2.0),
             CurvePt(VecSE2(3.0,0.0,0.0), 3.0)]),
         Lane(LaneTag(2,3),
            [CurvePt(VecSE2(0.0,0.0,0.0), 0.0),
             CurvePt(VecSE2(1.0,0.0,0.0), 1.0),
             CurvePt(VecSE2(2.0,0.0,0.0), 2.0),
             CurvePt(VecSE2(3.0,0.0,0.0), 3.0)])
         ])

    seg3 = RoadSegment(3,
        [Lane(LaneTag(3,1),
            [CurvePt(VecSE2(4.0,0.0,0.0), 0.0),
             CurvePt(VecSE2(5.0,0.0,0.0), 1.0)]),
         Lane(LaneTag(3,2),
            [CurvePt(VecSE2(4.0,0.0,0.0), 0.0),
             CurvePt(VecSE2(5.0,0.0,0.0), 1.0)]),
         Lane(LaneTag(3,3),
            [CurvePt(VecSE2(4.0,0.0,0.0), 0.0),
             CurvePt(VecSE2(5.0,0.0,0.0), 1.0)])
        ])

    for i in 1:3
        connect!(seg1.lanes[i], seg2.lanes[i])
        connect!(seg2.lanes[i], seg3.lanes[i])
    end

    push!(roadway.segments, seg1)
    push!(roadway.segments, seg2)
    push!(roadway.segments, seg3)

    roadway
end

let
    curve = get_test_curve1()

    lanetag = LaneTag(1,1)
    lane = Lane(lanetag, curve)

    @test !has_next(lane)
    @test !has_prev(lane)

    roadway = get_test_roadway()

    lane = roadway[LaneTag(1,1)]
    @test has_next(lane)
    @test !has_prev(lane)
    @test lane.next == LaneTag(2,1)

    lane = roadway[LaneTag(2,1)]
    @test has_next(lane)
    @test has_prev(lane)
    @test lane.next == LaneTag(3,1)
    @test lane.prev == LaneTag(1,1)

    res = proj(VecSE2(1.0,0.0,0.0), lane, roadway)
    @test res.curveproj.ind == CurveIndex(2, 0.0)
    @test isapprox(res.curveproj.t, 0.0)
    @test isapprox(res.curveproj.ϕ, 0.0)
    @test res.tag == lane.tag

    res = proj(VecSE2(1.5,0.25,0.1), lane, roadway)
    @test res.curveproj.ind == CurveIndex(2, 0.5)
    @test isapprox(res.curveproj.t, 0.25)
    @test isapprox(res.curveproj.ϕ, 0.1)
    @test res.tag == lane.tag

    res = proj(VecSE2(0.0,0.0,0.0), lane, roadway)
    @test res.curveproj.ind == CurveIndex(1, 0.0)
    @test isapprox(res.curveproj.t, 0.0)
    @test isapprox(res.curveproj.ϕ, 0.0)
    @test res.tag == lane.tag

    res = proj(VecSE2(-0.75,0.0,0.0), lane, roadway)
    @test res.curveproj.ind == CurveIndex(0, 0.25)
    @test isapprox(res.curveproj.t, 0.0)
    @test isapprox(res.curveproj.ϕ, 0.0)
    @test res.tag == lane.tag

    res = proj(VecSE2(-1.75,0.0,0.0), lane, roadway)
    @test res.curveproj.ind == CurveIndex(1, 0.25)
    @test isapprox(res.curveproj.t, 0.0)
    @test isapprox(res.curveproj.ϕ, 0.0)
    @test res.tag == lane.prev

    # @test res.ind == CurveIndex(1, 0.0)
    # @test isapprox(res.t, 0.0)
    # @test isapprox(res.ϕ, 0.0)
    # res = proj(VecSE2(0.25,0.5,0.1), lane)
    # @test res.ind == CurveIndex(1, 0.25)
    # @test isapprox(res.t, 0.5)
    # @test isapprox(res.ϕ, 0.1)
    # res = proj(VecSE2(0.25,-0.5,-0.1), lane)
    # @test res.ind == CurveIndex(1, 0.25)
    # @test isapprox(res.t, -0.5)
    # @test isapprox(res.ϕ, -0.1)
    # res = proj(VecSE2(1.5,0.5,-0.1), lane)
    # @test res.ind == CurveIndex(2, 0.25)
    # @test isapprox(res.t,  0.5)
    # @test isapprox(res.ϕ, -0.1)
end