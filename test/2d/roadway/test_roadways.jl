function get_test_roadway()
    roadway = Roadway()

    seg1 = RoadSegment(1,
        [Lane(LaneTag(1,1),
            [CurvePt(VecSE2(-2.0,0.0,0.0), 0.0),
             CurvePt(VecSE2(-1.0,0.0,0.0), 1.0)]),
         Lane(LaneTag(1,2),
            [CurvePt(VecSE2(-2.0,1.0,0.0), 0.0),
             CurvePt(VecSE2(-1.0,1.0,0.0), 1.0)]),
         Lane(LaneTag(1,3),
            [CurvePt(VecSE2(-2.0,2.0,0.0), 0.0),
             CurvePt(VecSE2(-1.0,2.0,0.0), 1.0)])
        ])

    seg2 = RoadSegment(2,
        [Lane(LaneTag(2,1),
            [CurvePt(VecSE2(0.0,0.0,0.0), 0.0),
             CurvePt(VecSE2(1.0,0.0,0.0), 1.0),
             CurvePt(VecSE2(2.0,0.0,0.0), 2.0),
             CurvePt(VecSE2(3.0,0.0,0.0), 3.0)]),
         Lane(LaneTag(2,2),
            [CurvePt(VecSE2(0.0,1.0,0.0), 0.0),
             CurvePt(VecSE2(1.0,1.0,0.0), 1.0),
             CurvePt(VecSE2(2.0,1.0,0.0), 2.0),
             CurvePt(VecSE2(3.0,1.0,0.0), 3.0)]),
         Lane(LaneTag(2,3),
            [CurvePt(VecSE2(0.0,2.0,0.0), 0.0),
             CurvePt(VecSE2(1.0,2.0,0.0), 1.0),
             CurvePt(VecSE2(2.0,2.0,0.0), 2.0),
             CurvePt(VecSE2(3.0,2.0,0.0), 3.0)])
         ])

    seg3 = RoadSegment(3,
        [Lane(LaneTag(3,1),
            [CurvePt(VecSE2(4.0,0.0,0.0), 0.0),
             CurvePt(VecSE2(5.0,0.0,0.0), 1.0)]),
         Lane(LaneTag(3,2),
            [CurvePt(VecSE2(4.0,1.0,0.0), 0.0),
             CurvePt(VecSE2(5.0,1.0,0.0), 1.0)]),
         Lane(LaneTag(3,3),
            [CurvePt(VecSE2(4.0,2.0,0.0), 0.0),
             CurvePt(VecSE2(5.0,2.0,0.0), 1.0)])
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
    show(IOBuffer(), lanetag)

    @test !has_next(lane)
    @test !has_prev(lane)

    show(IOBuffer(), RoadIndex(CurveIndex(1,0.0), lanetag))

    roadway = get_test_roadway()

    lane = roadway[LaneTag(1,1)]
    @test has_next(lane)
    @test !has_prev(lane)
    @test lane.exits[1].target == RoadIndex(CurveIndex(1,0.0), LaneTag(2,1))

    lane = roadway[LaneTag(2,1)]
    @test has_next(lane)
    @test has_prev(lane)
    @test lane.exits[1].target == RoadIndex(CurveIndex(1,0.0), LaneTag(3,1))
    @test lane.entrances[1].target == RoadIndex(CurveIndex(1,1.0), LaneTag(1,1))

    res = proj(VecSE2(1.0,0.0,0.0), lane, roadway)
    @test res.curveproj.ind == CurveIndex(2, 0.0)
    @test isapprox(res.curveproj.t, 0.0)
    @test isapprox(res.curveproj.ϕ, 0.0)
    @test res.tag == lane.tag

    res = proj(VecSE2(1.0,0.0,0.0), lane, roadway, move_along_curves=false)
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
    @test res.tag == prev_lane(lane, roadway).tag

    res = proj(VecSE2(-1.75,0.0,0.0), lane, roadway, move_along_curves=false)
    @test res.curveproj.ind == CurveIndex(0, 0.0)
    @test isapprox(res.curveproj.t, 0.0)
    @test isapprox(res.curveproj.ϕ, 0.0)
    @test res.tag == lane.tag

    res = proj(VecSE2(4.25,0.2,0.1), lane, roadway)
    @test res.curveproj.ind == CurveIndex(1, 0.25)
    @test isapprox(res.curveproj.t, 0.2)
    @test isapprox(res.curveproj.ϕ, 0.1)
    @test res.tag == next_lane(lane, roadway).tag

    res = proj(VecSE2(4.25,0.2,0.1), lane, roadway, move_along_curves=false)
    @test res.curveproj.ind == CurveIndex(4, 1.0)
    @test isapprox(res.curveproj.t, 0.2)
    @test isapprox(res.curveproj.ϕ, 0.1)
    @test res.tag == lane.tag

    res = proj(VecSE2(3.25,0.2,0.1), lane, roadway)
    @test res.curveproj.ind == CurveIndex(4, 0.25)
    @test isapprox(res.curveproj.t, 0.2)
    @test isapprox(res.curveproj.ϕ, 0.1)
    @test res.tag == lane.tag

    res = proj(VecSE2(4.25,0.2,0.1), prev_lane(lane, roadway), roadway)
    @test res.curveproj.ind == CurveIndex(1, 0.25)
    @test isapprox(res.curveproj.t, 0.2)
    @test isapprox(res.curveproj.ϕ, 0.1)
    @test res.tag == next_lane(lane, roadway).tag

    res = proj(VecSE2(-0.75,0.0,0.0), next_lane(lane, roadway), roadway)
    @test res.curveproj.ind == CurveIndex(0, 0.25)
    @test isapprox(res.curveproj.t, 0.0)
    @test isapprox(res.curveproj.ϕ, 0.0)
    @test res.tag == lane.tag

    ####

    seg = roadway[2]

    res = proj(VecSE2(1.0,0.0,0.0), seg, roadway)
    @test res.curveproj.ind == CurveIndex(2, 0.0)
    @test isapprox(res.curveproj.t, 0.0)
    @test isapprox(res.curveproj.ϕ, 0.0)
    @test res.tag == lane.tag

    res = proj(VecSE2(1.5,0.25,0.1), seg, roadway)
    @test res.curveproj.ind == CurveIndex(2, 0.5)
    @test isapprox(res.curveproj.t, 0.25)
    @test isapprox(res.curveproj.ϕ, 0.1)
    @test res.tag == lane.tag

    res = proj(VecSE2(0.0,0.0,0.0), seg, roadway)
    @test res.curveproj.ind == CurveIndex(1, 0.0)
    @test isapprox(res.curveproj.t, 0.0)
    @test isapprox(res.curveproj.ϕ, 0.0)
    @test res.tag == lane.tag

    res = proj(VecSE2(-0.75,0.0,0.0), seg, roadway)
    @test res.curveproj.ind == CurveIndex(0, 0.25)
    @test isapprox(res.curveproj.t, 0.0)
    @test isapprox(res.curveproj.ϕ, 0.0)
    @test res.tag == lane.tag

    res = proj(VecSE2(-1.75,0.0,0.0), seg, roadway)
    @test res.curveproj.ind == CurveIndex(1, 0.25)
    @test isapprox(res.curveproj.t, 0.0)
    @test isapprox(res.curveproj.ϕ, 0.0)
    @test res.tag == prev_lane(lane, roadway).tag

    res = proj(VecSE2(4.25,0.2,0.1), seg, roadway)
    @test res.curveproj.ind == CurveIndex(1, 0.25)
    @test isapprox(res.curveproj.t, 0.2)
    @test isapprox(res.curveproj.ϕ, 0.1)
    @test res.tag == next_lane(lane, roadway).tag

    res = proj(VecSE2(3.25,0.2,0.1), seg, roadway)
    @test res.curveproj.ind == CurveIndex(4, 0.25)
    @test isapprox(res.curveproj.t, 0.2)
    @test isapprox(res.curveproj.ϕ, 0.1)
    @test res.tag == lane.tag

    res = proj(VecSE2(4.25,0.2,0.1), roadway[1], roadway)
    @test res.curveproj.ind == CurveIndex(1, 0.25)
    @test isapprox(res.curveproj.t, 0.2)
    @test isapprox(res.curveproj.ϕ, 0.1)
    @test res.tag == next_lane(lane, roadway).tag

    res = proj(VecSE2(-0.75,0.0,0.0), roadway[3], roadway)
    @test res.curveproj.ind == CurveIndex(0, 0.25)
    @test isapprox(res.curveproj.t, 0.0)
    @test isapprox(res.curveproj.ϕ, 0.0)
    @test res.tag == lane.tag

    res = proj(VecSE2(1.0,1.0,0.0), seg, roadway)
    @test res.curveproj.ind == CurveIndex(2, 0.0)
    @test isapprox(res.curveproj.t, 0.0)
    @test isapprox(res.curveproj.ϕ, 0.0)
    @test res.tag == LaneTag(2,2)

    res = proj(VecSE2(1.0,2.0,0.0), seg, roadway)
    @test res.curveproj.ind == CurveIndex(2, 0.0)
    @test isapprox(res.curveproj.t, 0.0)
    @test isapprox(res.curveproj.ϕ, 0.0)
    @test res.tag == LaneTag(2,3)

    ###

    res = proj(VecSE2(1.0,0.0,0.0), roadway)
    @test res.curveproj.ind == CurveIndex(2, 0.0)
    @test isapprox(res.curveproj.t, 0.0)
    @test isapprox(res.curveproj.ϕ, 0.0)
    @test res.tag == lane.tag

    res = proj(VecSE2(1.5,0.25,0.1), roadway)
    @test res.curveproj.ind == CurveIndex(2, 0.5)
    @test isapprox(res.curveproj.t, 0.25)
    @test isapprox(res.curveproj.ϕ, 0.1)
    @test res.tag == lane.tag

    res = proj(VecSE2(0.0,0.0,0.0), roadway)
    @test res.curveproj.ind == CurveIndex(1, 0.0)
    @test isapprox(res.curveproj.t, 0.0)
    @test isapprox(res.curveproj.ϕ, 0.0)
    @test res.tag == lane.tag

    res = proj(VecSE2(-0.75,0.0,0.0), roadway)
    @test res.curveproj.ind == CurveIndex(2, 0.25)
    @test isapprox(res.curveproj.t, 0.0)
    @test isapprox(res.curveproj.ϕ, 0.0)
    @test res.tag == prev_lane(lane, roadway).tag

    res = proj(VecSE2(-1.75,0.0,0.0), roadway)
    @test res.curveproj.ind == CurveIndex(1, 0.25)
    @test isapprox(res.curveproj.t, 0.0)
    @test isapprox(res.curveproj.ϕ, 0.0)
    @test res.tag == prev_lane(lane, roadway).tag

    res = proj(VecSE2(4.25,0.2,0.1), roadway)
    @test res.curveproj.ind == CurveIndex(1, 0.25)
    @test isapprox(res.curveproj.t, 0.2)
    @test isapprox(res.curveproj.ϕ, 0.1)
    @test res.tag == next_lane(lane, roadway).tag

    res = proj(VecSE2(3.25,0.2,0.1), roadway)
    @test res.curveproj.ind == CurveIndex(4, 0.25)
    @test isapprox(res.curveproj.t, 0.2)
    @test isapprox(res.curveproj.ϕ, 0.1)
    @test res.tag == lane.tag

    res = proj(VecSE2(4.25,0.2,0.1), roadway[1], roadway)
    @test res.curveproj.ind == CurveIndex(1, 0.25)
    @test isapprox(res.curveproj.t, 0.2)
    @test isapprox(res.curveproj.ϕ, 0.1)
    @test res.tag == next_lane(lane, roadway).tag

    res = proj(VecSE2(-0.75,0.0,0.0), roadway[3], roadway)
    @test res.curveproj.ind == CurveIndex(0, 0.25)
    @test isapprox(res.curveproj.t, 0.0)
    @test isapprox(res.curveproj.ϕ, 0.0)
    @test res.tag == lane.tag

    res = proj(VecSE2(1.0,1.0,0.0), roadway)
    @test res.curveproj.ind == CurveIndex(2, 0.0)
    @test isapprox(res.curveproj.t, 0.0)
    @test isapprox(res.curveproj.ϕ, 0.0)
    @test res.tag == LaneTag(2,2)

    res = proj(VecSE2(1.0,2.0,0.0), roadway)
    @test res.curveproj.ind == CurveIndex(2, 0.0)
    @test isapprox(res.curveproj.t, 0.0)
    @test isapprox(res.curveproj.ϕ, 0.0)
    @test res.tag == LaneTag(2,3)

    ###

    roadind_0 = RoadIndex(CurveIndex(1,0), LaneTag(2,1))
    roadind = move_along(roadind_0, roadway, 0.0)
    @test roadind == roadind_0

    roadind = move_along(roadind_0, roadway, 1.0)
    @test roadind == RoadIndex(CurveIndex(2,0), LaneTag(2,1))

    roadind = move_along(roadind_0, roadway, 1.25)
    @test roadind == RoadIndex(CurveIndex(2,0.25), LaneTag(2,1))

    roadind = move_along(roadind_0, roadway, 3.0)
    @test roadind == RoadIndex(CurveIndex(3,1.0), LaneTag(2,1))

    roadind = move_along(roadind_0, roadway, 4.0)
    @test roadind == RoadIndex(CurveIndex(1,0.0), LaneTag(3,1))

    roadind = move_along(roadind_0, roadway, 4.5)
    @test roadind == RoadIndex(CurveIndex(1,0.5), LaneTag(3,1))

    roadind = move_along(roadind_0, roadway, 3.75)
    @test roadind == RoadIndex(CurveIndex(0,0.75), LaneTag(3,1))

    roadind = move_along(roadind_0, roadway, -1.0)
    @test roadind == RoadIndex(CurveIndex(0,0.0), LaneTag(2,1))

    roadind = move_along(roadind_0, roadway, -1.75)
    @test roadind == RoadIndex(CurveIndex(1,0.25), LaneTag(1,1))

    roadind = move_along(roadind_0, roadway, -0.75)
    @test roadind == RoadIndex(CurveIndex(0,0.25), LaneTag(2,1))

    roadind = move_along(roadind_0, roadway, -Inf)
    @test roadind == RoadIndex(CurveIndex(1,0.0), LaneTag(1,1))

    roadind = move_along(roadind_0, roadway, Inf)
    @test roadind == RoadIndex(CurveIndex(1,1.0), LaneTag(3,1))

    ####

    @test n_lanes_right(roadway[LaneTag(2,1)], roadway) == 0
    @test n_lanes_right(roadway[LaneTag(2,2)], roadway) == 1
    @test n_lanes_right(roadway[LaneTag(2,3)], roadway) == 2
    @test n_lanes_left(roadway[LaneTag(2,1)], roadway) == 2
    @test n_lanes_left(roadway[LaneTag(2,2)], roadway) == 1
    @test n_lanes_left(roadway[LaneTag(2,3)], roadway) == 0

end

let
    roadway = Roadway()

    seg1 = RoadSegment(1,
        [Lane(LaneTag(1,1),
            [CurvePt(VecSE2(0.0,0.0,0.0), 0.0),
             CurvePt(VecSE2(1.0,0.0,0.0), 1.0)],
             width=1.0,
             boundary_left = LaneBoundary(:solid, :white),
             boundary_right = LaneBoundary(:broken, :yellow)),
         Lane(LaneTag(1,2),
            [CurvePt(VecSE2(0.0,1.0,0.0), 0.0),
             CurvePt(VecSE2(1.0,1.0,0.0), 1.0)],
             width=2.0,
             boundary_left = LaneBoundary(:double, :white))],
         )

    seg2 = RoadSegment(2,
        [Lane(LaneTag(2,1),
            [CurvePt(VecSE2(4.0,0.0,0.0), 0.0),
             CurvePt(VecSE2(5.0,0.0,0.0), 1.0)]),
         Lane(LaneTag(2,2),
            [CurvePt(VecSE2(4.0,1.0,0.0), 0.0),
             CurvePt(VecSE2(5.0,1.0,0.0), 1.0)])],
         )

    connect!(seg1.lanes[1], seg2.lanes[2])

    push!(roadway.segments, seg1)
    push!(roadway.segments, seg2)

    ############

    path, io = mktemp()
    write(io, MIME"text/plain"(), roadway)
    close(io)

    lines = open(readlines, path)

    for (line_orig, line_test) in zip(lines,
            [
             "ROADWAY",
             "2",
             "1",
             "   2",
             "   1",
             "       1.000",
             "       -Inf Inf",
             "       solid white",
             "       broken yellow",
             "       1",
             "           D (1 1.000000) 1 0.000000 2 2",
             "       2",
             "           (0.0000 0.0000 0.000000) 0.0000 NaN NaN",
             "           (1.0000 0.0000 0.000000) 1.0000 NaN NaN",
             "   2",
             "       2.000",
             "       -Inf Inf",
             "       double white",
             "       unknown unknown",
             "       0",
             "       2",
             "           (0.0000 1.0000 0.000000) 0.0000 NaN NaN",
             "           (1.0000 1.0000 0.000000) 1.0000 NaN NaN",
             "2",
             "   2",
             "   1",
             "       3.000",
             "       -Inf Inf",
             "       unknown unknown",
             "       unknown unknown",
             "       0",
             "       2",
             "           (4.0000 0.0000 0.000000) 0.0000 NaN NaN",
             "           (5.0000 0.0000 0.000000) 1.0000 NaN NaN",
             "   2",
             "       3.000",
             "       -Inf Inf",
             "       unknown unknown",
             "       unknown unknown",
             "       1",
             "           U (1 0.000000) 1 1.000000 1 1",
             "       2",
             "           (4.0000 1.0000 0.000000) 0.0000 NaN NaN",
             "           (5.0000 1.0000 0.000000) 1.0000 NaN NaN",
             ]
        )
        @test strip(line_orig) == strip(line_test)
    end

    io = open(path)
    roadway2 = read(io, MIME"text/plain"(), Roadway)
    close(io)
    rm(path)

    @test length(roadway.segments) == length(roadway2.segments)
    for (seg1, seg2) in zip(roadway.segments, roadway2.segments)
        @test seg1.id == seg2.id
        @test length(seg1.lanes) == length(seg2.lanes)
        for (lane1, lane2) in zip(seg1.lanes, seg2.lanes)
            @test lane1.tag == lane2.tag
            @test isapprox(lane1.width, lane2.width, atol=1e-3)
            @test isapprox(lane1.speed_limit.lo, lane2.speed_limit.lo, atol=1e-3)
            @test isapprox(lane1.speed_limit.hi, lane2.speed_limit.hi, atol=1e-3)
            @test lane1.boundary_left == lane2.boundary_left
            @test lane1.boundary_right == lane2.boundary_right
            for (conn1, conn2) in zip(lane1.exits, lane2.exits)
                @test conn1.downstream == conn2.downstream
                @test conn1.mylane.i == conn2.mylane.i
                @test isapprox(conn1.mylane.t, conn2.mylane.t, atol=1e-3)
                @test conn1.target.tag == conn2.target.tag
                @test conn1.target.ind.i == conn2.target.ind.i
                @test isapprox(conn1.target.ind.t, conn2.target.ind.t, atol=1e-3)
            end
            for (conn1, conn2) in zip(lane1.entrances, lane2.entrances)
                @test conn1.downstream == conn2.downstream
                @test conn1.mylane.i == conn2.mylane.i
                @test isapprox(conn1.mylane.t, conn2.mylane.t, atol=1e-3)
                @test conn1.target.tag == conn2.target.tag
                @test conn1.target.ind.i == conn2.target.ind.i
                @test isapprox(conn1.target.ind.t, conn2.target.ind.t, atol=1e-3)
            end
            for (pt1, pt2) in zip(lane1.curve, lane2.curve)
                @test abs2(convert(VecE2, pt1.pos) - convert(VecE2, pt2.pos)) < 0.01
                @test angledist(pt1.pos.θ, pt2.pos.θ) < 1e-5
                @test isnan(pt1.s) || isapprox(pt1.s, pt2.s, atol=1e-3)
                @test isnan(pt1.k) || isapprox(pt1.k, pt2.k, atol=1e-6)
                @test isnan(pt1.kd) || isapprox(pt1.kd, pt2.kd, atol=1e-6)
            end
        end
    end
end

let
    lc = LaneConnection(true, CurveIndex(1,0.0), RoadIndex(CurveIndex(2,0.0), LaneTag(2,2)))
    show(IOBuffer(), lc)
end