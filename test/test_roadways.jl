get_test_curve1() = [CurvePt(VecSE2(0.0,0.0,0.0), 0.0),
                     CurvePt(VecSE2(1.0,0.0,0.0), 1.0),
                     CurvePt(VecSE2(3.0,0.0,0.0), 3.0)]


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

@testset "1d roadway" begin 
    roadway = StraightRoadway(20.0)
    s = 10.0 
    @test mod_position_to_roadway(s, roadway) == s
    s = 25.0 
    @test mod_position_to_roadway(s, roadway) == 5.0 
    s = 45.0
    @test mod_position_to_roadway(s, roadway) == 5.0
    s = -5.0 
    @test mod_position_to_roadway(s, roadway) == 15.0
    s_rear = 10.0 
    s_fore = 15.0
    @test get_headway(s_rear, s_fore, roadway) == 5.0
    s_fore = 25.0
    @test get_headway(s_rear, s_fore, roadway) == 15.0
    s_fore = 5.0 
    @test get_headway(s_rear, s_fore, roadway) == 15.0
end

@testset "Curves" begin 
    p = lerp(CurvePt(VecSE2(0.0,0.0,0.0), 0.0), CurvePt(VecSE2(1.0,2.0,3.0), 4.0), 0.25)
    show(IOBuffer(), p)
    @test isapprox(convert(VecE2, p.pos), VecE2(0.25, 0.5))
    @test isapprox(p.pos.θ, 0.75)
    @test isapprox(p.s, 1.0)

    show(IOBuffer(), CurveIndex(1,0.0))

    @test isapprox(get_lerp_time(VecE2(0.0,0.0), VecE2(1.0,0.0), VecE2(1.0,0.0)), 1.0)
    @test isapprox(get_lerp_time(VecE2(0.0,0.0), VecE2(1.0,0.0), VecE2(0.5,0.0)), 0.5)
    @test isapprox(get_lerp_time(VecE2(0.0,0.0), VecE2(1.0,0.0), VecE2(0.5,0.5)), 0.5)
    @test isapprox(get_lerp_time(VecE2(0.0,0.0), VecE2(1.0,1.0), VecE2(0.5,0.5)), 0.5)
    @test isapprox(get_lerp_time(VecE2(1.0,0.0), VecE2(2.0,0.0), VecE2(1.5,0.5)), 0.5)
    @test isapprox(get_lerp_time(VecE2(0.0,0.0), VecE2(-1.0,0.0), VecE2(1.0,0.0)), 0.0)
    @test isapprox(get_lerp_time(VecE2(0.0,0.0), VecE2(-1.0,0.0), VecE2(-0.75,0.0)), 0.75)

    curve = get_test_curve1()
    @inferred index_closest_to_point(curve, VecE2(0.0,0.0)) 
    @test index_closest_to_point(curve, VecE2(0.0,0.0)) == 1
    @test index_closest_to_point(curve, VecE2(1.0,0.0)) == 2
    @test index_closest_to_point(curve, VecE2(2.1,0.0)) == 3
    @test index_closest_to_point(curve, VecE2(0.49,0.0)) == 1
    @test index_closest_to_point(curve, VecE2(1.9,-100.0)) == 2
    @test index_closest_to_point(curve, VecSE2(1.9,-100.0,0.0)) == 2
    @test index_closest_to_point(curve, VecSE2(-1.0,0.0,0.0)) == 1
    @test index_closest_to_point([CurvePt(VecSE2(0.0,0.0,0.0),0.0)], VecE2(0.0,0.0)) == 1
    @test get_curve_index(curve, 0.0) == CurveIndex(1, 0.0)
    @test get_curve_index(curve, 1.0) == CurveIndex(2, 0.0)
    @test get_curve_index(curve, 1.5) == CurveIndex(2, 0.25)
    @test get_curve_index(curve, 3.5) == CurveIndex(2, 1.0)
    @test get_curve_index(curve,-0.5) == CurveIndex(1, 0.0)

    p = curve[CurveIndex(1, 0.75)]
    @test isapprox(convert(VecE2, p.pos), VecE2(0.75, 0.0))
    @test isapprox(p.pos.θ, 0.0)
    @test isapprox(p.s, 0.75)

    @test get_curve_index(CurveIndex(1, 0.0), curve,  0.25) == CurveIndex(1,0.25)
    @test get_curve_index(CurveIndex(1, 0.0), curve,  1.25) == CurveIndex(2,0.125)
    @test get_curve_index(CurveIndex(1, 0.5), curve,  1.50) == CurveIndex(2,0.5)
    @test get_curve_index(CurveIndex(1, 0.5), curve, -0.25) == CurveIndex(1,0.25)
    @test get_curve_index(CurveIndex(2, 0.0), curve, -0.50) == CurveIndex(1,0.50)

    res = proj(VecSE2(0.0,0.0,0.0), curve)
    @inferred proj(VecSE2(0.0,0.0,0.0), curve)
    show(IOBuffer(), res)
    @test res.ind == CurveIndex(1, 0.0)
    @test isapprox(res.t, 0.0)
    @test isapprox(res.ϕ, 0.0)
    res = proj(VecSE2(0.25,0.5,0.1), curve)
    @inferred proj(VecSE2(0.25,0.5,0.1), curve)
    @test res.ind == CurveIndex(1, 0.25)
    @test isapprox(res.t, 0.5)
    @test isapprox(res.ϕ, 0.1)
    res = proj(VecSE2(0.25,-0.5,-0.1), curve)
    @test res.ind == CurveIndex(1, 0.25)
    @test isapprox(res.t, -0.5)
    @test isapprox(res.ϕ, -0.1)
    res = proj(VecSE2(1.5,0.5,-0.1), curve)
    @test res.ind == CurveIndex(2, 0.25)
    @test isapprox(res.t,  0.5)
    @test isapprox(res.ϕ, -0.1)
    res = proj(VecSE2(10.0,0.0,0.0), curve)
    @test res.ind == CurveIndex(length(curve)-1, 1.0)
    @test isapprox(res.t, 0.0)
    @test isapprox(res.ϕ, 0.0)

    curve2 = [CurvePt(VecSE2(-1.0,-1.0,π/4), 0.0),
              CurvePt(VecSE2( 1.0, 1.0,π/4), hypot(2.0,2.0)),
              CurvePt(VecSE2( 3.0, 3.0,π/4), hypot(4.0,4.0))]

   res = proj(VecSE2(0.0,0.0,0.0), curve2)
   @test res.ind == CurveIndex(1, 0.5)
   @test isapprox(res.t, 0.0)
   @test isapprox(res.ϕ, -π/4)
end # curve tests

@testset "roadways" begin
    curve = get_test_curve1()

    lanetag = LaneTag(1,1)
    lane = Lane(lanetag, curve)
    show(IOBuffer(), lanetag)

    @test !has_next(lane)
    @test !has_prev(lane)

    show(IOBuffer(), RoadIndex(CurveIndex(1,0.0), lanetag))

    roadway = get_test_roadway()

    io = IOBuffer()
    show(io, roadway)
    close(io)

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
    @inferred proj(VecSE2(1.0,0.0,0.0), lane, roadway)
    @test res.curveproj.ind == CurveIndex(2, 0.0)
    @test isapprox(res.curveproj.t, 0.0)
    @test isapprox(res.curveproj.ϕ, 0.0)
    @test res.tag == lane.tag

    res = proj(VecSE2(1.0,0.0,0.0), lane, roadway, move_along_curves=false)
    @inferred proj(VecSE2(1.0,0.0,0.0), lane, roadway, move_along_curves=false)
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

    roadind_0 = RoadIndex(CurveIndex(1, 0.0), LaneTag(2,1))
    roadind = move_along(roadind_0, roadway, 0.0)
    @inferred move_along(roadind_0, roadway, 0.0)
    @test roadind == roadind_0

    roadind = move_along(roadind_0, roadway, 1.0)
    @test roadind == RoadIndex(CurveIndex(2,0.0), LaneTag(2,1))

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

end # roadway test

@testset "roadway read/write" begin  
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
                @test normsquared(convert(VecE2, pt1.pos) - convert(VecE2, pt2.pos)) < 0.01
                @test angledist(pt1.pos.θ, pt2.pos.θ) < 1e-5
                @test isnan(pt1.s) || isapprox(pt1.s, pt2.s, atol=1e-3)
                @test isnan(pt1.k) || isapprox(pt1.k, pt2.k, atol=1e-6)
                @test isnan(pt1.kd) || isapprox(pt1.kd, pt2.kd, atol=1e-6)
            end 
        end
    end
end # roadway read/write tests

@testset "Lane connection" begin
    lc = LaneConnection(true, CurveIndex(1,0.0), RoadIndex(CurveIndex(2,0.0), LaneTag(2,2)))
    show(IOBuffer(), lc)
end


@testset "Frenet" begin
    @test isapprox(AutomotiveDrivingModels._mod2pi2(0.0), 0.0)
    @test isapprox(AutomotiveDrivingModels._mod2pi2(0.5), 0.5)
    @test isapprox(AutomotiveDrivingModels._mod2pi2(2pi + 1), 1.0)
    @test isapprox(AutomotiveDrivingModels._mod2pi2(1 - 2pi), 1.0)

    roadway = get_test_roadway()
    roadproj = proj(VecSE2(0.0, 0.0, 0.0), roadway)
    roadind = RoadIndex(roadproj)

    @test roadind.ind.i == 1
    @test isapprox(roadind.ind.t, 0.0)
    @test roadind.tag == LaneTag(2,1)
    f = Frenet(roadind, roadway, t=0.1, ϕ=0.2)
    @test f.roadind === roadind
    @test isapprox(f.s, 0.0)
    @test f.t == 0.1
    @test f.ϕ == 0.2

    f = Frenet(roadproj, roadway)
    @test f.roadind === roadind
    @test isapprox(f.s, 0.0)
    @test f.t == 0.0
    @test f.ϕ == 0.0

    f = Frenet(VecSE2(0.0, 0.0, 0.0), roadway)
    @test f.roadind === roadind
    @test isapprox(f.s, 0.0)
    @test f.t == 0.0
    @test f.ϕ == 0.0
    @test get_posG(f, roadway) == VecSE2(0.0, 0.0, 0.0)
    
    lane = roadway[LaneTag(1,1)]
    f = Frenet(lane, 1.0, 0.0, 0.0)
    @test f.roadind.tag == lane.tag
    @test isapprox(f.s, 1.0)
    @test f.t == 0.0
    @test f.ϕ == 0.0
end

@testset "straight roadway" begin
    curve = gen_straight_curve(VecE2(25.0, -10.0), VecE2(25.0, 10.0), 2)
    @inferred gen_straight_curve(VecE2(25.0, -10.0), VecE2(25.0, 10.0), 2)
    @test length(curve) == 2
    @test curve[1].pos.x == 25.0

    roadway = gen_straight_roadway(1, 1000.0)
    @test length(roadway.segments) == 1

    seg = roadway[1]
    @test length(seg.lanes) == 1

    lane = seg.lanes[1]
    @test isapprox(lane.width, DEFAULT_LANE_WIDTH)
    @test lane.curve[1] == CurvePt(VecSE2(0.0,0.0,0.0), 0.0)
    @test lane.curve[2] == CurvePt(VecSE2(1000.0,0.0,0.0), 1000.0)

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

@testset "bezier curve" begin 
    # see intersection tutorial for visualization
    roadway = Roadway()
    # Define coordinates of the entry and exit points to the intersection
    r = 5.0 # turn radius
    A = VecSE2(0.0,DEFAULT_LANE_WIDTH,-π)
    B = VecSE2(0.0,0.0,0.0)
    C = VecSE2(r,-r,-π/2)
    D = VecSE2(r+DEFAULT_LANE_WIDTH,-r,π/2)
    E = VecSE2(2r+DEFAULT_LANE_WIDTH,0,0)
    F = VecSE2(2r+DEFAULT_LANE_WIDTH,DEFAULT_LANE_WIDTH,-π)

    # helper function 
    function append_to_curve!(target::Curve, newstuff::Curve)
        s_end = target[end].s
        for c in newstuff
            push!(target, CurvePt(c.pos, c.s+s_end, c.k, c.kd))
        end
        return target
    end

    # Append right turn coming from the left
    curve = gen_straight_curve(convert(VecE2, B+VecE2(-100,0)), convert(VecE2, B), 2)
    append_to_curve!(curve, gen_bezier_curve(B, C, 0.6r, 0.6r, 51)[2:end])
    @test length(curve) == 52
    append_to_curve!(curve, gen_straight_curve(convert(VecE2, C), convert(VecE2, C+VecE2(0,-50.0)), 2))
    @test length(curve) == 54
    lane = Lane(LaneTag(length(roadway.segments)+1,1), curve)
    push!(roadway.segments, RoadSegment(lane.tag.segment, [lane]))

end

@testset "stadium roadway" begin
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