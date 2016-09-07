get_test_curve1() = [CurvePt(VecSE2(0.0,0.0,0.0), 0.0),
                     CurvePt(VecSE2(1.0,0.0,0.0), 1.0),
                     CurvePt(VecSE2(3.0,0.0,0.0), 3.0)]

let
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
    show(IOBuffer(), res)
    @test res.ind == CurveIndex(1, 0.0)
    @test isapprox(res.t, 0.0)
    @test isapprox(res.ϕ, 0.0)
    res = proj(VecSE2(0.25,0.5,0.1), curve)
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
end