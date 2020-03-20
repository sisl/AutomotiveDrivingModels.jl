let
    @test isapprox(deltaangle(0.0, 0.0),  0.0)
    @test isapprox(deltaangle(0.0, 1.0),  1.0)
    @test isapprox(deltaangle(0.0, 2π),  0.0, atol=1e-10)
    @test isapprox(deltaangle(0.0, π + 1.0), -(π- 1.0))

    @test isapprox(angledist(0.0, 0.0), 0.0)
    @test isapprox(angledist(0.0, 1.0), 1.0)
    @test isapprox(angledist(0.0, 2π), 0.0, atol=1e-10)
    @test isapprox(angledist(0.0, π + 1.0), π- 1.0)

    @test isapprox(lerp_angle(0.0, 0.0, 1.0), 0.0)
    @test isapprox(lerp_angle(0.0, 2π, 0.5), 0.0, atol=1e-10)
    @test isapprox(lerp_angle(0.0, 2.0, 0.75),  1.5)
    @test isapprox(lerp_angle(0.0, 2π-2, 0.75), -1.5)

    @test are_collinear(VecE2(0.0,0.0), VecE2(1.0,1.0), VecE2(2.0,2.0))
    @test are_collinear(VecE2(0.0,0.0), VecE2(1.0,1.0), VecE2(-2.0,-2.0))
    @test are_collinear(VecE2(0.0,0.0), VecE2(0.5,1.0), VecE2(1.0,2.0))
    @test are_collinear(VecE2(1.0,2.0), VecE2(0.0,0.0), VecE2(0.5,1.0))
    @test !are_collinear(VecE2(0.0,0.0), VecE2(1.0,0.0), VecE2(0.0,1.0))
end

let
    seg = LineSegment1D(0,1)
    seg2 = seg + 0.5
    @test isapprox(seg2.a, 0.5)
    @test isapprox(seg2.b, 1.5)

    seg2 = seg - 0.3
    @test isapprox(seg2.a, -0.3)
    @test isapprox(seg2.b,  0.7)

    seg = LineSegment1D(2,5)
    @test isapprox(get_distance(seg, 1.5), 0.5)
    @test isapprox(get_distance(seg, -1.5), 3.5)
    @test isapprox(get_distance(seg,  2.5), 0.0)
    @test isapprox(get_distance(seg,  2.0), 0.0)
    @test isapprox(get_distance(seg,  5.0), 0.0)
    @test isapprox(get_distance(seg,  5.5), 0.5)
    @test isapprox(get_distance(seg, 15.5), 10.5)

    @test !in(-1.0, seg)
    @test !in(1.0, seg)
    @test in(2.0, seg)
    @test in(3.0, seg)
    @test in(5.0, seg)
    @test !in(5.5, seg)

    @test in(LineSegment1D(3.0, 3.5), seg)
    @test in(LineSegment1D(3.0, 3.0), seg)
    @test !in(LineSegment1D(3.0, 13.0), seg)
    @test !in(LineSegment1D(-3.0, 3.0), seg)
    @test !in(LineSegment1D(-3.0, -2.0), seg)
    @test !in(LineSegment1D(13.0, 15.0), seg)

    @test intersects(seg, LineSegment1D(3.0, 3.5))
    @test intersects(seg, LineSegment1D(3.0, 3.0))
    @test intersects(seg, LineSegment1D(3.0, 13.0))
    @test intersects(seg, LineSegment1D(-3.0, 3.0))
    @test !intersects(seg, LineSegment1D(-3.0, -2.0))
    @test !intersects(seg, LineSegment1D(13.0, 15.0))
end

let
    L = Line(VecE2(0,0), π/4)
    L2 = L + VecE2(-1,1)
    @test isapprox(L2.C, VecE2(-1,1))
    @test isapprox(L2.θ, π/4)
    L2 = L - VecE2(-1,1)
    @test isapprox(L2.C, VecE2(1,-1))
    @test isapprox(L2.θ, π/4)
    L2 = rot180(L)
    @test isapprox(L2.θ, π/4 + π)
    @test isapprox(get_polar_angle(L), atan(1,1))
    @test isapprox(get_distance(L, VecE2(0,0)), 0)
    @test isapprox(get_distance(L, VecE2(1,1)), 0, atol=1e-10)
    @test isapprox(get_distance(L, VecE2(1,0)), √2/2)
    @test isapprox(get_distance(L, VecE2(0,1)), √2/2)
    @test get_side(L, VecE2(0,0)) ==  0
    @test get_side(L, VecE2(1,1)) ==  0
    @test get_side(L, VecE2(0,1)) ==  1
    @test get_side(L, VecE2(1,0)) == -1
end

let
    L = LineSegment(VecE2(0,0), VecE2(1,1))
    L2 = L + VecE2(-1,1)
    @test isapprox(L2.A, VecE2(-1,1))
    @test isapprox(L2.B, VecE2( 0,2))
    L2 = L - VecE2(-1,1)
    @test isapprox(L2.A, VecE2(1,-1))
    @test isapprox(L2.B, VecE2(2, 0))
    @test isapprox(get_polar_angle(L), atan(1,1))
    @test isapprox(get_distance(L, VecE2(0,0)), 0)
    @test isapprox(get_distance(L, VecE2(1,1)), 0)
    @test isapprox(get_distance(L, VecE2(1,0)), √2/2)
    @test isapprox(get_distance(L, VecE2(0,1)), √2/2)
    @test get_side(L, VecE2(1,1)) ==  0
    @test get_side(L, VecE2(0,1)) ==  1
    @test get_side(L, VecE2(1,0)) == -1

    @test angledist(L, L2) ≈ 0.0
    @test parallel(L, L2)

    L2 = LineSegment(VecE2(0,0), VecE2(-1,1))
    @test angledist(L, L2) ≈ π/2
    @test !parallel(L, L2)
    @test intersects(L, L2)

    L2 = LineSegment(VecE2(0,0), VecE2(1,-1))
    @test angledist(L, L2) ≈ π/2
    @test !parallel(L, L2)
    @test intersects(L, L2)

    L2 = LineSegment(VecE2(0,0), VecE2(1,-1)) + VecE2(5,-7)
    @test angledist(L, L2) ≈ π/2
    @test !parallel(L, L2)
    @test !intersects(L, L2)

    @test  intersects(L, LineSegment(VecE2(1,1), VecE2(2,-1)))
    @test !intersects(L, LineSegment(VecE2(2,2), VecE2(5,6)))
    @test !intersects(L, LineSegment(VecE2(-5,-5), VecE2(5,-5)))
end

let
    @test isapprox(intersect(Ray(-1,0,0), Ray(0,-1,π/2)), VecE2(0.0,0.0))

    @test  intersects(Ray(0,0,0), Ray(1,-1,π/2))
    @test !intersects(Ray(0,0,0), Ray(1,-1,-π/2))
    # @test  intersects(Ray(0,0,0), Ray(1,0,0)) # TODO: get this to work

    @test  intersects(Ray(0,0,0),   Line(VecE2(0,0), VecE2(1,1)))
    @test !intersects(Ray(0,0,0),   Line(VecE2(0,1), VecE2(1,1)))
    @test  intersects(Ray(0,0,0),   Line(VecE2(1,0), VecE2(2,0)))
    @test  intersects(Ray(0,0,0),   Line(VecE2(1,-1), VecE2(1,1)))
    @test  intersects(Ray(0,0,π/2), Line(VecE2(-1,1), VecE2(1,1)))
    @test !intersects(Ray(0,0,π/2), Line(VecE2(-1,1), VecE2(-1,2)))
    @test  intersects(Ray(0,0,π/2), Line(VecE2(-1,1), VecE2(-1.5,1.5)))

    @test  intersects(Ray(0,0,0), LineSegment(VecE2(0,0), VecE2(1,1)))
    @test !intersects(Ray(0,0,0), LineSegment(VecE2(0,1), VecE2(1,1)))
    @test  intersects(Ray(0,0,0), LineSegment(VecE2(1,0), VecE2(2,0)))
    @test !intersects(Ray(0,0,0), LineSegment(VecE2(-1,0), VecE2(-2,0)))

    @test isapprox(intersect(VecSE2(4,0,3pi/4), LineSegment(VecE2(5.6,0), VecE2(0,3.6))),
                   VecE2(1.12, 2.88), atol=1e-3)
end

let
    proj1 = Projectile(VecSE2(0.0,0.0,0.0), 1.0)
    proj2 = Projectile(VecSE2(1.0,1.0,1.0), 1.0)
    @test isapprox(VecE2(Vec.propagate(proj1, 2.0).pos), VecE2(2.0,0.0))
    @test isapprox(VecE2(Vec.propagate(proj2, 0.0).pos), VecE2(1.0,1.0))

    t_PCA, d_PCA = closest_time_of_approach_and_distance(proj1, Projectile(VecSE2(0.0,1.0,0.0), 1.0))
    @test isapprox(t_PCA, 0.0)
    @test isapprox(d_PCA, 1.0)

    t_PCA, d_PCA = closest_time_of_approach_and_distance(proj1, Projectile(VecSE2(0.0,2.0,0.0), 1.0))
    @test isapprox(t_PCA, 0.0)
    @test isapprox(d_PCA, 2.0)

    t_PCA, d_PCA = closest_time_of_approach_and_distance(proj1, Projectile(VecSE2(0.0,1.0,0.0), 2.0))
    @test isapprox(t_PCA, 0.0)
    @test isapprox(d_PCA, 1.0)

    t_PCA, d_PCA = closest_time_of_approach_and_distance(Projectile(VecSE2(0.0,-0.0,0.2), 1.0), Projectile(VecSE2(0.0,1.0,-0.2), 1.0))
    @test t_PCA > 0.0
    @test isapprox(d_PCA, 0.0)

    @test isapprox(get_intersection_time(proj1, LineSegment(VecE2(0, 0), VecE2(0,1))), 0.0)
    @test isapprox(get_intersection_time(proj1, LineSegment(VecE2(1,-1), VecE2(1,1))), 1.0)
    @test    isinf(get_intersection_time(proj1, LineSegment(VecE2(-1,0), VecE2(-1,1))))
    @test isapprox(get_intersection_time(proj1, LineSegment(VecE2(3,0), VecE2(2,0))), 2.0)

    t_PCA, d_PCA = closest_time_of_approach_and_distance(proj1, LineSegment(VecE2(0, 0), VecE2(0,1)))
    @test isapprox(t_PCA, 0.0)
    @test isapprox(d_PCA, 0.0)
    t_PCA, d_PCA = closest_time_of_approach_and_distance(proj1, LineSegment(VecE2(1,-1), VecE2(1,1)))
    @test isapprox(t_PCA, 1.0)
    @test isapprox(d_PCA, 0.0)
    t_PCA, d_PCA = closest_time_of_approach_and_distance(proj1, LineSegment(VecE2(1,0.5), VecE2(1,1)))
    @test isapprox(t_PCA, 1.0)
    @test isapprox(d_PCA, 0.5)
    t_PCA, d_PCA = closest_time_of_approach_and_distance(proj1, LineSegment(VecE2(1,-0.5), VecE2(1,-1)))
    @test isapprox(t_PCA, 1.0)
    @test isapprox(d_PCA, 0.5)
    t_PCA, d_PCA = closest_time_of_approach_and_distance(proj1, LineSegment(VecE2(1,-0.5), VecE2(2,-1)))
    @test isapprox(t_PCA, 1.0)
    @test isapprox(d_PCA, 0.5)
    t_PCA, d_PCA = closest_time_of_approach_and_distance(proj1, LineSegment(VecE2(2,-1), VecE2(1,-0.5)))
    @test isapprox(t_PCA, 1.0)
    @test isapprox(d_PCA, 0.5)
    t_PCA, d_PCA = closest_time_of_approach_and_distance(proj1, LineSegment(VecE2(2,-1), VecE2(1,-0.5)), true)
    @test isinf(t_PCA)
    @test isinf(d_PCA)
    # t_PCA, d_PCA = closest_time_of_approach_and_distance(proj1, LineSegment(VecE2(-1,1), VecE2(1,1)))
    # println((t_PCA, d_PCA ))
    # @test isapprox(t_PCA, 0.0)
    # @test isapprox(d_PCA, 1.0)
end

let
    @test  in(VecE2(0,0), Circ(0,0,1.0))
    @test !in(VecE2(0,0), Circ(2,0,1.0))
    @test  in(VecE2(1.5, 0), Circ(2,0,1.0))
    @test  in(VecE3(1.5,0,3), Circ(2,0,3,1.0))

    box = AABB(VecE2(0.0, 0.5), 2.0, 5.0)
    @test  in(VecE2( 0.0,0.0), box)
    @test  in(VecE2(-1.0,0.0), box)
    @test !in(VecE2(-2.0,0.0), box)
    @test !in(VecE2( 1.0,3.1), box)

    box = AABB(VecE2(-1.0, -2.0), VecE2(1.0, 3.0))
    @test  in(VecE2( 0.0,0.0), box)
    @test  in(VecE2(-1.0,0.0), box)
    @test !in(VecE2(-2.0,0.0), box)
    @test !in(VecE2( 1.0,3.1), box)
end

let
    box = OBB(VecSE2(0.0, 0.5, 0.0), 2.0, 5.0)
    @test  in(VecE2( 0.0,0.0), box)
    @test  in(VecE2(-1.0,0.0), box)
    @test !in(VecE2(-2.0,0.0), box)
    @test !in(VecE2( 1.0,3.1), box)

    box = OBB(VecSE2(0.0, 0.0, π/4), 2.0, 2.0)
    @test  in(VecE2( 0.0,0.0), box)
    @test  in(VecE2( 1.0,0.0), box)
    @test  in(VecE2( √2/2-0.1,√2/2-0.1), box)
    @test !in(VecE2( 1.0,1.0), box)
    @test !in(VecE2( √2/2+0.1,√2/2+0.1), box)

    box = OBB(VecSE2(0.0, 0.0, 0.0), 1.0, 1.0)
    box2 = inertial2body(box, VecSE2(1.0,0.0,0.0))
    @test isapprox(box2.aabb.center.x, -1.0)
    @test isapprox(box2.aabb.center.y,  0.0)
    @test isapprox(box2.θ,  0.0)

    box2 = inertial2body(box, VecSE2(1.0,0.0,π))
    @test isapprox(box2.aabb.center.x,  1.0)
    @test isapprox(box2.aabb.center.y,  0.0, atol=1e-10)
    @test isapprox(angledist(box2.θ,  π), 0.0, atol=1e-10)
end

let
    plane = Plane3()
    @test plane.normal == VecE3(1.0,0.0,0.0)
    @test plane.offset == 0.0

    plane = Plane3(VecE3(2.0,0.0,0.0), 0.5)
    @test plane.normal == VecE3(1.0,0.0,0.0)
    @test plane.offset == 0.5

    plane = Plane3(VecE3(2.0,0.0,0.0), VecE3(0.0,0.0,0.0))
    @test plane.normal == VecE3(1.0,0.0,0.0)
    @test plane.offset == 0.0

    plane = Plane3(VecE3(2.0,0.0,0.0), VecE3(1.0,1.0,1.0))
    @test plane.normal == VecE3(1.0,0.0,0.0)
    @test isapprox(plane.offset, -1.0, atol=1e-10)

    plane = Plane3(VecE3(1.0,1.0,0.0), VecE3(1.0,0.0,0.0), VecE3(0.0,0.0,0.0))
    @test norm(plane.normal - VecE3(0.0,0.0,1.0)) < 1e-8
    @test isapprox(plane.offset, 0.0, atol=1e-10)

    plane = Plane3(VecE3(0.0,0.0,0.0), VecE3(1.0,0.0,0.0), VecE3(1.0,1.0,0.0))
    @test norm(plane.normal - VecE3(0.0,0.0,-1.0)) < 1e-8
    @test isapprox(plane.offset, 0.0, atol=1e-10)

    plane = Plane3()
    @test isapprox(get_signed_distance(plane, VecE3( 0.0,0.0,0.0)),  0.0, atol=1e-10)
    @test isapprox(get_signed_distance(plane, VecE3( 1.0,0.0,0.0)),  1.0, atol=1e-10)
    @test isapprox(get_signed_distance(plane, VecE3(-1.0,0.0,0.0)), -1.0, atol=1e-10)
    @test isapprox(get_signed_distance(plane, VecE3( 1.0,0.5,0.7)),  1.0, atol=1e-10)
    @test isapprox(get_signed_distance(plane, VecE3(-1.0,0.7,0.5)), -1.0, atol=1e-10)
    @test isapprox(get_distance(plane, VecE3(-1.0,0.7,0.5)), 1.0, atol=1e-10)
    @test norm(proj(VecE3(-1.0,0.7,0.5), plane) - VecE3(0.0,0.7,0.5)) < 1e-10
    @test get_side(plane, VecE3( 0.0,0.0,0.0)) ==  0
    @test get_side(plane, VecE3( 1.0,0.0,0.0)) ==  1
    @test get_side(plane, VecE3(-1.0,0.0,0.0)) == -1
    @test get_side(plane, VecE3( 1.0,0.5,0.7)) ==  1
    @test get_side(plane, VecE3(-1.0,0.7,0.5)) == -1
end
