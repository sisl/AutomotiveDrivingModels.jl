a = VecSE2()
@test typeof(a) <: VecSE
@test typeof(a) <: AbstractVec
@test a.x == 0.0
@test a.y == 0.0
@test a.θ == 0.0

b = VecSE2(0.0,0.0,0.0)
@test b.x == 0.0
@test b.y == 0.0
@test b.θ == 0.0

c = VecSE2(VecE2(0.0,0).0,0.0)
@test c.x == 0.0
@test c.y == 0.0
@test c.θ == 0.0

@test length(a) == 3
@test a == b
@test b == c
@test a == c
@test isequal(b, a)
@test copy(a) == a
@test convert(Vector{Float64}, a) == [0.0,0.0,0.0]
@test convert(VecSE2, [1.0,2.0,3.0]) == VecSE2(1.0,2.0,3.0)
@test convert(VecE2, VecSE2(1.0,2.0,3.0)) == VecE2(1.0,2.0)
@test convert(VecE3, VecSE2(1.0,2.0,3.0)) == VecE3(1.0,2.0,3.0)

# @test isapprox(polar(1.0,0.0,0.5), VecSE2(1.0,0.0,0.5))
# @test isapprox(polar(2.0,π/2,-0.5), VecSE2(0.0,2.0,-0.5))
# @test isapprox(polar(3.0,-π/2,1), VecSE2(0.0,-3.0,1.0))
# @test isapprox(polar(-0.5,1.0*π,-1), VecSE2(0.5,0.0,-1))

# a = VecSE2(0.0,1.0,π/2)
# b = VecSE2(0.5,2.0,-π)

# @test a != b

# @test a + b == VecSE2(0.5, 3.0, )
# @test a + 1 == VecSE2(1.0, 2.0)
# @test a + 0.5 == VecSE2(0.5, 1.5)

# @test a - b == VecSE2(-0.5, -1.0)
# @test a - 1 == VecSE2(-1.0,  0.0)
# @test a - 0.5 == VecSE2(-0.5, 0.5)

# @test a * 2 == VecSE2(0.0, 2.0)
# @test a * 0.5 == VecSE2(0.0, 0.5)

# @test a / 2 == VecSE2(0.0, 0.5)
# @test a / 0.5 == VecSE2(0.0, 2.0)

# @test b^2 == VecSE2(0.25, 4.0)
# @test b^0.5 == VecSE2(0.5^0.5, 2.0^0.5)

# @test a % 2.0 == VecSE2(0.0, 1.0)

# @test isfinite(a)
# @test !isfinite(VecSE2(Inf,0))
# @test !isfinite(VecSE2(0,Inf))
# @test !isfinite(VecSE2(0,-Inf))
# @test !isfinite(VecSE2(Inf,Inf))

# @test !isinf(a)
# @test isinf(VecSE2(Inf,0))
# @test isinf(VecSE2(0,Inf))
# @test isinf(VecSE2(0,-Inf))
# @test isinf(VecSE2(Inf,Inf))

# @test !isnan(a)
# @test isnan(VecSE2(NaN,0))
# @test isnan(VecSE2(0,NaN))
# @test isnan(VecSE2(NaN,NaN))

# @test round(VecSE2(0.25,1.75)) == VecSE2(0.0,2.0)
# @test round(VecSE2(-0.25,-1.75)) == VecSE2(-0.0,-2.0)

# @test floor(VecSE2(0.25,1.75)) == VecSE2(0.0,1.0)
# @test floor(VecSE2(-0.25,-1.75)) == VecSE2(-1.0,-2.0)

# @test ceil(VecSE2(0.25,1.75)) == VecSE2(1.0,2.0)
# @test ceil(VecSE2(-0.25,-1.75)) == VecSE2(-0.0,-1.0)

# @test trunc(VecSE2(0.25,1.75)) == VecSE2(0.0,1.0)
# @test trunc(VecSE2(-0.25,-1.75)) == VecSE2(-0.0,-1.0)

# c = VecSE2(3.0,4.0)

# @test isapprox(abs(a), 1.0)
# @test isapprox(abs(c), 5.0)
# @test isapprox(hypot(a), 1.0)
# @test isapprox(hypot(c), 5.0)
# @test isapprox(abs2(a), 1.0)
# @test isapprox(abs2(c), 25.0)
# @test isapprox(norm(a), VecSE2(0.0,1.0))
# @test isapprox(norm(b), VecSE2(0.5/sqrt(4.25),2.0/sqrt(4.25)))
# @test isapprox(norm(c), VecSE2(3.0/5,4.0/5))

# @test isapprox(dist(a,a), 0.0)
# @test isapprox(dist(b,b), 0.0)
# @test isapprox(dist(c,c), 0.0)
# @test isapprox(dist(a,b), hypot(0.5, 1.0))
# @test isapprox(dist(a,c), hypot(3.0, 3.0))

# @test isapprox(dist2(a,a), 0.0)
# @test isapprox(dist2(b,b), 0.0)
# @test isapprox(dist2(c,c), 0.0)
# @test isapprox(dist2(a,b), 0.5^2 + 1^2)
# @test isapprox(dist2(a,c), 3^2 + 3^2)

# @test isapprox(dot(a, b), 2.0)
# @test isapprox(dot(b, c), 1.5 + 8.0)
# @test isapprox(dot(c, c), abs2(c))

# @test isapprox(proj(b, a, Float64), 2.0)
# @test isapprox(proj(c, a, Float64), 4.0)

# @test isapprox(proj(b, a, VecSE2), VecSE2(0.0, 2.0))
# @test isapprox(proj(c, a, VecSE2), VecSE2(0.0, 4.0))

# @test isapprox(lerp(VecSE2(0,0), VecSE2(2,-2), 0.25), VecSE2(0.5,-0.5))
# @test isapprox(lerp(VecSE2(2,-2), VecSE2(3,-3), 0.5), VecSE2(2.5,-2.5))

# @test isapprox(rot180(b), VecSE2(-0.5,-2.0))
# @test isapprox(rotr90(b), VecSE2( 2.0,-0.5))
# @test isapprox(rotl90(b), VecSE2(-2.0, 0.5))

# @test isapprox(rot(b, 1.0*pi), VecSE2(-0.5,-2.0))
# @test isapprox(rot(b, -π/2), VecSE2( 2.0,-0.5))
# @test isapprox(rot(b,  π/2), VecSE2(-2.0, 0.5))
# @test isapprox(rot(b, 2π), b)
