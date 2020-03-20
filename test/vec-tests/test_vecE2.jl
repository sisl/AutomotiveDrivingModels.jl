let
	a = VecE2()
	@test typeof(a) <: VecE
	@test typeof(a) <: AbstractVec
	@test a.x == 0.0
	@test a.y == 0.0

	b = VecE2(0.0,0.0)
	@test b.x == 0.0
	@test b.y == 0.0

	@test length(a) == 2
	@test a == b
	@test isequal(a, b)
	@test copy(a) == a
	@test convert(Vector{Float64}, a) == [0.0,0.0]
	@test convert(VecE2, [0.0,0.0]) == a

	@test isapprox(polar(1.0,0.0), VecE2(1.0,0.0))
	@test isapprox(polar(2.0,π/2), VecE2(0.0,2.0))
	@test isapprox(polar(3.0,-π/2), VecE2(0.0,-3.0))
	@test isapprox(polar(-0.5,1.0*π), VecE2(0.5,0.0))

	a = VecE2(0.0,1.0)
	b = VecE2(0.5,2.0)

	@test a != b

	@test a + b == VecE2(0.5, 3.0)
	@test a .+ 1 == VecE2(1.0, 2.0)
	@test a .+ 0.5 == VecE2(0.5, 1.5)

	@test a - b == VecE2(-0.5, -1.0)
	@test a .- 1 == VecE2(-1.0,  0.0)
	@test a .- 0.5 == VecE2(-0.5, 0.5)

	@test a * 2 == VecE2(0.0, 2.0)
	@test a * 0.5 == VecE2(0.0, 0.5)

	@test a / 2 == VecE2(0.0, 0.5)
	@test a / 0.5 == VecE2(0.0, 2.0)

	@test b.^2 == VecE2(0.25, 4.0)
	@test b.^0.5 == VecE2(0.5^0.5, 2.0^0.5)

	@test a.%2.0 == VecE2(0.0, 1.0)

	@test isfinite(a)
	@test !isfinite(VecE2(Inf,0))
	@test !isfinite(VecE2(0,Inf))
	@test !isfinite(VecE2(0,-Inf))
	@test !isfinite(VecE2(Inf,Inf))

	@test !isinf(a)
	@test isinf(VecE2(Inf,0))
	@test isinf(VecE2(0,Inf))
	@test isinf(VecE2(0,-Inf))
	@test isinf(VecE2(Inf,Inf))

	@test !isnan(a)
	@test isnan(VecE2(NaN,0))
	@test isnan(VecE2(0,NaN))
	@test isnan(VecE2(NaN,NaN))

	@test round.(VecE2(0.25,1.75)) == VecE2(0.0,2.0)
	@test round.(VecE2(-0.25,-1.75)) == VecE2(-0.0,-2.0)

	@test floor.(VecE2(0.25,1.75)) == VecE2(0.0,1.0)
	@test floor.(VecE2(-0.25,-1.75)) == VecE2(-1.0,-2.0)

	@test ceil.(VecE2(0.25,1.75)) == VecE2(1.0,2.0)
	@test ceil.(VecE2(-0.25,-1.75)) == VecE2(-0.0,-1.0)

	@test trunc.(VecE2(0.25,1.75)) == VecE2(0.0,1.0)
	@test trunc.(VecE2(-0.25,-1.75)) == VecE2(-0.0,-1.0)

	@test clamp.(VecE2(1.0, 10.0), 0.0, 5.0) == VecE2(1.0, 5.0)
	@test clamp.(VecE2(-1.0, 4.0), 0.0, 5.0) == VecE2(0.0, 4.0)

	c = VecE2(3.0,4.0)

	@test isapprox(abs.(a), VecE2(0.0, 1.0))
	@test isapprox(abs.(c), VecE2(3.0, 4.0))
	@test isapprox(norm(a), 1.0)
	@test isapprox(norm(c), 5.0)
	@test isapprox(normalize(a), VecE2(0.0,1.0))
	@test isapprox(normalize(b), VecE2(0.5/sqrt(4.25),2.0/sqrt(4.25)))
	@test isapprox(normalize(c), VecE2(3.0/5,4.0/5))

	@test isapprox(dist(a,a), 0.0)
	@test isapprox(dist(b,b), 0.0)
	@test isapprox(dist(c,c), 0.0)
	@test isapprox(dist(a,b), hypot(0.5, 1.0))
	@test isapprox(dist(a,c), hypot(3.0, 3.0))

	@test isapprox(dist2(a,a), 0.0)
	@test isapprox(dist2(b,b), 0.0)
	@test isapprox(dist2(c,c), 0.0)
	@test isapprox(dist2(a,b), 0.5^2 + 1^2)
	@test isapprox(dist2(a,c), 3^2 + 3^2)

	@test isapprox(dot(a, b), 2.0)
	@test isapprox(dot(b, c), 1.5 + 8.0)
	@test isapprox(dot(c, c), norm(c)^2)

	@test isapprox(proj(b, a, Float64), 2.0)
	@test isapprox(proj(c, a, Float64), 4.0)

	@test isapprox(proj(b, a, VecE2), VecE2(0.0, 2.0))
	@test isapprox(proj(c, a, VecE2), VecE2(0.0, 4.0))

	@test isapprox(lerp(VecE2(0,0), VecE2(2,-2), 0.25), VecE2(0.5,-0.5))
	@test isapprox(lerp(VecE2(2,-2), VecE2(3,-3), 0.5), VecE2(2.5,-2.5))

	@test isapprox(rot180(b), VecE2(-0.5,-2.0))
	@test isapprox(rotr90(b), VecE2( 2.0,-0.5))
	@test isapprox(rotl90(b), VecE2(-2.0, 0.5))

	@test isapprox(rot(b, 1.0*pi), VecE2(-0.5,-2.0))
	@test isapprox(rot(b, -π/2), VecE2( 2.0,-0.5))
	@test isapprox(rot(b,  π/2), VecE2(-2.0, 0.5))
	@test isapprox(rot(b, 2π), b)
end
