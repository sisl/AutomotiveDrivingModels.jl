let
	a = VecE3()
	@test typeof(a) <: VecE3
	@test typeof(a) <: AbstractVec
	@test a.x == 0.0
	@test a.y == 0.0
	@test a.z == 0.0

	b = VecE3(0.0,0.0,0.0)
	@test b.x == 0.0
	@test b.y == 0.0
	@test b.z == 0.0

	@test length(a) == 3
	@test a == b
	@test isequal(a, b)
	@test copy(a) == a
	@test convert(Vector{Float64}, a) == [0.0,0.0,0.0]
	@test convert(VecE3, [0.0,0.0,0.0]) == a

	@test isapprox(polar(1.0,0.0,0.0), VecE3(1.0,0.0,0.0))
	@test isapprox(polar(2.0,π/2,1.0), VecE3(0.0,2.0,1.0))
	@test isapprox(polar(3.0,-π/2,-0.5), VecE3(0.0,-3.0,-0.5))
	@test isapprox(polar(-0.5,1.0*π,1.0*π), VecE3(0.5,0.0,1.0*π))

	a = VecE3(0.0,1.0, 2.0)
	b = VecE3(0.5,2.0,-3.0)

	@test a != b

	@test a + b == VecE3(0.5, 3.0, -1.0)
	@test a .+ 1 == VecE3(1.0, 2.0,  3.0)
	@test a .+ 0.5 == VecE3(0.5, 1.5, 2.5)

	@test a - b == VecE3(-0.5, -1.0, 5.0)
	@test a .- 1 == VecE3(-1.0,  0.0, 1.0)
	@test a .- 0.5 == VecE3(-0.5, 0.5, 1.5)

	@test a * 2 == VecE3(0.0, 2.0, 4.0)
	@test a * 0.5 == VecE3(0.0, 0.5, 1.0)

	@test a / 2 == VecE3(0.0, 0.5, 1.0)
	@test a / 0.5 == VecE3(0.0, 2.0, 4.0)

	@test b.^2 == VecE3(0.25, 4.0, 9.0)
	@test VecE3(0.5, 2.0, 3.0).^0.5 == VecE3(0.5^0.5, 2.0^0.5, 3.0^0.5)

	@test isfinite(a)
	@test !isfinite(VecE3(Inf,0,0))
	@test !isfinite(VecE3(0,Inf,0))
	@test !isfinite(VecE3(0,-Inf,0))
	@test !isfinite(VecE3(Inf,Inf,Inf))

	@test !isinf(a)
	@test isinf(VecE3(Inf,0,0))
	@test isinf(VecE3(0,Inf,0))
	@test isinf(VecE3(0,-Inf,0))
	@test isinf(VecE3(Inf,Inf,Inf))

	@test !isnan(a)
	@test isnan(VecE3(NaN,0,NaN))
	@test isnan(VecE3(0,NaN,NaN))
	@test isnan(VecE3(NaN,NaN,NaN))

	@test round.(VecE3(0.25,1.75,0)) == VecE3(0.0,2.0,0.0)
	@test round.(VecE3(-0.25,-1.75,0)) == VecE3(-0.0,-2.0,0.0)

	@test floor.(VecE3(0.25,1.75,0.0)) == VecE3(0.0,1.0,0.0)
	@test floor.(VecE3(-0.25,-1.75,0.0)) == VecE3(-1.0,-2.0,0.0)

	@test ceil.(VecE3(0.25,1.75,0.0)) == VecE3(1.0,2.0,0.0)
	@test ceil.(VecE3(-0.25,-1.75,0.0)) == VecE3(-0.0,-1.0,0.0)

	@test trunc.(VecE3(0.25,1.75,0.0)) == VecE3(0.0,1.0,0.0)
	@test trunc.(VecE3(-0.25,-1.75,0.0)) == VecE3(-0.0,-1.0,0.0)

	@test clamp.(VecE3(1.0, 10.0, 0.5), 0.0, 5.0) == VecE3(1.0, 5.0, 0.5)
	@test clamp.(VecE3(-1.0, 4.0, 5.5), 0.0, 5.0) == VecE3(0.0, 4.0, 5.0)

	c = VecE3(3.0,4.0,5.0)

	@test isapprox(abs.(a), a)
	@test isapprox(abs.(c), c)
	@test isapprox(norm(c), sqrt(3^2 + 4^2 + 5^2))
	@test isapprox(normalize(a), VecE3(0.0,1.0/hypot(1.0,2.0), 2.0/hypot(1.0,2.0)))

	@test isapprox(dist(a,a), 0.0)
	@test isapprox(dist(b,b), 0.0)
	@test isapprox(dist(c,c), 0.0)
	@test isapprox(dist(a,b), norm([0.5, 1.0, 5.0]))
	@test isapprox(dist(a,c), norm([3.0, 3.0, 3.0]))

	@test isapprox(dist2(a,a), 0.0)
	@test isapprox(dist2(b,b), 0.0)
	@test isapprox(dist2(c,c), 0.0)
	@test isapprox(dist2(a,b), 0.5^2 + 1^2 + 5^2)
	@test isapprox(dist2(a,c), 3^2 + 3^2 + 3^2)

	@test isapprox(dot(a, b), 2.0 - 6.0)
	@test isapprox(dot(b, c), 1.5 + 8.0 - 15.0)
	@test isapprox(dot(c, c), norm(c)^2)

	@test isapprox(proj(b, a, Float64), -norm(VecE3(0.0, -0.8, -1.6)))
	@test isapprox(proj(c, a, Float64),  norm(VecE3(0.0,  2.8,  5.6)))

	@test isapprox(proj(b, a, VecE3), VecE3(0.0, -0.8, -1.6))
	@test isapprox(proj(c, a, VecE3), VecE3(0.0,  2.8,  5.6))

	@test isapprox(lerp(VecE3(0,0,0), VecE3(2,-2,2), 0.25), VecE3(0.5,-0.5,0.5))
	@test isapprox(lerp(VecE3(2,-2,2), VecE3(3,-3,3), 0.5), VecE3(2.5,-2.5,2.5))

	@test isapprox(rot(VecE3(1,2,3), VecE3(2,0,0), 0.0), VecE3(1,2,3))
	@test isapprox(rot(VecE3(1,2,3), VecE3(2,0,0), π/2), VecE3(1,-3,2))
	@test isapprox(rot(VecE3(1,2,3), VecE3(2,0,0), -π/2), VecE3(1,3,-2))
	@test isapprox(rot(VecE3(1,2,3), VecE3(2,0,0), 1.0π), VecE3(1,-2,-3))
	@test isapprox(rot(VecE3(1,2,3), VecE3(0,2,0), π/2), VecE3(3,2,-1))
	@test isapprox(rot(VecE3(1,2,3), VecE3(0,0,2), π/2), VecE3(-2,1,3))

	@test isapprox(rot_normalized(VecE3(1,2,3), VecE3(1,0,0), 0.0), VecE3(1,2,3))
	@test isapprox(rot_normalized(VecE3(1,2,3), VecE3(1,0,0), π/2), VecE3(1,-3,2))
	@test isapprox(rot_normalized(VecE3(1,2,3), VecE3(1,0,0), -π/2), VecE3(1,3,-2))
	@test isapprox(rot_normalized(VecE3(1,2,3), VecE3(1,0,0), 1.0π), VecE3(1,-2,-3))
	@test isapprox(rot_normalized(VecE3(1,2,3), VecE3(0,1,0), π/2), VecE3(3,2,-1))
	@test isapprox(rot_normalized(VecE3(1,2,3), VecE3(0,0,1), π/2), VecE3(-2,1,3))
end
