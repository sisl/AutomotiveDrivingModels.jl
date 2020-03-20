let
    q = Quat(1.0,2.0,3.0,4.5)
    @test length(q) == 4
    @test imag(q) == VecE3(1.0,2.0,3.0)
    @test conj(q) == Quat(-1.0,-2.0,-3.0,4.5)
    @test vec(q) == [1.0,2.0,3.0,4.5]
    @test vec(copy(q)) == vec(q)
    @test convert(Quat, [1.0,2.0,3.0,4.5]) == q
    @test norm(q) == norm(vec(q))
    @test vec(normalize(q)) == vec(q) ./ norm(vec(q))


    q = Quat(-0.002, -0.756, 0.252, -0.604)
    sol = push!(vec(get_axis(q)), get_rotation_angle(q))
    @test isapprox(sol, [-0.00250956, -0.948614,  0.316205, mod2pi(-1.84447)], atol=1e-3) ||
          isapprox(sol, [ 0.00250956,  0.948614, -0.316205, mod2pi( 1.84447)], atol=1e-3)

    seed!(0)
    for i in 1 : 5
        a = normalize(VecE3(rand(),rand(),rand()))
        b = normalize(VecE3(rand(),rand(),rand()))
        q = quat_for_a2b(a,b)
        @test norm(rot(q, a) - b) < 1e-8
    end

    @test isapprox(angledist(Quat(1,0,0,0), Quat(0,1,0,0)), π, atol=1e-6)
    @test norm(lerp(Quat(1,0,0,0), Quat(0,1,0,0), 0.5) - Quat(√2/2, √2/2, 0, 0)) < 1e-6

    seed!(0)
    for i in 1 : 5
        q = rand(Quat)
        @test isapprox(norm(q), 1.0, atol=1e-8)
        @test norm(inv(q)*q - Quat(0,0,0,1)) < 1e-8
        @test norm(q*inv(q) - Quat(0,0,0,1)) < 1e-8
    end
end
