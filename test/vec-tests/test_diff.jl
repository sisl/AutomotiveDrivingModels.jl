using ForwardDiff
using LinearAlgebra

let
	f(x) = x+x
	g(x) = 5*x
	h(x) = dot(x, x)

	# Once we get ForwardDiff working, this should work
	# h(x::VecE) = dot(x, x)

	@test ForwardDiff.jacobian(f, VecE2(1.0, 2.0)) == [2.0 0.0; 0.0 2.0]
	@test ForwardDiff.jacobian(g, VecE2(1.0, 2.0)) == [5.0 0.0; 0.0 5.0]
	@test ForwardDiff.gradient(h, VecE2(1.0, 2.0)) == [2.0, 4.0]
	@test ForwardDiff.hessian(h, VecE2(1.0, 2.0)) == [2.0 0.0; 0.0 2.0]

	@test ForwardDiff.jacobian(f, VecE3(1.0, 2.0, 3.0)) == 2.0*Matrix{Float64}(I, 3, 3)
	@test ForwardDiff.jacobian(g, VecE3(1.0, 2.0, 3.0)) == 5.0*Matrix{Float64}(I, 3, 3)
	@test ForwardDiff.gradient(h, VecE3(1.0, 2.0, 3.0)) == [2.0, 4.0, 6.0]
	@test ForwardDiff.hessian(h, VecE3(1.0, 2.0, 3.0)) == 2.0*Matrix{Float64}(I, 3, 3)

	@test ForwardDiff.jacobian(f, VecSE2(1.0, 2.0, 3.0)) == 2.0*Matrix{Float64}(I, 3, 3)

	# Once we get ForwardDiff working, this should work...
	# h(x::VecSE2) = normsquared(VecE2(x))
	# @test ForwardDiff.jacobian(g, VecSE2(1.0, 2.0, 3.0)) == diagm([5.0, 5.0, 1.0])
	# @test ForwardDiff.gradient(h, VecSE2(1.0, 2.0, 3.0)) == [2.0, 4.0, 0.0]
	# @test ForwardDiff.hessian(h, VecSE2(1.0, 2.0, 3.0)) == [2.0 0.0 0.0; 0.0 2.0 0.0; 0.0 0.0 0.0]
end
