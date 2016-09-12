let

    vec_A = Array(Matrix{Float64}, 2)
    vec_b = Array(Vector{Float64}, 2)
    vec_A[1] = [0.5,]'
    vec_A[2] = [2.0,]'
    vec_b[1] = [2.0]
    vec_b[2] = [0.5]
    mix_obs = MixtureModel([MvNormal([0.0], [1.0]), MvNormal([1.0], [1.0])],[0.5,0.5])
    mix_AgO = MixtureModel([MvNormal([NaN], [1.0]), MvNormal([NaN], [3.0])])

    gmr = GMR(vec_A, vec_b, mix_obs, mix_AgO)

    show(IOBuffer(), gmr)

    pred = gmr([0.0])
    weights = probs(pred.prior)
    @test isapprox(weights[1], 0.6225, atol=1e-4)
    @test isapprox(weights[2], 0.3775, atol=1e-4)
    @test isapprox(pred.components[1].μ[1], 2.0, atol=1e-8)
    @test isapprox(pred.components[2].μ[1], 0.5, atol=1e-8)
    @test full(pred.components[1].Σ)[1] == 1.0
    @test full(pred.components[2].Σ)[1] == 9.0


    mix = MixtureModel([
            MvNormal([0.0, 0.0], [1.0, 1.0]),
            MvNormal([1.0, 1.0], [1.0, 1.0]),
        ], [0.25,0.75]
        )
    gmr = GMR(mix, 1)
    @test isapprox(gmr.vec_A[1][1], 0.0, atol=1e-8)
    @test isapprox(gmr.vec_A[2][1], 0.0, atol=1e-8)
    @test isapprox(gmr.vec_b[1][1], 0.0, atol=1e-8)
    @test isapprox(gmr.vec_b[2][1], 1.0, atol=1e-8)
    @test isapprox(full(gmr.mixture_Act_given_Obs.components[1].Σ)[1], 1.0, atol=1e-8)
    @test isapprox(full(gmr.mixture_Act_given_Obs.components[2].Σ)[1], 1.0, atol=1e-8)
    @test probs(gmr.mixture_Obs.prior) == [0.25, 0.75]
    @test isapprox(gmr.mixture_Obs.components[1].μ[1], 0.0, atol=1e-8)
    @test isapprox(gmr.mixture_Obs.components[2].μ[1], 1.0, atol=1e-8)
    @test isapprox(full(gmr.mixture_Obs.components[1].Σ)[1], 1.0, atol=1e-8)
    @test isapprox(full(gmr.mixture_Obs.components[2].Σ)[1], 1.0, atol=1e-8)
end