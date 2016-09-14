export
    GMR,

    n_learned_components,
    nsuffstats

type GMR{M<:MvNormal}

    # μ₁₋₂ = μ₁ + Σ₁₂ * Σ₂₂⁻¹ * (x₂ - μ₂) = A*x₂ + b
    vec_A::Vector{Matrix{Float64}} # [n_components [ntargets×nindicators]]
    vec_b::Vector{Vector{Float64}} # [n_components [ntargets]]

    # pdf(p), all pre-computed. Used to compute βⱼ(p)
    mixture_Obs::MixtureModel{Multivariate,Continuous,M} # p(obs), all pre-computed, should never be edited

    # pdf(a|p), means μⱼ_ₚ and weights βⱼ(p) are functions of p and must be updated every time, covariance is constant
    mixture_Act_given_Obs::MixtureModel{Multivariate,Continuous,M}
end
function GMR{M<:MvNormal}(mix::MixtureModel{Multivariate,Continuous,M}, n_targets::Int=2)

    #=
    Construct a Gaussian Mixture Regressor using a Gaussian mixture over both
    the features and the actions, and the features are at the end of the input
    =#

    weights = probs(mix.prior) # [n_components]
    n_components = length(weights)
    n_indicators = length(mix) - n_targets

    vec_A = Array(Matrix{Float64}, n_components) # μ₁₋₂ = μ₁ + Σ₁₂ * Σ₂₂⁻¹ * (x₂ - μ₂) = A*x₂ + b
    vec_b = Array(Vector{Float64}, n_components)
    vec_G = Array(MvNormal, n_components)
    vec_H = Array(MvNormal, n_components)

    for i = 1 : n_components

        μ = mix.components[i].μ
        μₐ = μ[1:n_targets]
        μₚ = μ[n_targets+1:end]

        Σ = full(mix.components[i].Σ)
        Σₐₐ = Σ[1:n_targets,1:n_targets]
        Σₐₚ = Σ[1:n_targets,n_targets+1:end]
        Σₚₚ = nearestSPD(Σ[n_targets+1:end,n_targets+1:end])
        iΣₚₚ = inv(Σₚₚ)

        A = Σₐₚ * iΣₚₚ
        vec_A[i] = A
        vec_b[i] = vec(μₐ - A*μₚ)
        C = nearestSPD(Σₐₐ - Σₐₚ * iΣₚₚ * ((Σₐₚ)'))

        vec_G[i] = MvNormal(Array(Float64, n_targets), C) # p(action|obs), mean and weighting must be updated with each observation, cov is pre-computed
        vec_H[i] = MvNormal(μₚ, Σₚₚ) # p(obs), all pre-computed, should never be edited
    end

    mixture_Act_given_Obs = MixtureModel(vec_G) # p(action|obs), mean and weighting must be updated with each observation, cov is pre-computed
    mixture_Obs = MixtureModel(vec_H, weights) # p(obs), all pre-computed, should never be edited
    GMR(vec_A, vec_b, mixture_Obs, mixture_Act_given_Obs)
end

function Base.print(model::GMR)
    println("GMR:")
    for (i, mat) in enumerate(model.vec_A)
        println(i)
        print("\t[")
        for j in 1:size(mat,2)
            @printf("  %10.6f", mat[1,j])
        end
        @printf("] + [  %10.6f]\n", model.vec_b[i][1])
        print("\t[")
        for j in 1:size(mat,2)
            @printf("  %10.6f", mat[2,j])
        end
        @printf("] + [  %10.6f]\n", model.vec_b[i][2])
    end
    println("\tmixture_Obs: ")
    println("\t\tprior: ", model.mixture_Obs.prior)
end

n_targets(gmr::GMR) = size(gmr.vec_A[1], 1)
n_features(gmr::GMR) = size(gmr.vec_A[1], 2)
n_components(gmr::GMR) = length(gmr.vec_A)
function nsuffstats(gmr::GMR)
    dimA = length(gmr.vec_A[1])
    n_components(gmr) * (2*dimA + 2 # bias
                                  + 3 # covariance in mixture_Act_given_Obs
                                  + div(dimA*dimA,2)) # covariance for mixture_Obs
end
function nearestSPD(A::Matrix{Float64})

    # see http://www.mathworks.com/matlabcentral/fileexchange/42885-nearestspd

    # output:
    #  α, β ≥ 0.0 such that
    #         α ≤ δ₂(A) ≤ β ≤ α + 2 max(fα, tol)
    #  and a PSD matrix X such that |A - X|₂ = β

    n = size(A, 1)
    @assert(n == size(A, 2)) # ensure it is square

    I = eye(n)

    # symmetrize A into B
    B = (A+A')./2

    # Compute the symmetric polar factor of B. Call it H.
    # Clearly H is itself SPD.
    U, σ, V = svd(B)
    H = V*diagm(σ)*V'

    # get Ahat in the above formula
    Ahat = (B+H)/2

    # ensure symmetry
    Ahat = (Ahat + Ahat')/2;

    # test that Ahat is in fact PD. if it is not so, then tweak it just a bit.
    worked = false
    iteration_count = 0
    while !worked && iteration_count < 100

        iteration_count += 1

        try
            chol(Ahat)
            worked = true
        catch
            # do nothing
        end

        if !worked
            # Ahat failed the chol test. It must have been just a hair off,
            # due to floating point trash, so it is simplest now just to
            # tweak by adding a tiny multiple of an identity matrix.

            min_eig = minimum(eigvals(Ahat))
            Ahat = Ahat + (-min_eig*iteration_count.^2 + eps(Float32))*I
        end
    end

    Ahat
end

@compat function (gmr::GMR)(features::Vector{Float64})

    mixture_Act_given_Obs = gmr.mixture_Act_given_Obs
    mixture_Obs = gmr.mixture_Obs
    nc = n_components(gmr)

    for j in 1 : nc

        # compute the β value, unweighted
        # βⱼ(f) ∝ wⱼ Nⱼ(μₚ, Σₚ)
        wⱼ = mixture_Obs.prior.p[j]
        Nⱼ = mixture_Obs.components[j]
        mixture_Act_given_Obs.prior.p[j] = wⱼ * pdf(Nⱼ, features)

        # compute the conditional mean
        #  μₐ_ₚ = A⋅f + b
        A = gmr.vec_A[j]
        b = gmr.vec_b[j]
        copy!(mixture_Act_given_Obs.components[j].μ, A*features + b)
    end

    # normalize the β values
    sum_β = sum(mixture_Act_given_Obs.prior.p)

    if sum_β > 0.0 && !isnan(sum_β) && !isinf(sum_β)
        for i in 1 : nc
            mixture_Act_given_Obs.prior.p[i] /= sum_β
        end
    else
        fill!(mixture_Act_given_Obs.prior.p, 1/nc) # set all to equal weight
        for i in 1 : nc
            fill!(mixture_Act_given_Obs.components[i].μ, 0.0) # set mean to zero
        end
    end

    mixture_Act_given_Obs
end
