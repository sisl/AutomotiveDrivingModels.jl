export
    GMRTrainParams,
    calc_bic_score,
    train

# import GaussianMixtures
using PyCall
const SKLEARN_MIXTURE = PyCall.PyNULL()
function __init__()
    copy!(SKLEARN_MIXTURE, pyimport("sklearn.mixture"))
end

type GMRTrainParams{A<:DriveAction}
    extractor::AbstractFeatureExtractor
    context::IntegratedContinuous

    max_n_indicators::Int
    max_n_train_examples::Int

    n_components::Int
    random_state::Int
    n_iter::Int
    n_init::Int
    min_covar::Float64
    tol::Float64
    verbose::Int

    unlearned_component::MvNormal
    unlearned_component_weight::Float64

    prime_history::Int
end
function GMRTrainParams{A<:DriveAction}(::Type{A},
    extractor::AbstractFeatureExtractor,
    context::IntegratedContinuous;
    max_n_indicators::Int = 3,
    max_n_train_examples::Int = typemax(Int),
    n_components::Int = 2,
    random_state::Int = 1,
    n_init::Int = 3,
    n_iter::Int = 100,
    min_covar::Float64 = 1e-6,
    tol::Float64 = 1e-3,
    verbose::Int = 0,
    prime_history::Int = 0,
    unlearned_component_weight::Float64 = NaN,
    unlearned_component::MvNormal = MvNormal(eye(length(A))),
    )

    GMRTrainParams{A}(extractor, context, max_n_indicators, max_n_train_examples,
                      n_components, random_state, n_iter, n_init, min_covar, tol, verbose,
                      unlearned_component, unlearned_component_weight, prime_history)
end

# GMR(gmm::GaussianMixtures.GMM, n_targets::Int=2) = GMR(GaussianMixtures.MixtureModel(gmm), n_targets)
function GMR(gmm::PyObject, n_targets::Int=2)

    println(gmm[:converged_])

    if gmm[:converged_]

        weights = deepcopy(gmm[:weights_]::Vector{Float64})
        means = deepcopy(gmm[:means_]::Matrix{Float64})     # [n_components, n_features]
        covars = deepcopy(gmm[:covars_]::Array{Float64, 3}) # shape depends on covariance_type
        @assert(gmm[:covariance_type] == "full")

        n_components = length(weights)
        n_indicators = size(means, 2) - n_targets

        vec_A = Array(Matrix{Float64}, n_components) # μ₁₋₂ = μ₁ + Σ₁₂ * Σ₂₂⁻¹ * (x₂ - μ₂) = A*x₂ + b
        vec_b = Array(Vector{Float64}, n_components)
        vec_G = Array(MvNormal, n_components)
        vec_H = Array(MvNormal, n_components)

        for i = 1 : n_components

            μₐ = vec(means[i,1:n_targets])
            μₚ = vec(means[i,n_targets+1:end])

            Σ = reshape(covars[i, :, :], (n_targets + n_indicators,n_targets + n_indicators))

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
    else
        GMR(Matrix{Float64}[], Vector{Float64}[], MvNormal[], MvNormal[])
    end
end
function GMR(gmm::PyObject, unlearned_component::MvNormal, unlearned_component_weight::Float64)

    println(gmm[:converged_])

    if gmm[:converged_]

        weights = deepcopy(gmm[:weights_]::Vector{Float64})
        means = deepcopy(gmm[:means_]::Matrix{Float64})     # [n_components, n_features]
        covars = deepcopy(gmm[:covars_]::Array{Float64, 3}) # shape depends on covariance_type
        @assert(gmm[:covariance_type] == "full")

        n_components = length(weights)
        n_targets = length(unlearned_component)
        n_indicators = size(means, 2) - n_targets

        vec_A = Array(Matrix{Float64}, n_components+1) # μ₁₋₂ = μ₁ + Σ₁₂ * Σ₂₂⁻¹ * (x₂ - μ₂) = A*x₂ + b
        vec_b = Array(Vector{Float64}, n_components+1)
        vec_G = Array(MvNormal, n_components+1)
        vec_H = Array(MvNormal, n_components+1)

        for i = 1 : n_components

            μₐ = vec(means[i,1:n_targets])
            μₚ = vec(means[i,n_targets+1:end])

            Σ = reshape(covars[i, :, :], (n_targets + n_indicators,n_targets + n_indicators))

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

        # unlearned component
        vec_A[end] = zeros(Float64, n_targets, n_indicators)
        vec_b[end] = deepcopy(unlearned_component.μ)
        vec_G[end] = unlearned_component

        mixture_Act_given_Obs = MixtureModel(vec_G) # p(action|obs), mean and weighting must be updated with each observation, cov is pre-computed
        mixture_Obs = MixtureModel(vec_H, weights) # p(obs), all pre-computed, should never be edited
        GMR(vec_A, vec_b, mixture_Obs, mixture_Act_given_Obs)
    else
        GMR(Matrix{Float64}[], Vector{Float64}[], MvNormal[], MvNormal[])
    end
end

function calc_bic_score(gmr::GMR, YX::Matrix{Float64}, chosen_indicators::Vector{Int})

    bic = -Inf

    action_len = length(gmr.mixture_Act_given_Obs)
    a = Array(Float64, action_len)
    f = Array(Float64, length(chosen_indicators))

    m = size(YX, 1)
    logl = 0.0
    for i in 1 : m

        for j in 1 : action_len
            a[j] = YX[i,j]
        end
        for (j,p) in enumerate(chosen_indicators)
            f[j] = YX[i, action_len+p]
        end

        logl += logpdf(gmr(f), a)
    end


    logl - log(m)*nsuffstats(gmr)/2
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

function train{A}(
    data::EvaluationData,
    params::GMRTrainParams{A},
    foldset::FoldSet,
    )

    action_len = length(A)
    n_indicators = length(params.extractor)

    rowcount = 0
    for i in foldset
        seg = data.segments[i]
        rowcount += nsteps(seg) - params.prime_history
    end

    println("rowcount: ", rowcount)

    rowcount = min(rowcount, params.max_n_train_examples)

    # YX[i,j] has is ith sample of jth feature
    YX = Array(Float64, rowcount, action_len + n_indicators)
    means = Array(Float64, n_indicators)
    stdevs = Array(Float64, n_indicators)
    action = Array(Float64, action_len)
    features = Array(Float64, n_indicators)

    let
        println("extracting features"); tic()

        row = 0
        for i in foldset
            seg = data.segments[i]
            rec = pull_record(seg, data)
            for pastframe in -length(rec) + params.prime_history + 1 : 0
                row += 1

                trajdata = data.trajdatas[seg.trajdata_index]
                roadway = trajdata.roadway
                vehicle_index = get_index_of_first_vehicle_with_id(rec, seg.egoid, pastframe)
                pull_action!(A, action, rec, roadway, vehicle_index, pastframe)
                for j in 1 : action_len
                    YX[row, j] = action[j]
                end

                pull_features!(params.extractor, features, rec, roadway, vehicle_index, pastframe)
                for j in 1 : n_indicators
                    YX[row, j+action_len] = features[j]
                end

                if row == rowcount
                    break
                end
            end

            if row == rowcount
                break
            end
        end

        toc()

        @assert row == size(YX, 1)

        # standardize
        println("standardize"); tic()
        for j in 1 : n_indicators
            means[j] = mean(YX[:,j+action_len])
            stdevs[j] = stdm(YX[:,j+action_len], means[j])

            if stdevs[j] == 0.0 || isnan(stdevs[j]) || isinf(stdevs[j])
                warn("stdev was bad for column $j")
                for i in 1:rowcount
                    YX[i,j+action_len] = 0.0
                end
            else
                for i in 1:rowcount
                    YX[i,j+action_len] = (YX[i,j+action_len] - means[j]) / stdevs[j]
                end
            end
        end
        toc()
    end

    @assert(findfirst(v->isnan(v) || isinf(v), YX) == 0)

    # -----------------------------------------
    # run greedy ascent
    #  - always add the next best parent

    println("fitting first one"); tic()

    max_n_indicators = min(params.max_n_indicators, n_indicators)
    chosen_indicators = Int[] # start with no parents
    columns = collect(1:action_len)

    gmm = SKLEARN_MIXTURE[:GMM](
            n_components = params.n_components, # int, optional. Number of mixture components. Defaults to 1.
            covariance_type = "full", # string, optional. ‘spherical’, ‘tied’, ‘diag’, ‘full’. Defaults to ‘diag’
            random_state    = params.random_state, # RandomState or an int seed (None by default)
            min_covar       = params.min_covar, # float, optional
            n_iter          = params.n_iter,
            n_init          = params.n_init,
            tol             = params.tol,
            verbose         = params.verbose
        )

    gmm[:fit](YX[:,columns])
    best_model = GMR(gmm, params.unlearned_component, params.unlearned_component_weight)

    # best_model = GMR(GaussianMixtures.GMM(params.n_components, deepcopy(YX[:,columns]),
    #                   method = params.method,
    #                   kind = :full,
    #                   nInit = params.nInit,
    #                   nIter = params.nIter,
    #                 ))

    best_score = calc_bic_score(best_model, YX, chosen_indicators)
    println("best_score: ", best_score)

    toc()

    finished = false
    while !finished && length(chosen_indicators) < max_n_indicators

        push!(chosen_indicators, 0)
        push!(columns, 0)

        println("trying n_indicators: ", length(chosen_indicators))
        best_indicator = 0

        for i in 1:n_indicators
            if i ∉ chosen_indicators

                chosen_indicators[end] = i
                columns[end] = i + action_len

                try
                    # test_model = GMR(GaussianMixtures.GMM(params.n_components, deepcopy(YX[:,columns]),
                    #                   method = params.method,
                    #                   kind = :full,
                    #                   nInit = params.nInit,
                    #                   nIter = params.nIter,
                    #                 ))
                    gmm[:fit](YX[:,columns])
                    test_model = GMR(gmm)
                    test_score = calc_bic_score(test_model, YX, chosen_indicators)
                    println(chosen_indicators, " -> bic ", test_score)

                    if test_score > best_score
                        println("better than ", best_score)
                        best_score = test_score
                        best_model = test_model
                        best_indicator = i
                    end
                catch
                    warn("FAILED to train with $chosen_indicators")
                end
            end
        end

        chosen_indicators[end] = best_indicator
        columns[end] = best_indicator + action_len
        if best_indicator == 0
            pop!(chosen_indicators)
            pop!(columns)
            finished = true
        end
    end

    println("extractors"); tic()

    subset_extractor = SubsetExtractor(params.extractor, chosen_indicators)
    standardizing_extractor = StandardizingExtractor(subset_extractor, means[chosen_indicators], stdevs[chosen_indicators])

    toc()

    GaussianMixtureRegressionDriver{A, typeof(standardizing_extractor)}(
        SceneRecord(rec_length(standardizing_extractor), params.context.Δt),
        params.context, best_model, standardizing_extractor,
        Array(Float64, length(standardizing_extractor)),
        Array(Float64, action_len)
    )
end