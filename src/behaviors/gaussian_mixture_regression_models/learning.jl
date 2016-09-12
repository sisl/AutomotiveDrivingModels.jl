
using GaussianMixtures

type GMRTrainParams{A<:DriveAction}
    max_n_indicators::Int,
    n_components::Int,
    method::Symbol # = :kmeans
    nInit::Int # = 50
    nIter::Int # = 10
    extractor::AbstractFeatureExtractor
    context::IntegratedContinuous
end

GMR(gmm::GMM, n_targets::Int=2) = GMR(convert(MixtureModel, gmm), n_targets)
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

function train{A}(
    data::EvaluationData,
    params::GMRTrainParams{A},
    foldset::FoldSet,
    )

    action_len = length(A)

    YX # TODO YX[i,j] has is ith sample of jth feature

    means = ...
    sigmas = ...

    # -----------------------------------------
    # run greedy ascent
    #  - always add the next best parent

    n_indicators = length(params.extractor)
    max_n_indicators = min(params.max_n_indicators, n_indicators)
    chosen_indicators = Int[] # start with no parents
    best_model = GMR(GMM(params.n_components, YX[:,1:action_len],
                      method = params.method,
                      kind = :full,
                      nInit = params.nInit,
                      nIter = params.nIter,
                    ))
    best_score = calc_bic_score(best_model, YX, chosen_indicators)

    # finished = false
    # while !finished && length(chosen_indicators) < max_n_indicators

    #     finished = true
    #     push!(chosen_indicators, 0)

    #     for i in 1:n_indicators
    #         if i ∉ chosen_indicators
    #             chosen_indicators[end] = i

    #             columns = append!(collect(1:action_len), chosen_indicators .+ action_len)
    #             test_model = GMR(GMM(params.n_components, YX[:,columns],
    #                               method = params.method,
    #                               kind = :full,
    #                               nInit = params.nInit,
    #                               nIter = params.nIter,
    #                             ))
    #             test_score = calc_bic_score(test_model, YX, chosen_indicators)

    #             if test_score > best_score
    #                 best_score = test_score
    #                 best_model = test_model
    #                 finished = false
    #             else
    #                 chosen_indicators[end] = 0
    #             end
    #         end
    #     end

    #     if chosen_indicators[end] == 0
    #         pop!(chosen_indicators)
    #     end
    # end

    subset_extractor = SubsetExtractor(params.extractor, chosen_indicators)
    standardizing_extractor = StandardizingExtractor(subset_extractor, means[chosen_indicators], sigmas[chosen_indicators])

    GaussianMixtureRegressionDriver{A, typeof(standardizing_extractor)}(
        SceneRecord(rec_length(standardizing_extractor), params.context.Δt),
        params.context, best_model, standardizing_extractor,
        Array(Float64, length(standardizing_extractor)),
        Array(Float64, action_len)
    )
end