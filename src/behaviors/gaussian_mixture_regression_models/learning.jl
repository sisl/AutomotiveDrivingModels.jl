export
    GMRTrainParams,
    calc_bic_score,
    train

import GaussianMixtures

type GMRTrainParams{A<:DriveAction}
    max_n_indicators::Int
    max_n_train_examples::Int
    n_components::Int
    prime_history::Int
    method::Symbol
    nInit::Int
    nIter::Int
    extractor::AbstractFeatureExtractor
    context::IntegratedContinuous
end
function GMRTrainParams{A<:DriveAction}(::Type{A},
    extractor::AbstractFeatureExtractor,
    context::IntegratedContinuous;
    max_n_indicators::Int = 5,
    max_n_train_examples::Int = typemax(Int),
    n_components::Int = 3,
    prime_history::Int = 0,
    method::Symbol = :kmeans,
    nInit::Int = 50,
    nIter::Int = 10,
    )

    GMRTrainParams{A}(max_n_indicators, max_n_train_examples, n_components, prime_history,
        method, nInit, nIter, extractor, context)
end

GMR(gmm::GaussianMixtures.GMM, n_targets::Int=2) = GMR(GaussianMixtures.MixtureModel(gmm), n_targets)
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
            for i in 1:rowcount
                YX[i,j+action_len] = (YX[i,j+action_len] - means[j]) / stdevs[j]
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
    best_model = GMR(GaussianMixtures.GMM(params.n_components, YX[:,1:action_len],
                      method = params.method,
                      kind = :full,
                      nInit = params.nInit,
                      nIter = params.nIter,
                    ))
    best_score = calc_bic_score(best_model, YX, chosen_indicators)

    toc()

    finished = false
    while !finished && length(chosen_indicators) < max_n_indicators

        finished = true
        push!(chosen_indicators, 0)

        println("tring n_indicators: ", length(chosen_indicators))
        best_indicator = 0

        for i in 1:n_indicators
            if i ∉ chosen_indicators
                chosen_indicators[end] = i

                columns = append!(collect(1:action_len), chosen_indicators .+ action_len)
                test_score = -Inf

                try
                    test_model = GMR(GaussianMixtures.GMM(params.n_components, YX[:,columns],
                                      method = params.method,
                                      kind = :full,
                                      nInit = params.nInit,
                                      nIter = params.nIter,
                                    ))
                    test_score = calc_bic_score(test_model, YX, chosen_indicators)

                    if test_score > best_score
                        best_score = test_score
                        best_model = test_model
                        best_indicator = i
                        finished = false
                    end
                catch
                    warn("FAILED to train")
                end
            end
        end

        if best_indicator == 0
            pop!(chosen_indicators)
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