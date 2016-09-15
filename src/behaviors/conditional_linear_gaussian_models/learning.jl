type CLBNTrainParams{A<:DriveAction}
    max_n_parents::Int
    max_n_train_examples::Int
    prime_history::Int
    prior::ConjugatePriors.MvNormalInverseGamma
    features::Vector{AbstractFeature}
    context::IntegratedContinuous
    verbose::Int
end
function CLBNTrainParams{A<:DriveAction}(::Type{A},
    features::Vector{AbstractFeature},
    context::IntegratedContinuous;
    max_n_parents::Int = 5,
    max_n_train_examples::Int = typemax(Int),
    prime_history::Int = 0,
    prior::ConjugatePriors.MvNormalInverseGamma=ConjugatePriors.MvNormalInverseGamma(zeros(length(features)+1), eye(length(features)+1), 1.0, 1.0),
    verbose::Int = 0,
    )

    CLBNTrainParams{A}(max_n_parents, max_n_train_examples, prime_history, prior, features, context, verbose)
end

# function calc_bic_score(gmr::GMR, YX::Matrix{Float64}, chosen_indicators::Vector{Int})

#     bic = -Inf

#     action_len = length(gmr.mixture_Act_given_Obs)
#     a = Array(Float64, action_len)
#     f = Array(Float64, length(chosen_indicators))

#     m = size(YX, 1)
#     logl = 0.0
#     for i in 1 : m

#         for j in 1 : action_len
#             a[j] = YX[i,j]
#         end
#         for (j,p) in enumerate(chosen_indicators)
#             f[j] = YX[i, action_len+p]
#         end

#         logl += logpdf(gmr(f), a)
#     end


#     logl - log(m)*nsuffstats(gmr)/2
# end

function _pull_training_dataframe{A}(
    data::EvaluationData,
    params::CLBNTrainParams{A},
    foldset::FoldSet,
    )

    rowcount = 0
    for i in foldset
        seg = data.segments[i]
        rowcount += nsteps(seg) - params.prime_history
    end
    rowcount = min(rowcount, params.max_n_train_examples)

    targets = [ACC, TURNRATEG] # TODO: make general by having a get_features(A) or something
    dataframe = DataFrame()
    for target in targets
        dataframe[symbol(target)] = Array(Float64, rowcount)
    end
    for feature in params.features
        dataframe[symbol(feature)] = Array(inherent_type(feature), rowcount)
    end

    let
        println("extracting features"); tic()
        row = 0
        for i in foldset
            seg = data.segments[i]
            rec = pull_record(seg, data)
            for pastframe in -length(rec) + params.prime_history + 1 : -1
                row += 1

                trajdata = data.trajdatas[seg.trajdata_index]
                roadway = trajdata.roadway
                vehicle_index = get_index_of_first_vehicle_with_id(rec, seg.egoid, pastframe)

                for target in targets
                    v = convert(Float64, get(target, rec, roadway, vehicle_index, pastframe+1))
                    @assert(!isnan(v) && !isinf(v))
                    dataframe[row, symbol(target)] = v
                end
                for feature in params.features
                    fval = get(feature, rec, roadway, vehicle_index, pastframe)
                    if is_feature_valid(fval)
                        if inherent_type(feature) <: AbstractFloat
                            dataframe[row, symbol(feature)] = convert(Float64, fval)
                        else
                            dataframe[row, symbol(feature)] = convert(Int, convert(Float64, fval))
                        end
                    else
                        dataframe[row, symbol(feature)] = NaN
                    end

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

        @assert row == nrow(dataframe)
    end

    dataframe
end

function train{A}(
    data::EvaluationData,
    params::CLBNTrainParams{A},
    foldset::FoldSet;
    dataframe = _pull_training_dataframe(data, params, foldset)
    )

    action_len = length(A)
    n_indicators = length(params.features)
    targets = [ACC, TURNRATEG] # TODO - rm

    # -----------------------------------------
    # run greedy ascent
    #  - always add the next best parent

    println("fitting first one"); tic()



    Distributions.fit(NannableConditionalLinearGaussianCPD, dataframe,
    data::DataFrame,
    target::NodeName,
    parents::Vector{NodeName},
    prior::ConjugatePriors.MvNormalInverseGamma;
    parents_nannable = parents[find(c->eltype(data[c])<:Float64 && any(v->isnan(v),data[c]), 1:ncol(data))],
    )

    max_n_indicators = min(params.max_n_indicators, n_indicators)
    chosen_indicators = Int[] # start with no parents
    columns = collect(1:action_len)
    best_model = GMR(GaussianMixtures.GMM(params.n_components, deepcopy(YX[:,columns]),
                      method = params.method,
                      kind = :full,
                      nInit = params.nInit,
                      nIter = params.nIter,
                    ))
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
                    test_model = GMR(GaussianMixtures.GMM(params.n_components, deepcopy(YX[:,columns]),
                                      method = params.method,
                                      kind = :full,
                                      nInit = params.nInit,
                                      nIter = params.nIter,
                                    ))
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