export
    BehaviorParameter,
    BehaviorTrainDefinition

    # CVFoldResults,
    # ModelCrossValidationResults,
    # CrossValidationResults,

    # cross_validate

    # add_behavior!(behaviorset, GindeleRandomForestBehavior, "Random Forest")
    # model_trainparams["Random Forest"] = GRF_TrainParams(indicators=INDICATOR_SET)
    # model_hyperparams["Random Forest"] = [
    #         BehaviorParameter(:ntrees, 1:10:51, 3),
    #         BehaviorParameter(:max_tree_depth, 1:20, 5),
    #         # BehaviorParameter(:min_samples_split, 10:10:50, 3),
    #         # BehaviorParameter(:min_samples_leaves, [2,4,10,20,50], 3),
    #         # BehaviorParameter(:min_split_improvement, [10.0, 5.0, 1.0,0.5,0.1,0.0], 3),
    #         # BehaviorParameter(:partial_sampling, [0.5,0.6,0.7,0.8,0.9,0.95,1.0], 5),
    #         # BehaviorParameter(:n_split_tries, [10,25,50,100,200,500,1000], 5),
    #     ]

###############################################################

type BehaviorParameter
    sym::Symbol # symbol of the associated field
    range::AbstractVector # set of values that the param can take on
    index_of_default::Int # index of the default value

    function BehaviorParameter(sym::Symbol, range::AbstractVector, index_of_default::Int=1)
        @assert(!isempty(range))
        @assert(index_of_default > 0)
        @assert(index_of_default ≤ length(range))
        new(sym, range, index_of_default)
    end
end

###############################################################

type BehaviorTrainDefinition
    trainparams::AbstractVehicleBehaviorTrainParams # object of train parameters for the model
    hyperparams::Vector{BehaviorParameter} # define the search space over parameters

    function BehaviorTrainDefinition(trainparams::AbstractVehicleBehaviorTrainParams, hyperparams::Vector{BehaviorParameter}=BehaviorParameter[])
        field_names = fieldnames(trainparams)
        for λ in hyperparams
            @assert(in(λ.sym, field_names))
        end
        new(trainparams, hyperparams)
    end
end

###############################################################

# function train( behaviorset::BehaviorSet, training_frames::DataFrame, training_frames_nona::DataFrame )
#
#     retval = Array(AbstractVehicleBehavior, length(behaviorset.behaviors))
#     for (i,b) in enumerate(behaviorset.behaviors)
#         if trains_with_nona(b)
#             retval[i] = train(behaviorset.behaviors[i], training_frames_nona, args=behaviorset.additional_params[i])
#         else
#             retval[i] = train(behaviorset.behaviors[i], training_frames,      args=behaviorset.additional_params[i])
#         end
#     end
#     retval
# end
# function train(
#     behaviorset::BehaviorSet,
#     dset::ModelTrainingData,
#     train_test_split::FoldAssignment,
#     cross_validation_split::FoldAssignment,
#     model_param_sets::Dict{AbstractString, BehaviorParameterSet}; # model name to params
#     max_cv_opt_time_per_model::Float64=60.0 # [sec]
#     )

#     training_frames = dset.dataframe[train_test_split.frame_assignment.==FOLD_TRAIN, :]
#     training_frames_nona = dset.dataframe_nona[train_test_split.frame_assignment.==FOLD_TRAIN, :]

#     retval = Array(AbstractVehicleBehavior, length(behaviorset.behaviors))
#     for i = 1 : length(retval)
#         behavior_type = behaviorset.behaviors[i]
#         behavior_name = behaviorset.names[i]
#         params = model_param_sets[behavior_name]
#         (behavior_train_params, param_indeces) = optimize_hyperparams_cyclic_coordinate_ascent(
#                                                     behavior_type, behavior_name, params,
#                                                     dset, cross_validation_split,
#                                                     max_time=max_cv_opt_time_per_model)

#         if trains_with_nona(behavior_type)
#             retval[i] = train(behavior_type, training_frames_nona, args=behavior_train_params)
#         else
#             retval[i] = train(behavior_type, training_frames,      args=behavior_train_params)
#         end
#     end
#     retval
# end

# function optimize_hyperparams_cyclic_coordinate_ascent{D<:AbstractVehicleBehavior}(
#     behavior_type::Type{D},
#     behavior_name::AbstractString,
#     params::BehaviorParameterSet,
#     dset::ModelTrainingData,
#     cross_validation_split::FoldAssignment;
#     max_iter::Int = 500,
#     max_time::Float64 = 60.0, # [sec]
#     )

#     # returns the optimal param set
#     behavior_train_params = get_default_params(params)

#     n_varying_params = length(params.varying_params)
#     if n_varying_params == 0
#         return (behavior_train_params, Int[])
#     end

#     param_indeces = Array(Int, n_varying_params)
#     for i in 1 : n_varying_params
#         param_indeces[i] = params.varying_params[i].index_of_default
#     end

#     best_param_score = -Inf
#     param_hash = Set{UInt}() # set of param hashes that have already been done
#     metric_types_train_frames = DataType[]
#     metric_types_test_frames = [LoglikelihoodMetric]
#     param_range = 1:n_varying_params

#     behaviorset = BehaviorSet()
#     add_behavior!(behaviorset, behavior_type, behavior_name, behavior_train_params)

#     t_start = time()
#     iteration_count = 0
#     while (time() - t_start < max_time) && (iteration_count < max_iter)
#         iteration_count += 1
#         println("iteration_count: ", iteration_count)

#         # sample new params
#         param_index = rand(param_range)
#         prev_index = param_indeces[param_index]
#         param_indeces[param_index] = rand(1:length(params.varying_params[param_index].range))
#         while in(hash(param_indeces), param_hash) && (time() - t_start < max_time)
#             param_indeces[param_index] = prev_index
#             param_index = rand(param_range)
#             param_indeces[param_index] = rand(1:length(params.varying_params[param_index].range))
#             sleep(0.5)
#         end
#         if in(hash(param_indeces), param_hash)
#             println("timed out")
#             break # we timed out
#         end

#         push!(param_hash, hash(param_indeces))
#         sym = params.varying_params[param_index].sym
#         val = params.varying_params[param_index].range[param_indeces[param_index]]
#         prev_param = behavior_train_params[sym]
#         behavior_train_params[sym] = val
#         cv_res = cross_validate(behaviorset, dset, cross_validation_split,
#                                 metric_types_train_frames, metric_types_test_frames)

#         μ = 0.0
#         n_samples = 0
#         for (fold,cvfold_results) in enumerate(cv_res.models[1].results)
#             m = cvfold_results.metrics_test_frames[1]::LoglikelihoodMetric
#             μ += m.logl
#             n_samples += 1
#         end
#         μ /= n_samples

#         if μ > best_param_score
#             best_param_score = μ
#         else # previous was better
#             behavior_train_params[sym] = prev_param
#             param_indeces[param_index] = prev_index
#         end
#     end

#     (behavior_train_params, param_indeces)
# end
function optimize_hyperparams_cyclic_coordinate_ascent!(
    traindef::BehaviorTrainDefinition,
    dset::ModelTrainingData,
    preallocated_data::AbstractVehicleBehaviorPreallocatedData,
    cross_validation_split::FoldAssignment,
    )

    hyperparams = traindef.hyperparams
    trainparams = traindef.trainparams

    if isempty(hyperparams) # nothing varies
        return traindef
    end

    n_varying_params = length(hyperparams)
    param_indeces = Array(Int, n_varying_params)
    for i in 1 : n_varying_params
        param_indeces[i] = hyperparams[i].index_of_default
    end

    best_logl = -Inf # mean cross-validated log likelihood
    param_hash_set = Set{UInt}() # set of param hashes that have already been done

    t_start = time()
    t_prev_printout = t_start
    iteration_count = 0
    finished = false
    while !finished

        iteration_count += 1
        has_improved = false

        if (time() - t_prev_printout) > 5.0
            println("optimize_hyperparams_cyclic_coordinate_ascent iteration ", iteration_count, "  et: ", time() - t_start)
            t_prev_printout = time()
        end

        for (param_index, behavior_param) in enumerate(hyperparams)

            sym = behavior_param.sym
            best_param_val = getfield(trainparams, sym) # current one

            for (i, v) in enumerate(behavior_param.range)
                param_indeces[param_index] = i

                param_hash = hash(param_indeces)
                if !in(param_hash, param_hash_set)
                    push!(param_hash_set, param_hash)

                    setfield!(trainparams, sym, v)

                    logl = 0.0
                    for fold in 1 : cross_validation_split.nfolds
                        model = train(dset, preallocated_data, trainparams, fold, cross_validation_split, false)
                        logl += extract(LoglikelihoodMetric, dset, model, cross_validation_split, fold, true).logl
                    end
                    logl /= cross_validation_split.nfolds

                    @printf("%3d %3d %6.4f %6.4f\n", param_index, i, logl, best_logl)

                    if logl > best_logl
                        best_logl = logl
                        best_param_val = v
                        has_improved = true
                    end
                end
            end

            # reset to best param val
            setfield!(trainparams, sym, best_param_val)
        end

        finished = !has_improved # terminate if we never improved
    end

    traindef
end

####################################################

# type CVFoldResults
#     metrics_train_frames::Vector{BehaviorFrameMetric}
#     metrics_test_frames::Vector{BehaviorFrameMetric}
#     metrics_train_traces::Vector{BehaviorTraceMetric}
#     metrics_test_traces::Vector{BehaviorTraceMetric}
#
#     CVFoldResults() = new(BehaviorFrameMetric[], BehaviorFrameMetric[], BehaviorTraceMetric[], BehaviorTraceMetric[])
#     function CVFoldResults(
#         train_frames::Vector{BehaviorFrameMetric},
#         test_frames::Vector{BehaviorFrameMetric},
#         train_traces::Vector{BehaviorTraceMetric},
#         test_traces::Vector{BehaviorTraceMetric},
#         )
#
#         new(train_frames, test_frames, train_traces, test_traces)
#     end
# end
# type ModelCrossValidationResults
#     results::Vector{CVFoldResults}
#
#     ModelCrossValidationResults(nfolds::Int) = new(Array(CVFoldResults, nfolds))
# end
# type CrossValidationResults
#     realworld::ModelCrossValidationResults
#     models::Vector{ModelCrossValidationResults}
# end
#
# function cross_validate(
#     behaviorset::BehaviorSet,
#     dset::ModelTrainingData,
#     assignment::FoldAssignment, # assignment for CV Folds
#     metric_types_train_frames::Vector{DataType}, # evaluated only on the training dataframe for each cv component
#     metric_types_test_frames::Vector{DataType}, # evaluated only on the withheld dataframe for each cv component
#     )
#
#     #=
#     For each cross validation fold:
#       - train each model
#       - compute the validation metrics on the witheld dataset
#       - compute the train metrics on the training data
#
#     Returns a CrossValidationResults object
#     =#
#
#     nfolds = assignment.nfolds
#     nmodels = length(behaviorset.behaviors)
#
#     retval_realworld = ModelCrossValidationResults(nfolds)
#     retval_models = [ModelCrossValidationResults(nfolds) for i = 1 : nmodels]
#
#     max_dataframe_training_frames = length(assignment.frame_assignment) - _min_count_of_unique_value(assignment.frame_assignment)
#     dataframe_for_training = similar(dset.dataframe, max_dataframe_training_frames)
#     dataframe_for_training_nona = deepcopy(dataframe_for_training)
#
#     for fold = 1 : nfolds
#
#         _copy_in_fold_frames!(dataframe_for_training, dset.dataframe, assignment, fold, false)
#         _copy_in_fold_frames!(dataframe_for_training_nona, dset.dataframe_nona, assignment, fold, false)
#         behaviors = train(behaviorset, dataframe_for_training, dataframe_for_training_nona)
#
#         for i in 1 : nmodels
#             retval_models[i].results[fold] = CVFoldResults(
#                 extract_metrics(metric_types_train_frames, dset, behaviors[i], assignment, fold, true),
#                 extract_metrics(metric_types_test_frames, dset, behaviors[i], assignment, fold, true),
#                 BehaviorTraceMetric[], BehaviorTraceMetric[])
#         end
#     end
#
#     CrossValidationResults(retval_realworld, retval_models)
# end
# function cross_validate(
#     behaviorset::BehaviorSet,
#     dset::ModelTrainingData,
#     assignment::FoldAssignment, # assignment for CV Folds
#     metric_types_train_frames::Vector{DataType}, # evaluated only on the training dataframe for each cv component
#     metric_types_test_frames::Vector{DataType}, # evaluated only on the withheld dataframe for each cv component
#     metric_types_train_traces::Vector{DataType}, # evaluated only on the training traces for each cv component
#     metric_types_test_traces::Vector{DataType}, # evaluated only on the withheld traces for each cv component
#     )
#
#     #=
#     For each cross validation fold:
#       - train each model
#       - compute the validation metrics on the witheld dataset
#       - compute the train metrics on the training data
#
#     Returns a CrossValidationResults object
#     =#
#
#     pdsets_original = load_pdsets(dset)
#     streetnets = load_streetnets(dset)
#     pdsets_for_simulation = deepcopy(pdsets_original)
#
#     nfolds = assignment.nfolds
#     nmodels = length(behaviorset.behaviors)
#
#     retval_realworld = ModelCrossValidationResults(nfolds)
#     retval_models = [ModelCrossValidationResults(nfolds) for i = 1 : nmodels]
#
#     max_dataframe_training_frames = length(assignment.frame_assignment) - _min_count_of_unique_value(assignment.frame_assignment)
#     dataframe_for_training = similar(dset.dataframe, max_dataframe_training_frames)
#
#     for fold = 1 : nfolds
#
#         _copy_in_fold_frames!(dataframe_for_training, dset.dataframe, assignment, fold, false)
#         behaviors = train(behaviorset, dataframe_for_training)
#
#         all_metrics_train_traces = extract_metrics_from_traces(
#                                     metric_types_train_traces, behaviors,
#                                     pdsets_original, pdsets_for_simulation, streetnets,
#                                     dset.pdset_segments, assignment, fold, false
#                                    )
#         all_metrics_test_traces = extract_metrics_from_traces(
#                                    metric_types_test_traces, behaviors,
#                                    pdsets_original, pdsets_for_simulation, streetnets,
#                                    dset.pdset_segments, assignment, fold, true
#                                   )
#
#         retval_realworld.results[fold] = CVFoldResults(BehaviorFrameMetric[], BehaviorFrameMetric[], all_metrics_train_traces[1], all_metrics_test_traces[1])
#
#         for i in 1 : nmodels
#             metrics_train_frames = extract_metrics(metric_types_train_frames, dset, behaviors[i], assignment, fold, true)
#             metrics_test_frames = extract_metrics(metric_types_test_frames, dset, behaviors[i], assignment, fold, true)
#             metrics_train_traces = all_metrics_train_traces[i+1]
#             metrics_test_traces = all_metrics_test_traces[i+1]
#
#             retval_models[i].results[fold] = CVFoldResults(
#                                                 metrics_train_frames,
#                                                 metrics_test_frames,
#                                                 metrics_train_traces,
#                                                 metrics_test_traces
#                                             )
#
#         end
#
#     end
#
#     CrossValidationResults(retval_realworld, retval_models)
# end
#
# function _get_training_and_validation(
#     fold::Int,
#     pdset_segments::Vector{PdsetSegment},
#     dataframe::DataFrame,
#     frame_assignment::Vector{Int},
#     pdsetseg_fold_assignment::Vector{Int},
#     )
#
#     #=
#     OUTPUT:
#         (training_frames::DataFrame, training_tracesets::Vector{Vector{VehicleTrace}}, validation_tracesets::Vector{Vector{VehicleTrace}})
#         for use in training and then validating a behavior model
#
#     frame_assignment and pdsetseg_fold_assignment should have been created with cross_validation_sets
#     to assign to kfolds.
#
#     Fold must be within [1,kfolds].
#
#     returns training_frames corresponding to all frames in folds that are not fold
#     and validation_tracesets corresponding to fold
#     =#
#
#     @assert(fold > 0)
#
#     training_frame_indeces = falses(length(frame_assignment))
#     for (i,a) in enumerate(frame_assignment)
#         training_frame_indeces[i] = (a!=fold)
#     end
#     training_frames = dataframe[training_frame_indeces, :]
#
#     training_trace_indeces = falses(length(pdsetseg_fold_assignment))
#     for (i,a) in enumerate(pdsetseg_fold_assignment)
#         training_trace_indeces[i] = (a!=fold)
#     end
#     training_pdset_segments = pdset_segments[training_trace_indeces]
#
#     validation_trace_indeces = falses(length(pdsetseg_fold_assignment))
#     for (i,a) in enumerate(pdsetseg_fold_assignment)
#         validation_trace_indeces[i] = (a==fold)
#     end
#     validation_pdset_segments = pdset_segments[validation_trace_indeces]
#
#     (training_frames, training_pdset_segments, validation_pdset_segments)
# end
#
# function _max_count_of_unique_value(arr::AbstractArray)
#     dict = Dict{Any,Int}()
#     for v in arr
#         dict[v] = get(dict, v, 0) + 1
#     end
#     maximum(values(dict))::Int
# end
# function _min_count_of_unique_value(arr::AbstractArray)
#     dict = Dict{Any,Int}()
#     for v in arr
#         dict[v] = get(dict, v, 0) + 1
#     end
#     minimum(values(dict))::Int
# end
#
# function _copy_in_fold_frames!(
#     dest::DataFrame,
#     src::DataFrame,
#     assignment::FoldAssignment,
#     fold::Int,
#     match_fold::Bool
#     )
#
#     # NOTE(tim): assumes dest is preallocated with enough rows
#
#     nrow_dest = nrow(dest)
#
#     framecount = 0
#     for (i,assignment) in enumerate(assignment.frame_assignment)
#         if is_in_fold(fold, assignment, match_fold)
#             framecount += 1
#             for j = 1 : size(src, 2)
#                 dest[framecount, j] = src[i, j]
#             end
#         end
#     end
#     while framecount < nrow_dest
#
#         # now we need to fill in any extra training frames so that the entire thing is populated
#         # If properly used this will only be one extra row so the overall effect will be negligible
#
#         row = rand(1:framecount) # choose something at random
#         framecount += 1
#         for j = 1 : size(src, 2)
#             dest[framecount, j] = src[row, j]
#         end
#     end
#
#     dest
# end
