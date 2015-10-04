export
    BehaviorSet,

    add_behavior!,
    cross_validate,
    cross_validate_logl,

    create_metrics_sets,
    create_metrics_sets_no_tracemetrics

type BehaviorSet
    behaviors::Vector{Type}
    names::Vector{String}
    additional_params::Vector{Dict}

    BehaviorSet() = new(Type{AbstractVehicleBehavior}[], String[], Dict[])
end

function add_behavior!{B<:AbstractVehicleBehavior}(
    behaviorset::BehaviorSet,
    behavior::Type{B},
    name::String,
    additional_params::Dict = Dict{Symbol,Any}()
    )

    @assert(!in(name, behaviorset.names))

    push!(behaviorset.behaviors, behavior)
    push!(behaviorset.names, name)
    push!(behaviorset.additional_params, additional_params)

    behaviorset
end

function train( behaviorset::BehaviorSet, training_frames::DataFrame )

    retval = Array(AbstractVehicleBehavior, length(behaviorset.behaviors))
    for i = 1 : length(retval)
        retval[i] = train(behaviorset.behaviors[i], training_frames, args=behaviorset.additional_params[i])
    end
    retval
end

####################################################

type CVFoldResults
    metrics_train::Vector{BehaviorMetric}
    metrics_validation::Vector{BehaviorMetric}

    CVFoldResults() = new(BehaviorMetric[], BehaviorMetric[])
    CVFoldResults(train::Vector{BehaviorMetric}, validate::Vector{BehaviorMetric}) = new(train, validate)
end
type ModelCrossValidationResults
    results::Vector{CVFoldResults}

    ModelCrossValidationResults(nfolds::Int) = new(Array(CVFoldResults, nfolds))
end
type CrossValidationResults
    realworld::ModelCrossValidationResults
    models::Vector{ModelCrossValidationResults}
end

function cross_validate{DV<:DataType, DT<:DataType}(
    behaviorset::BehaviorSet,
    dset::ModelTrainingData,
    assignment::FoldAssignment,
    metrics_validation::Vector{DV},
    metrics_train::Vector{DT},
    )

    #=
    For each cross validation fold:
      - train each model
      - compute the validation metrics on the witheld dataset
      - compute the train metrics on the training data

    Returns a CrossValidationResults object
    =#

    nfolds = assignment.nfolds
    nmodels = length(behaviorset.behaviors)

    retval_realworld = CrossValidationResults(nfolds)
    retval_models = [CrossValidationResults(nfolds) for i = 1 : nmodels]

    max_dataframe_training_frames = length(assignment.frame_assignment) - _min_count_of_unique_value(assignment.frame_assignment)
    dataframe_for_training = similar(dset.dataframe, max_dataframe_training_frames)

    for fold = 1 : nfolds

        _copy_in_fold_frames!(dataframe_for_training, dset.dataframe, assignment, fold, false)
        behaviors = train(behaviorset, dataframe_for_training)

        for i in 1 : nmodels
            retval_models[i].results[fold] = CVFoldResults(
                extract_metrics(metrics_train, dset, behaviors[i], assignment, fold, true),
                extract_metrics(metrics_validation, dset, behaviors[i], assignment, fold, true))
            end
        end
    end

    CrossValidationResults(retval_realworld, retval_models)
end

function _cross_validate(
    behaviorset::BehaviorSet,
    pdsets::Vector{PrimaryDataset},
    streetnets::Vector{StreetNetwork},
    pdset_segments::Vector{PdsetSegment},
    dataframe::DataFrame,
    frame_assignment::Vector{Int},
    pdsetseg_fold_assignment::Vector{Int},
    evalparams::EvaluationParams;
    verbosity::Int=1,
    incremental_model_save_file::Union(Nothing, String)=nothing
    )

    nfolds = maximum(frame_assignment)
    nbehaviors = length(behaviorset.behaviors)

    aggregate_metric_sets_training = Array(Vector{Dict{Symbol,Any}}, nbehaviors+1)
    for i = 1:length(aggregate_metric_sets_training)
        aggregate_metric_sets_training[i] = Array(Dict{Symbol,Any}, nfolds)
    end

    aggregate_metric_sets_validation = Array(Vector{Dict{Symbol,Any}}, nbehaviors+1)
    for i = 1:length(aggregate_metric_sets_validation)
        aggregate_metric_sets_validation[i] = Array(Dict{Symbol,Any}, nfolds)
    end


    max_dataframe_training_frames = length(frame_assignment) - _min_count_of_unique_value(frame_assignment)
    dataframe_for_training = similar(dataframe, max_dataframe_training_frames)
    pdsets_for_simulation = deepcopy(pdsets) # these will be overwritten for the simulation

    # ------------------

    for fold = 1 : nfolds

        if verbosity > 0
            println("FOLD ", fold, " / ", nfolds)
            tic()
        end

        # copy the training frames into dataframe_for_training
        framecount = 0
        for (i,assignment) in enumerate(frame_assignment)
            if is_in_fold(fold, assignment, false)
                framecount += 1
                for j = 1 : size(dataframe, 2)
                    dataframe_for_training[framecount, j] = dataframe[i, j]
                end
            end
        end
        while framecount < max_dataframe_training_frames

            # now we need to fill in any extra training frames so that the entire thing is populated
            # If properly used this will only be one extra row so the overall effect will be negligible

            row = rand(1:framecount)
            framecount += 1
            for j = 1 : size(dataframe, 2)
                dataframe_for_training[framecount, j] = dataframe_for_training[row, j]
            end
        end

        model_behaviors = train(behaviorset, dataframe_for_training)

        println("done training")

        print("computing validation metrics sets"); tic()
        metrics_sets_validation = create_metrics_sets(model_behaviors, pdsets, pdsets_for_simulation,
                                                            streetnets, pdset_segments, evalparams,
                                                            fold, pdsetseg_fold_assignment, true)
        for (i,metrics_set) in enumerate(metrics_sets_validation)
            aggregate_metric_sets_validation[i][fold] = metrics_set.aggmetrics
            metrics_set.tracemetrics = Dict{Symbol, Any}[] # delete it so we don't save it later
        end
        print(" [DONE] "); toc()


        if isa(incremental_model_save_file, String)

            print("computing training metrics sets"); tic()
            metrics_sets_training = create_metrics_sets(model_behaviors, pdsets, pdsets_for_simulation,
                                                              streetnets, pdset_segments, evalparams,
                                                              fold, pdsetseg_fold_assignment, false)
            for (i,metrics_set) in enumerate(metrics_sets_training)
                aggregate_metric_sets_training[i][fold] = metrics_set.aggmetrics
                metrics_set.tracemetrics = Dict{Symbol, Any}[] # delete it so we don't save it later
            end
            print(" [DONE] "); toc()

            println("saving incremental file"); tic()
            # NOTE: "dir/file.jld" → "dir/file_<fold>.jld"
            filename = splitext(incremental_model_save_file)[1] * @sprintf("_%02d.jld", fold)
            JLD.save(filename,
                "behaviorset", behaviorset,
                "models", model_behaviors,
                "metrics_sets_training", metrics_sets_training,
                "metrics_sets_validation", metrics_sets_validation)

            print(" [DONE] "); toc()
        end

        if verbosity > 0
            toc()
        end
    end

    cross_validation_metrics = Array(Dict{Symbol,Any}, length(aggregate_metric_sets_validation))
    for (i,aggs) in enumerate(aggregate_metric_sets_validation)
        cross_validation_metrics[i] = calc_mean_cross_validation_metrics(aggs)
    end
    cross_validation_metrics
end
function _cross_validate(
    behaviorset::BehaviorSet,
    pdset_filepaths::Vector{String},
    streetnet_filepaths::Vector{String},
    pdset_segments::Vector{PdsetSegment},
    dataframe::DataFrame,
    frame_assignment::Vector{Int},
    pdsetseg_fold_assignment::Vector{Int},
    evalparams::EvaluationParams;
    verbosity::Int=1,
    incremental_model_save_file::Union(Nothing, String)=nothing
    )

    pdsets = Array(PrimaryDataset, length(pdset_filepaths))
    for (i,pdset_filepath) in enumerate(pdset_filepaths)
        pdsets[i] = load(pdset_filepath, "pdset")
    end

    streetnets = Array(StreetNetwork, length(streetnet_filepaths))
    for (i,streetnet_filepath) in enumerate(streetnet_filepaths)
        streetnets[i] = load(streetnet_filepath, "streetmap")
    end

    _cross_validate(behaviorset, pdsets, streetnets, pdset_segments, dataframe, frame_assignment, pdsetseg_fold_assignment,
                    evalparams, verbosity=verbosity, incremental_model_save_file=incremental_model_save_file)
end
function _cross_validate_logl{B<:AbstractVehicleBehavior, A<:Any}(
    behavior_type::Type{B},
    behavior_train_params::Dict{Symbol,A},
    dataframe::DataFrame,
    frame_assignment::Vector{Int},
    )

    nfolds = maximum(frame_assignment)
    logls = zeros(Float64, nfolds)

    max_dataframe_training_frames = length(frame_assignment) - _min_count_of_unique_value(frame_assignment)
    dataframe_for_training = similar(dataframe, max_dataframe_training_frames)

    # ------------------

    for fold = 1 : nfolds

        # copy the training frames into dataframe_for_training
        framecount = 0
        for (i,assignment) in enumerate(frame_assignment)
            if is_in_fold(fold, assignment, false)
                framecount += 1
                for j = 1 : size(dataframe, 2)
                    dataframe_for_training[framecount, j] = dataframe[i, j]
                end
            end
        end
        while framecount < max_dataframe_training_frames

            # now we need to fill in any extra training frames so that the entire thing is populated
            # If properly used this will only be one extra row so the overall effect will be negligible

            row = rand(1:framecount)
            framecount += 1
            for j = 1 : size(dataframe, 2)
                dataframe_for_training[framecount, j] = dataframe_for_training[row, j]
            end
        end

        model = train(behavior_type, dataframe_for_training, args=behavior_train_params)

        for (frameind,assignment) in enumerate(frame_assignment)
            if is_in_fold(fold, assignment, true)

                action_lat = dataframe[frameind, symbol(FUTUREDESIREDANGLE_250MS)]::Float64
                action_lon = dataframe[frameind, symbol(FUTUREACCELERATION_250MS)]::Float64

                if !isinf(action_lat) && !isinf(action_lon)
                    logls[fold] += calc_action_loglikelihood(model, dataframe, frameind)
                end

                if isinf(logls[fold])
                    n = names(dataframe)
                    for f in 1 : ncol(dataframe)
                        println(n[f], ":  ", dataframe[frameind, f])
                    end
                    exit()
                end
            end
        end
    end

    μ = mean(logls)
    σ = stdm(logls, μ)

    (μ,σ)
end

function _cross_validate_fold(
    fold::Integer,
    behaviorset::BehaviorSet,
    pdsets::Vector{PrimaryDataset},
    streetnet_filepaths::Vector{String},
    pdset_segments::Vector{PdsetSegment},
    dataframe::DataFrame,
    frame_assignment::Vector{Int},
    pdsetseg_fold_assignment::Vector{Int},
    evalparams::EvaluationParams;
    incremental_model_save_file::Union(Nothing, String)=nothing
    )

    pdsets_for_simulation = deepcopy(pdsets)

    streetnets = Array(StreetNetwork, length(streetnet_filepaths))
    for (i,streetnet_filepath) in enumerate(streetnet_filepaths)
        streetnets[i] = load(streetnet_filepath, "streetmap")
    end

    nbehaviors = length(behaviorset.behaviors)
    model_behaviors = train(behaviorset, dataframe[frame_assignment .!= fold,:])

    metrics_sets_validation = create_metrics_sets_no_tracemetrics(model_behaviors, pdsets, pdsets_for_simulation,
                                                                  streetnets, pdset_segments, evalparams,
                                                                  fold, pdsetseg_fold_assignment, true)
    aggregate_metric_sets_validation = Array(Dict{Symbol,Any}, nbehaviors+1)
    for (i,metrics_set) in enumerate(metrics_sets_validation)
        aggregate_metric_sets_validation[i] = metrics_set.aggmetrics
    end

    if isa(incremental_model_save_file, String)

        metrics_sets_training = create_metrics_sets_no_tracemetrics(model_behaviors, pdsets, pdsets_for_simulation,
                                                                    streetnets, pdset_segments, evalparams,
                                                                    fold, pdsetseg_fold_assignment, false)
        aggregate_metric_sets_training = Array(Dict{Symbol,Any}, nbehaviors+1)
        for (i,metrics_set) in enumerate(metrics_sets_training)
            aggregate_metric_sets_training[i] = metrics_set.aggmetrics
        end

        # NOTE: "dir/file.jld" → "dir/file_<fold>.jld"
        filename = splitext(incremental_model_save_file)[1] * @sprintf("_%02d.jld", fold)
        JLD.save(filename,
            "behaviorset", behaviorset,
            "models", model_behaviors,
            "metrics_sets_training", metrics_sets_training,
            "metrics_sets_validation", metrics_sets_validation)
    end

    aggregate_metric_sets_validation
end
function _cross_validate_parallel(
    behaviorset::BehaviorSet,
    pdset_filepaths::Vector{String},
    streetnet_filepaths::Vector{String},
    pdset_segments::Vector{PdsetSegment},
    dataframe::DataFrame,
    frame_assignment::Vector{Int},
    pdsetseg_fold_assignment::Vector{Int},
    evalparams::EvaluationParams;
    incremental_model_save_file::Union(Nothing, String)=nothing
    )

    pdsets = Array(PrimaryDataset, length(pdset_filepaths))
    for (i,pdset_filepath) in enumerate(pdset_filepaths)
        pdsets[i] = load(pdset_filepath, "pdset")
    end

    nfolds = maximum(frame_assignment)
    all_the_aggmetric_sets_validation = pmap(1:nfolds) do fold
        _cross_validate_fold(fold, behaviorset, pdsets, streetnet_filepaths, pdset_segments,
                             dataframe, frame_assignment, pdsetseg_fold_assignment, evalparams,
                             incremental_model_save_file=incremental_model_save_file)
    end

    n_cv_metrics = length(all_the_aggmetric_sets_validation[1])
    cross_validation_metrics = Array(Dict{Symbol,Any}, n_cv_metrics)
    aggs = Array(Dict{Symbol,Any}, nfolds)
    for i in 1 : n_cv_metrics
        for j = 1 : nfolds
            aggs[j] = all_the_aggmetric_sets_validation[j][i]
        end
        cross_validation_metrics[i] = calc_mean_cross_validation_metrics(aggs)
    end
    cross_validation_metrics
end
function _cross_validate_fold_logl{B<:AbstractVehicleBehavior, A<:Any}(
    fold::Integer,
    behavior_type::Type{B},
    behavior_train_params::Dict{Symbol,A},
    dataframe::DataFrame,
    frame_assignment::Vector{Int},
    )


    model = train(behavior_type, dataframe, args=behavior_train_params)

    logl = 0.0
    for (frameind,assignment) in enumerate(frame_assignment)
        if is_in_fold(fold, assignment, true)
            logl += calc_action_loglikelihood(model, dataframe, frameind)
        end
    end
    logl
end
function _cross_validate_parallel_logl{B<:AbstractVehicleBehavior, A<:Any}(
    behavior_type::Type{B},
    behavior_train_params::Dict{Symbol,A},
    dataframe::DataFrame,
    frame_assignment::Vector{Int},
    )


    nfolds = maximum(frame_assignment)

    logls = pmap(1:nfolds) do fold
        _cross_validate_fold_logl(fold, behavior_type, behavior_train_params, dataframe, frame_assignment)
    end

    μ = mean(logls)
    σ = stdm(logls, μ)
    (μ, σ)
end

function cross_validate(
    behaviorset::BehaviorSet,
    pdset_filepaths::Vector{String},
    streetnet_filepaths::Vector{String},
    pdset_segments::Vector{PdsetSegment},
    dataframe::DataFrame,
    frame_assignment::Vector{Int},
    pdsetseg_fold_assignment::Vector{Int},
    evalparams::EvaluationParams;
    verbosity::Int=1,
    incremental_model_save_file::Union(Nothing, String)=nothing
    )

    if nworkers() > 1
        _cross_validate_parallel(behaviorset, pdset_filepaths, streetnet_filepaths, pdset_segments,
                                 dataframe, frame_assignment, pdsetseg_fold_assignment,
                                 evalparams, incremental_model_save_file=incremental_model_save_file)
    else
        _cross_validate(behaviorset, pdset_filepaths, streetnet_filepaths, pdset_segments,
                        dataframe, frame_assignment, pdsetseg_fold_assignment,
                        evalparams, verbosity=verbosity,
                        incremental_model_save_file=incremental_model_save_file)
    end
end
function cross_validate_logl{B<:AbstractVehicleBehavior, A<:Any}(
    behavior_type::Type{B},
    behavior_train_params::Dict{Symbol,A},
    dataframe::DataFrame,
    frame_assignment::Vector{Int},
    )

    #=
    Returns the mean and stdev of the cross validation scores
    =#

    if nworkers() > 1
        _cross_validate_parallel_logl(behavior_type, behavior_train_params,
                                 dataframe, frame_assignment)
    else
        _cross_validate_logl(behavior_type, behavior_train_params,
                                 dataframe, frame_assignment)
    end
end

function _get_training_and_validation(
    fold::Int,
    pdset_segments::Vector{PdsetSegment},
    dataframe::DataFrame,
    frame_assignment::Vector{Int},
    pdsetseg_fold_assignment::Vector{Int},
    )

    #=
    OUTPUT:
        (training_frames::DataFrame, training_tracesets::Vector{Vector{VehicleTrace}}, validation_tracesets::Vector{Vector{VehicleTrace}})
        for use in training and then validating a behavior model

    frame_assignment and pdsetseg_fold_assignment should have been created with cross_validation_sets
    to assign to kfolds.

    Fold must be within [1,kfolds].

    returns training_frames corresponding to all frames in folds that are not fold
    and validation_tracesets corresponding to fold
    =#

    @assert(fold > 0)

    training_frame_indeces = falses(length(frame_assignment))
    for (i,a) in enumerate(frame_assignment)
        training_frame_indeces[i] = (a!=fold)
    end
    training_frames = dataframe[training_frame_indeces, :]

    training_trace_indeces = falses(length(pdsetseg_fold_assignment))
    for (i,a) in enumerate(pdsetseg_fold_assignment)
        training_trace_indeces[i] = (a!=fold)
    end
    training_pdset_segments = pdset_segments[training_trace_indeces]

    validation_trace_indeces = falses(length(pdsetseg_fold_assignment))
    for (i,a) in enumerate(pdsetseg_fold_assignment)
        validation_trace_indeces[i] = (a==fold)
    end
    validation_pdset_segments = pdset_segments[validation_trace_indeces]

    (training_frames, training_pdset_segments, validation_pdset_segments)
end

function _max_count_of_unique_value(arr::AbstractArray)
    dict = Dict{Any,Int}()
    for v in arr
        dict[v] = get(dict, v, 0) + 1
    end
    maximum(values(dict))::Int
end
function _min_count_of_unique_value(arr::AbstractArray)
    dict = Dict{Any,Int}()
    for v in arr
        dict[v] = get(dict, v, 0) + 1
    end
    minimum(values(dict))::Int
end

function _copy_in_fold_frames!(
    dest::DataFrame,
    src::DataFrame,
    assignment::FoldAssignment,
    fold::Int,
    match_fold::Bool
    )

    # NOTE(tim): assumes dest is preallocated with enough rows

    nrow_dest = nrow(dest)

    framecount = 0
    for (i,assignment) in enumerate(frame_assignment)
        if is_in_fold(fold, assignment.frame_assignment, match_fold)
            framecount += 1
            for j = 1 : size(src, 2)
                dest[framecount, j] = src[i, j]
            end
        end
    end
    while framecount < nrow_dest

        # now we need to fill in any extra training frames so that the entire thing is populated
        # If properly used this will only be one extra row so the overall effect will be negligible

        row = rand(1:framecount) # choose something at random
        framecount += 1
        for j = 1 : size(src, 2)
            dest[framecount, j] = src[row, j]
        end
    end

    dest
end