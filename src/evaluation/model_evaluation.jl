export
    BehaviorSet,
    EvaluationParams,

    add_behavior!,
    cross_validate

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

type EvaluationParams
    sim_history_in_frames::Int
    road::StraightRoadway
    simparams::SimParams
    histobin_params::ParamsHistobin
end

function train( models::BehaviorSet, training_frames::DataFrame )

    # TODO(tim): parallel training

    retval = Array(AbstractVehicleBehavior, length(models.behaviors))
    for i = 1 : length(retval)
        retval[i] = train(models.behaviors[i], training_frames, args=models.additional_params[i])
    end
    retval
end

function create_metrics_sets{B<:AbstractVehicleBehavior}(
    model_behaviors::Vector{B},
    tracesets::Vector{Vector{VehicleTrace}},
    evalparams::EvaluationParams;
    nframes_total::Int=get_nframes(tracesets[1][1]),
    )

    #=
    Takes trained behavior models and computes MetricsSet for each
    given the tracesets
    =#

    # pull out the parameters
    sim_history_in_frames = evalparams.sim_history_in_frames
    road = evalparams.road
    simparams = evalparams.simparams
    histobin_params = evalparams.histobin_params

    # compute original simlogs from tracesets
    simlogs_original = allocate_simlogs_for_all_traces(tracesets, nframes_total)
    for (simlog,traceset) in zip(simlogs_original, tracesets)
        for (carind, trace) in enumerate(traceset)
            fill_log_with_trace!(simlog, 1, trace, 1, get_nframes(trace), carind)
        end
    end

    # compute original metrics
    metrics_set_orig = create_metrics_set(simlogs_original, road, sim_history_in_frames, simparams, histobin_params)
    
    # create default behavior vector
    behaviors = Array(AbstractVehicleBehavior, get_max_vehicle_count(tracesets))
    fill!(behaviors, VEHICLE_BEHAVIOR_NONE)

    # simulate each behavior
    behavior_simlogs = Array(Vector{Matrix{Float64}}, length(model_behaviors))
    for (i, model_behavior) in enumerate(model_behaviors)
        behaviors[1] = model_behavior
        behavior_simlogs[i] = simulate!(deepcopy(simlogs_original), behaviors, road, sim_history_in_frames, simparams)
    end

    # compute the behavior metrics
    behavior_metrics_sets = create_metrics_sets(model_behaviors, metrics_set_orig, simlogs_original, behavior_simlogs, 
                                                road, sim_history_in_frames, simparams, histobin_params)

    retval = Array(MetricsSet, 1 + length(behavior_metrics_sets))
    retval[1] = metrics_set_orig
    retval[2:end] = behavior_metrics_sets
    retval::Vector{MetricsSet}
end
function create_metrics_sets{B<:AbstractVehicleBehavior}(
    model_behaviors::Vector{B},
    pdsets_original::Vector{PrimaryDataset},
    pdsets_for_simulation::Vector{PrimaryDataset},
    streetnets::Vector{StreetNetwork},
    pdset_segments::Vector{PdsetSegment},
    evalparams::EvaluationParams,
    fold::Integer,
    pdsetseg_fold_assignment::Vector{Int},
    match_fold::Bool, # if true, then we want to only include folds that match target (validation)
                      # if false, then we want to only include folds that do not (training)
    )

    #=
    Takes trained behavior models and computes MetricsSet for each
    =#


    retval = Array(MetricsSet, 1+ length(model_behaviors))

    # pull out the parameters
    simparams = evalparams.simparams
    histobin_params = evalparams.histobin_params

    # compute original metrics
    metrics_set_orig = create_metrics_set_no_tracemetrics(pdsets_original, streetnets, pdset_segments, simparams, histobin_params,
                                                          fold, pdsetseg_fold_assignment, match_fold)
    histobin_original_with_prior = copy(metrics_set_orig.histobin) .+ 1.0
    retval[1] = metrics_set_orig

    # simulate each behavior and compute the behavior metrics
    basics = FeatureExtractBasicsPdSet(pdsets_for_simulation[1], streetnets[1])
    behavior_pairs = Array((AbstractVehicleBehavior,Int), 1)
    fold_size = calc_fold_size(fold, pdsetseg_fold_assignment, match_fold)
    for (i, behavior) in enumerate(model_behaviors)

        histobin = allocate_empty_histobin(histobin_params)
        tracemetrics = Array(Dict{Symbol, Any}, fold_size)

        fold_entry_count = 0
        for (fold_assignment, seg) in zip(pdsetseg_fold_assignment, pdset_segments)

            if is_in_fold(fold, fold_assignment, match_fold)

                fold_entry_count += 1

                pdset_orig = pdsets_original[seg.pdset_id]
                basics.pdset = pdsets_for_simulation[seg.pdset_id]
                basics.sn = streetnets[seg.streetnet_id]
                basics.runid += 1
                behavior_pairs[1] = (behavior, seg.carid)
                simulate!(basics, behavior_pairs, seg.validfind_start, seg.validfind_end)

                # update the MetricsSet calculation
                update_histobin!(histobin, basics.pdset, basics.sn, seg, histobin_params)
                tracemetrics[fold_entry_count] = calc_tracemetrics(behavior, pdset_orig, basics.pdset, basics.sn, seg, simparams, basics=basics)

                # now override the pdset with the original values once more
                copy_trace!(basics.pdset, pdset_orig, seg.carid, seg.validfind_start, seg.validfind_end)
            end
        end

        histobin_kldiv = KL_divergence_dirichlet(histobin_original_with_prior, histobin .+ 1.0 )
        aggmetrics = calc_aggregate_metrics(tracemetrics, metrics_set_orig.aggmetrics, histobin_kldiv)

        retval[i+1] = MetricsSet(histobin, histobin_kldiv, tracemetrics, aggmetrics)
    end

    retval
end
function create_metrics_sets_no_tracemetrics{B<:AbstractVehicleBehavior}(
    model_behaviors::Vector{B},
    pdsets_original::Vector{PrimaryDataset},
    pdsets_for_simulation::Vector{PrimaryDataset},
    streetnets::Vector{StreetNetwork},
    pdset_segments::Vector{PdsetSegment},
    evalparams::EvaluationParams,
    fold::Integer,
    pdsetseg_fold_assignment::Vector{Int},
    match_fold::Bool, # if true, then we want to only include folds that match target (validation)
                      # if false, then we want to only include folds that do not (training)
    )

    #=
    Takes trained behavior models and computes MetricsSet for each
    =#


    retval = Array(MetricsSet, 1+ length(model_behaviors))

    # pull out the parameters
    simparams = evalparams.simparams
    histobin_params = evalparams.histobin_params

    # compute original metrics
    metrics_set_orig = create_metrics_set_no_tracemetrics(pdsets_original, streetnets, pdset_segments, simparams, histobin_params,
                                                          fold, pdsetseg_fold_assignment, match_fold)
    histobin_original_with_prior = copy(metrics_set_orig.histobin) .+ 1.0
    retval[1] = metrics_set_orig

    # simulate each behavior and compute the behavior metrics
    basics = FeatureExtractBasicsPdSet(pdsets_for_simulation[1], streetnets[1])
    behavior_pairs = Array((AbstractVehicleBehavior,Int), 1)
    fold_size = calc_fold_size(fold, pdsetseg_fold_assignment, match_fold)
    for (i, behavior) in enumerate(model_behaviors)

        histobin = allocate_empty_histobin(histobin_params)
        tracemetrics = Array(Dict{Symbol, Any}, fold_size)

        fold_entry_count = 0
        for (fold_assignment, seg) in zip(pdsetseg_fold_assignment, pdset_segments)

            if is_in_fold(fold, fold_assignment, match_fold)

                fold_entry_count += 1

                pdset_orig = pdsets_original[seg.pdset_id]
                basics.pdset = pdsets_for_simulation[seg.pdset_id]
                basics.sn = streetnets[seg.streetnet_id]
                basics.runid += 1
                behavior_pairs[1] = (behavior, seg.carid)
                simulate!(basics, behavior_pairs, seg.validfind_start, seg.validfind_end)

                # update the MetricsSet calculation
                update_histobin!(histobin, basics.pdset, basics.sn, seg, histobin_params)
                tracemetrics[fold_entry_count] = calc_tracemetrics(behavior, pdset_orig, basics.pdset, basics.sn, seg, simparams, basics=basics)

                # now override the pdset with the original values once more
                copy_trace!(basics.pdset, pdset_orig, seg.carid, seg.validfind_start, seg.validfind_end)
            end
        end

        histobin_kldiv = KL_divergence_dirichlet(histobin_original_with_prior, histobin .+ 1.0 )
        aggmetrics = calc_aggregate_metrics(tracemetrics, metrics_set_orig.aggmetrics, histobin_kldiv)

        retval[i+1] = MetricsSet(histobin, histobin_kldiv, Dict{Symbol, Any}[], aggmetrics)
    end

    retval
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

function cross_validate(                                                                                                              
    behaviorset::BehaviorSet,
    tracesets::Vector{Vector{VehicleTrace}},
    dataframe::DataFrame,
    startframes::Vector{Int},
    frame_assignment::Vector{Int},
    trace_assignment::Vector{Int},
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

    for fold = 1 : nfolds
        if verbosity > 0
            println("FOLD ", fold, " / ", nfolds)
            tic()
        end
        training_frames, training_tracesets, validation_tracesets = _get_training_and_validation(
                                                                        fold, tracesets, dataframe, 
                                                                        startframes, frame_assignment, 
                                                                        trace_assignment)

        model_behaviors = train(behaviorset, training_frames)

        print("training metrics sets ($(length(training_tracesets)))"); tic()
        metrics_sets_training = create_metrics_sets(model_behaviors, training_tracesets, evalparams)
        for (i,metrics_set) in enumerate(metrics_sets_training)
            aggregate_metric_sets_training[i][fold] = metrics_set.aggmetrics
            metrics_set.tracemetrics = Dict{Symbol, Any}[] # delete it so we don't save it later
        end
        print(" [DONE] "); toc()

        print("validation metrics sets ($(length(validation_tracesets)))"); tic()
        metrics_sets_validation = create_metrics_sets(model_behaviors, validation_tracesets, evalparams)
        for (i,metrics_set) in enumerate(metrics_sets_validation)
            aggregate_metric_sets_validation[i][fold] = metrics_set.aggmetrics
            metrics_set.tracemetrics = Dict{Symbol, Any}[] # delete it so we don't save it later
        end
        print(" [DONE] "); toc()

        println("saving incremental file"); tic()
        if isa(incremental_model_save_file, String)
            # NOTE: "dir/file.jld" → "dir/file_<fold>.jld"
            filename = splitext(incremental_model_save_file)[1] * @sprintf("_%02d.jld", fold)
            JLD.save(filename,
                "behaviorset", behaviorset,
                "models", model_behaviors,
                "metrics_sets_training", metrics_sets_training,
                "metrics_sets_validation", metrics_sets_validation)
        end
        print(" [DONE] "); toc()

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

    pdsets = Array(PrimaryDataset, length(pdset_filepaths))
    for (i,pdset_filepath) in enumerate(pdset_filepaths)
        pdsets[i] = load(pdset_filepath, "pdset")
    end

    streetnets = Array(StreetNetwork, length(streetnet_filepaths))
    for (i,streetnet_filepath) in enumerate(streetnet_filepaths)
        streetnets[i] = load(streetnet_filepath, "streetmap")
    end

    CarEM._cross_validate(behaviorset, pdsets, streetnets, pdset_segments, dataframe, frame_assignment, pdsetseg_fold_assignment,
                    evalparams, verbosity=verbosity, incremental_model_save_file=incremental_model_save_file)
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
function cross_validate_parallel(
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

function calc_fold_size{I<:Integer}(fold::Integer, fold_assignment::AbstractArray{I}, match_fold::Bool)
    fold_size = 0
    for a in fold_assignment
        if is_in_fold(fold, a, match_fold)
            fold_size += 1
        end
    end
    fold_size
end
function is_in_fold(fold::Integer, fold_assignment::Integer, match_fold::Bool)
    (match_fold && fold_assignment == fold) ||
    (!match_fold && fold_assignment != fold)
end

function _get_training_and_validation(
    fold::Int,
    tracesets::Vector{Vector{VehicleTrace}},
    dataframe::DataFrame,
    frame_assignment::Vector{Int},
    trace_assignment::Vector{Int},
    )

    #=
    OUTPUT:
        (training_frames::DataFrame, training_tracesets::Vector{Vector{VehicleTrace}}, validation_tracesets::Vector{Vector{VehicleTrace}})
        for use in training and then validating a behavior model

    frame_assignment and trace_assignment should have been created with cross_validation_sets
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

    training_trace_indeces = falses(length(trace_assignment))
    for (i,a) in enumerate(trace_assignment)
        training_trace_indeces[i] = (a!=fold)
    end
    training_tracesets = tracesets[training_trace_indeces]

    validation_trace_indeces = falses(length(trace_assignment))
    for (i,a) in enumerate(trace_assignment)
        validation_trace_indeces[i] = (a==fold)
    end
    validation_tracesets = tracesets[validation_trace_indeces]

    (training_frames, training_tracesets, validation_tracesets)
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