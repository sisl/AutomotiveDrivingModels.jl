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

# function train(
#     models::BehaviorSet,
    # frame_assignment::Vector{Int},
    # trace_assignment::Vector{Int}
#     )


# end
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
    Takes trained bejavior models and computes MetricsSet for each
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
    aggregate_metric_sets_validation = Array(Vector{Dict{Symbol,Any}}, nbehaviors+1)
    for i = 1:length(aggregate_metric_sets)
        aggregate_metric_sets_training[i] = Array(Dict{Symbol,Any}, nfolds)
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

        metrics_sets_training = create_metrics_sets(model_behaviors, training_tracesets, evalparams)
        for (i,metrics_set) in enumerate(metrics_sets_training)
            aggregate_metric_sets_training[i][fold] = metrics_set.aggmetrics
        end

        metrics_sets_validation = create_metrics_sets(model_behaviors, validation_tracesets, evalparams)
        for (i,metrics_set) in enumerate(metrics_sets_validation)
            aggregate_metric_sets_validation[i][fold] = metrics_set.aggmetrics
        end

        if isa(incremental_model_save_file, String)
            # NOTE: "dir/file.jld" → "dir/file_<fold>.jld"
            filename = splitext(incremental_model_save_file)[1] * @sprintf("_%02d.jld", fold)
            JLD.save(filename,
                "behaviorset", behaviorset,
                "models", model_behaviors,
                "metrics_sets_training", metrics_sets,
                "metrics_sets_validation", metrics_sets)
        end

        verbosity > 0 && toc()
    end

    cross_validation_metrics = Array(Dict{Symbol,Any}, length(aggregate_metric_sets_validation))
    for (i,aggs) in enumerate(aggregate_metric_sets_validation)
        cross_validation_metrics[i] = calc_mean_cross_validation_metrics(aggs)
    end
    cross_validation_metrics
end

function _get_training_and_validation(
    fold::Int,
    tracesets::Vector{Vector{VehicleTrace}},
    dataframe::DataFrame,
    trace_to_framestart::Vector{Int},
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
        validation_trace_indeces[i] = (a!=fold)
    end
    training_tracesets = tracesets[training_trace_indeces]

    validation_trace_indeces = falses(length(trace_assignment))
    for (i,a) in enumerate(trace_assignment)
        validation_trace_indeces[i] = (a==fold)
    end
    validation_tracesets = tracesets[validation_trace_indeces]

    (training_frames, training_tracesets, validation_tracesets)
end