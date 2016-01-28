using AutomotiveDrivingModels
using RandomForestBehaviors
using DynamicBayesianNetworkBehaviors

##############################
# PARAMETERS
##############################

include(Pkg.dir("AutomotiveDrivingModels", "scripts", "extract_params.jl"))
include(Pkg.dir("AutomotiveDrivingModels", "scripts", "model_params.jl"))

################################
# METRICS
################################

metric_types_test_traces = [
                            EmergentKLDivMetric{symbol(SPEED)},
                            EmergentKLDivMetric{symbol(D_CL)},
                            # EmergentKLDivMetric{symbol(TIMEGAP_X_FRONT)},
                            RootWeightedSquareError{symbol(SPEED), 0.5},
                            RootWeightedSquareError{symbol(SPEED), 1.0},
                            RootWeightedSquareError{symbol(SPEED), 1.5},
                            RootWeightedSquareError{symbol(SPEED), 2.0},
                            RootWeightedSquareError{symbol(SPEED), 2.5},
                            RootWeightedSquareError{symbol(SPEED), 3.0},
                            RootWeightedSquareError{symbol(SPEED), 3.5},
                            RootWeightedSquareError{symbol(SPEED), 4.0},
                            RootWeightedSquareError{symbol(D_CL), 0.5},
                            RootWeightedSquareError{symbol(D_CL), 1.0},
                            RootWeightedSquareError{symbol(D_CL), 1.5},
                            RootWeightedSquareError{symbol(D_CL), 2.0},
                            RootWeightedSquareError{symbol(D_CL), 2.5},
                            RootWeightedSquareError{symbol(D_CL), 3.0},
                            RootWeightedSquareError{symbol(D_CL), 3.5},
                            RootWeightedSquareError{symbol(D_CL), 4.0},
                            RootWeightedSquareError{symbol(FeaturesNew.INV_TIMEGAP_FRONT), 0.5},
                            RootWeightedSquareError{symbol(FeaturesNew.INV_TIMEGAP_FRONT), 1.0},
                            RootWeightedSquareError{symbol(FeaturesNew.INV_TIMEGAP_FRONT), 1.5},
                            RootWeightedSquareError{symbol(FeaturesNew.INV_TIMEGAP_FRONT), 2.0},
                            RootWeightedSquareError{symbol(FeaturesNew.INV_TIMEGAP_FRONT), 2.5},
                            RootWeightedSquareError{symbol(FeaturesNew.INV_TIMEGAP_FRONT), 3.0},
                            RootWeightedSquareError{symbol(FeaturesNew.INV_TIMEGAP_FRONT), 3.5},
                            RootWeightedSquareError{symbol(FeaturesNew.INV_TIMEGAP_FRONT), 4.0},
                           ]
metric_types_test_traces_bagged = [
                                   EmergentKLDivMetric{symbol(SPEED)},
                                   EmergentKLDivMetric{symbol(D_CL)},
                                   # EmergentKLDivMetric{symbol(TIMEGAP_X_FRONT)},
                                   # RootWeightedSquareError{symbol(SPEED), 0.5},
                                   # RootWeightedSquareError{symbol(SPEED), 1.0},
                                   # RootWeightedSquareError{symbol(SPEED), 1.5},
                                   # RootWeightedSquareError{symbol(SPEED), 2.0},
                                   # RootWeightedSquareError{symbol(SPEED), 2.5},
                                   # RootWeightedSquareError{symbol(SPEED), 3.0},
                                   # RootWeightedSquareError{symbol(SPEED), 3.5},
                                   # RootWeightedSquareError{symbol(SPEED), 4.0},
                                   # RootWeightedSquareError{symbol(D_CL), 0.5},
                                   # RootWeightedSquareError{symbol(D_CL), 1.0},
                                   # RootWeightedSquareError{symbol(D_CL), 1.5},
                                   # RootWeightedSquareError{symbol(D_CL), 2.0},
                                   # RootWeightedSquareError{symbol(D_CL), 2.5},
                                   # RootWeightedSquareError{symbol(D_CL), 3.0},
                                   # RootWeightedSquareError{symbol(D_CL), 3.5},
                                   # RootWeightedSquareError{symbol(D_CL), 4.0},
                                   # RootWeightedSquareError{symbol(TIMEGAP_X_FRONT), 0.5},
                                   # RootWeightedSquareError{symbol(TIMEGAP_X_FRONT), 1.0},
                                   # RootWeightedSquareError{symbol(TIMEGAP_X_FRONT), 1.5},
                                   # RootWeightedSquareError{symbol(TIMEGAP_X_FRONT), 2.0},
                                   # RootWeightedSquareError{symbol(TIMEGAP_X_FRONT), 2.5},
                                   # RootWeightedSquareError{symbol(TIMEGAP_X_FRONT), 3.0},
                                   # RootWeightedSquareError{symbol(TIMEGAP_X_FRONT), 3.5},
                                   # RootWeightedSquareError{symbol(TIMEGAP_X_FRONT), 4.0},
                                  ]

################################
# MAIN LOOP
################################

nmodels = length(behaviorset)
model_names = collect(keys(behaviorset))

for dset_filepath_modifier in (
    # "_freeflow",
    "_following",
    # "_lanechange",
    )

    println(dset_filepath_modifier)

    METRICS_OUTPUT_FILE = joinpath(EVALUATION_DIR, "validation_results" * dset_filepath_modifier * ".jld")
    DATASET_JLD_FILE = joinpath(EVALUATION_DIR, "dataset2" * dset_filepath_modifier * ".jld")

    dset = JLD.load(DATASET_JLD_FILE, "model_training_data")::ModelTrainingData2

    preallocated_data_dict = Dict{AbstractString, AbstractVehicleBehaviorPreallocatedData}()
    for (model_name, train_def) in behaviorset
        preallocated_data_dict[model_name] = preallocate_learning_data(dset, train_def.trainparams)
    end

    hyperparam_counts = Dict{AbstractString, Matrix{Int}}()
    for model_name in model_names
        train_def = behaviorset[model_name]
        if !isempty(train_def.hyperparams)
            max_range = maximum([length(λ) for λ in train_def.hyperparams])
            hyperparam_counts[model_name] = zeros(length(train_def.hyperparams), max_range)
        else
            hyperparam_counts[model_name] = zeros(1, 1)
        end
    end

    print("loading sim resources "); tic()
    runlogs_original = load_runlogs(dset)
    streetnets = load_streetnets(runlogs_original)
    toc()

    cv_split_outer = get_fold_assignment_across_drives(dset, N_FOLDS)

    nframes = nrow(dset.dataframe)
    ntraces = length(cv_split_outer.seg_assignment)
    frame_logls = Array(Float64, nframes, ntraces, nmodels) # logl for each frame under each run (we can back out TRAIN and TEST)

    foldinds = collect(1:ntraces)
    bagged_selection = collect(1:ntraces)

    # make pdset copies that are only as large as needed
    # (contain history and horizon from runlogs_original)
    println("preallocating memory for traces"); tic()

    arr_runlogs_for_simulation = Array(Matrix{RunLog}, nmodels)
    frame_starts_sim = Array(Int, ntraces) # new frame_start for the truncated arr_runlogs_for_simulation
    for k in 1 : nmodels
        arr_runlogs_for_simulation[k] = Array(RunLog, ntraces, N_SIMULATIONS_PER_TRACE)
    end
    for (i,ind) in enumerate(foldinds)

        seg = dset.runlog_segments[ind]

        where_to_start_copying_from_original_runlog = max(1, seg.frame_start - DEFAULT_TRACE_HISTORY)
        where_to_start_simulating_from_runlog_sim = seg.frame_start - where_to_start_copying_from_original_runlog + 1

        runlog_sim = deepcopy(runlogs_original[seg.runlog_id], where_to_start_copying_from_original_runlog, seg.frame_end)
        frame_starts_sim[i] = where_to_start_simulating_from_runlog_sim

        for k in 1 : nmodels
            for j in 1 : N_SIMULATIONS_PER_TRACE
                arr_runlogs_for_simulation[k][i,j] = deepcopy(runlog_sim)
            end
        end
    end
    toc()

    ######################################
    # TRAIN A MODEL FOR EACH FOLD USING CV
    ######################################

    for fold in 1 : cv_split_outer.nfolds
        #=
        1 - find optimal hyperparam set
        2 - update trace metrics for given traces
        3 - update mean frame likelihood
        =#

        println("fold ", fold, " / ", cv_split_outer.nfolds)

        # create an inner split where we remove the current fold
        cv_split_inner = drop_fold!(deepcopy(cv_split_outer), fold)
        @assert(cv_split_inner.nfolds > 0)

        ##############

        print("\toptimizing hyperparameters\n"); tic()
        for (model_name, train_def) in behaviorset
            println(model_name)
            preallocated_data = preallocated_data_dict[model_name]
            AutomotiveDrivingModels.optimize_hyperparams_cyclic_coordinate_ascent!(
                    train_def, dset, preallocated_data, cv_split_inner)
        end
        toc()

        # update the count
        for (model_name, train_def) in behaviorset
            hyperparam_count = hyperparam_counts[model_name]
            for (i, λ) in enumerate(train_def.hyperparams)
                sym = λ.sym
                val = getfield(train_def.trainparams, sym)
                ind = findfirst(λ.range, val)
                if ind == 0
                    println("sym: ", sym)
                    println("range: ", λ.range)
                    println("val: ", val)
                end
                @assert(ind != 0)
                hyperparam_count[i, ind] += 1
            end
        end

        print("\ttraining models  "); tic()
        models = train(behaviorset, dset, preallocated_data_dict, fold, cv_split_outer)
        toc()

        # println("MODELS: ", models)

        print("\tcomputing likelihoods  "); tic()
        for (i,model_name) in enumerate(model_names)
            behavior = models[model_name]
            for frameind in 1 : nframes
                if trains_with_nona(behavior)
                    frame_logls[frameind, fold, i] = calc_action_loglikelihood(behavior, dset.dataframe_nona, frameind)
                else
                    frame_logls[frameind, fold, i] = calc_action_loglikelihood(behavior, dset.dataframe, frameind)
                end
            end
        end

        println("\tsimulating"); tic()
        for (k,model_name) in enumerate(model_names)
            behavior = models[model_name]

            print("\t\t", model_name, "  "); tic()
            for i in 1 : ntraces
                if cv_split_outer.seg_assignment[i] == fold # in test
                    # simulate
                    seg = dset.runlog_segments[i]
                    seg_duration = seg.frame_end - seg.frame_start
                    where_to_start_simulating_from_runlog_sim = frame_starts_sim[i]
                    where_to_end_simulating_from_runlog_sim = where_to_start_simulating_from_runlog_sim + seg_duration

                    for l in 1 : N_SIMULATIONS_PER_TRACE
                        runlog = arr_runlogs_for_simulation[k][i, l]
                        sn = streetnets[runlog.header.map_name]
                        simulate!(runlog, sn, behavior, seg.carid,
                            where_to_start_simulating_from_runlog_sim, where_to_end_simulating_from_runlog_sim)
                    end
                end
            end
            toc()
        end
        toc()
    end

    #########################################################

    print_hyperparam_statistics(STDOUT, behaviorset, hyperparam_counts)

    print("Exctracting frame stats  "); tic()

    metrics_sets_test_frames = Array(Vector{BehaviorFrameMetric}, nmodels)
    metrics_sets_train_frames = Array(Vector{BehaviorFrameMetric}, nmodels)
    metrics_sets_test_frames_bagged = Array(Vector{BaggedMetricResult}, nmodels)
    metrics_sets_train_frames_bagged = Array(Vector{BaggedMetricResult}, nmodels)
    metrics_sets_test_traces = Array(Vector{BehaviorTraceMetric}, nmodels)
    metrics_sets_test_traces_bagged = Array(Vector{BaggedMetricResult}, nmodels)

    for (k, model_name) in enumerate(model_names)
        print("\tmodel: ", k, "  "); tic()

        arr_logl_test = Float64[]
        arr_logl_train = Float64[]

        for j in 1 : cv_split_outer.nfolds
            for i in 1 : nframes
                if cv_split_outer.frame_assignment[i] == j
                    push!(arr_logl_test, frame_logls[i,j,k])
                elseif cv_split_outer.frame_assignment[i] != 0
                    push!(arr_logl_train, frame_logls[i,j,k])
                end
            end
        end

        metrics_sets_test_frames[k] = BehaviorFrameMetric[MedianLoglikelihoodMetric(median(arr_logl_test))]
        metrics_sets_train_frames[k] = BehaviorFrameMetric[MedianLoglikelihoodMetric(median(arr_logl_train))]
        metrics_sets_test_frames_bagged[k] = BaggedMetricResult[BaggedMetricResult(MedianLoglikelihoodMetric, arr_logl_test, N_BAGGING_SAMPLES)]
        metrics_sets_train_frames_bagged[k] = BaggedMetricResult[BaggedMetricResult(MedianLoglikelihoodMetric, arr_logl_train, N_BAGGING_SAMPLES)]

        # TRACES
        retval_straight = Array(BehaviorTraceMetric, length(metric_types_test_traces))
        retval_bagged = Array(BaggedMetricResult, length(metric_types_test_traces_bagged))
        for (i,M) in enumerate(metric_types_test_traces)
            retval_straight[i] = extract(M, dset.runlog_segments,
                                         runlogs_original, arr_runlogs_for_simulation[k], frame_starts_sim,
                                         streetnets, foldinds, bagged_selection)
            if i ≤ length(retval_bagged)
                retval_bagged[i] = BaggedMetricResult(M, dset.runlog_segments,
                                             runlogs_original, arr_runlogs_for_simulation[k], frame_starts_sim,
                                             streetnets, foldinds, bagged_selection,
                                             N_BAGGING_SAMPLES, CONFIDENCE_LEVEL)
            end
        end

        metrics_sets_test_traces[k] = retval_straight
        metrics_sets_test_traces_bagged[k] = retval_bagged

        model_output_name = replace(lowercase(model_name), " ", "_")
        model_results_path_jld = joinpath(EVALUATION_DIR, "validation_results" * dset_filepath_modifier * "_" * model_output_name * ".jld")
        JLD.save(model_results_path_jld,
             "model_name",                      model_name,
             "metrics_set_test_frames",         metrics_sets_test_frames[k],
             "metrics_set_test_frames_bagged",  metrics_sets_test_frames_bagged[k],
             "metrics_set_train_frames",        metrics_sets_train_frames[k],
             "metrics_set_train_frames_bagged", metrics_sets_train_frames_bagged[k],
             "metrics_set_test_traces",         metrics_sets_test_traces[k],
             "metrics_set_test_traces_bagged",  metrics_sets_test_traces_bagged[k],
            )

        model_results_path_txt = joinpath(EVALUATION_DIR, "validation_results" * dset_filepath_modifier * "_" * model_output_name * ".txt")

        open(model_results_path_txt, "w") do fh
            println("")

            train_def = behaviorset[model_name]
            counts = hyperparam_counts[model_name]
            print_hyperparam_statistics(fh, model_name, train_def, counts)
            println(fh)
            @printf(fh, "LOGL TEST: %6.3f ± %6.3f\n", get_score(metrics_sets_test_frames[k][1]), metrics_sets_test_frames_bagged[k][1].confidence_bound)
            @printf(fh, "LOGL TEST: %6.3f ± %6.3f\n", get_score(metrics_sets_train_frames[k][1]), metrics_sets_train_frames_bagged[k][1].confidence_bound)
        end

        toc()
    end
    toc()

    println("\tLOGL TEST")
    for i in 1 : length(metrics_sets_test_frames)
        logl_μ = get_score(metrics_sets_test_frames[i][1])
        logl_b = metrics_sets_test_frames_bagged[i][1].confidence_bound
        @printf("\t%-20s logl %6.3f ± %6.3f\n", model_names[i], logl_μ, logl_b)
    end
    println("")

    println("\tLOGL TRAIN")
    for i in 1 : length(metrics_sets_train_frames)
        logl_μ = get_score(metrics_sets_train_frames[i][1])
        logl_b = metrics_sets_train_frames_bagged[i][1].confidence_bound
        @printf("\t%-20s logl %6.3f ± %6.3f\n", model_names[i], logl_μ, logl_b)
    end
    println("")

    println("metrics_sets_test_traces: ")
    println(metrics_sets_test_traces)
    println("metrics_sets_test_traces_bagged: ")
    println(metrics_sets_test_traces_bagged)

    # JLD.save(METRICS_OUTPUT_FILE,
    #          "model_names",                      model_names,
    #          "metrics_sets_test_frames",         metrics_sets_test_frames,
    #          "metrics_sets_test_frames_bagged",  metrics_sets_test_frames_bagged,
    #          "metrics_sets_train_frames",        metrics_sets_train_frames,
    #          "metrics_sets_train_frames_bagged", metrics_sets_train_frames_bagged,
    #          "metrics_sets_test_traces",         metrics_sets_test_traces,
    #          "metrics_sets_test_traces_bagged",  metrics_sets_test_traces_bagged,
    #         )
end

# println("DONE")
println("DONE")
exit()