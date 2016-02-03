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
                            # EmergentKLDivMetric{symbol(SPEED)},
                            # EmergentKLDivMetric{symbol(POSFT)},
                            # EmergentKLDivMetric{symbol(INV_TIMEGAP_FRONT)},
                            # RootWeightedSquareError{symbol(SPEED), 0.5},
                            # RootWeightedSquareError{symbol(SPEED), 1.0},
                            # RootWeightedSquareError{symbol(SPEED), 1.5},
                            # RootWeightedSquareError{symbol(SPEED), 2.0},
                            # RootWeightedSquareError{symbol(SPEED), 2.5},
                            # RootWeightedSquareError{symbol(SPEED), 3.0},
                            # RootWeightedSquareError{symbol(SPEED), 3.5},
                            RootWeightedSquareError{symbol(SPEED), 4.0},
                            # RootWeightedSquareError{symbol(POSFT), 0.5},
                            # RootWeightedSquareError{symbol(POSFT), 1.0},
                            # RootWeightedSquareError{symbol(POSFT), 1.5},
                            # RootWeightedSquareError{symbol(POSFT), 2.0},
                            # RootWeightedSquareError{symbol(POSFT), 2.5},
                            # RootWeightedSquareError{symbol(POSFT), 3.0},
                            # RootWeightedSquareError{symbol(POSFT), 3.5},
                            RootWeightedSquareError{symbol(POSFT), 4.0},
                            # RootWeightedSquareError{symbol(INV_TIMEGAP_FRONT), 0.5},
                            # RootWeightedSquareError{symbol(INV_TIMEGAP_FRONT), 1.0},
                            # RootWeightedSquareError{symbol(INV_TIMEGAP_FRONT), 1.5},
                            # RootWeightedSquareError{symbol(INV_TIMEGAP_FRONT), 2.0},
                            # RootWeightedSquareError{symbol(INV_TIMEGAP_FRONT), 2.5},
                            # RootWeightedSquareError{symbol(INV_TIMEGAP_FRONT), 3.0},
                            # RootWeightedSquareError{symbol(INV_TIMEGAP_FRONT), 3.5},
                            # RootWeightedSquareError{symbol(INV_TIMEGAP_FRONT), 4.0},
                           ]
metric_types_test_traces_bagged = [
                                   # EmergentKLDivMetric{symbol(SPEED)},
                                   # EmergentKLDivMetric{symbol(POSFT)},
                                   # EmergentKLDivMetric{symbol(INV_TIMEGAP_FRONT)},
                                   # RootWeightedSquareError{symbol(SPEED), 0.5},
                                   # RootWeightedSquareError{symbol(SPEED), 1.0},
                                   # RootWeightedSquareError{symbol(SPEED), 1.5},
                                   # RootWeightedSquareError{symbol(SPEED), 2.0},
                                   # RootWeightedSquareError{symbol(SPEED), 2.5},
                                   # RootWeightedSquareError{symbol(SPEED), 3.0},
                                   # RootWeightedSquareError{symbol(SPEED), 3.5},
                                   # RootWeightedSquareError{symbol(SPEED), 4.0},
                                   # RootWeightedSquareError{symbol(POSFT), 0.5},
                                   # RootWeightedSquareError{symbol(POSFT), 1.0},
                                   # RootWeightedSquareError{symbol(POSFT), 1.5},
                                   # RootWeightedSquareError{symbol(POSFT), 2.0},
                                   # RootWeightedSquareError{symbol(POSFT), 2.5},
                                   # RootWeightedSquareError{symbol(POSFT), 3.0},
                                   # RootWeightedSquareError{symbol(POSFT), 3.5},
                                   # RootWeightedSquareError{symbol(POSFT), 4.0},
                                   # RootWeightedSquareError{symbol(INV_TIMEGAP_FRONT), 0.5},
                                   # RootWeightedSquareError{symbol(INV_TIMEGAP_FRONT), 1.0},
                                   # RootWeightedSquareError{symbol(INV_TIMEGAP_FRONT), 1.5},
                                   # RootWeightedSquareError{symbol(INV_TIMEGAP_FRONT), 2.0},
                                   # RootWeightedSquareError{symbol(INV_TIMEGAP_FRONT), 2.5},
                                   # RootWeightedSquareError{symbol(INV_TIMEGAP_FRONT), 3.0},
                                   # RootWeightedSquareError{symbol(INV_TIMEGAP_FRONT), 3.5},
                                   # RootWeightedSquareError{symbol(INV_TIMEGAP_FRONT), 4.0},
                                  ]

################################
# MAIN LOOP
################################

# remove models for which we have no hyperparams to maximize
for (model_name, train_def) in behaviorset
    if isempty(train_def.hyperparams)
        delete!(behaviorset, model_name)
    end
end
if isempty(behaviorset)
    println("terminating due to lack of models with tuneable hyperparams")
    exit()
end

nmodels = length(behaviorset)
model_names = collect(keys(behaviorset))

for dset_filepath_modifier in (
    # "_freeflow",
    "_following",
    # "_lanechange",
    )

    output_files = map(model_names) do model_name
        model_output_name = replace(lowercase(model_name), " ", "_")
        joinpath(EVALUATION_DIR, "validation_results" * dset_filepath_modifier * "_" * model_output_name * ".csv")
    end

    println(dset_filepath_modifier)

    METRICS_OUTPUT_FILE = joinpath(EVALUATION_DIR, "validation_results" * dset_filepath_modifier * ".jld")
    DATASET_JLD_FILE = joinpath(EVALUATION_DIR, "dataset2" * dset_filepath_modifier * ".jld")

    dset = JLD.load(DATASET_JLD_FILE, "model_training_data")::ModelTrainingData2

    print("\t\tpreallocating data   "); tic()
    preallocated_data_dict = Dict{AbstractString, AbstractVehicleBehaviorPreallocatedData}()
    for (model_name, train_def) in behaviorset
        preallocated_data_dict[model_name] = preallocate_learning_data(dset, train_def.trainparams)
    end
    toc()

    hyperparam_dataframes = Dict{AbstractString, DataFrame}()
    for model_name in model_names
        train_def = behaviorset[model_name]

        df = DataFrame()

        df[:mean_logl_test] = Float64[]
        df[:mean_logl_train] = Float64[]
        df[:median_logl_test] = Float64[]
        df[:median_logl_train] = Float64[]
        df[:rwse_speed_4] = Float64[]
        df[:rwse_posft_4] = Float64[]

        for λ in train_def.hyperparams
            df[λ.sym] = Array(eltype(λ.range), 0)
        end

        hyperparam_dataframes[model_name] = df
    end

    println("hyperparam_dataframes")
    println(hyperparam_dataframes)

    print("loading sim resources "); tic()
    runlogs_original = load_runlogs(dset)
    streetnets = load_streetnets(runlogs_original)
    toc()

    cv_split = get_fold_assignment_across_drives(dset, N_FOLDS)

    nframes = nrow(dset.dataframe)
    ntraces = length(cv_split.seg_assignment)
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

    t_start = time()
    n_iter = 0
    while n_iter ≥ 0
        n_iter += 1

        println("ITERATION: ", n_iter)
        println("TOTAL ELAPSED TIME: ", time() - t_start)


        # sample new training params
        for (model_name, train_def) in behaviorset
            rand!(train_def)
        end

        for fold in 1 : cv_split.nfolds

            println("\tfold: ", fold, " / ", cv_split.nfolds)

            # create an inner split where we remove the current fold
            cv_split_inner = drop_fold!(deepcopy(cv_split), fold)
            @assert(cv_split_inner.nfolds > 0)

            ##############

            print("\t\ttraining models   "); tic()
            models = train(behaviorset, dset, preallocated_data_dict, fold, cv_split_inner)
            toc()

            print("\t\tcomputing likelihoods  "); tic()
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
            toc()

            println("\t\tsimulating"); tic()
            for (k,model_name) in enumerate(model_names)
                behavior = models[model_name]

                print("\t\t\t", model_name, "  "); tic()
                for i in 1 : ntraces
                    if cv_split.seg_assignment[i] == fold # in test
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

        print("Exctracting frame stats  "); tic()

        for (k, model_name) in enumerate(model_names)

            arr_logl_test = Float64[]
            arr_logl_train = Float64[]

            for j in 1 : cv_split.nfolds
                for i in 1 : nframes
                    if cv_split.frame_assignment[i] == j
                        push!(arr_logl_test, frame_logls[i,j,k])
                    elseif cv_split.frame_assignment[i] != 0
                        push!(arr_logl_train, frame_logls[i,j,k])
                    end
                end
            end

            mean_logl_test = mean(arr_logl_test)
            mean_logl_train = mean(arr_logl_train)
            median_logl_test = median(arr_logl_test)
            median_logl_train = median(arr_logl_train)

            rwse_speed_4 = get_score(extract(RootWeightedSquareError{symbol(SPEED), 4.0}, dset.runlog_segments,
                                             runlogs_original, arr_runlogs_for_simulation[k], frame_starts_sim,
                                             streetnets, foldinds, bagged_selection))
            rwse_posft_4 = get_score(extract(RootWeightedSquareError{symbol(POSFT), 4.0}, dset.runlog_segments,
                                             runlogs_original, arr_runlogs_for_simulation[k], frame_starts_sim,
                                             streetnets, foldinds, bagged_selection))

            @printf("%25s  %10.6f  %10.6f  %10.6f  %10.6f  %10.6f  %10.6f", model_name, mean_logl_train, mean_logl_test,
                    median_logl_train, median_logl_test, rwse_speed_4, rwse_posft_4)

            df_row = Any[]
            push!(df_row, mean_logl_test)
            push!(df_row, mean_logl_train)
            push!(df_row, median_logl_test)
            push!(df_row, median_logl_train)
            push!(df_row, rwse_speed_4)
            push!(df_row, rwse_posft_4)

            train_def = behaviorset[model_name]
            for λ in train_def.hyperparams
                val = getfield(train_def.trainparams, λ.sym)
                print("  ", val)
                push!(df_row, val)
            end
            println("")

            println("df_row: ", df_row)
            println("ncol: ", ncol(hyperparam_dataframes[model_name]))

            push!(hyperparam_dataframes[model_name], df_row)

            writetable(output_files[k], hyperparam_dataframes[model_name])
        end

        toc()

        print("Sleeping to possibly terminate..."); sleep(1.0)
        println("DONE")
    end
end

println("DONE")
exit()