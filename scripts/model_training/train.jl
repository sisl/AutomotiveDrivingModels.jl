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

trace_metrics = BehaviorTraceMetric[
                    SumSquareJerk(),
                    JerkSignInversions(),
                    LagOneAutocorrelation(),
                    EmergentKLDivMetric(SumSquareJerk()),
                    EmergentKLDivMetric(JerkSignInversions()),
                    EmergentKLDivMetric(LagOneAutocorrelation()),
                    # RootWeightedSquareError(SPEED, 0.5),
                    # RootWeightedSquareError(SPEED, 1.0),
                    # RootWeightedSquareError(SPEED, 1.5),
                    # RootWeightedSquareError(SPEED, 2.0),
                    # RootWeightedSquareError(SPEED, 2.5),
                    # RootWeightedSquareError(SPEED, 3.0),
                    # RootWeightedSquareError(SPEED, 3.5),
                    # RootWeightedSquareError(SPEED, 4.0),
                    # RootWeightedSquareError(POSFT, 0.5),
                    # RootWeightedSquareError(POSFT, 1.0),
                    # RootWeightedSquareError(POSFT, 1.5),
                    # RootWeightedSquareError(POSFT, 2.0),
                    # RootWeightedSquareError(POSFT, 2.5),
                    # RootWeightedSquareError(POSFT, 3.0),
                    # RootWeightedSquareError(POSFT, 3.5),
                    # RootWeightedSquareError(POSFT, 4.0),
                    # RootWeightedSquareError(INV_TIMEGAP_FRONT, 0.5),
                    # RootWeightedSquareError(INV_TIMEGAP_FRONT, 1.0),
                    # RootWeightedSquareError(INV_TIMEGAP_FRONT, 1.5),
                    # RootWeightedSquareError(INV_TIMEGAP_FRONT, 2.0),
                    # RootWeightedSquareError(INV_TIMEGAP_FRONT, 2.5),
                    # RootWeightedSquareError(INV_TIMEGAP_FRONT, 3.0),
                    # RootWeightedSquareError(INV_TIMEGAP_FRONT, 3.5),
                    # RootWeightedSquareError(INV_TIMEGAP_FRONT, 4.0),
                    # RootWeightedSquareError(DIST_FRONT, 0.5),
                    # RootWeightedSquareError(DIST_FRONT, 1.0),
                    # RootWeightedSquareError(DIST_FRONT, 1.5),
                    # RootWeightedSquareError(DIST_FRONT, 2.0),
                    # RootWeightedSquareError(DIST_FRONT, 2.5),
                    # RootWeightedSquareError(DIST_FRONT, 3.0),
                    # RootWeightedSquareError(DIST_FRONT, 3.5),
                    # RootWeightedSquareError(DIST_FRONT, 4.0),
                ]
trace_metric_names = map(m->get_name(m), trace_metrics)::Vector{Symbol}

# metric_types_test_traces = [
                            # EmergentKLDivMetric{symbol(SPEED)},
                            # EmergentKLDivMetric{symbol(POSFT)},
                            # EmergentKLDivMetric{symbol(INV_TIMEGAP_FRONT)},
                            # SumSquareJerk,
                            # JerkSignInversions,
                            # LagOneAutocorrelation,
                            # EmergentKLDivMetric{SumSquareJerk},
                            # EmergentKLDivMetric{JerkSignInversions},
                            # EmergentKLDivMetric{LagOneAutocorrelation},
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
                            # RootWeightedSquareError{symbol(DIST_FRONT), 0.5},
                            # RootWeightedSquareError{symbol(DIST_FRONT), 1.0},
                            # RootWeightedSquareError{symbol(DIST_FRONT), 1.5},
                            # RootWeightedSquareError{symbol(DIST_FRONT), 2.0},
                            # RootWeightedSquareError{symbol(DIST_FRONT), 2.5},
                            # RootWeightedSquareError{symbol(DIST_FRONT), 3.0},
                            # RootWeightedSquareError{symbol(DIST_FRONT), 3.5},
                            # RootWeightedSquareError{symbol(DIST_FRONT), 4.0},
                           # ]

################################
# MAIN LOOP
################################

behaviorset_full = behaviorset

for (model_name, traindef) in behaviorset_full

    behaviorset = Dict{AbstractString, BehaviorTrainDefinition}()
    behaviorset[model_name] = traindef
    model_output_name = replace(lowercase(model_name), " ", "_")    # ex: bayesian_network
    model_short_name = convert_model_name_to_short_name(model_name) # ex: BN

    # try
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

            print("loading dataset  "); tic()
            dset = JLD.load(DATASET_JLD_FILE, "model_training_data")::ModelTrainingData2
            toc()

            print("loading evaluation data  "); tic()
            evaldata = EvaluationData(dset)
            toc()

            print("allocating runlogs for simulation  "); tic()
            arr_runlogs_for_simulation = allocate_runlogs_for_simulation(evaldata, nmodels, 1) # TODO: investigate whether this needs to be fixed
            toc()

            print("\t\tpreallocating data   "); tic()
            preallocated_data_dict = Dict{AbstractString, AbstractVehicleBehaviorPreallocatedData}()
            for (model_name, train_def) in behaviorset
                preallocated_data_dict[model_name] = preallocate_learning_data(dset, train_def.trainparams)
            end
            toc()

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

            cv_split_outer = get_fold_assignment_across_drives(dset, N_FOLDS)

            nframes = nrow(dset.dataframe)
            ntraces = length(dset.runlog_segments)
            ntracemetrics = length(trace_metrics)

            # NOTE: metrics is shared between models to conserve memory
            metrics_df = DataFrame()
            metrics_df[:mean_logl_train] = Array(Float64, cv_split_outer.nfolds)
            metrics_df[:mean_logl_test] = Array(Float64, cv_split_outer.nfolds)
            metrics_df[:median_logl_train] = Array(Float64, cv_split_outer.nfolds)
            metrics_df[:median_logl_test] = Array(Float64, cv_split_outer.nfolds)
            for (metric_index,metric) in enumerate(trace_metrics)
                metrics_df[trace_metric_names[metric_index]] = Array(Float64, cv_split_outer.nfolds)
            end

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
                cv_split_inner = drop_fold!(deepcopy(cv_split_outer), fold) # TODO: can I do this with pre-allocated memory?
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

                # update the traindef count
                for (model_name, train_def) in behaviorset
                    hyperparam_count = hyperparam_counts[model_name]
                    for (i, λ) in enumerate(train_def.hyperparams)
                        sym = λ.sym
                        val = getfield(train_def.trainparams, sym)
                        ind = findfirst(λ.range, val)
                        if ind == 0
                            println("sym: ", sym) # DEBUG
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

                # save the models, ex: BN_following_fold02.jld
                println("\tsaving model  "); tic()
                model_for_fold_path_jld = joinpath(EVALUATION_DIR, model_short_name * dset_filepath_modifier * "_fold" * @sprintf("%02d", fold) * ".jld")
                JLD.save(model_for_fold_path_jld,
                     "model_name", model_name,
                     "model",      models[model_name],
                     "train_def",  behaviorset[model_name],
                     "time",       now(),
                    )
                toc()

                print("\tcomputing metrics  "); tic()
                foldset_seg_test = FoldSet(cv_split_outer, fold, true, :seg)
                arr_logl_test  = Array(Float64, length(FoldSet(cv_split_outer, fold, true,  :frame)))
                arr_logl_train = Array(Float64, length(FoldSet(cv_split_outer, fold, false, :frame)))

                for (model_index,model_name) in enumerate(model_names)

                    behavior = models[model_name]

                    count_logl_train = 0
                    count_logl_test = 0

                    for frame in 1 : nframes
                        if cv_split_outer.frame_assignment[frame] == fold
                            count_logl_test += 1
                            arr_logl_test[count_logl_test] = calc_action_loglikelihood(behavior, dset.dataframe, frame)
                        elseif cv_split_outer.frame_assignment[frame] != 0
                            count_logl_train += 1
                            arr_logl_train[count_logl_train] = calc_action_loglikelihood(behavior, dset.dataframe, frame)
                        end
                    end

                    metrics_df[fold, :mean_logl_train] = mean(arr_logl_train)
                    metrics_df[fold, :mean_logl_test] = mean(arr_logl_test)
                    metrics_df[fold, :median_logl_train] = median(arr_logl_train)
                    metrics_df[fold, :median_logl_test] = median(arr_logl_test)

                    # reset metrics
                    for metric in trace_metrics
                        reset!(metric)
                    end

                    # simulate traces and perform online metric extraction
                    for seg_index in foldset_seg_test

                        seg = evaldata.segments[seg_index]
                        seg_duration = seg.frame_end - seg.frame_start
                        where_to_start_simulating_from_runlog_sim = evaldata.frame_starts_sim[seg_index]
                        where_to_end_simulating_from_runlog_sim = where_to_start_simulating_from_runlog_sim + seg_duration
                        runlog_true = evaldata.runlogs[seg.runlog_id]
                        runlog_sim = arr_runlogs_for_simulation[model_index][seg.runlog_id, 1]
                        sn = evaldata.streetnets[runlog_true.header.map_name]
                        frame_starts_sim = evaldata.frame_starts_sim[seg_index]

                        for sim_index in 1 : N_SIMULATIONS_PER_TRACE
                            simulate!(runlog_sim, sn, behavior, seg.carid,
                                      where_to_start_simulating_from_runlog_sim,
                                      where_to_end_simulating_from_runlog_sim)

                            for metric in trace_metrics
                                extract!(metric, seg, runlog_true, runlog_sim, sn, frame_starts_sim)
                            end
                        end
                    end

                    # compute metric scores
                    for metric_index in 1 : ntracemetrics
                        metric_name = trace_metric_names[metric_index]
                        metrics_df[fold, metric_name] = get_score(trace_metrics[metric_index])
                    end

                    # save model results
                    model_results_path_df = joinpath(EVALUATION_DIR, "validation_results" * dset_filepath_modifier * "_" * model_output_name * ".csv")
                    writetable(model_results_path_df, metrics_df)

                    println(metrics_df)
                end
                toc()
            end

            #########################################################

            print_hyperparam_statistics(STDOUT, behaviorset, hyperparam_counts)
        end
    # catch err
    #     println("CAUGHT SOME ERROR, model: ", model_name)

    #     error_filename = @sprintf("error_%s.txt", model_name)
    #     open(error_filename, "w") do fh
    #         println(fh, "ERROR training ", model_name)
    #         println(fh, "TIME: ", now())
    #         println(fh, "")
    #         println(fh, err)
    #     end

    #     println(err)
    # end
end

# println("DONE")
println("DONE")
exit()