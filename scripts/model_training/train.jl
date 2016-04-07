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
                    RootWeightedSquareError(SPEED, 0.5),
                    RootWeightedSquareError(SPEED, 1.0),
                    RootWeightedSquareError(SPEED, 1.5),
                    RootWeightedSquareError(SPEED, 2.0),
                    RootWeightedSquareError(SPEED, 2.5),
                    RootWeightedSquareError(SPEED, 3.0),
                    RootWeightedSquareError(SPEED, 3.5),
                    RootWeightedSquareError(SPEED, 4.0),
                    RootWeightedSquareError(POSFT, 0.5),
                    RootWeightedSquareError(POSFT, 1.0),
                    RootWeightedSquareError(POSFT, 1.5),
                    RootWeightedSquareError(POSFT, 2.0),
                    RootWeightedSquareError(POSFT, 2.5),
                    RootWeightedSquareError(POSFT, 3.0),
                    RootWeightedSquareError(POSFT, 3.5),
                    RootWeightedSquareError(POSFT, 4.0),
                    RootWeightedSquareError(INV_TIMEGAP_FRONT, 0.5),
                    RootWeightedSquareError(INV_TIMEGAP_FRONT, 1.0),
                    RootWeightedSquareError(INV_TIMEGAP_FRONT, 1.5),
                    RootWeightedSquareError(INV_TIMEGAP_FRONT, 2.0),
                    RootWeightedSquareError(INV_TIMEGAP_FRONT, 2.5),
                    RootWeightedSquareError(INV_TIMEGAP_FRONT, 3.0),
                    RootWeightedSquareError(INV_TIMEGAP_FRONT, 3.5),
                    RootWeightedSquareError(INV_TIMEGAP_FRONT, 4.0),
                    RootWeightedSquareError(DIST_FRONT, 0.5),
                    RootWeightedSquareError(DIST_FRONT, 1.0),
                    RootWeightedSquareError(DIST_FRONT, 1.5),
                    RootWeightedSquareError(DIST_FRONT, 2.0),
                    RootWeightedSquareError(DIST_FRONT, 2.5),
                    RootWeightedSquareError(DIST_FRONT, 3.0),
                    RootWeightedSquareError(DIST_FRONT, 3.5),
                    RootWeightedSquareError(DIST_FRONT, 4.0),
                ]
trace_metric_names = map(m->get_name(m), trace_metrics)::Vector{Symbol}

################################
# MAIN LOOP
################################

behaviorset_full = behaviorset

for (model_name, traindef) in behaviorset_full

    model_output_name = replace(lowercase(model_name), " ", "_")    # ex: bayesian_network
    model_short_name = convert_model_name_to_short_name(model_name) # ex: BN

    # try

        for dset_filepath_modifier in (
            "_freeflow",
            "_following",
            "_lanechange",
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
            arr_runlogs_for_simulation = allocate_runlogs_for_simulation(evaldata) # TODO: investigate whether this needs to be fixed
            toc()

            print("\t\tpreallocating data   "); tic()
            preallocated_data = preallocate_learning_data(dset, traindef.trainparams)
            toc()

            if !isempty(traindef.hyperparams)
                max_range = maximum([length(λ) for λ in traindef.hyperparams])
                hyperparam_counts = zeros(Int, length(traindef.hyperparams), max_range)
            else
                hyperparam_counts = zeros(Int, 1, 1)
            end

            cv_split_outer = get_fold_assignment_across_drives(dset, N_FOLDS)

            # NOTE: metrics_df is shared between models to conserve memory
            metrics_df = create_metrics_df(cv_split_outer.nfolds, trace_metric_names)

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

                print("\toptimizing hyperparameters for $(model_name)\n"); tic()
                AutomotiveDrivingModels.optimize_hyperparams_cyclic_coordinate_ascent!(
                                        traindef, dset, preallocated_data, cv_split_inner)
                toc()

                # update the traindef count
                for (i, λ) in enumerate(traindef.hyperparams)
                    sym = λ.sym
                    val = getfield(traindef.trainparams, sym)
                    ind = findfirst(λ.range, val)
                    if ind == 0
                        println("sym: ", sym) # DEBUG
                        println("range: ", λ.range)
                        println("val: ", val)
                    end
                    @assert(ind != 0)
                    hyperparam_counts[i, ind] += 1
                end

                print("\ttraining models  "); tic()
                foldset_frame_train = FoldSet(cv_split_outer, fold, false, :frame)
                model = train(dset, preallocated_data, traindef.trainparams, foldset_frame_train)
                toc()

                # save the models, ex: BN_following_fold02.jld
                println("\tsaving model  "); tic()
                model_for_fold_path_jld = joinpath(EVALUATION_DIR, model_short_name * dset_filepath_modifier * "_fold" * @sprintf("%02d", fold) * ".jld")
                JLD.save(model_for_fold_path_jld,
                     "model_name", model_name,
                     "model",      model,
                     "train_def",  traindef,
                     "time",       now(),
                    )
                toc()

                print("\tsimulating and computing metrics  "); tic()
                calc_metrics!(metrics_df, dset, model, cv_split_outer, fold,
                              trace_metrics, evaldata, arr_runlogs_for_simulation, N_SIMULATIONS_PER_TRACE)
                toc()
            end

            println(metrics_df)

            # save model results
            model_results_path_df = joinpath(EVALUATION_DIR, "validation_results" * dset_filepath_modifier * "_" * model_output_name * ".csv")
            writetable(model_results_path_df, metrics_df)

            #########################################################

            print_hyperparam_statistics(STDOUT, model_name, traindef, hyperparam_counts)
            println("\n")
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