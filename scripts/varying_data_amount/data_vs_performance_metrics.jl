using AutomotiveDrivingModels
using RandomForestBehaviors
using DynamicBayesianNetworkBehaviors

##############################
# PARAMETERS
##############################

include(Pkg.dir("AutomotiveDrivingModels", "scripts", "extract_params.jl"))
include(Pkg.dir("AutomotiveDrivingModels", "scripts", "model_params.jl"))

const DATASET_PERCENTAGES = logspace(-3.0, 0.0, 3)

trace_metrics = BehaviorTraceMetric[
                    SumSquareJerk(),
                    JerkSignInversions(),
                    LagOneAutocorrelation(),
                    EmergentKLDivMetric(SumSquareJerk()),
                    EmergentKLDivMetric(JerkSignInversions()),
                    EmergentKLDivMetric(LagOneAutocorrelation()),
                    RootWeightedSquareError(SPEED, 4.0),
                    RootWeightedSquareError(POSFT, 4.0),
                    RootWeightedSquareError(INV_TIMEGAP_FRONT, 4.0),
                    RootWeightedSquareError(DIST_FRONT, 4.0),
                ]
trace_metric_names = map(m->get_name(m), trace_metrics)::Vector{Symbol}

tic()
for (model_name, traindef) in behaviorset

    println("MODEL: ", model_name)

    model_output_name = replace(lowercase(model_name), " ", "_")    # ex: bayesian_network
    model_short_name = convert_model_name_to_short_name(model_name) # ex: BN

    # try

        df_results = DataFrame()
        df_results[:model_name] = AbstractString[]
        df_results[:dataset_percentage] = Float64[]
        df_results[:nframes_train] = Float64[]
        df_results[:nframes_test] = Float64[]
        df_results[:nseg_test] = Float64[]
        df_results[:context_class] = AbstractString[]
        df_results[:logl_train] = Float64[]
        df_results[:logl_test] = Float64[]
        df_results[:rwse_speed_test] = Float64[]
        df_results[:rwse_dcl_test] = Float64[]
        df_results[:rwse_headway_test] = Float64[]
        df_results[:smooth_sumsquare] = Float64[]
        df_results[:smooth_autocor] = Float64[]
        df_results[:smooth_jerkinvs] = Float64[]

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
            arr_runlogs_for_simulation = allocate_runlogs_for_simulation(evaldata)
            toc()

            print("\t\tpreallocating data   "); tic()
            preallocated_data = preallocate_learning_data(dset, traindef.trainparams)
            toc()

            cv_split_outer = get_fold_assignment_across_drives(dset, N_FOLDS)
            metrics_df = create_metrics_df(cv_split_outer.nfolds, trace_metric_names)

            ##############################
            # RUN FOR EACH DATA SUBSET
            ##############################

            for dset_percentage in DATASET_PERCENTAGES

                nframes_train_reduced_mean = 0.0
                nframes_test_mean = 0.0
                nsegs_test_mean = 0.0

                for fold_test in 1 : cv_split_outer.nfolds

                    nframes_train = length(FoldSet(cv_split_outer, fold_test, false, :frame))
                    nframes_test = length(FoldSet(cv_split_outer, fold_test, true, :frame))
                    nsegs_test = length(FoldSet(cv_split_outer, fold_test, false, :seg))

                    nframes_train_reduced = round(Int, nframes_train * dset_percentage)
                    if nframes_train_reduced < 10
                        continue
                    end
                    @assert(nframes_train_reduced ≤ nframes_train)

                    nframes_train_reduced_mean += nframes_train_reduced
                    nframes_test_mean += nframes_test
                    nsegs_test_mean += nsegs_test

                    # build a subset to train on
                    cv_split_outer_copy = deepcopy(cv_split_outer)
                    for (i,a) in enumerate(cv_split_outer_copy.frame_assignment)
                        if a == fold_test
                            cv_split_outer_copy.frame_assignment[i] = FOLD_TEST
                        elseif a > 0
                            cv_split_outer_copy.frame_assignment[i] = FOLD_TRAIN
                        end
                    end

                    # println("length train before:        ", length(FoldSet(cv_split_outer_copy, FOLD_TRAIN, true, :frame)))
                    # println("length test  before:        ", length(FoldSet(cv_split_outer_copy, FOLD_TEST, true, :frame)))

                    # remove nframes_train_reduced entries
                    for nframes_train_partial in nframes_train : -1 : nframes_train_reduced + 1
                        remove_this_train_index = rand(1:nframes_train_partial)

                        i = 0
                        for j in FoldSet(cv_split_outer_copy, FOLD_TRAIN, true, :frame)
                            i += 1
                            if i == remove_this_train_index
                                cv_split_outer_copy.frame_assignment[j] = 0
                                break
                            end
                        end
                    end
                    # println("nframes_train_reduced: ", nframes_train_reduced)
                    # println("length foldset:        ", length(FoldSet(cv_split_outer_copy, FOLD_TRAIN, true, :frame)))
                    @assert(nframes_train_reduced == length(FoldSet(cv_split_outer_copy, FOLD_TRAIN, true, :frame)))

                    sum_fold_size = 0
                    cv_split_inner = get_cross_validation_fold_assignment_frameonly(cv_split_outer.nfolds-1, dset, cv_split_outer_copy)
                    for fold in 1:cv_split_outer.nfolds-1
                        fold_size = length(FoldSet(cv_split_inner, fold, true, :frame))
                        # println(fold, "  ", fold_size)
                        sum_fold_size += fold_size
                    end
                    @assert(sum_fold_size == nframes_train_reduced)

                    println("\tdset_percentage: ", dset_percentage, "   nframes train reduced: ", nframes_train_reduced)

                    print("\toptimizing hyperparameters for $(model_name)\n"); tic()
                    AutomotiveDrivingModels.optimize_hyperparams_cyclic_coordinate_ascent!(
                                            traindef, dset, preallocated_data, cv_split_outer_copy)
                    toc()

                    print("\ttraining models  "); tic()
                    foldset_frame_train = FoldSet(cv_split_outer_copy, FOLD_TEST, false, :frame)
                    model = train(dset, preallocated_data, traindef.trainparams, foldset_frame_train)
                    toc()

                    print("\tsimulating and computing metrics  "); tic()
                    foldset_seg_test = FoldSet(cv_split_outer_copy, FOLD_TEST, true, :seg)
                    arr_logl_test  = Array(Float64, length(FoldSet(cv_split_outer_copy, FOLD_TEST, true,  :frame)))
                    arr_logl_train = Array(Float64, length(FoldSet(cv_split_outer_copy, FOLD_TEST, false, :frame)))
                    calc_metrics!(metrics_df, dset, model, cv_split_outer_copy, FOLD_TRAIN,
                                  trace_metrics, evaldata, arr_runlogs_for_simulation, N_SIMULATIONS_PER_TRACE,
                                  fold_test)
                    toc()

                end

                rwse_speed_test   = mean(metrics_df[:RWSE_speed_4_00])
                rwse_dcl_test     = mean(metrics_df[:RWSE_posFt_4_00])
                rwse_headway_test = mean(metrics_df[:RWSE_d_front_4_00])
                smooth_sumsquare  = mean(metrics_df[:kldiv_sumsquarejerk])
                smooth_autocor    = mean(metrics_df[:kldiv_jerksigninvs])
                smooth_jerkinvs   = mean(metrics_df[:kldiv_lagoneautocor])
                median_logl_train = mean(metrics_df[:median_logl_train])
                median_logl_test = mean(metrics_df[:median_logl_test])

                nframes_train_reduced_mean /= cv_split_outer.nfolds
                nframes_test_mean /= cv_split_outer.nfolds
                nsegs_test_mean /= cv_split_outer.nfolds

                push!(df_results, [model_name, dset_percentage,
                                   nframes_train_reduced_mean, nframes_test_mean, nsegs_test_mean,
                                   dset_filepath_modifier, median_logl_train, median_logl_test,
                                   rwse_speed_test, rwse_dcl_test, rwse_headway_test,
                                   smooth_sumsquare, smooth_autocor, smooth_jerkinvs])

                    # println("\tsimulating"); tic()
                    # foldset = FoldSet(cv_split_outer_copy, FOLD_TEST, true, :seg)
                    # for (k,model_name) in enumerate(model_names)
                    #     behavior = models[model_name]

                    #     print("\t\t", model_name, "  "); tic()
                    #     simulate!(behavior, evaldata, arr_runlogs_for_simulation[k], foldset)
                    #     toc()
                    # end
                    # toc()

                    # print("\tcomputing metrics  "); tic()
                    # logl_train_arr = Array(Float64, nframes_train_reduced)
                    # logl_test_arr = Array(Float64, nframes_test)

                    # seg_indeces = collect(foldset)
                    # bagsamples = collect(1:length(seg_indeces))
                    # for (k,model_name) in enumerate(model_names)

                    #     behavior = models[model_name]
                    #     ind_test = 0
                    #     ind_train = 0

                    #     for frameind in 1 : nframes
                    #         logl = calc_action_loglikelihood(behavior, dset.dataframe, frameind)

                    #         if cv_split_outer_copy.frame_assignment[frameind] == FOLD_TRAIN
                    #             logl_train_arr[ind_train += 1] = logl
                    #         elseif cv_split_outer_copy.frame_assignment[frameind] == FOLD_TEST
                    #             logl_test_arr[ind_test += 1] = logl
                    #         end
                    #     end

                    #     rwse_speed_test   = get_score(extract(RootWeightedSquareError{symbol(SPEED),      4.0}, evaldata, arr_runlogs_for_simulation[k], seg_indeces, bagsamples))
                    #     rwse_dcl_test     = get_score(extract(RootWeightedSquareError{symbol(POSFT),      4.0}, evaldata, arr_runlogs_for_simulation[k], seg_indeces, bagsamples))
                    #     rwse_headway_test = get_score(extract(RootWeightedSquareError{symbol(DIST_FRONT), 4.0}, evaldata, arr_runlogs_for_simulation[k], seg_indeces, bagsamples))
                    #     smooth_sumsquare  = get_score(extract(EmergentKLDivMetric{SumSquareJerk},               evaldata, arr_runlogs_for_simulation[k], seg_indeces, bagsamples))
                    #     smooth_autocor    = get_score(extract(EmergentKLDivMetric{JerkSignInversions},          evaldata, arr_runlogs_for_simulation[k], seg_indeces, bagsamples))
                    #     smooth_jerkinvs   = get_score(extract(EmergentKLDivMetric{LagOneAutocorrelation},       evaldata, arr_runlogs_for_simulation[k], seg_indeces, bagsamples))

                    #     @assert(ind_train == length(logl_train_arr))
                    #     @assert(ind_test == length(logl_test_arr))

                    #     median_logl_train = median(logl_train_arr)
                    #     median_logl_test = median(logl_test_arr)

                    #     push!(df_results[model_name], [model_name, fold_test, dset_percentage, nframes_train_reduced, nframes_test, nsegs_test, dset_filepath_modifier, median_logl_train, median_logl_test,
                    #                                    rwse_speed_test, rwse_dcl_test, rwse_headway_test, smooth_sumsquare, smooth_autocor, smooth_jerkinvs])
                    # end
                #     # toc()
                # end
            end
        end

        # print results
        println(model_name)
        println(df_results)

        # export results
        outpath = joinpath(".", "results", "data_vs_performance_metrics_" * model_output_name * ".csv")
        writetable(outpath, df_results)
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
println("DONE")
toc()