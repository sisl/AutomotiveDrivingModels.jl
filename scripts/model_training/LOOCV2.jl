using AutomotiveDrivingModels

using RandomForestBehaviors
using DynamicBayesianNetworkBehaviors
using StreamStats

#=
1 - load the best inputs for every model
2 - train a final model on the test dataset
3 - save them
4 - compute validation metrics on the validation dataset
5 - save results to a .csv
=#

# For LOOCV2:
#=
- load full dataset
- construct split over drives
- run CV split get the best hyperparams for each model based on likelihood
- for each drive:
   - train a model on the other drives
   - compute train and test metrics (logl, emergent kldiv counts, rwse) on the withheld set
- aggregate the resuts
=#

##############################
# PARAMETERS
##############################

const INCLUDE_FILE_BASE = "realworld"
const N_SIMULATIONS_PER_TRACE = 50
const DEFAULT_TRACE_HISTORY = 2*DEFAULT_FRAME_PER_SEC
const N_BAGGING_SAMPLES = 10
const CONFIDENCE_LEVEL = 0.95

const MAX_CV_OPT_TIME_PER_MODEL = 60.0 # [s]
const AM_ON_TULA = gethostname() == "tula"
const INCLUDE_FILE = AM_ON_TULA ? joinpath("/home/wheelert/PublicationData/2015_TrafficEvolutionModels", INCLUDE_FILE_BASE, "extract_params.jl") :
                                  joinpath("/media/tim/DATAPART1/PublicationData/2015_TrafficEvolutionModels", INCLUDE_FILE_BASE, "extract_params.jl")
const INCLUDE_NAME = splitdir(splitext(INCLUDE_FILE)[1])[2]

include(INCLUDE_FILE)

################################
# METRICS
################################

metric_types_test_frames = [LoglikelihoodMetric]
metric_types_test_frames_bagged = [LoglikelihoodMetric]
metric_types_train_frames = [LoglikelihoodMetric]
metric_types_train_frames_bagged = [LoglikelihoodMetric]

metric_types_test_traces = [
                            EmergentKLDivMetric{symbol(SPEED)},
                            EmergentKLDivMetric{symbol(TIMEGAP_X_FRONT)},
                            EmergentKLDivMetric{symbol(D_CL)},
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
                            RootWeightedSquareError{symbol(TIMEGAP_X_FRONT), 0.5},
                            RootWeightedSquareError{symbol(TIMEGAP_X_FRONT), 1.0},
                            RootWeightedSquareError{symbol(TIMEGAP_X_FRONT), 1.5},
                            RootWeightedSquareError{symbol(TIMEGAP_X_FRONT), 2.0},
                            RootWeightedSquareError{symbol(TIMEGAP_X_FRONT), 2.5},
                            RootWeightedSquareError{symbol(TIMEGAP_X_FRONT), 3.0},
                            RootWeightedSquareError{symbol(TIMEGAP_X_FRONT), 3.5},
                            RootWeightedSquareError{symbol(TIMEGAP_X_FRONT), 4.0},
                           ]
metric_types_test_traces_bagged = [
                                   EmergentKLDivMetric{symbol(SPEED)},
                                   EmergentKLDivMetric{symbol(TIMEGAP_X_FRONT)},
                                   EmergentKLDivMetric{symbol(D_CL)},
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
# LOAD TRAIN AND VALIDATION SETS
################################

include(Pkg.dir("AutomotiveDrivingModels", "scripts", "model_params.jl"))
nmodels = length(behaviorset)

for dset_filepath_modifier in (
    "_subset_car_following",
    # "_subset_free_flow",
    # "_subset_lane_crossing",
    )

    println(dset_filepath_modifier)

    METRICS_OUTPUT_FILE = joinpath(EVALUATION_DIR, "validation_results" * dset_filepath_modifier * ".jld")
    MODEL_OUTPUT_JLD_FILE = joinpath(EVALUATION_DIR, "validation_models" * dset_filepath_modifier * ".jld")
    TRAIN_VALIDATION_JLD_FILE = joinpath(EVALUATION_DIR, "train_validation_split" * dset_filepath_modifier * ".jld")
    DATASET_JLD_FILE = joinpath(EVALUATION_DIR, "dataset" * dset_filepath_modifier * ".jld")

    dset = JLD.load(DATASET_JLD_FILE, "model_training_data")

    if AM_ON_TULA
        # replace foldernames
        for (i,str) in enumerate(dset.pdset_filepaths)
            dset.pdset_filepaths[i] = joinpath(EVALUATION_DIR, "pdsets", splitdir(str)[2])
        end

        for (i,str) in enumerate(dset.streetnet_filepaths)
            dset.streetnet_filepaths[i] = joinpath(EVALUATION_DIR, "streetmaps", splitdir(str)[2])
        end
    end

    split_drives = get_fold_assignment_across_drives(dset)

    preallocated_data_dict = Dict{AbstractString, AbstractVehicleBehaviorPreallocatedData}()
    for (behavior_name, train_def) in behaviorset
        preallocated_data_dict[behavior_name] = preallocate_learning_data(dset, train_def.trainparams)
    end

    ###################################
    # DETERMINE OPTIMAL PARAMS FROM CV
    ###################################

    print("optimizing hyperparameters  "); tic()
    for (behavior_name, train_def) in behaviorset
        preallocated_data = preallocated_data_dict[behavior_name]
        AutomotiveDrivingModels.optimize_hyperparams_cyclic_coordinate_ascent!(
                train_def, dset, preallocated_data, split_drives)
    end
    toc()

    ##############################
    # COMPUTE VALIDATION METRICS
    ##############################
    # run over drive folds
    #  - compute model on non-matching indeces using optimal hyperparams
    #  - update trace metrics on given traces
    #  - update mean frame likelihood

    nframes = nrow(dset.dataframe)
    ntraces = length(split_drives.pdsetseg_assignment)
    frame_logls = Array(Float64, nframes, ntraces, nmodels) # logl for each frame under each run (we can back out TRAIN and TEST)

    print("Loading sim resources "); tic()
    pdsets_original = load_pdsets(dset)
    streetnets = load_streetnets(dset)
    toc()

    foldinds = [1:ntraces]
    bagged_selection = [1:ntraces]

    println("preallocating memory for traces"); tic()
    # make pdset copies that are only as large as needed (contain history and horizon from pdsets_original)
    arr_pdsets_for_simulation = Array(Matrix{PrimaryDataset}, nmodels)
    for k = 1 : nmodels
        arr_pdsets_for_simulation[k] = Array(PrimaryDataset, ntraces, N_SIMULATIONS_PER_TRACE)
    end
    validfind_starts_sim = Array(Int, ntraces) # new validfind_start for the truncated pdsets_for_simulation
    basics = FeatureExtractBasicsPdSet(pdsets_original[1], streetnets[1])

    for (i,ind) in enumerate(foldinds)
        seg = dset.pdset_segments[ind]
        validfind_start = max(1, seg.validfind_start - DEFAULT_TRACE_HISTORY)
        pdset_sim = deepcopy(pdsets_original[seg.pdset_id], validfind_start, seg.validfind_end)
        validfind_starts_sim[i] = clamp(seg.validfind_start-DEFAULT_TRACE_HISTORY, 1, DEFAULT_TRACE_HISTORY+1)

        for k in 1 : nmodels
            for j in 1 : N_SIMULATIONS_PER_TRACE
                arr_pdsets_for_simulation[k][i,j] = deepcopy(pdset_sim)
            end
        end
    end
    toc()

    tic()
    for fold in 1 : split_drives.nfolds
        println("Fold: ", fold, "  /  ", split_drives.nfolds)

        print("\ttraining models  "); tic()
        training_frames = dset.dataframe[split_drives.frame_assignment.!=fold, :]
        training_frames_nona = dset.dataframe_nona[split_drives.frame_assignment.!=fold, :]
        models = train(behaviorset, training_frames, training_frames_nona)
        toc()

        print("\tcomputing likelihoods  "); tic()
        for (i,behavior) in enumerate(models)
            for frameind in 1 : nframes
                if trains_with_nona(behavior)
                    frame_logls[frameind, fold, i] = calc_action_loglikelihood(behavior, dset.dataframe_nona, frameind)
                else
                    frame_logls[frameind, fold, i] = calc_action_loglikelihood(behavior, dset.dataframe, frameind)
                end
            end
        end
        toc()

        println("\tsimulating"); tic()
        for (k,behavior) in enumerate(models)
            print("\t\t", behaviorset.names[k], "  "); tic()
            for i in 1 : ntraces
                if split_drives.pdsetseg_assignment[i] == fold # in test
                    # simulate
                    seg = dset.pdset_segments[i]
                    basics.sn = streetnets[seg.streetnet_id]
                    validfind_start = validfind_starts_sim[i]
                    validfind_end = validfind_start + seg.validfind_end - seg.validfind_start

                    for l in 1 : N_SIMULATIONS_PER_TRACE
                        basics.pdset = arr_pdsets_for_simulation[k][i, l]
                        basics.runid += 1

                        simulate!(basics, behavior, seg.carid, validfind_start, validfind_end)
                    end
                end
            end
            toc()
        end
        toc()
    end
    toc()

    print("Exctracting frame stats  "); tic()

    metrics_sets_test_frames = Array(Vector{BehaviorFrameMetric}, nmodels)
    metrics_sets_train_frames = Array(Vector{BehaviorFrameMetric}, nmodels)
    metrics_sets_test_frames_bagged = Array(Vector{BaggedMetricResult}, nmodels)
    metrics_sets_train_frames_bagged = Array(Vector{BaggedMetricResult}, nmodels)
    metrics_sets_test_traces = Array(Vector{BehaviorTraceMetric}, nmodels)
    metrics_sets_test_traces_bagged = Array(Vector{BaggedMetricResult}, nmodels)

    let
        for k in 1:nmodels
            print("\tmodel: ", k, "  "); tic()

            arr_logl_test = Float64[]
            arr_logl_train = Float64[]

            for j in 1 : split_drives.nfolds
                for i in 1 : nframes
                    if split_drives.frame_assignment[i] == j
                        push!(arr_logl_test, frame_logls[i,j,k])
                    elseif split_drives.frame_assignment[i] != 0
                        push!(arr_logl_train, frame_logls[i,j,k])
                    end
                end
            end

            metrics_sets_test_frames[k] = BehaviorFrameMetric[LoglikelihoodMetric(mean(arr_logl_test))]
            metrics_sets_train_frames[k] = BehaviorFrameMetric[LoglikelihoodMetric(mean(arr_logl_train))]
            metrics_sets_test_frames_bagged[k] = BaggedMetricResult[BaggedMetricResult(LoglikelihoodMetric, arr_logl_test, N_BAGGING_SAMPLES)]
            metrics_sets_train_frames_bagged[k] = BaggedMetricResult[BaggedMetricResult(LoglikelihoodMetric, arr_logl_train, N_BAGGING_SAMPLES)]

            # TRACES

            retval_straight = Array(BehaviorTraceMetric, length(metric_types_test_traces))
            retval_bagged = Array(BaggedMetricResult, length(metric_types_test_traces_bagged))
            for (i,M) in enumerate(metric_types_test_traces)
                println("\t\t", M, "  "); tic()
                retval_straight[i] = extract(M, dset.pdset_segments,
                                             pdsets_original, arr_pdsets_for_simulation[k], validfind_starts_sim,
                                             streetnets, foldinds, basics, bagged_selection)
                toc()
                if i ≤ length(retval_bagged)
                    println("\t\tbagged   "); tic()
                    retval_bagged[i] = BaggedMetricResult(M, dset.pdset_segments,
                                                 pdsets_original, arr_pdsets_for_simulation[k], validfind_starts_sim,
                                                 streetnets, foldinds, basics,
                                                 bagged_selection, N_BAGGING_SAMPLES, CONFIDENCE_LEVEL)
                    toc()
                end
            end

            metrics_sets_test_traces[k] = retval_straight
            metrics_sets_test_traces_bagged[k] = retval_bagged

            toc()
        end
    end
    toc()

    println("\tLOGL TEST")
    for i in 1 : length(metrics_sets_test_frames)
        logl_μ = get_score(metrics_sets_test_frames[i][1])
        logl_b = metrics_sets_test_frames_bagged[i][1].confidence_bound
        @printf("\t%-20s logl %6.3f ± %6.3f\n", behaviorset.names[i], logl_μ, logl_b)
    end
    println("")

    println("\tLOGL TRAIN")
    for i in 1 : length(metrics_sets_train_frames)
        logl_μ = get_score(metrics_sets_train_frames[i][1])
        logl_b = metrics_sets_train_frames_bagged[i][1].confidence_bound
        @printf("\t%-20s logl %6.3f ± %6.3f\n", behaviorset.names[i], logl_μ, logl_b)
    end
    println("")

    println("metrics_sets_test_traces: ")
    println(metrics_sets_test_traces)
    println("metrics_sets_test_traces_bagged: ")
    println(metrics_sets_test_traces_bagged)

    JLD.save(METRICS_OUTPUT_FILE,
             "metrics_sets_test_frames",         metrics_sets_test_frames,
             "metrics_sets_test_frames_bagged",  metrics_sets_test_frames_bagged,
             "metrics_sets_train_frames",        metrics_sets_train_frames,
             "metrics_sets_train_frames_bagged", metrics_sets_train_frames_bagged,
             "metrics_sets_test_traces",         metrics_sets_test_traces,
             "metrics_sets_test_traces_bagged",  metrics_sets_test_traces_bagged,
            )
end

# println("DONE")
println("DONE")
exit()
