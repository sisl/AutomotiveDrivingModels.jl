using AutomotiveDrivingModels

using RandomForestBehaviors
using DynamicBayesianNetworkBehaviors

#=
1 - load the best inputs for every model
2 - train a final model on the test dataset
3 - save them
4 - compute validation metrics on the validation dataset
5 - save results to a .csv
=#

# For LOOCV:
#=
- load full dataset
- run CV to get the best model inputs
- for each trace:
   - train a model on the other traces
   - compute the train and test metrics (logl, emergent kldiv counts, rwse) on the withheld set
- aggregate the resuts
=#

##############################
# PARAMETERS
##############################

# const INCLUDE_FILE_BASE = "vires_highway_2lane_sixcar"
const INCLUDE_FILE_BASE = "realworld"

const MAX_CV_OPT_TIME_PER_MODEL = 10.0 # [s]
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

metric_types_cv_train_frames = DataType[]
metric_types_cv_test_frames = [LoglikelihoodMetric]
metric_types_cv_train_traces = DataType[]
metric_types_cv_test_traces = DataType[]

################################
# LOAD TRAIN AND VALIDATION SETS
################################

include(Pkg.dir("AutomotiveDrivingModels", "scripts", "model_params.jl"))

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

    train_test_split = JLD.load(TRAIN_VALIDATION_JLD_FILE, "train_test_split")
    cross_validation_split = JLD.load(TRAIN_VALIDATION_JLD_FILE, "cross_validation_split")

    if AM_ON_TULA
        # replace foldernames
        for (i,str) in enumerate(dset.pdset_filepaths)
            dset.pdset_filepaths[i] = joinpath(EVALUATION_DIR, "pdsets", splitdir(str)[2])
        end

        for (i,str) in enumerate(dset.streetnet_filepaths)
            dset.streetnet_filepaths[i] = joinpath(EVALUATION_DIR, "streetmaps", splitdir(str)[2])
        end
    end

    ##############################
    # MODELS
    ##############################

    print("training models    "); tic()
    models = train(behaviorset, dset, train_test_split, cross_validation_split, model_param_sets, max_cv_opt_time_per_model=MAX_CV_OPT_TIME_PER_MODEL)
    toc()

    # print("saving models    "); tic()
    # JLD.save(MODEL_OUTPUT_JLD_FILE,
    #         "behaviorset", behaviorset,
    #         "models", models,
    #         )
    # toc()

    # models = JLD.load(MODEL_OUTPUT_JLD_FILE, "models")

    ##############################
    # COMPUTE VALIDATION METRICS
    ##############################

    print("\textracting test frames  "); tic()
    metrics_sets_test_frames, metrics_sets_test_frames_bagged = extract_frame_metrics(
            metric_types_test_frames, metric_types_test_frames_bagged,
            models, dset, train_test_split, FOLD_TEST, true)
    toc()

    println("\tLOGL TEST")
    for i in 1 : length(metrics_sets_test_frames)
        logl_μ = get_score(metrics_sets_test_frames[i][1])
        logl_b = metrics_sets_test_frames_bagged[i][1].confidence_bound
        @printf("\t%-20s logl %6.3f ± %6.3f\n", behaviorset.names[i], logl_μ, logl_b)
    end
    println("")

    print("\textracting train frames  "); tic()
    metrics_sets_train_frames, metrics_sets_train_frames_bagged = extract_frame_metrics(
            metric_types_train_frames, metric_types_train_frames_bagged,
            models, dset, train_test_split, FOLD_TRAIN, true)
    toc()

    println("\tLOGL TRAIN")
    for i in 1 : length(metrics_sets_train_frames)
        logl_μ = get_score(metrics_sets_train_frames[i][1])
        logl_b = metrics_sets_train_frames_bagged[i][1].confidence_bound
        @printf("\t%-20s logl %6.3f ± %6.3f\n", behaviorset.names[i], logl_μ, logl_b)
    end
    println("")

    exit()

    print("\tloading sim resources "); tic()
    pdsets_original = load_pdsets(dset)
    streetnets = load_streetnets(dset)
    toc()

    print("\textracting trace metrics  "); tic()
    metrics_sets_test_traces, metrics_sets_test_traces_bagged = extract_trace_metrics(
            metric_types_test_traces, metric_types_test_traces_bagged,
            models, dset, train_test_split, FOLD_TEST, true,
            pdsets_original, streetnets,
            )
    toc()

    # println("metrics_sets_test_traces: ")
    # println(metrics_sets_test_traces)
    # println("metrics_sets_test_traces_bagged: ")
    # println(metrics_sets_test_traces_bagged)

    # JLD.save(METRICS_OUTPUT_FILE,
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