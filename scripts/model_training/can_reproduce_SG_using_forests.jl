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

##############################
# PARAMETERS
##############################

# const INCLUDE_FILE_BASE = "vires_highway_2lane_sixcar"
const INCLUDE_FILE_BASE = "realworld"

const AM_ON_TULA = gethostname() == "tula"
const INCLUDE_FILE = AM_ON_TULA ? joinpath("/home/wheelert/PublicationData/2015_TrafficEvolutionModels", INCLUDE_FILE_BASE, "extract_params.jl") :
                                  joinpath("/media/tim/DATAPART1/PublicationData/2015_TrafficEvolutionModels", INCLUDE_FILE_BASE, "extract_params.jl")
const INCLUDE_NAME = splitdir(splitext(INCLUDE_FILE)[1])[2]

include(INCLUDE_FILE)

################################
# LOAD TRAIN AND VALIDATION SETS
################################

for dset_filepath_modifier in (
    "_subset_car_following",
    "_subset_free_flow",
    "_subset_lane_crossing",
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

    # TODO(tim): load optimal behavior set params from file
    behaviorset = BehaviorSet()
    add_behavior!(behaviorset, VehicleBehaviorGaussian, "Static Gaussian")
    # add_behavior!(behaviorset, VehicleBehaviorLinearGaussian, "Linear Gaussian",
    #     [:indicators=>INDICATOR_SET,
    #      :ridge_regression_constant=>0.3157894736842105,
    #     ])
    add_behavior!(behaviorset, GindeleRandomForestBehavior, "Random Forest",
        [:indicators=>INDICATOR_SET,
         :ntrees=>1,
         :max_depth=>0,
         :partial_sampling=>1.0,
        ])
    add_behavior!(behaviorset, DynamicForestBehavior, "Dynamic Forest",
        [:indicators=>INDICATOR_SET,
         :ntrees=>1,
         :max_depth=>0,
         :partial_sampling=>1.0,
        ])
    # add_behavior!(behaviorset, DynamicBayesianNetworkBehavior, "Bayesian Network",
    #     [:indicators=>INDICATOR_SET,
    #      :preoptimize_target_bins=>true,
    #      :preoptimize_parent_bins=>true,
    #      :optimize_structure=>true,
    #      :optimize_target_bins=>false,
    #      :optimize_parent_bins=>false,
    #      :ncandidate_bins=>35,
    #      :max_parents=>5,
    #      # :nbins_lat=>5,
    #      # :nbins_lon=>5,
    #      ])

    models = train(behaviorset, dset.dataframe[train_test_split.frame_assignment.==FOLD_TRAIN, :])
    # JLD.save(MODEL_OUTPUT_JLD_FILE,
    #         "behaviorset", behaviorset,
    #         "models", models,
    #         )

    println(models)
    continue

    # models = JLD.load(MODEL_OUTPUT_JLD_FILE, "models")

    ##############################
    # COMPUTE VALIDATION METRICS
    ##############################

    # evalparams = EvaluationParams(SIM_HISTORY_IN_FRAMES, SIMPARAMS, HISTOBIN_PARAMS)

    metric_types_test_frames = [LoglikelihoodMetric]
    metric_types_test_traces = [
                                EmergentKLDivMetric{symbol(SPEED)},
                                EmergentKLDivMetric{symbol(TIMEGAP_X_FRONT)},
                                EmergentKLDivMetric{symbol(D_CL)},
                                RootWeightedSquareError{symbol(SPEED), 1.0},
                                RootWeightedSquareError{symbol(SPEED), 2.0},
                                RootWeightedSquareError{symbol(SPEED), 3.0},
                                RootWeightedSquareError{symbol(SPEED), 4.0},
                                # RootWeightedSquareError{symbol(TIMEGAP_X_FRONT)},
                                # RootWeightedSquareError{symbol(D_CL)},
                               ]
    metric_types_test_frames_bagged = [LoglikelihoodMetric]
    metric_types_test_traces_bagged = [
                                       EmergentKLDivMetric{symbol(SPEED)},
                                       EmergentKLDivMetric{symbol(TIMEGAP_X_FRONT)},
                                       EmergentKLDivMetric{symbol(D_CL)},
                                       RootWeightedSquareError{symbol(SPEED), 1.0},
                                       RootWeightedSquareError{symbol(SPEED), 2.0},
                                       RootWeightedSquareError{symbol(SPEED), 3.0},
                                       RootWeightedSquareError{symbol(SPEED), 4.0},
                                      ]

    metric_types_cv_train_frames = DataType[]
    metric_types_cv_test_frames = [LoglikelihoodMetric]
    metric_types_cv_train_traces = DataType[]
    metric_types_cv_test_traces = DataType[]

    metrics_sets_test_frames = extract_metrics(metric_types_test_frames, models, dset, train_test_split, FOLD_TEST, true)
    metrics_sets_test_frames_bagged = extract_bagged_metrics(metric_types_test_frames_bagged, models, dset, train_test_split, FOLD_TEST, true)

    pdsets_original = load_pdsets(dset)
    streetnets = load_streetnets(dset)
    pdsets_for_simulation = deepcopy(pdsets_original)

    metrics_sets_test_traces = extract_metrics_from_traces(metric_types_test_traces, models,
                                    pdsets_original, pdsets_for_simulation, streetnets,
                                    dset.pdset_segments, train_test_split, FOLD_TEST, true) # NOTE(tim): includes realworld entry

    metrics_sets_test_traces_bagged = extract_bagged_metrics_from_traces(metric_types_test_traces, models,
                                           pdsets_original, pdsets_for_simulation, streetnets,
                                           dset.pdset_segments, train_test_split, FOLD_TEST, true)

    metrics_sets_cv = cross_validate(behaviorset, dset, cross_validation_split,
                                metric_types_cv_train_frames, metric_types_cv_test_frames,
                                metric_types_cv_train_traces, metric_types_cv_test_traces)

    JLD.save(METRICS_OUTPUT_FILE,
             "metrics_sets_test_frames", metrics_sets_test_frames,
             "metrics_sets_test_traces", metrics_sets_test_traces,
             "metrics_sets_test_frames_bagged", metrics_sets_test_frames_bagged,
             "metrics_sets_test_traces_bagged", metrics_sets_test_traces_bagged,
             "metrics_sets_cv", metrics_sets_cv,
            )
end

# println("DONE")
println("DONE")