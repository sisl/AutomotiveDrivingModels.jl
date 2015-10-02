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

const SAVE_FILE_MODIFIER = ""
const METRICS_OUTPUT_FILE = joinpath(EVALUATION_DIR, "validation_results" * SAVE_FILE_MODIFIER * ".jld")
const MODEL_OUTPUT_JLD_FILE = joinpath(EVALUATION_DIR, "validation_models" * SAVE_FILE_MODIFIER * ".jld")
const TRAIN_VALIDATION_JLD_FILE = joinpath(EVALUATION_DIR, "train_validation_split" * SAVE_FILE_MODIFIER * ".jld")
const DATASET_JLD_FILE = joinpath(EVALUATION_DIR, "dataset" * SAVE_FILE_MODIFIER * ".jld")

println("results will be exported to:")
println("\t", METRICS_OUTPUT_FILE)

################################
# LOAD TRAIN AND VALIDATION SETS
################################


dset = JLD.load(DATASET_JLD_FILE, "model_training_data")
train_test_split = JLD.load(TRAIN_VALIDATION_JLD_FILE, "train_test_split")

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

behaviorset = BehaviorSet()
add_behavior!(behaviorset, VehicleBehaviorGaussian, "Gaussian Filter")
add_behavior!(behaviorset, GindeleRandomForestBehavior, "Random Forest",
    [:indicators=>INDICATOR_SET,
     :ntrees=>5,
     :max_depth=>5,
     :min_samples_split=>20,
     :min_samples_leaves=>10,
     :min_split_improvement=>0.2,
     :partial_sampling=>0.7,
    ])
add_behavior!(behaviorset, DynamicForestBehavior, "Dynamic Forest",
    [:indicators=>INDICATOR_SET,
     :ntrees=>5,
     :max_depth=>5,
     :min_samples_split=>20,
     :min_samples_leaves=>10,
     :min_split_improvement=>0.2,
     :partial_sampling=>0.7,
    ])
add_behavior!(behaviorset, DynamicBayesianNetworkBehavior, "Bayesian Network",
    [:indicators=>INDICATOR_SET,
     :preoptimize_target_bins=>true,
     :preoptimize_parent_bins=>true,
     :optimize_structure=>true,
     :optimize_target_bins=>false,
     :optimize_parent_bins=>false,
     :ncandidate_bins=>20,
     ])

# TODO(tim): fix train test split.....
models = train(behaviorset, dset.dataframe[train_test_split.frame_assignment.==FOLD_TRAIN, :])

JLD.save(MODEL_OUTPUT_JLD_FILE,
        "behaviorset", behaviorset,
        "models", models,
        )

##############################
# COMPUTE VALIDATION METRICS
##############################

evalparams = EvaluationParams(SIM_HISTORY_IN_FRAMES, SIMPARAMS, HISTOBIN_PARAMS)

metric_types_train = [LoglikelihoodMetric]#, LoglikelihoodBoundsCVMetric]
metric_types_validation = [LoglikelihoodMetric]#, LoglikelihoodBaggedBoundsMetric]

# pdsets = load_pdsets(dset)
# streetnets = load_pdsets(dset)
# pdsets_for_simulation = deepcopy(pdsets)

metrics_sets_train = extract_metrics(metric_types_train, models, dset, train_test_split, FOLD_TRAIN, true)

# metrics_sets_train = create_metrics_sets_no_tracemetrics(models, pdsets, pdsets_for_simulation,
#                                                               streetnets, pdset_segments, evalparams,
#                                                               1, pdsetseg_tv_assignment, true)
# metrics_sets_validation = create_metrics_sets_no_tracemetrics(models, pdsets, pdsets_for_simulation,
#                                                               streetnets, pdset_segments, evalparams,
#                                                               2, pdsetseg_tv_assignment, true)

# JLD.save(METRICS_OUTPUT_FILE,
#             "metrics_sets_train", metrics_sets_train,
#             "metrics_sets_validation", metrics_sets_validation,
#             "evalparams", evalparams,
#             "nframes", size(dset.dataframe, 1),
#             "npdset_segments", length(dset.pdset_segments)
#             )

println(metrics_sets_train)

# println("DONE")
println("DONE")