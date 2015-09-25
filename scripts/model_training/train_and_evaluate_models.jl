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

const AM_ON_TULA = gethostname() == "tula"
const INCLUDE_FILE = AM_ON_TULA ?
                        "/home/wheelert/PublicationData/2015_TrafficEvolutionModels/vires_highway_2lane_sixcar/extract_params.jl" :
                        "/media/tim/DATAPART1/PublicationData/2015_TrafficEvolutionModels/vires_highway_2lane_sixcar/extract_params.jl"
const INCLUDE_NAME = splitdir(splitext(INCLUDE_FILE)[1])[2]

include(INCLUDE_FILE)

const SAVE_FILE_MODIFIER = ""
const METRICS_OUTPUT_FILE = joinpath(EVALUATION_DIR, "validation_results" * SAVE_FILE_MODIFIER * ".jld")
const MODEL_OUTPUT_JLD_FILE = joinpath(EVALUATION_DIR, "validation_models" * SAVE_FILE_MODIFIER * ".jld")
const TRAIN_VALIDATION_JLD_FILE = joinpath(EVALUATION_DIR, "train_validation_split" * SAVE_FILE_MODIFIER * ".jld")
const DATASET_JLD_FILE = joinpath(EVALUATION_DIR, "dataset_small" * SAVE_FILE_MODIFIER * ".jld")

println("results will be exported to:")
println("\t", METRICS_OUTPUT_FILE)

################################
# LOAD TRAIN AND VALIDATION SETS
################################

pdset_filepaths, streetnet_filepaths, pdset_segments, dataframe, startframes, extract_params_loaded =
            load_pdsets_streetnets_segements_and_dataframe(DATASET_JLD_FILE)

frame_tv_assignment, pdsetseg_tv_assignment = let
    jld = JLD.load(TRAIN_VALIDATION_JLD_FILE)
    frame_tv_assignment = jld["frame_tv_assignment"]
    pdsetseg_tv_assignment = jld["pdsetseg_tv_assignment"]
    (frame_tv_assignment, pdsetseg_tv_assignment)
end

if AM_ON_TULA
    # replace foldernames
    for (i,str) in enumerate(pdset_filepaths)
        pdset_filepaths[i] = joinpath(EVALUATION_DIR, "pdsets", splitdir(str)[2])
    end

    for (i,str) in enumerate(streetnet_filepaths)
        streetnet_filepaths[i] = joinpath(EVALUATION_DIR, "streetmaps", splitdir(str)[2])
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

models = train(behaviorset, dataframe[frame_tv_assignment.==1, :])

JLD.save(MODEL_OUTPUT_JLD_FILE,
        "behaviorset", behaviorset,
        "models", models,
        )

##############################
# COMPUTE VALIDATION METRICS
##############################

evalparams = EvaluationParams(SIM_HISTORY_IN_FRAMES, SIMPARAMS, HISTOBIN_PARAMS)

pdsets = Array(PrimaryDataset, length(pdset_filepaths))
for (i,pdset_filepath) in enumerate(pdset_filepaths)
    pdsets[i] = load(pdset_filepath, "pdset")
end

streetnets = Array(StreetNetwork, length(streetnet_filepaths))
for (i,streetnet_filepath) in enumerate(streetnet_filepaths)
    streetnets[i] = load(streetnet_filepath, "streetmap")
end

pdsets_for_simulation = deepcopy(pdsets)

nbehaviors = length(behaviorset.behaviors)

metrics_sets_train = create_metrics_sets_no_tracemetrics(models, pdsets, pdsets_for_simulation,
                                                              streetnets, pdset_segments, evalparams,
                                                              1, pdsetseg_tv_assignment, true)
metrics_sets_validation = create_metrics_sets_no_tracemetrics(models, pdsets, pdsets_for_simulation,
                                                              streetnets, pdset_segments, evalparams,
                                                              2, pdsetseg_tv_assignment, true)

JLD.save(METRICS_OUTPUT_FILE,
            "metrics_sets_train", metrics_sets_train,
            "metrics_sets_validation", metrics_sets_validation,
            "evalparams", evalparams,
            "nframes", size(dataframe, 1),
            "npdset_segments", length(pdset_segments)
            )

# println("DONE")
println("DONE")