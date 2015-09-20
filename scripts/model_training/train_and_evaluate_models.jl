using AutomotiveDrivingModels

using RandomForestBehaviors
using DynamicBayesianNetworkBehaviors

##############################
# PARAMETERS
##############################

const INCLUDE_FILE = let
    hostname = gethostname()
    if hostname == "Cupertino"
        "/media/tim/DATAPART1/PublicationData/2015_TrafficEvolutionModels/vires_highway_2lane_sixcar/extract_params.jl"
    elseif hostname == "tula"
        "/home/wheelert/PublicationData/2015_TrafficEvolutionModels/vires_highway_2lane_sixcar/extract_params.jl"
    else
        "unknown"
    end
end
const INCLUDE_NAME = splitdir(splitext(INCLUDE_FILE)[1])[2]

include(INCLUDE_FILE)

const SAVE_FILE_MODIFIER = "_test"
const INCREMENTAL_SAVE_FILE = joinpath(EVALUATION_DIR, "incremental_save" * SAVE_FILE_MODIFIER * ".jld")
const CVMETRIC_SAVE_FILE = joinpath(EVALUATION_DIR, "cross_validation_metrics" * SAVE_FILE_MODIFIER * ".jld")
const DATASET_SUMMARY_PATH = joinpath(EVALUATION_DIR, "dataset_summary" * SAVE_FILE_MODIFIER)

println("results will be exported to:")
println("\t", INCREMENTAL_SAVE_FILE)
println("\t", CVMETRIC_SAVE_FILE)
println("\t", DATASET_SUMMARY_PATH)

##############################
# DATA
##############################

# dataset_filepath = joinpath(EVALUATION_DIR, "dataset_small.jld")
# dataset_filepath = joinpath(EVALUATION_DIR, "dataset_medium.jld")
dataset_filepath = joinpath(EVALUATION_DIR, "dataset.jld")
pdsets, streetnets, pdset_segments, dataframe, startframes, extract_params_loaded =
            load_pdsets_streetnets_segements_and_dataframe(dataset_filepath)

frame_assignment, pdsetseg_fold_assignment = cross_validation_sets(NFOLDS, pdset_segments, dataframe, startframes)

##############################
# DATASET VISUALIZATION
##############################

# using PGFPlots
# for key in (:f_accel_250ms, :f_des_angle_250ms, :speed, :d_x_front)

#     xlabel = replace(string(key), "_", " ") * " [" * units(symbol2feature(key)) * "]"
#     filepath = DATASET_SUMMARY_PATH * string(key) * ".pdf"
#     println(filepath)
#     tic()

#     if isfile(filepath)
#         rm(filepath)
#     end
#     save(filepath, Axis(Plots.Histogram(convert(Vector{Float64}, dataframe[key]), bins=10), ymin=0, xlabel=xlabel))

#     toc()
# end

# using PyPlot

# fig = figure(facecolor="white")
# ax = fig[:add_subplot](111)
# ax[:hist](convert(Vector{Float64}, filter!(v->!isinf(v), dropna(dataframe[:f_accel_250ms]))))
# ax[:set_xlabel]("acceleration [m/s2]")

# fig = figure(facecolor="white")
# ax = fig[:add_subplot](111)
# ax[:hist](convert(Vector{Float64}, filter!(v->!isinf(v), dropna(dataframe[:f_des_angle_250ms]))))
# ax[:set_xlabel]("f_des_angle_250ms [rad]")

# fig = figure(facecolor="white")
# ax = fig[:add_subplot](111)
# ax[:hist](convert(Vector{Float64}, filter!(v->!isinf(v), dropna(dataframe[:speed]))))
# ax[:set_xlabel]("speed [m/s]")

# fig = figure(facecolor="white")
# ax = fig[:add_subplot](111)
# ax[:hist](convert(Vector{Float64}, filter!(v->!isinf(v), dropna(dataframe[:d_x_front]))))
# ax[:set_xlabel]("d x front [m]")

# sleep(10000)

##############################
# MODELS
##############################

behaviorset = BehaviorSet()
add_behavior!(behaviorset, VehicleBehaviorGaussian, "Gaussian Filter")
# add_behavior!(behaviorset, GindeleRandomForestBehavior, "Random Forest",
#     [:indicators=>INDICATOR_SET])

# add_behavior!(behaviorset, DynamicBayesianNetworkBehavior, "Bayesian Network None",
#     [:preoptimize_target_bins=>false, :preoptimize_parent_bins=>false, :optimize_structure=>false, :optimize_target_bins=>false, :optimize_parent_bins=>false, :indicators=>INDICATOR_SET])
# add_behavior!(behaviorset, DynamicBayesianNetworkBehavior, "Bayesian Network Bare",
#     [:preoptimize_target_bins=>true,  :preoptimize_parent_bins=>false, :optimize_structure=>false, :optimize_target_bins=>false, :optimize_parent_bins=>false, :indicators=>INDICATOR_SET])
# add_behavior!(behaviorset, DynamicBayesianNetworkBehavior, "Bayesian Network Struct",
#     [:preoptimize_target_bins=>false, :preoptimize_parent_bins=>false, :optimize_structure=>true,  :optimize_target_bins=>false, :optimize_parent_bins=>false, :indicators=>INDICATOR_SET])
# add_behavior!(behaviorset, DynamicBayesianNetworkBehavior, "Bayesian Network PreOptT Struct",
#     [:preoptimize_target_bins=>true,  :preoptimize_parent_bins=>false, :optimize_structure=>true,  :optimize_target_bins=>false, :optimize_parent_bins=>false, :indicators=>INDICATOR_SET])
# add_behavior!(behaviorset, DynamicBayesianNetworkBehavior, "Bayesian Network PreOptI Struct",
#     [:preoptimize_target_bins=>false, :preoptimize_parent_bins=>true,  :optimize_structure=>true,  :optimize_target_bins=>false, :optimize_parent_bins=>false, :indicators=>INDICATOR_SET])
# add_behavior!(behaviorset, DynamicBayesianNetworkBehavior, "Bayesian Network PreOptTI Struct",
    # [:preoptimize_target_bins=>true,  :preoptimize_parent_bins=>true,  :optimize_structure=>true,  :optimize_target_bins=>false, :optimize_parent_bins=>false, :indicators=>INDICATOR_SET])

# discretizers_13_9 = deepcopy(DEFAULT_DISCRETIZERS)
# discretizers_13_9[:f_des_angle_250ms] = LinearDiscretizer(linspace(-0.09,0.09,13+1), Int)
# discretizers_13_9[:f_accel_250ms] = LinearDiscretizer(linspace(-3.25,1.8,9+1), Int)
# add_behavior!(behaviorset, DynamicBayesianNetworkBehavior, "BN PreOptT Struct 13-9",
#     [:preoptimize_target_bins=>true,  :preoptimize_parent_bins=>false, :optimize_structure=>true,  :optimize_target_bins=>false, :optimize_parent_bins=>false, :indicators=>INDICATOR_SET,
#      :discretizerdict=>discretizers_13_9])

# add_behavior!(behaviorset, DynamicBayesianNetworkBehavior, "Bayesian Network Struct OptT, Target",
#     [:preoptimize_target_bins=>false, :preoptimize_parent_bins=>false, :optimize_structure=>true,  :optimize_target_bins=>true,  :optimize_parent_bins=>false, :indicators=>INDICATOR_SET])
# add_behavior!(behaviorset, DynamicBayesianNetworkBehavior, "Bayesian Network Struct OptI, Target",
#     [:preoptimize_target_bins=>false, :preoptimize_parent_bins=>false, :optimize_structure=>true,  :optimize_target_bins=>false, :optimize_parent_bins=>true,  :indicators=>INDICATOR_SET])
# add_behavior!(behaviorset, DynamicBayesianNetworkBehavior, "Bayesian Network Struct OptTI, Target",
#     [:preoptimize_target_bins=>false, :preoptimize_parent_bins=>false, :optimize_structure=>true,  :optimize_target_bins=>true,  :optimize_parent_bins=>true,  :indicators=>INDICATOR_SET])

##############################
# EVALUATION
##############################

evalparams = EvaluationParams(SIM_HISTORY_IN_FRAMES, SIMPARAMS, HISTOBIN_PARAMS)

# Profile.clear()  # in case we have any previous profiling data
# Profile.init(n=10^7)
# tic()
# @profile CarEM.cross_validate_parallel(behaviorset, pdsets, streetnets,
#                                        pdset_segments, dataframe,
#                                        frame_assignment, pdsetseg_fold_assignment, evalparams)
# toc()

# using ProfileView
# ProfileView.view()
# sleep(100000000)
# exit()

# tic()
# cross_validation_metrics = CarEM.cross_validate(behaviorset, pdsets, streetnets,
#                                                 pdset_segments, dataframe,
#                                                 frame_assignment, pdsetseg_fold_assignment, evalparams)
#                                                 verbosity=1)
#                                                 # , incremental_model_save_file=INCREMENTAL_SAVE_FILE
# toc()

tic()
cross_validation_metrics = cross_validate(behaviorset, pdsets, streetnets,
                                          pdset_segments, dataframe,
                                          frame_assignment, pdsetseg_fold_assignment, evalparams,
                                          incremental_model_save_file=INCREMENTAL_SAVE_FILE)
toc()


JLD.save(CVMETRIC_SAVE_FILE,
            "cross_validation_metrics", cross_validation_metrics,
            "behaviorset", behaviorset,
            "evalparams", evalparams,
            "nframes", size(dataframe, 1),
            "npdset_segments", length(pdset_segments),
            "nfolds", NFOLDS
            )

println("DONE")
println("DONE")