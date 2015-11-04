using AutomotiveDrivingModels

using RandomForestBehaviors
using DynamicBayesianNetworkBehaviors
using PyPlot
pygui(true)

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

behaviorset = BehaviorSet()
model_param_sets = Dict{String, BehaviorParameterSet}()
add_behavior!(behaviorset, VehicleBehaviorGaussian, "Static Gaussian")
# model_param_sets["Static Gaussian"] = BehaviorParameterSet()
# add_behavior!(behaviorset, VehicleBehaviorLinearGaussian, "Linear Gaussian")
# model_param_sets["Linear Gaussian"] = BehaviorParameterSet(
#     convert(Vector{(Symbol,Any)}, [(:indicators,INDICATOR_SET)]),
#     [BehaviorParameter(:ridge_regression_constant, linspace(0.0,1.0,20), 5)]
#     )
rf_params = Dict{Symbol, Any}()
rf_params[:ntrees] = 5
rf_params[:max_depth] = 5
rf_params[:min_samples_split] = 20
rf_params[:min_samples_leaves] = 10
rf_params[:min_split_improvement] = 0.1
rf_params[:partial_sampling] = 0.9
rf_params[:n_split_tries] = 1000
rf_params[:indicators] = INDICATOR_SET
add_behavior!(behaviorset, GindeleRandomForestBehavior, "Random Forest", rf_params)
# model_param_sets["Random Forest"] = BehaviorParameterSet(
#     convert(Vector{(Symbol,Any)}, [(:indicators,INDICATOR_SET)]),
#     # [BehaviorParameter(:ntrees, 1:5:51, 2),
#     #  BehaviorParameter(:max_depth, 1:20, 5),
#     #  BehaviorParameter(:min_samples_split, 10:10:50, 3),
#     #  BehaviorParameter(:min_samples_leaves, [2,4,10,20,50], 3),
#     #  BehaviorParameter(:min_split_improvement, [10.0, 5.0, 1.0,0.5,0.1,0.0], 5),
#     #  BehaviorParameter(:partial_sampling, [0.5,0.6,0.7,0.8,0.9,0.95,1.0], 5),
#     #  BehaviorParameter(:n_split_tries, [10,25,50,100,200,500,1000], 7),]
#     [BehaviorParameter(:ntrees, [1], 1),
#      BehaviorParameter(:max_depth, [1], 1),
#      BehaviorParameter(:min_samples_split, [20], 1),
#      BehaviorParameter(:min_samples_leaves, [10], 1),
#      BehaviorParameter(:min_split_improvement, [0.1], 1),
#      BehaviorParameter(:partial_sampling, [0.9], 1),
#      BehaviorParameter(:n_split_tries, [1000], 1),]
#     )
# add_behavior!(behaviorset, DynamicForestBehavior, "Dynamic Forest")
# model_param_sets["Dynamic Forest"] = BehaviorParameterSet(
#     convert(Vector{(Symbol,Any)}, [(:indicators,INDICATOR_SET)]),
#     [BehaviorParameter(:ntrees, 1:5:51, 3),
#      BehaviorParameter(:max_depth, 1:20, 5),
#      BehaviorParameter(:min_samples_split, 10:10:50, 3),
#      BehaviorParameter(:min_samples_leaves, [2,4,10,20,50], 3),
#      BehaviorParameter(:min_split_improvement, [10.0, 5.0, 1.0,0.5,0.1,0.0], 3),
#      BehaviorParameter(:partial_sampling, [0.5,0.6,0.7,0.8,0.9,0.95,1.0], 5),
#      BehaviorParameter(:n_split_tries, [10,25,50,100,200,500,1000], 5),]
#     )
# add_behavior!(behaviorset, GMRBehavior, "Mixture Regression")
# model_param_sets["Mixture Regression"] = BehaviorParameterSet(
#     convert(Vector{(Symbol,Any)}, [(:indicators,[YAW, SPEED, VELFX, VELFY, TURNRATE, ACC, ACCFX, ACCFY, A_REQ_STAYINLANE, TIME_CONSECUTIVE_THROTTLE])]),
#     [BehaviorParameter(:n_components, 2:5, 1),
#      BehaviorParameter(:max_n_indicators, 2:5, 1),
#      ]
#     )
# add_behavior!(behaviorset, DynamicBayesianNetworkBehavior, "Bayesian Network")
# model_param_sets["Bayesian Network"] = BehaviorParameterSet(
#     convert(Vector{(Symbol,Any)}, [(:indicators,INDICATOR_SET),
#                                    (:preoptimize_target_bins,true),
#                                    (:preoptimize_parent_bins,true),
#                                    (:optimize_structure,true),
#                                    (:optimize_target_bins,false),
#                                    (:optimize_parent_bins,false),
#         ]),
#     [BehaviorParameter(:ncandidate_bins, 1:5:51, 7),
#      BehaviorParameter(:max_parents, 1:20, 5)],
#     )

dset_filepath_modifier = "_subset_car_following"

METRICS_OUTPUT_FILE = joinpath(EVALUATION_DIR, "validation_results" * dset_filepath_modifier * ".jld")
MODEL_OUTPUT_JLD_FILE = joinpath(EVALUATION_DIR, "validation_models" * dset_filepath_modifier * ".jld")
TRAIN_VALIDATION_JLD_FILE = joinpath(EVALUATION_DIR, "train_validation_split" * dset_filepath_modifier * ".jld")
DATASET_JLD_FILE = joinpath(EVALUATION_DIR, "dataset" * dset_filepath_modifier * ".jld")

dset = JLD.load(DATASET_JLD_FILE, "model_training_data")

train_test_split = JLD.load(TRAIN_VALIDATION_JLD_FILE, "train_test_split")
cross_validation_split = JLD.load(TRAIN_VALIDATION_JLD_FILE, "cross_validation_split")

dset_train = dset.dataframe[train_test_split.frame_assignment.==FOLD_TRAIN, :]
dset_train_nona = dset.dataframe_nona[train_test_split.frame_assignment.==FOLD_TRAIN, :]

function eval_logl()

    println("yo")

    tic()
    models = train(behaviorset, dset_train, dset_train_nona)
    toc()

    metrics_sets_test_frames, metrics_sets_test_frames_bagged = extract_frame_metrics(
            metric_types_test_frames, metric_types_test_frames_bagged,
            models, dset, train_test_split, FOLD_TEST, true)

    # println("\tLOGL TEST")
    # for i in 1 : length(metrics_sets_test_frames)
    #     
    #     logl_b = metrics_sets_test_frames_bagged[i][1].confidence_bound
    #     @printf("\t%-20s logl %6.3f ± %6.3f\n", behaviorset.names[i], logl_μ, logl_b)
    # end
    # println("")

    metrics_sets_train_frames, metrics_sets_train_frames_bagged = extract_frame_metrics(
            metric_types_train_frames, metric_types_train_frames_bagged,
            models, dset, train_test_split, FOLD_TRAIN, true)

    # println("\tLOGL TRAIN")
    # for i in 1 : length(metrics_sets_train_frames)
    #     logl_μ = get_score(metrics_sets_train_frames[i][1])
    #     logl_b = metrics_sets_train_frames_bagged[i][1].confidence_bound
    #     @printf("\t%-20s logl %6.3f ± %6.3f\n", behaviorset.names[i], logl_μ, logl_b)
    # end
    # println("")

    logl_μ_test = get_score(metrics_sets_test_frames[2][1])
    logl_μ_train = get_score(metrics_sets_train_frames[2][1])
    logl_b_test = metrics_sets_test_frames_bagged[2][1].confidence_bound
    logl_b_train = metrics_sets_train_frames_bagged[2][1].confidence_bound

    (logl_μ_test, logl_μ_train, logl_b_test, logl_b_train)
end

# rf_params[:ntrees] = 1 # no effect
# rf_params[:max_depth] = 1 # bad bad
# rf_params[:min_samples_split] = 20
# rf_params[:min_samples_leaves] = 10
# rf_params[:min_split_improvement] = 0.1
# rf_params[:partial_sampling] = 0.9
# rf_params[:n_split_tries] = 1000

sym = :ntrees
instances = 1:10
arr_μ_test = Array(Float64, length(instances))
arr_μ_train = Array(Float64, length(instances))
arr_b_test = Array(Float64, length(instances))
arr_b_train = Array(Float64, length(instances))

for (i,v) in enumerate(instances)
    rf_params[sym] = v
    (μ_test, μ_train, b_test, b_train) = eval_logl()
    arr_μ_test[i] = μ_test
    arr_μ_train[i] = μ_train
    arr_b_test[i] = b_test
    arr_b_train[i] = b_train
end


fig = figure(facecolor="white")
ax = fig[:add_subplot](111)
ax[:errorbar](instances, arr_μ_test, yerr=arr_b_test, color="g", label="test")
ax[:errorbar](instances, arr_μ_train, yerr=arr_b_train, color="b", label="train")
ax[:set_xlabel](symbol(sym))
ax[:set_ylabel]("LOGL")
ax[:legend]()

savefig("fig.png")
run(`eog fig.png`)
println("done")