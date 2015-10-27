using AutomotiveDrivingModels

using RandomForestBehaviors
using DynamicBayesianNetworkBehaviors

#=
We are testing how much model performance degrades as the amount of data decreases
The way we are testing this is to:
 - load the full datasets for each context class
 - split them into train/test
 - train a model for each reduced-quantity context class (based on cross-validated optimal hyperparams)
 - evaluate the model on the withheld training set via the logl
 - save results to csv file
=#

##############################
# PARAMETERS
##############################

const DATASET_PERCENTAGES = logspace(-2.0, 0.0, 20)
const MAX_CV_OPT_TIME_PER_MODEL = 10.0 # [s]
const NFOLDS = 5
const METRIC_TYPES_TEST_FRAMES = [LoglikelihoodMetric]
const SCENARIO_DATASETS = [
    "_subset_car_following",
    "_subset_free_flow",
    "_subset_lane_crossing",
]
const EVALUATION_DIR = "/media/tim/DATAPART1/PublicationData/2015_TrafficEvolutionModels/realworld/"
const INDICATOR_SET = [
                    YAW, SPEED, VELFX, VELFY, #DELTA_SPEED_LIMIT,
                    D_CL, D_ML, D_MR, #D_MERGE, D_SPLIT,
                    TIMETOCROSSING_RIGHT, TIMETOCROSSING_LEFT, TIMESINCELANECROSSING, ESTIMATEDTIMETOLANECROSSING,
                    N_LANE_L, N_LANE_R, HAS_LANE_L, HAS_LANE_R,
                    TURNRATE, ACC, ACCFX, ACCFY, A_REQ_STAYINLANE,
                    TURNRATE_GLOBAL, LANECURVATURE,

                    HAS_FRONT, D_X_FRONT, D_Y_FRONT, V_X_FRONT, V_Y_FRONT, YAW_FRONT, TURNRATE_FRONT,
                    # HAS_REAR,  D_X_REAR,  D_Y_REAR,  V_X_REAR,  V_Y_REAR,  YAW_REAR,  TURNRATE_REAR,
                               D_X_LEFT,  D_Y_LEFT,  V_X_LEFT,  V_Y_LEFT,  YAW_LEFT,  TURNRATE_LEFT,
                               D_X_RIGHT, D_Y_RIGHT, V_X_RIGHT, V_Y_RIGHT, YAW_RIGHT, TURNRATE_RIGHT,
                    A_REQ_FRONT, TTC_X_FRONT, TIMEGAP_X_FRONT,
                    # A_REQ_REAR,  TTC_X_REAR,  TIMEGAP_X_REAR,
                    A_REQ_LEFT,  TTC_X_LEFT,  TIMEGAP_X_LEFT,
                    A_REQ_RIGHT, TTC_X_RIGHT, TIMEGAP_X_RIGHT,

                    SCENEVELFX,

                    TIME_CONSECUTIVE_THROTTLE, # TIME_CONSECUTIVE_BRAKE, TIME_CONSECUTIVE_ACCEL,
                    #      PASTACC250MS,      PASTACC500MS,      PASTACC750MS,      PASTACC1S,
                    # PASTTURNRATE250MS, PASTTURNRATE500MS, PASTTURNRATE750MS, PASTTURNRATE1S,
                    #    PASTVELFY250MS,    PASTVELFY500MS,    PASTVELFY750MS,    PASTVELFY1S,
                        # PASTD_CL250MS,     PASTD_CL500MS,     PASTD_CL750MS,     PASTD_CL1S,

                     #     MAXACCFX500MS,     MAXACCFX750MS,     MAXACCFX1S,     MAXACCFX1500MS,     MAXACCFX2S,     MAXACCFX2500MS,     MAXACCFX3S,     MAXACCFX4S,
                     #     MAXACCFY500MS,     MAXACCFY750MS,     MAXACCFY1S,     MAXACCFY1500MS,     MAXACCFY2S,     MAXACCFY2500MS,     MAXACCFY3S,     MAXACCFY4S,
                     #  MAXTURNRATE500MS,  MAXTURNRATE750MS,  MAXTURNRATE1S,  MAXTURNRATE1500MS,  MAXTURNRATE2S,  MAXTURNRATE2500MS,  MAXTURNRATE3S,  MAXTURNRATE4S,
                     #    MEANACCFX500MS,    MEANACCFX750MS,    MEANACCFX1S,    MEANACCFX1500MS,    MEANACCFX2S,    MEANACCFX2500MS,    MEANACCFX3S,    MEANACCFX4S,
                     #    MEANACCFY500MS,    MEANACCFY750MS,    MEANACCFY1S,    MEANACCFY1500MS,    MEANACCFY2S,    MEANACCFY2500MS,    MEANACCFY3S,    MEANACCFY4S,
                     # MEANTURNRATE500MS, MEANTURNRATE750MS, MEANTURNRATE1S, MEANTURNRATE1500MS, MEANTURNRATE2S, MEANTURNRATE2500MS, MEANTURNRATE3S, MEANTURNRATE4S,
                     #     STDACCFX500MS,     STDACCFX750MS,     STDACCFX1S,     STDACCFX1500MS,     STDACCFX2S,     STDACCFX2500MS,     STDACCFX3S,     STDACCFX4S,
                     #     STDACCFY500MS,     STDACCFY750MS,     STDACCFY1S,     STDACCFY1500MS,     STDACCFY2S,     STDACCFY2500MS,     STDACCFY3S,     STDACCFY4S,
                     #  STDTURNRATE500MS,  STDTURNRATE750MS,  STDTURNRATE1S,  STDTURNRATE1500MS,  STDTURNRATE2S,  STDTURNRATE2500MS,  STDTURNRATE3S,  STDTURNRATE4S,
                ]

df_results = DataFrame()
df_results[:dataset_percentage] = Float64[]
df_results[:nframes] = Int[]
df_results[:context_class] = String[]
df_results[:logl_train] = Float64[]
df_results[:logl_test] = Float64[]
df_results[:model_name] = String[]

behaviorset = BehaviorSet()
model_param_sets = Dict{String, BehaviorParameterSet}()
add_behavior!(behaviorset, VehicleBehaviorGaussian, "Static Gaussian")
model_param_sets["Static Gaussian"] = BehaviorParameterSet()
add_behavior!(behaviorset, VehicleBehaviorLinearGaussian, "Linear Gaussian")
model_param_sets["Linear Gaussian"] = BehaviorParameterSet(
    convert(Vector{(Symbol,Any)}, [(:indicators,INDICATOR_SET)]),
    [BehaviorParameter(:ridge_regression_constant, linspace(0.0,1.0,20), 5)]
    )
add_behavior!(behaviorset, GindeleRandomForestBehavior, "Random Forest")
model_param_sets["Random Forest"] = BehaviorParameterSet(
    convert(Vector{(Symbol,Any)}, [(:indicators,INDICATOR_SET)]),
    [BehaviorParameter(:ntrees, 1:5:51, 3),
     BehaviorParameter(:max_depth, 1:20, 5),
     BehaviorParameter(:min_samples_split, 10:10:50, 3),
     BehaviorParameter(:min_samples_leaves, [2,4,10,20,50], 3),
     BehaviorParameter(:min_split_improvement, [10.0, 5.0, 1.0,0.5,0.1,0.0], 3),
     BehaviorParameter(:partial_sampling, [0.5,0.6,0.7,0.8,0.9,0.95,1.0], 5),
     BehaviorParameter(:n_split_tries, [10,25,50,100,200,500,1000], 5),]
    )
add_behavior!(behaviorset, DynamicForestBehavior, "Dynamic Forest")
model_param_sets["Dynamic Forest"] = BehaviorParameterSet(
    convert(Vector{(Symbol,Any)}, [(:indicators,INDICATOR_SET)]),
    [BehaviorParameter(:ntrees, 1:5:51, 3),
     BehaviorParameter(:max_depth, 1:20, 5),
     BehaviorParameter(:min_samples_split, 10:10:50, 3),
     BehaviorParameter(:min_samples_leaves, [2,4,10,20,50], 3),
     BehaviorParameter(:min_split_improvement, [10.0, 5.0, 1.0,0.5,0.1,0.0], 3),
     BehaviorParameter(:partial_sampling, [0.5,0.6,0.7,0.8,0.9,0.95,1.0], 5),
     BehaviorParameter(:n_split_tries, [10,25,50,100,200,500,1000], 5),]
    )
add_behavior!(behaviorset, GMRBehavior, "Gaussian Mixture Regression")
model_param_sets["Gaussian Mixture Regression"] = BehaviorParameterSet(
    convert(Vector{(Symbol,Any)}, [(:indicators,[YAW, SPEED, VELFX, VELFY, TURNRATE, ACC, ACCFX, ACCFY, A_REQ_STAYINLANE, TIME_CONSECUTIVE_THROTTLE])]),
    [BehaviorParameter(:n_components, 2:10, 3),
     BehaviorParameter(:max_n_indicators, 2:8, 1),
     #BehaviorParameter(:Σ_type, [:full, :diag], 1),
     ]
    )
add_behavior!(behaviorset, DynamicBayesianNetworkBehavior, "Bayesian Network")
model_param_sets["Bayesian Network"] = BehaviorParameterSet(
    convert(Vector{(Symbol,Any)}, [(:indicators,INDICATOR_SET),
                                   (:preoptimize_target_bins,true),
                                   (:preoptimize_parent_bins,true),
                                   (:optimize_structure,true),
                                   (:optimize_target_bins,false),
                                   (:optimize_parent_bins,false),
        ]),
    [BehaviorParameter(:ncandidate_bins, 1:5:51, 7),
     BehaviorParameter(:max_parents, 1:20, 5)],
    )

for dset_filepath_modifier in SCENARIO_DATASETS

    context_class = dset_filepath_modifier
    println("context_class: ", context_class)

    METRICS_OUTPUT_FILE = joinpath(EVALUATION_DIR, "validation_results" * dset_filepath_modifier * ".jld")
    MODEL_OUTPUT_JLD_FILE = joinpath(EVALUATION_DIR, "validation_models" * dset_filepath_modifier * ".jld")
    TRAIN_VALIDATION_JLD_FILE = joinpath(EVALUATION_DIR, "train_validation_split" * dset_filepath_modifier * ".jld")
    DATASET_JLD_FILE = joinpath(EVALUATION_DIR, "dataset" * dset_filepath_modifier * ".jld")

    dset = JLD.load(DATASET_JLD_FILE, "model_training_data")
    train_test_split = JLD.load(TRAIN_VALIDATION_JLD_FILE, "train_test_split")

    # extract training set
    df_train = dset.dataframe[train_test_split.frame_assignment .== FOLD_TRAIN,:]
    nframes_train = nrow(df_train)

    println("nframes train: ", nframes_train)
    println("nframes test:  ", calc_fold_size(FOLD_TEST, train_test_split.frame_assignment, true))

    ##############################
    # TRAIN MODELS
    ##############################

    for dset_percentage in DATASET_PERCENTAGES

        nframes_train_reduced = int(nframes_train * dset_percentage)
        @assert(nframes_train_reduced ≤ nframes_train)

        train_test_split_copy = deepcopy(train_test_split)
        for nframes_train_partial in nframes_train : -1 : nframes_train_reduced + 1
            remove_this_train_index = rand(1:nframes_train_partial)

            i = 0
            for (j,v) in enumerate(train_test_split_copy.frame_assignment)
                if v == FOLD_TRAIN
                    i += 1
                    if i == remove_this_train_index
                        train_test_split_copy.frame_assignment[j] = 0
                        break
                    end
                end
            end
        end
        @assert(calc_fold_size(FOLD_TRAIN, train_test_split_copy.frame_assignment, true) == nframes_train_reduced)

        cross_validation_split = get_cross_validation_fold_assignment(NFOLDS, dset, train_test_split_copy)

        println("\tdset_percentage: ", dset_percentage, "   nframes train reduced: ", nframes_train_reduced)

        tic()
        models = train(behaviorset, dset, train_test_split_copy, cross_validation_split, model_param_sets, max_cv_opt_time_per_model=MAX_CV_OPT_TIME_PER_MODEL)
        toc()

        metrics_sets_test_frames = extract_metrics(METRIC_TYPES_TEST_FRAMES, models,
                                                   dset, train_test_split, FOLD_TEST, true)
        metrics_sets_train_frames = extract_metrics(METRIC_TYPES_TEST_FRAMES, models,
                                                   dset, train_test_split, FOLD_TEST, false)

        for (i, model_name) in enumerate(behaviorset.names)
            logl_train = (metrics_sets_train_frames[i][1]::LoglikelihoodMetric).logl
            logl_test = (metrics_sets_test_frames[i][1]::LoglikelihoodMetric).logl
            push!(df_results, [dset_percentage, nframes_train_reduced, context_class, logl_train, logl_test, model_name])
        end

        println(df_results)
    end
end

writetable("results/data_vs_performance_metrics.csv", df_results)