using AutomotiveDrivingModels

using RandomForestBehaviors
using DynamicBayesianNetworkBehaviors

#=
1 - load the full datasets for each scenario used
2 - train a model using its optimal hyperparameter set on each dataset amount
3 - compute the validation log likelihood
4 - save results to a .csv file in results/
=#

##############################
# PARAMETERS
##############################


const SCENARIO_DATASETS = [
    # "/media/tim/DATAPART1/PublicationData/2015_TrafficEvolutionModels/vires_highway_1lane_freeflow/dataset.jld",
    # "/media/tim/DATAPART1/PublicationData/2015_TrafficEvolutionModels/vires_highway_2lane_sixcar/dataset_small.jld",
    "/media/tim/DATAPART1/PublicationData/2015_TrafficEvolutionModels/realworld/dataset_subset_car_following.jld",
    "/media/tim/DATAPART1/PublicationData/2015_TrafficEvolutionModels/realworld/dataset_subset_free_flow.jld",
    "/media/tim/DATAPART1/PublicationData/2015_TrafficEvolutionModels/realworld/dataset_subset_lane_crossing.jld",
]
const OPTIMAL_MODEL_PARAMS = "/home/tim/.julia/v0.3/AutomotiveDrivingModels/scripts/model_training/opt_model_params.jld"
const DATASET_PERCENTAGES = logspace(-2.0, 0.0, 10)

# construct a mega dset
dset = JLD.load(SCENARIO_DATASETS[1], "model_training_data")
for i = 2 : length(SCENARIO_DATASETS)
    append!(dset, JLD.load(SCENARIO_DATASETS[i], "model_training_data"))
end

# construct a train/test split
srand(1)
TRAIN_TEST_SPLIT_TEST_FRACTION = 0.2
train_test_split = get_train_test_fold_assignment(TRAIN_TEST_SPLIT_TEST_FRACTION, dset)

df_train = dset.dataframe[train_test_split.frame_assignment .== FOLD_TRAIN,:]

# TODO(tim): load optimal behavior set params from file
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

behaviorset = BehaviorSet()
add_behavior!(behaviorset, VehicleBehaviorGaussian, "Gaussian Filter")
add_behavior!(behaviorset, VehicleBehaviorLinearGaussian, "Single Variable",
    [:indicators=>INDICATOR_SET,
     :ridge_regression_constant=>0.1,
    ])
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
     :max_depth=>2,
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

##############################
# TRAIN MODELS
##############################

df_results = DataFrame(dataset_amount=DATASET_PERCENTAGES, nframes=Array(Int, length(DATASET_PERCENTAGES)))
for name in behaviorset.names
    df_results[symbol(name)] = Array(Float64, length(DATASET_PERCENTAGES))
end

metric_types_test_frames = [LoglikelihoodMetric]

nframes_train = nrow(df_train)
println("max n training frames: ", nframes_train)

for i in 1 : nrow(df_results)

    dataset_percent = df_results[i, :dataset_amount]::Float64

    nframes = int(nframes_train * dataset_percent)
    @assert(nframes ≤ nframes_train)

    println(i, ")  ", dataset_percent, "% -> ", nframes, " frames")

    models = train(behaviorset, df_train[1:nframes, :])
    metrics_sets_test_frames = extract_metrics(metric_types_test_frames, models,
                                               dset, train_test_split, FOLD_TEST, true)

    df_results[i, :nframes] = nframes
    for (metric_set, name) in zip(metrics_sets_test_frames, behaviorset.names)
        df_results[i, symbol(name)] = (metric_set[1]::LoglikelihoodMetric).logl
    end

    println(df_results)
end

writetable("results/data_vs_performance_metrics.csv", df_results)