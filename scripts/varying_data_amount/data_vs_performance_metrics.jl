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

const TRAIN_TEST_SPLIT_TEST_FRACTION = 0.1
const OPTIMAL_MODEL_PARAMS = "/home/tim/.julia/v0.3/AutomotiveDrivingModels/scripts/model_training/opt_model_params.jld"
const DATASET_PERCENTAGES = logspace(-2.0, 0.0, 21)
const METRIC_TYPES_TEST_FRAMES = [LoglikelihoodMetric]
const SCENARIO_DATASETS = [
    # "/media/tim/DATAPART1/PublicationData/2015_TrafficEvolutionModels/vires_highway_1lane_freeflow/dataset.jld",
    # "/media/tim/DATAPART1/PublicationData/2015_TrafficEvolutionModels/vires_highway_2lane_sixcar/dataset_small.jld",
    "/media/tim/DATAPART1/PublicationData/2015_TrafficEvolutionModels/realworld/dataset_subset_car_following.jld",
    "/media/tim/DATAPART1/PublicationData/2015_TrafficEvolutionModels/realworld/dataset_subset_free_flow.jld",
    "/media/tim/DATAPART1/PublicationData/2015_TrafficEvolutionModels/realworld/dataset_subset_lane_crossing.jld",
]
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

for scenario_datasetpath in SCENARIO_DATASETS

    context_class = splitdir(scenario_datasetpath)[2]
    println("context_class: ", context_class)

    dset = JLD.load(scenario_datasetpath, "model_training_data")
    println(dset)

    # construct a train/test split
    srand(1)
    train_test_split = get_train_test_fold_assignment(TRAIN_TEST_SPLIT_TEST_FRACTION, dset)
    df_train = dset.dataframe[train_test_split.frame_assignment .== FOLD_TRAIN,:]
    nframes_train = nrow(df_train)

    println("nframes train: ", nframes_train)
    println("nframes test:  ", calc_fold_size(FOLD_TEST, train_test_split.frame_assignment, true))

    # TODO(tim): determine optimal behavior set params using CV

    behaviorset = BehaviorSet()
    add_behavior!(behaviorset, VehicleBehaviorGaussian, "Static Gaussian")
    add_behavior!(behaviorset, VehicleBehaviorLinearGaussian, "Linear Gaussian",
        [:indicators=>INDICATOR_SET,
         :ridge_regression_constant=>0.3157894736842105,
        ])
    add_behavior!(behaviorset, GindeleRandomForestBehavior, "Random Forest",
        [:indicators=>INDICATOR_SET,
         :ntrees=>31,
         :max_depth=>20,
         :min_samples_split=>10,
         :min_samples_leaves=>2,
         :min_split_improvement=>0.0,
         :partial_sampling=>1.0,
         :n_split_tries=>1000,
        ])
    add_behavior!(behaviorset, DynamicForestBehavior, "Dynamic Forest",
        [:indicators=>INDICATOR_SET,
         :ntrees=>20,
         :max_depth=>10,
         :min_samples_split=>10,
         :min_samples_leaves=>8,
         :min_split_improvement=>0.0,
         :partial_sampling=>0.8,
         :n_split_tries=>1000,
        ])
    add_behavior!(behaviorset, DynamicBayesianNetworkBehavior, "Bayesian Network",
        [:indicators=>INDICATOR_SET,
         :preoptimize_target_bins=>true,
         :preoptimize_parent_bins=>true,
         :optimize_structure=>true,
         :optimize_target_bins=>false,
         :optimize_parent_bins=>false,
         :ncandidate_bins=>35,
         :max_parents=>5,
         # :nbins_lat=>5,
         # :nbins_lon=>5,
         ])

    ##############################
    # TRAIN MODELS
    ##############################

    for dset_percentage in DATASET_PERCENTAGES

        nframes_train_reduced = int(nframes_train * dset_percentage)
        @assert(nframes_train_reduced ≤ nframes_train)

        println("\tdset_percentage: ", dset_percentage, "   nframes train reduced: ", nframes_train_reduced)

        models = train(behaviorset, df_train[1:nframes_train_reduced, :])
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