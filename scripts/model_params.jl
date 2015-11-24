behaviorset = BehaviorSet()
model_trainparams = Dict{AbstractString, AbstractVehicleBehaviorTrainParams}()
model_hyperparams = Dict{AbstractString, Vector{BehaviorParameter}}()

add_behavior!(behaviorset, VehicleBehaviorGaussian, "Static Gaussian")
model_trainparams["Static Gaussian"] = SG_TrainParams()
model_hyperparams["Static Gaussian"] = BehaviorParameter[]

add_behavior!(behaviorset, VehicleBehaviorLinearGaussian, "Linear Gaussian")
model_trainparams["Linear Gaussian"] = LG_TrainParams(indicators=INDICATOR_SET)
model_hyperparams["Linear Gaussian"] = [
        BehaviorParameter(:ridge_regression_constant, collect(linspace(0.0,1.0,20)), 5)
    ]

add_behavior!(behaviorset, GindeleRandomForestBehavior, "Random Forest")
model_trainparams["Random Forest"] = RF_TrainParams(indicators=INDICATOR_SET)
model_hyperparams["Random Forest"] = [
        BehaviorParameter(:ntrees, 1:10:51, 3),
        BehaviorParameter(:max_tree_depth, 1:20, 5),
        # BehaviorParameter(:min_samples_split, 10:10:50, 3),
        # BehaviorParameter(:min_samples_leaves, [2,4,10,20,50], 3),
        # BehaviorParameter(:min_split_improvement, [10.0, 5.0, 1.0,0.5,0.1,0.0], 3),
        # BehaviorParameter(:partial_sampling, [0.5,0.6,0.7,0.8,0.9,0.95,1.0], 5),
        # BehaviorParameter(:n_split_tries, [10,25,50,100,200,500,1000], 5),
    ]

add_behavior!(behaviorset, DynamicForestBehavior, "Dynamic Forest")
model_trainparams["Dynamic Forest"] = DF_TrainParams(indicators=INDICATOR_SET)
model_hyperparams["Dynamic Forest"] = [
        BehaviorParameter(:ntrees, 1:10:51, 3),
        BehaviorParameter(:max_tree_depth, 1:20, 5),
        # BehaviorParameter(:min_samples_split, 10:10:50, 3),
        # BehaviorParameter(:min_samples_leaves, [2,4,10,20,50], 3),
        # BehaviorParameter(:min_split_improvement, [10.0, 5.0, 1.0,0.5,0.1,0.0], 3),
        # BehaviorParameter(:partial_sampling, [0.5,0.6,0.7,0.8,0.9,0.95,1.0], 5),
        # BehaviorParameter(:n_split_tries, [10,25,50,100,200,500,1000], 5),
    ]

add_behavior!(behaviorset, GMRBehavior, "Mixture Regression")
model_trainparams["Mixture Regression"] = GMR_TrainParams(indicators=[YAW, ACC, TIME_CONSECUTIVE_THROTTLE]) # [YAW, SPEED, VELFX, VELFY, TURNRATE, ACC, ACCFX, ACCFY, A_REQ_STAYINLANE, TIME_CONSECUTIVE_THROTTLE]
model_hyperparams["Mixture Regression"] = [
        BehaviorParameter(:n_components, 2:5, 1),
        BehaviorParameter(:max_n_indicators, 2:5, 1),
    ]


add_behavior!(behaviorset, DynamicBayesianNetworkBehavior, "Bayesian Network")
model_trainparams["Bayesian Network"] = BN_TrainParams(
                                            indicators=INDICATOR_SET,
                                            preoptimize_target_bins=true,
                                            preoptimize_indicator_bins=true,
                                            optimize_structure=true,
                                            optimize_target_bins=false,
                                            optimize_parent_bins=false,
                                        )
model_hyperparams["Bayesian Network"] = [
        BehaviorParameter(:ncandidate_bins, 1:5:36, 6),
        BehaviorParameter(:max_parents, 1:10, 5)
    ]
