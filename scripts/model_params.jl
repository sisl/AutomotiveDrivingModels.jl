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

add_behavior!(behaviorset, GMRBehavior, "Mixture Regression")
model_param_sets["Mixture Regression"] = BehaviorParameterSet(
    convert(Vector{(Symbol,Any)}, [(:indicators,[YAW, ACC, TIME_CONSECUTIVE_THROTTLE])]), # [YAW, SPEED, VELFX, VELFY, TURNRATE, ACC, ACCFX, ACCFY, A_REQ_STAYINLANE, TIME_CONSECUTIVE_THROTTLE]
    [BehaviorParameter(:n_components, 2:5, 1),
     BehaviorParameter(:max_n_indicators, 2:5, 1),
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
