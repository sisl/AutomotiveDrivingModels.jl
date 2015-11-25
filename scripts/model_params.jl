behaviorset = Dict{AbstractString, BehaviorTrainDefinition}()

behaviorset["Static Gaussian"] = BehaviorTrainDefinition(SG_TrainParams())
behaviorset["Linear Gaussian"] = BehaviorTrainDefinition(
                                            LG_TrainParams(indicators=INDICATOR_SET),
                                            [
                                                BehaviorParameter(:ridge_regression_constant, collect(linspace(0.0,1.0,5)), 5)
                                            ])
# behaviorset["Random Forest"] = BehaviorTrainDefinition(
#                                             GRF_TrainParams(indicators=INDICATOR_SET),
#                                             [
#                                                 BehaviorParameter(:ntrees, 1:10:51, 3),
#                                                 BehaviorParameter(:max_tree_depth, 1:5:16, 3),
#                                                 # BehaviorParameter(:min_samples_split, 10:10:50, 3),
#                                                 # BehaviorParameter(:min_samples_leaves, [2,4,10,20,50], 3),
#                                                 # BehaviorParameter(:min_split_improvement, [10.0, 5.0, 1.0,0.5,0.1,0.0], 3),
#                                                 # BehaviorParameter(:partial_sampling, [0.5,0.6,0.7,0.8,0.9,0.95,1.0], 5),
#                                                 # BehaviorParameter(:n_split_tries, [10,25,50,100,200,500,1000], 5),
#                                             ])
# behaviorset["Dynamic Forest"] = BehaviorTrainDefinition(
#                                             DF_TrainParams(indicators=INDICATOR_SET),
#                                             [
#                                                 BehaviorParameter(:ntrees, 1:10:51, 3),
#                                                 BehaviorParameter(:max_tree_depth, 1:5:16, 3),
#                                                 # BehaviorParameter(:min_samples_split, 10:10:50, 3),
#                                                 # BehaviorParameter(:min_samples_leaves, [2,4,10,20,50], 3),
#                                                 # BehaviorParameter(:min_split_improvement, [10.0, 5.0, 1.0,0.5,0.1,0.0], 3),
#                                                 # BehaviorParameter(:partial_sampling, [0.5,0.6,0.7,0.8,0.9,0.95,1.0], 5),
#                                                 # BehaviorParameter(:n_split_tries, [10,25,50,100,200,500,1000], 5),
#                                             ])
# behaviorset["Mixture Regression"] = BehaviorTrainDefinition(
#                                             GMR_TrainParams(indicators=[YAW, SPEED, ACC, VELFY, TIME_CONSECUTIVE_THROTTLE]), # [YAW, SPEED, VELFX, VELFY, TURNRATE, ACC, ACCFX, ACCFY, A_REQ_STAYINLANE, TIME_CONSECUTIVE_THROTTLE]
#                                             [
#                                                 BehaviorParameter(:n_components, 2:5, 1),
#                                                 BehaviorParameter(:max_n_indicators, 2:5, 1),
#                                             ])
# behaviorset["Mixture Regression"] = BehaviorTrainDefinition(
#                                             BN_TrainParams(
#                                                 indicators=INDICATOR_SET,
#                                                 preoptimize_target_bins=true,
#                                                 preoptimize_indicator_bins=true,
#                                                 optimize_structure=true,
#                                                 optimize_target_bins=false,
#                                                 optimize_parent_bins=false,
#                                             ),
#                                             [
#                                                 BehaviorParameter(:ncandidate_bins, 5:5:25, 2),
#                                                 BehaviorParameter(:max_parents, 1:7, 5),
#                                             ])
