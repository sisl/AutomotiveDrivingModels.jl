behaviorset = Dict{AbstractString, BehaviorTrainDefinition}()

INDICATOR_SET2 = [
                    FeaturesNew.POSFYAW, FeaturesNew.POSFT, FeaturesNew.SPEED, FeaturesNew.VELBX, FeaturesNew.VELBY, FeaturesNew.VELFS, FeaturesNew.VELFT,
                    FeaturesNew.TURNRATE, FeaturesNew.ACC, FeaturesNew.ACCFS, FeaturesNew.ACCFT, FeaturesNew.ACCBX, FeaturesNew.ACCBY,
                    FeaturesNew.MARKERDIST_LEFT, FeaturesNew.MARKERDIST_RIGHT,
                    FeaturesNew.TIMETOCROSSING_LEFT, FeaturesNew.TIMETOCROSSING_RIGHT, FeaturesNew.ESTIMATEDTIMETOLANECROSSING, FeaturesNew.A_REQ_STAYINLANE,
                    FeaturesNew.N_LANE_LEFT, FeaturesNew.N_LANE_RIGHT, FeaturesNew.HAS_LANE_RIGHT, FeaturesNew.HAS_LANE_LEFT, FeaturesNew.LANECURVATURE,
                ]

behaviorset["Static Gaussian"] = BehaviorTrainDefinition(SG_TrainParams())

# behaviorset["Random Forest"] = BehaviorTrainDefinition(GRF_TrainParams(indicators=INDICATOR_SET2, ntrees=3, max_tree_depth=3, n_PCA_features=5))
# behaviorset["Dynamic Forest"] = BehaviorTrainDefinition(DF_TrainParams(indicators=INDICATOR_SET2, ntrees=3, max_tree_depth=3, n_PCA_features=5))
behaviorset["Mixture Regression"] = BehaviorTrainDefinition(GMR_TrainParams(indicators=INDICATOR_SET2, n_components=3, max_n_indicators=2, n_PCA_features=5))

# behaviorset["Linear Gaussian"] = BehaviorTrainDefinition(
#                                             LG_TrainParams(indicators=INDICATOR_SET2),
#                                             [
#                                                 BehaviorParameter(:ridge_regression_constant, collect(linspace(0.0,1.0,5)), 5)
#                                             ])
# behaviorset["Random Forest"] = BehaviorTrainDefinition(
#                                             GRF_TrainParams(indicators=INDICATOR_SET2),
#                                             [
#                                                 BehaviorParameter(:ntrees, 1:10:51, 3),
#                                                 BehaviorParameter(:max_tree_depth, 1:6, 3),
#                                                 # BehaviorParameter(:min_samples_split, 10:10:50, 3),
#                                                 # BehaviorParameter(:min_samples_leaves, [2,4,10,20,50], 3),
#                                                 # BehaviorParameter(:min_split_improvement, [10.0, 5.0, 1.0,0.5,0.1,0.0], 3),
#                                                 # BehaviorParameter(:partial_sampling, [0.5,0.6,0.7,0.8,0.9,0.95,1.0], 5),
#                                                 BehaviorParameter(:n_split_tries, [2,3,4,5,10,20], 5),
#                                                 BehaviorParameter(:n_PCA_features, [5,10,20], 2),
#                                             ])
# behaviorset["Dynamic Forest"] = BehaviorTrainDefinition(
#                                             DF_TrainParams(indicators=INDICATOR_SET2),
#                                             [
#                                                 BehaviorParameter(:ntrees, 1:10:51, 3),
#                                                 BehaviorParameter(:max_tree_depth, 1:6, 3),
#                                                 # BehaviorParameter(:min_samples_split, 10:10:50, 3),
#                                                 # BehaviorParameter(:min_samples_leaves, [2,4,10,20,50], 3),
#                                                 # BehaviorParameter(:min_split_improvement, [10.0, 5.0, 1.0,0.5,0.1,0.0], 3),
#                                                 # BehaviorParameter(:partial_sampling, [0.5,0.6,0.7,0.8,0.9,0.95,1.0], 5),
#                                                 BehaviorParameter(:n_split_tries, [2,3,4,5,10,20], 5),
#                                                 BehaviorParameter(:n_PCA_features, [5,10,20], 2),
#                                             ])

# behaviorset["Bayesian Network"] = BehaviorTrainDefinition(
#                                             BN_TrainParams(
#                                                 starting_structure=ParentFeatures(FeaturesNew.AbstractFeature[], FeaturesNew.AbstractFeature[]),
#                                                 forced=ParentFeatures(FeaturesNew.AbstractFeature[], FeaturesNew.AbstractFeature[]),
#                                                 indicators=INDICATOR_SET2,
#                                                 preoptimize_target_bins=true,
#                                                 preoptimize_indicator_bins=true,
#                                                 optimize_structure=true,
#                                                 optimize_target_bins=false,
#                                                 optimize_parent_bins=false,
#                                                 ncandidate_bins=25,
#                                                 max_parents=5,
#                                                 dirichlet_prior=UniformPrior(),
#                                             ))

# behaviorset["Bayesian Network"] = BehaviorTrainDefinition(
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
#                                                 BehaviorParameter(:max_parents, 3:7, 2),
#                                                 BehaviorParameter(:dirichlet_prior, [UniformPrior(), BDeuPrior(0.5), BDeuPrior(1.0), BDeuPrior(2.0), BDeuPrior(10.0)], 1),
#                                             ])

# behaviorset["Mixture Regression"] = BehaviorTrainDefinition(
#                                             GMR_TrainParams(indicators=INDICATOR_SET), #[YAW, SPEED, ACC, VELFY, A_REQ_STAYINLANE, TIME_CONSECUTIVE_THROTTLE, TTC_X_FRONT]), # [YAW, SPEED, VELFX, VELFY, TURNRATE, ACC, ACCFX, ACCFY, A_REQ_STAYINLANE, TIME_CONSECUTIVE_THROTTLE, TTC_X_FRONT]
#                                             [
#                                                 BehaviorParameter(:n_components, 2:5, 1),
#                                                 BehaviorParameter(:max_n_indicators, 2:5, 1),
#                                                 BehaviorParameter(:fraction_test, [0.1,0.2,0.3,0.5], 1),
#                                                 BehaviorParameter(:min_covar, [1e-3, 1e-4, 1e-5, 1e-6], 2),
#                                             ])
