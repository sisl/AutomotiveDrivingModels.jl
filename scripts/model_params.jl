behaviorset = Dict{AbstractString, BehaviorTrainDefinition}()


####################
# Empty Parameter Set

# behaviorset["Static Gaussian"] = BehaviorTrainDefinition(SG_TrainParams())
# behaviorset["Linear Gaussian"] = BehaviorTrainDefinition(LG_TrainParams(indicators=INDICATOR_SET2, ridge_regression_constant=0.8))
# behaviorset["Random Forest"] = BehaviorTrainDefinition(
#                                                 GRF_TrainParams(indicators=INDICATOR_SET2,
#                                                     ntrees=25,
#                                                     max_tree_depth=3,
#                                                     n_split_tries=3,
#                                                     use_PCA=false,
#                                                     min_split_improvement=0.836,
#                                                     partial_sampling=0.2,
#                                                 ))
# behaviorset["Dynamic Forest"] = BehaviorTrainDefinition(
#                                                 DF_TrainParams(indicators=INDICATOR_SET2,
#                                                     ntrees=17,
#                                                     max_tree_depth=3,
#                                                     n_split_tries=3,
#                                                     use_PCA=false,
#                                                     min_split_improvement=0.493,
#                                                     partial_sampling=0.895,
#                                                 ))
# behaviorset["Bayesian Network"] = BehaviorTrainDefinition(
#                                             BN_TrainParams(
#                                                 indicators=INDICATOR_SET2,
#                                                 preoptimize_target_bins=true,
#                                                 preoptimize_indicator_bins=true,
#                                                 optimize_structure=true,
#                                                 optimize_target_bins=false,
#                                                 optimize_parent_bins=false,
#                                                 ncandidate_bins=20,
#                                                 max_parents=5,
#                                                 nbins_lat=7,
#                                                 nbins_lon=7,
#                                                 dirichlet_prior=BDeuPrior(0.5),
#                                                 )
#                                             )
# behaviorset["Linear Bayesian"] = BehaviorTrainDefinition(
#                                                 LB_TrainParams(indicators=INDICATOR_SET2,
#                                                     ridge_regression_constant=0.00486,
#                                                     min_σ_lat=1e-6,
#                                                     min_σ_lon=7e-6,
#                                                     max_parents=3,
#                                                 ))
# behaviorset["Mixture Regression"] = BehaviorTrainDefinition(
#                                                 GMR_TrainParams(indicators=INDICATOR_SET_SMALL,
#                                                     n_components=20,
#                                                     n_gmm_iter=100,
#                                                     n_init=3,
#                                                     tol=0.166,
#                                                     min_covar=0.000092,
#                                                     max_n_indicators=4,
#                                                     use_PCA=false,
#                                                     unlearned_component_weight=0.054,
#                                                 ))

###################
# Partial Parameter Set

# behaviorset["Static Gaussian"] = BehaviorTrainDefinition(SG_TrainParams(),
#                                     BehaviorParameter[
#                                         BehaviorParameter(:targets, [
#                                                                         ModelTargets(FUTUREDESIREDANGLE, FUTUREACCELERATION),
#                                                                         ModelTargets(FUTUREVELFT, FUTUREACCELERATION),
#                                                                         ModelTargets(FUTURETURNRATE, FUTUREACCELERATION),
#                                                                     ], 1),
#                                     ])
# behaviorset["Static Gaussian"] = BehaviorTrainDefinition(SG_TrainParams())

# behaviorset["Linear Gaussian"] = BehaviorTrainDefinition(
#                                             LG_TrainParams(indicators=INDICATOR_SET2),
#                                             [
#                                                 BehaviorParameter(:ridge_regression_constant, [0.0,1e-5,1e-4,1e-3,1e-2,0.1,0.25,0.5,0.9], 2)
#                                             ])
# behaviorset["Random Forest"] = BehaviorTrainDefinition(
#                                             GRF_TrainParams(indicators=INDICATOR_SET2),
#                                             [
#                                                 BehaviorParameter(:ntrees, [3,10,20,30], 2),
#                                                 BehaviorParameter(:max_tree_depth, [2,3,4], 2),
#                                                 # BehaviorParameter(:use_PCA, [false, true], 1),
#                                                 # BehaviorParameter(:n_split_tries, [2,10,20,30], 2),
#                                                 # BehaviorParameter(:min_split_improvement, [0.0,1.0], 1),
#                                                 BehaviorParameter(:partial_sampling, [0.2,0.5,0.6,0.8], 2),
#                                             ])
# behaviorset["Dynamic Forest"] = BehaviorTrainDefinition(
#                                             DF_TrainParams(indicators=INDICATOR_SET2),
#                                             [
#                                                 BehaviorParameter(:ntrees, [3,10,20,30], 2),
#                                                 BehaviorParameter(:max_tree_depth, [2,3,4], 2),
#                                                 # BehaviorParameter(:use_PCA, [false, true], 1),
#                                                 # BehaviorParameter(:n_split_tries, [10,20,30], 2),
#                                                 # BehaviorParameter(:min_split_improvement, [0.0,1.0], 2),
#                                                 BehaviorParameter(:partial_sampling, [0.2,0.5,0.6,0.8], 2),
#                                             ])
# behaviorset["Bayesian Network"] = BehaviorTrainDefinition(
#                                             BN_TrainParams(
#                                                 indicators=INDICATOR_SET2,
#                                                 preoptimize_target_bins=true,
#                                                 preoptimize_indicator_bins=true,
#                                                 optimize_structure=true,
#                                                 optimize_target_bins=false,
#                                                 optimize_parent_bins=false,
#                                                 ncandidate_bins=20,
#                                                 max_parents=5,
#                                                 nbins_lat=7,
#                                                 nbins_lon=7,
#                                                 dirichlet_prior=BDeuPrior(0.5),
#                                             ),
#                                             [
#                                                 # BehaviorParameter(:max_parents, [2,3,4,5], 2),
#                                                 # BehaviorParameter(:nbins_lat, 6:8, 2),
#                                                 # BehaviorParameter(:nbins_lon, 6:8, 2),
#                                                 BehaviorParameter(:dirichlet_prior, [UniformPrior(), UniformPrior(0.01), BDeuPrior(0.5), BDeuPrior(1.0), BDeuPrior(10.0)], 1),
#                                             ]
#                                             )
# behaviorset["Linear Bayesian"] = BehaviorTrainDefinition(
#                                             LB_TrainParams(
#                                                 # targets=ModelTargets(FUTUREDESIREDANGLE, FUTURE_DELTA_ACCEL),
#                                                 indicators=INDICATOR_SET2,
#                                             ),
#                                             [
#                                                 # BehaviorParameter(:targets, [
#                                                 #                                 ModelTargets(FUTUREDESIREDANGLE, FUTUREACCELERATION),
#                                                 #                                 ModelTargets(FUTUREVELFT, FUTUREACCELERATION),
#                                                 #                                 ModelTargets(FUTURETURNRATE, FUTUREACCELERATION),
#                                                 #                             ], 1),
#                                                 BehaviorParameter(:ridge_regression_constant, [0.005, 0.01, 0.1], 2),
#                                                 BehaviorParameter(:min_σ_lat, [1e-7, 1e-6, 1e-5], 2),
#                                                 BehaviorParameter(:min_σ_lon, [1e-7, 5e-6, 1e-6], 2),
#                                                 BehaviorParameter(:max_parents, [2,3,4,5], 2),
#                                             ])
# behaviorset["Mixture Regression"] = BehaviorTrainDefinition(
#                                             GMR_TrainParams(indicators=INDICATOR_SET2, n_gmm_iter=20, n_init=1, min_covar=1e-7, n_components=5, max_n_indicators=3, unlearned_component_weight=0.05),
#                                             BehaviorParameter[
#                                                 BehaviorParameter(:n_components, [3,5], 2),
#                                                 # BehaviorParameter(:tol, [0.1, 0.15, 0.2], 2),
#                                                 # BehaviorParameter(:min_covar, [1e-7,1e-6,1e-5,1e-4], 2),
#                                                 BehaviorParameter(:max_n_indicators, [3,4,5], 2),
#                                                 # BehaviorParameter(:unlearned_component_weight, [0.01,0.05,0.1], 2)
#                                                 # BehaviorParameter(:use_PCA, [false, true], 1),
#                                             ])

###################
# Partial Parameter Set

behaviorset["Static Gaussian Clean"] = BehaviorTrainDefinition(SG_TrainParams())
# behaviorset["Linear Gaussian Clean"] = BehaviorTrainDefinition(
#                                             LG_TrainParams(indicators=INDICATOR_SET_CLEAN),
#                                             [
#                                                 BehaviorParameter(:ridge_regression_constant, [0.0,1e-5,1e-4,1e-3,1e-2,0.1,0.25,0.5,0.9], 2)
#                                             ])
# behaviorset["Random Forest Clean"] = BehaviorTrainDefinition(
#                                             GRF_TrainParams(indicators=INDICATOR_SET_CLEAN),
#                                             [
#                                                 BehaviorParameter(:ntrees, [3,10,20,30], 2),
#                                                 BehaviorParameter(:max_tree_depth, [2,3,4], 2),
#                                                 # BehaviorParameter(:use_PCA, [false, true], 1),
#                                                 # BehaviorParameter(:n_split_tries, [2,10,20,30], 2),
#                                                 # BehaviorParameter(:min_split_improvement, [0.0,1.0], 1),
#                                                 BehaviorParameter(:partial_sampling, [0.2,0.5,0.6,0.8], 2),
#                                             ])
# behaviorset["Dynamic Forest Clean"] = BehaviorTrainDefinition(
#                                             DF_TrainParams(indicators=INDICATOR_SET_CLEAN),
#                                             [
#                                                 BehaviorParameter(:ntrees, [3,10,20,30], 2),
#                                                 BehaviorParameter(:max_tree_depth, [2,3,4], 2),
#                                                 # BehaviorParameter(:use_PCA, [false, true], 1),
#                                                 # BehaviorParameter(:n_split_tries, [10,20,30], 2),
#                                                 # BehaviorParameter(:min_split_improvement, [0.0,1.0], 2),
#                                                 BehaviorParameter(:partial_sampling, [0.2,0.5,0.6,0.8], 2),
#                                             ])
# behaviorset["Bayesian Network Clean"] = BehaviorTrainDefinition(
#                                             BN_TrainParams(
#                                                 indicators=INDICATOR_SET_CLEAN,
#                                                 preoptimize_target_bins=true,
#                                                 preoptimize_indicator_bins=true,
#                                                 optimize_structure=true,
#                                                 optimize_target_bins=false,
#                                                 optimize_parent_bins=false,
#                                                 ncandidate_bins=20,
#                                                 max_parents=5,
#                                                 nbins_lat=7,
#                                                 nbins_lon=7,
#                                                 dirichlet_prior=BDeuPrior(0.5),
#                                             ),
#                                             [
#                                                 # BehaviorParameter(:max_parents, [2,3,4,5], 2),
#                                                 # BehaviorParameter(:nbins_lat, 6:8, 2),
#                                                 # BehaviorParameter(:nbins_lon, 6:8, 2),
#                                                 BehaviorParameter(:dirichlet_prior, [UniformPrior(), UniformPrior(0.01), BDeuPrior(0.5), BDeuPrior(1.0), BDeuPrior(10.0)], 1),
#                                             ]
#                                             )
# behaviorset["Linear Bayesian Clean"] = BehaviorTrainDefinition(
#                                             LB_TrainParams(
#                                                 # targets=ModelTargets(FUTUREDESIREDANGLE, FUTURE_DELTA_ACCEL),
#                                                 indicators=INDICATOR_SET_CLEAN,
#                                             ),
#                                             [
#                                                 # BehaviorParameter(:targets, [
#                                                 #                                 ModelTargets(FUTUREDESIREDANGLE, FUTUREACCELERATION),
#                                                 #                                 ModelTargets(FUTUREVELFT, FUTUREACCELERATION),
#                                                 #                                 ModelTargets(FUTURETURNRATE, FUTUREACCELERATION),
#                                                 #                             ], 1),
#                                                 BehaviorParameter(:ridge_regression_constant, [0.005, 0.01, 0.1], 2),
#                                                 BehaviorParameter(:min_σ_lat, [1e-7, 1e-6, 1e-5], 2),
#                                                 BehaviorParameter(:min_σ_lon, [1e-7, 5e-6, 1e-6], 2),
#                                                 BehaviorParameter(:max_parents, [2,3,4,5], 2),
#                                             ])

# behaviorset["Mixture Regression Clean"] = BehaviorTrainDefinition(
#                                             GMR_TrainParams(indicators=INDICATOR_SET_CLEAN, n_gmm_iter=20, n_init=1, min_covar=1e-7, unlearned_component_weight=0.05),
#                                             BehaviorParameter[
#                                                 BehaviorParameter(:n_components, [3,5], 2),
#                                                 # BehaviorParameter(:tol, [0.1, 0.15, 0.2], 2),
#                                                 # BehaviorParameter(:min_covar, [1e-7,1e-6,1e-5,1e-4], 2),
#                                                 BehaviorParameter(:max_n_indicators, [3,4,5], 2),
#                                                 # BehaviorParameter(:unlearned_component_weight, [0.01,0.05,0.1], 2)
#                                                 # BehaviorParameter(:use_PCA, [false, true], 1),
#                                             ])