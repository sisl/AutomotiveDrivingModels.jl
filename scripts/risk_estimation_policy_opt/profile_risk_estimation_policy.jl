

using AutomotiveDrivingModels

reload("/home/tim/.julia/v0.3/AutomotiveDrivingModels/scripts/risk_estimation_policy_opt/scenario_follow_equal.jl")

sn = scenario.sn
scenario_pdset = create_scenario_pdset(scenario)
horizon = get_horizon(scenario)
history = scenario.history
basics = FeatureExtractBasicsPdSet(scenario_pdset, sn)

human_behavior = VehicleBehaviorGaussian(0.01, 0.1)
policy = RiskEstimationPolicy(human_behavior, nsimulations = 2, speed_deltas = get_speed_deltas(0, 0.0))

active_carid = CARID_EGO
nruns = 2

@time evaluate_policy(scenario, active_carid, policy, nruns)

Profile.clear()
@profile evaluate_policy(scenario, active_carid, policy, nruns)

using ProfileView
ProfileView.view()

sleep(100000)