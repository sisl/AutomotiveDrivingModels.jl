#=
This script is meant to be called by other processes
(ex: from Python in Spearmint)

ARGS:
    should be passed in as pairs:

    julia run_risk_estimation_policy.jl speed_delta_count 2 speed_delta_jump 5.0

    nruns              - Integer > 0  number of runs conducted to evaluate policy performance
    nsimulations       - Integer > 0  number of simulations conducted per trajectory to estimate collision risk
    speed_delta_count  - Integer ≥ 0  number of speed deltas to perform above and below 0
    speed_delta_jump   - Float        the target longitudinal speed difference above and below 0 for each speed_delta_count
    k_c                - Float        cost scalar on collision probability
    k_s                - Float        cost scalar relating lateral and longitudinal jerk
    k_v                - Float        cost scalar for deviation from desired speed

RETURNS:
    returns by printing to stdout
    returns the performance
=#

using AutomotiveDrivingModels

# reload(Pkg.dir("AutomotiveDrivingModels", "scripts", "risk_estimation_policy_opt", "scenario_follow_equal.jl"))
# reload(Pkg.dir("AutomotiveDrivingModels", "scripts", "risk_estimation_policy_opt", "scenario_follow_faster.jl"))
reload(Pkg.dir("AutomotiveDrivingModels", "scripts", "risk_estimation_policy_opt", "scenario_follow_faster_with_other.jl"))

sn = scenario.sn
scenario_pdset = create_scenario_pdset(scenario)
horizon = get_horizon(scenario)
history = scenario.history
basics = FeatureExtractBasicsPdSet(scenario_pdset, sn)

active_carid = CARID_EGO

human_behavior = VehicleBehaviorGaussian(0.00001, 0.1)
policy = RiskEstimationPolicy(human_behavior)

nruns = 50
speed_delta_count = 1
speed_delta_jump  = 2.2352 # 5 MPH

ARGS = ["nsimulations", "1", "speed_delta_jump", "1.000000", "k_s",          "0.000000", "k_v",          "0.000000", "speed_delta_count", "0", "k_c",          "0.000000", "nruns",        "1",]

arg_index = 1
while arg_index < length(ARGS)
    if ARGS[arg_index] == "nsimulations"
        nsimulations = int(ARGS[arg_index+=1])
        @assert(nsimulations > 0)
        policy.nsimulations = nsimulations
    elseif ARGS[arg_index] == "speed_delta_count"
        speed_delta_count = int(ARGS[arg_index+=1])
        @assert(speed_delta_count ≥ 0)
    elseif ARGS[arg_index] == "speed_delta_jump"
        speed_delta_jump = float(ARGS[arg_index+=1])
    elseif ARGS[arg_index] == "k_c"
        policy.k_c = float(ARGS[arg_index+=1])
    elseif ARGS[arg_index] == "k_s"
        policy.k_s = float(ARGS[arg_index+=1])
    elseif ARGS[arg_index] == "k_v"
        policy.k_v = float(ARGS[arg_index+=1])
    else
        warn("UNRECOGNIZED PARAMETER $(ARGS[arg_index])")
    end
    arg_index += 1
end

policy.speed_deltas = get_speed_deltas(speed_delta_count, speed_delta_jump)

evaluation = evaluate_policy(scenario, active_carid, policy, nruns)
println(evaluation.performance)