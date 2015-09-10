using AutomotiveDrivingModels

require(Pkg.dir("AutomotiveDrivingModels", "viz", "Renderer.jl")); @everywhere using .Renderer
require(Pkg.dir("AutomotiveDrivingModels", "viz", "ColorScheme.jl")); @everywhere using .ColorScheme
require(Pkg.dir("AutomotiveDrivingModels", "viz", "incl_cairo_utils.jl"))

# reload(Pkg.dir("AutomotiveDrivingModels", "scripts", "risk_estimation_policy_opt", "scenario_follow_equal.jl"))
# reload(Pkg.dir("AutomotiveDrivingModels", "scripts", "risk_estimation_policy_opt", "scenario_follow_faster.jl"))
reload(Pkg.dir("AutomotiveDrivingModels", "scripts", "risk_estimation_policy_opt", "scenario_follow_faster_with_other.jl"))

println("Loaded scenario: ", scenario.name)

sn = scenario.sn
scenario_pdset = create_scenario_pdset(scenario)
horizon = get_horizon(scenario)
history = scenario.history
basics = FeatureExtractBasicsPdSet(scenario_pdset, sn)

active_carid = CARID_EGO

# write("scenario_" * scenario.name * ".gif", roll(reel_pdset(scenario_pdset, sn, active_carid), fps=40))

human_behavior = VehicleBehaviorGaussian(0.00001, 0.1)
policy = RiskEstimationPolicy(human_behavior)
policy.speed_deltas = get_speed_deltas(2, 5.0)
policy.k_c = 100000.0
policy.k_v = 100.0

#################

write("scenario_" * scenario.name * "_default_policy.gif", roll(reel_scenario_playthrough(scenario, policy, pdset=scenario_pdset), fps=2))

exit()

#################

candidate_policies = RiskEstimationPolicy[]

df_results = DataFrame(p_nsimulations      = Int[],     # number of simulations per risk est in policy
                       p_speed_delta_count = Int[],     # 0 → only generate trajectories with current speed, 1 → do [-δ,0,+δ], 2 → [-2δ,-δ,0,+δ,+2δ], etc.
                       p_speed_delta_jump  = Float64[], # δ used in speed generation
                       e_time_per_tick     = Float64[], # average time per policy tick
                       e_ncollisions       = Int[],     # number of collisions during policy evaluation
                       e_performance       = Float64[], # overall policy performance measure
                       total_eval_time     = Float64[], # total time spent in evaluation
                      )

MPH_5 = 2.235

nsimulations = 1
speed_delta_count = 0
speed_delta_jump = 0.0
for nsimulations in (1,10,100) #,1000)
    for (speed_delta_count, speed_delta_jump) in [(0,0.0), (1,MPH_5), (1,2MPH_5)]#, (2,MPH_5)]

        speed_deltas = get_speed_deltas(speed_delta_count, speed_delta_jump)

        push!(candidate_policies, RiskEstimationPolicy(human_behavior, 
                                     nsimulations=nsimulations, speed_deltas=speed_deltas))
        push!(df_results, (nsimulations, speed_delta_count, speed_delta_jump, NaN, -999, NaN, NaN))
    end
end

ncandidate_policies = length(candidate_policies)
evaluations = Array(PolicyEvaluationResults, ncandidate_policies)

nruns = 10

file_risk_estimation_results = "risk_estimation_results_" * scenario.name * ".csv"

tic()
for i in 1 : ncandidate_policies
    println(i, " / ", ncandidate_policies)

    policy = candidate_policies[i]

    starttime = time()
    evaluations[i] = evaluate_policy(scenario, active_carid, policy, nruns)
    total_eval_time = time() - starttime
    
    println("eval: ", evaluations[i])

    df_results[i, :e_time_per_tick] = evaluations[i].time_per_tick
    df_results[i, :e_ncollisions] = evaluations[i].ncollisions
    df_results[i, :e_performance] = evaluations[i].performance
    df_results[i, :total_eval_time] = total_eval_time

    println("total eval time: ", total_eval_time, " [s]")
    writetable(file_risk_estimation_results, df_results)
end
toc()

writetable(file_risk_estimation_results, df_results)

println("DONE")