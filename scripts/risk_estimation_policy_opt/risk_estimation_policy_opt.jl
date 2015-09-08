using AutomotiveDrivingModels

# include(Pkg.dir("AutomotiveDrivingModels", "viz", "Renderer.jl")); using .Renderer
# include(Pkg.dir("AutomotiveDrivingModels", "viz", "ColorScheme.jl")); using .ColorScheme
# reload(Pkg.dir("AutomotiveDrivingModels", "viz", "incl_cairo_utils.jl"))

scenario = let

    sn = generate_straight_nlane_streetmap(2)

    speed_65mph = 29.06

    lanetagL = get_lanetag(0.0,0.0,sn)
    lanetagR = get_lanetag(0.0,-5.0,sn)

    history     = 4*DEFAULT_FRAME_PER_SEC # [pdset frames]
    horizon     = 4*DEFAULT_FRAME_PER_SEC # [pdset frames]
    x₀          = 10.5 # [m]
    delta_speed = 0.93 # [%]
    delta_x     = 20.0 # [m]

    trajs = Array(TrajDef, 3)
    trajs[1] = TrajDef(sn, VecE2(x₀,-4.5), speed_65mph)
    push!(trajs[1], TrajDefLinkTargetSpeed(history, lanetagR, 0.0, speed_65mph))
    push!(trajs[1], TrajDefLinkTargetSpeed(horizon*4, lanetagR, 0.0, speed_65mph))

    trajs[2] = TrajDef(sn, VecE2(x₀+delta_x,-4.5), delta_speed*speed_65mph)
    push!(trajs[2], TrajDefLinkTargetSpeed(history, lanetagR, 0.0, delta_speed*speed_65mph))
    push!(trajs[2], TrajDefLinkTargetSpeed(horizon*4, lanetagR, 0.0, delta_speed*speed_65mph))

    trajs[3] = TrajDef(sn, VecE2(x₀-10,-1.5), speed_65mph)
    push!(trajs[3], TrajDefLinkTargetSpeed(history, lanetagL, 0.0, speed_65mph))
    push!(trajs[3], TrajDefLinkTargetSpeed(horizon*4, lanetagL, 0.0, speed_65mph))

    Scenario("three_car", sn, history, DEFAULT_SEC_PER_FRAME, trajs)
end

sn = scenario.sn
scenario_pdset = create_scenario_pdset(scenario)
horizon = get_horizon(scenario)
history = scenario.history
basics = FeatureExtractBasicsPdSet(scenario_pdset, sn)

active_carid = CARID_EGO

# write("scenario.gif", reel_pdset(scenario_pdset, sn, active_carid=active_carid))

human_behavior = VehicleBehaviorGaussian(0.01, 0.1)
policy = RiskEstimationPolicy(human_behavior)

#################

# write("scenario_default_policy.gif", reel_scenario_playthrough(scenario, policy, pdset=scenario_pdset))

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
# for nsimulations in (1,10) #,100,1000)
#     for (speed_delta_count, speed_delta_jump) in [(0,0.0), (1,MPH_5), (1,2MPH_5)]#, (2,MPH_5)]

        if speed_delta_count == 0
            speed_deltas = [0.0]
        else
            speed_deltas = [-speed_delta_count : speed_delta_count].*speed_delta_jump
        end

        push!(candidate_policies, RiskEstimationPolicy(human_behavior, 
                                     nsimulations=nsimulations, speed_deltas=speed_deltas))
        push!(df_results, (nsimulations, speed_delta_count, speed_delta_jump, NaN, -999, NaN, NaN))
#     end
# end

ncandidate_policies = length(candidate_policies)
evaluations = Array(PolicyEvaluationResults, ncandidate_policies)

nruns = 10

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
    writetable("risk_estimation_results.csv", df_results)
end
toc()

writetable("risk_estimation_results.csv", df_results)

println("DONE")