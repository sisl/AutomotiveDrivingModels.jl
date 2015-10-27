#=
Produces the data for a collision risk table:

                            Estimated Collision Probability
Scenario  Interaction          Move Left       Hold Lane
            Aware            <0   ~0   >0    <0   ~0   >0
------------------------------------------------------------
A              N             80   10   -     -     -    50
               Y             60    5   -     -     -    20
B              N             30   70   20    -     -     -
...
------------------------------------------------------------
=#

# TODO: train models on FULL dataset
# TODO: select appropriate model based on context

using AutomotiveDrivingModels

using RandomForestBehaviors
using DynamicBayesianNetworkBehaviors

#############################################################
# CONSTANTS
#############################################################

const N_TRIALS = 1000
const ACTION_LON = [
        :accel =>  1.0,
        :decel => -1.0,
        :hold  =>  0.0,
    ]
const HUMAN_BEHAVIOR_MODELS = [
        ("SG", VehicleBehaviorGaussian(0.00001, 0.1)),
    ]
const SCENARIOS = [
        Pkg.dir("AutomotiveDrivingModels", "scripts", "risk_estimation_policy_opt", "scenario_follow_faster_with_other.jl"),
    ]

#############################################################
# Load Scenarios
#############################################################

scenarios = Scenario[]
for scenario_includepath in SCENARIOS
    include(scenario_includepath)
    push!(scenarios, scenario)
    println("Loaded scenario: ", scenario.name)
end

#############################################################
# Estimate Collision Probabilities
#############################################################

df_res = DataFrame()
df_res[:scenario] = String[]
df_res[:model] = String[]
df_res[:ego_action_lat] = Symbol[] # ∈ [:left, :right, :hold]
df_res[:ego_action_lon] = Symbol[] # ∈ [:accel, :decel, :hold]
df_res[:p_collision] = Float64[]

active_carid = CARID_EGO

for scenario in scenarios

    pdset = create_scenario_pdset(scenario)
    sn = scenario.sn
    basics = FeatureExtractBasicsPdSet(pdset, sn)
    policy = RiskEstimationPolicy(
                speed_deltas = [-2.235*2, 0.0, 2.235*2],
                generate_follow_trajectory = false,
                trailing_distance = 40.0,
                k_relative_speed = 0.0,
                nsimulations = N_TRIALS,
                history = 2*DEFAULT_FRAME_PER_SEC,
                horizon = 8*DEFAULT_FRAME_PER_SEC,
                trajectory_durations = [2*DEFAULT_FRAME_PER_SEC],
                desired_speed = 29.06,
                sec_per_frame = DEFAULT_SEC_PER_FRAME,
                col_method = :OBB,
            )
    candidate_trajectories = generate_candidate_trajectories(basics, policy, active_carid, scenario.history)
    extracted_trajdefs, extracted_polies = extract_trajdefs(basics, candidate_trajectories, active_carid, scenario.history)

    #############################################################
    # export scenario to tikz

    # \draw          (0,0)   -- (7.5,0);
    # \draw [dashed] (0,0.5) -- (7.5,0.5);
    # \draw          (0,1)   -- (7.5,1);

    # \draw [->,>=stealth,shorten >=1pt, thick, dotted] (1.25,0.25) -- (1.40,0.25) -- (1.60,0.30) -- (1.80,0.40) -- (2.00,0.50) -- (2.20,0.60) -- (2.40,0.63) -- (2.60,0.69) -- (2.80,0.73) -- (6.00,0.73);
    # \draw [->,>=stealth,shorten >=1pt, thick] (2.00,0.25) -- (7.00,0.25);
    # \draw [->,>=stealth,shorten >=1pt, thick, dashed] (0.50,0.75) -- (6.50,0.75);

    # for carind in -1 : 1
    #     print("\\draw [->,>=stealth,shorten >=1pt, thick] ")
    #     validfind_start = scenario.history
    #     validfind_end = validfind_start + policy.horizon
    #     for validfind in validfind_start : validfind_end
    #         posGx = get(pdset, :posGx, carind, validfind)
    #         posGy = get(pdset, :posGy, carind, validfind)
    #         @printf("(%.3f,%.3f)", posGx, posGy)
    #         if validfind != validfind_end
    #             print(" -- ")
    #         end
    #     end
    #     print(";\n")
    # end

    # \node [] (ego) at (1.0,0.25) {\includegraphics[width=7mm]{Car_Top_View_Sedan.eps}};
    # \node [] (ot1) at (2.0,0.25) {\includegraphics[width=7mm]{Car_Top_View_Sedan.eps}};
    # \node [] (ot2) at (0.5,0.75) {\includegraphics[width=7mm]{Car_Top_View_Sedan.eps}};

    # for (extract_trajdef, nodename) in zip(extracted_trajdefs, ["ego", "ot1", "ot2"])
    #     posGx = extract_trajdef.df[scenario.history, :posGx]
    #     posGy = extract_trajdef.df[scenario.history, :posGy]
    #     @printf("\\node [] (%s) at (%.3f,%.3f) {\\includegraphics[width=7mm]{Car_Top_View_Sedan.eps}};\n",
    #         nodename, posGx, posGy)
    # end

    # \node [] at ($(ego.center) + (0,-1.2em)$) {\scriptsize Ego};
    # \node [] at ($(ot1.center) + (0,-1.2em)$) {\scriptsize Other 1};
    # \node [] at ($(ot2.center) + (0, 1.2em)$) {\scriptsize Other 2};

    #############################################################

    validfind = scenario.history

    for (behavior_name, human_behavior) in HUMAN_BEHAVIOR_MODELS

        policy.human_behavior = human_behavior
        sec_per_frame = calc_sec_per_frame(basics.pdset)
        collision_risk = calc_collision_risk_monte_carlo!(basics, policy, candidate_trajectories,
                                                          active_carid, validfind, sec_per_frame)

        col_risk_ind = 0
        for ego_action_lat in (:hold, :left)
            for ego_action_lon in [:decel, :hold, :accel]
                col_risk_ind += 1
                push!(df_res, [scenario.name, behavior_name, ego_action_lat, ego_action_lon, collision_risk[col_risk_ind]])
            end
        end
    end
end

println(df_res)