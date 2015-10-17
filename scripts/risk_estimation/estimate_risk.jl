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

using AutomotiveDrivingModels

using RandomForestBehaviors
using DynamicBayesianNetworkBehaviors

#############################################################
# CONSTANTS
#############################################################

const N_TRIALS = 10
const ACTION_LON = [
        :accel =>  1.0,
        :decel => -1.0,
        :hold  =>  0.0.
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

for scenario in scenarios

    pdset = create_scenario_pdset(scenario)
    sn = scenario.sn
    basics = FeatureExtractBasicsPdSet(pdset, sn)

    validfind = scenario.history
    candidate_trajectories = generate_candidate_trajectories(basics, policy, active_carid, validfind)
    extracted_trajdefs, extracted_polies = extract_trajdefs(basics, candidate_trajectories, active_carid, validfind)
    sec_per_frame = calc_sec_per_frame(basics.pdset)

    for (behavior_name, human_behavior) in HUMAN_BEHAVIOR_MODELS
        collision_risk = calc_collision_risk_monte_carlo!(basics, policy, candidate_trajectories,
                                                          active_carid, validfind, sec_per_frame)
    end
end



