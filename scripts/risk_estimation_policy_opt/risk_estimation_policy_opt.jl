using AutomotiveDrivingModels

include(Pkg.dir("AutomotiveDrivingModels", "viz", "Renderer.jl")); using .Renderer
include(Pkg.dir("AutomotiveDrivingModels", "viz", "ColorScheme.jl")); using .ColorScheme
reload(Pkg.dir("AutomotiveDrivingModels", "viz", "incl_cairo_utils.jl"))

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
    push!(trajs[1], TrajDefLink(history, lanetagR, 0.0, speed_65mph))
    push!(trajs[1], TrajDefLink(horizon*4, lanetagR, 0.0, speed_65mph))

    trajs[2] = TrajDef(sn, VecE2(x₀+delta_x,-4.5), delta_speed*speed_65mph)
    push!(trajs[2], TrajDefLink(history, lanetagR, 0.0, delta_speed*speed_65mph))
    push!(trajs[2], TrajDefLink(horizon*4, lanetagR, 0.0, delta_speed*speed_65mph))

    trajs[3] = TrajDef(sn, VecE2(x₀-10,-1.5), speed_65mph)
    push!(trajs[3], TrajDefLink(history, lanetagL, 0.0, speed_65mph))
    push!(trajs[3], TrajDefLink(horizon*4, lanetagL, 0.0, speed_65mph))

    Scenario("three_car", sn, history, DEFAULT_SEC_PER_FRAME, trajs)
end

sn = scenario.sn
scenario_pdset = create_scenario_pdset(scenario)
horizon = get_horizon(scenario)
history = scenario.history
basics = FeatureExtractBasicsPdSet(scenario_pdset, sn)

active_carid = CARID_EGO

write("scenario.gif", reel_pdset(scenario_pdset, sn, active_carid=active_carid))

human_behavior = VehicleBehaviorGaussian(0.01, 0.1)
policy = RiskEstimationPolicy(human_behavior)

#################

canvas_width = 1100
canvas_height = 200
rendermodel = RenderModel()

camerazoom = 8.0

validfind_start = history
validfind_end   = nvalidfinds(scenario_pdset)

frames = CairoSurface[]
validfind = validfind_start

s = CairoRGBSurface(canvas_width, canvas_height)
ctx = creategc(s)
clear_setup!(rendermodel)

frameind = validfind2frameind(scenario_pdset, validfind)
render_streetnet_roads!(rendermodel, sn)
render_scene!(rendermodel, scenario_pdset, frameind, active_carid=active_carid)

camerax = get(scenario_pdset, :posGx, active_carid, validfind) + 60.0
cameray = get(scenario_pdset, :posGy, active_carid, validfind)

camera_setzoom!(rendermodel, camerazoom)
camera_set_pos!(rendermodel, camerax, cameray)

render(rendermodel, ctx, canvas_width, canvas_height)

push!(frames, s)

while validfind < validfind_end
        
    candidate_trajectories = generate_candidate_trajectories(basics, policy, active_carid, validfind)
    extracted_trajdefs = extract_trajdefs(basics, candidate_trajectories, active_carid, validfind)

    camerax = get(scenario_pdset, :posGx, active_carid, validfind) + 60.0
    push!(frames, plot_extracted_trajdefs(scenario_pdset, sn, extracted_trajdefs, validfind, active_carid, camerax=camerax, camerazoom=8.0, canvas_height=200))
    
    collision_risk = calc_collision_risk_monte_carlo!(basics, policy, extracted_trajdefs, active_carid, validfind)
    push!(frames, plot_extracted_trajdefs(scenario_pdset, sn, extracted_trajdefs, collision_risk, validfind, active_carid, camerax=camerax, camerazoom=8.0, canvas_height=200))
    
    costs = calc_costs(policy, candidate_trajectories, collision_risk)
    push!(frames, plot_extracted_trajdefs(scenario_pdset, sn, extracted_trajdefs, costs, validfind, active_carid, camerax=camerax, camerazoom=8.0, canvas_height=200))
    
    best = indmin(costs)
    extracted_best = extracted_trajdefs[best]
    push!(frames, plot_extracted_trajdefs(scenario_pdset, sn, [extracted_best], [costs[best]], validfind, active_carid, camerax=camerax, camerazoom=8.0, canvas_height=200))
    
    insert!(scenario_pdset, extracted_best, validfind+1, validfind+N_FRAMES_PER_SIM_FRAME)
    for validfind_vis in validfind+1 : min(validfind+N_FRAMES_PER_SIM_FRAME, validfind_end)
        camerax = get(scenario_pdset, :posGx, active_carid, validfind_vis) + 60.0
        push!(frames, plot_extracted_trajdefs(scenario_pdset, sn, [extracted_best], [costs[best]], validfind_vis, active_carid, camerax=camerax, camerazoom=8.0, canvas_height=200))
    end
    
    validfind += N_FRAMES_PER_SIM_FRAME
end

#################

candidate_policies = RiskEstimationPolicy[]

nsimulations = 1
speed_deltas = [0.0]
# for nsimulations in [1,10,100,1000]
    # for speed_deltas in ([0.0], [-2.235, 0.0, 2.235], [-2.235*1.5, 0.0, 2.235*1.5])
        push!(candidate_policies, RiskEstimationPolicy(human_behavior, 
                                     nsimulations=nsimulations, speed_deltas=speed_deltas))
    # end
# end

ncandidate_policies = length(candidate_policies)
evaluations = Array(Float64, ncandidate_policies)
times = Array(Float64, ncandidate_policies)

tic()
for i in 1 : ncandidate_policies
    println(i, " / ", ncandidate_policies)
    tic()
    evaluations[i] = map(policy->evaluate_policy(scenario, active_carid, policy, 2), policy)
    times[i] = toq()
end
toc()

println("evaluations: ", evaluations)
println("times:       ", times)

