export 
    RiskEstimationPolicy,

    calc_integrated_squared_jerk,
    calc_cost,
    calc_costs,

    generate_candidate_trajectories,
    calc_collision_risk_monte_carlo!,
    calc_costs,
    evaluate_policy

type RiskEstimationPolicy <: AbstractVehicleBehavior
    speed_deltas   :: Vector{Float64} # various speed delta that we will check
    nsimulations   :: Uint            # number of simulations per risk estimation
    history        :: Uint            # number of pdset frames in the past to record
    horizon        :: Uint            # number of pdset frames to predict forward
    desired_speed  :: Float64         # desired speed [m/s]
    human_behavior :: AbstractVehicleBehavior # behavior to assign to humans in sim

    function RiskEstimationPolicy(
        human_behavior::AbstractVehicleBehavior = VehicleBehaviorGaussian(0.001,0.1);

        speed_deltas::Vector{Float64}=[-2.235*2, 0.0, 2.235*2],
        nsimulations::Integer=100,
        desired_speed::Float64=29.06
        )

        retval = new()
        retval.speed_deltas   = speed_deltas
        retval.nsimulations   = nsimulations
        retval.horizon        = 2*DEFAULT_FRAME_PER_SEC
        retval.horizon        = 4*DEFAULT_FRAME_PER_SEC
        retval.desired_speed  = desired_speed
        retval.human_behavior = human_behavior
        retval
    end
end

function calc_integrated_squared_jerk(p::Quintic, τ::Float64)
    a = p.x₄
    b = p.x₅
    c = p.x₆

    cτ = c*τ
    τ² = τ*τ

    6τ*(
        3a*a + 4a*τ*(3b+5cτ) + 4τ²*(4b*b + 15b*cτ + 15c*c*τ²)
        )
end
function calc_integrated_squared_jerk(p::Quartic, τ::Float64)
    a = p.x₄
    b = p.x₅

    6τ*(3a*a + 12a*b*τ + 16*b*b*τ*τ)
end

function calc_cost(
    trajdef::Vector{TrajDefLink},
    collision_prob::Real,
    desired_speed::Real;  # m/s

    k_c::Float64=1.0, # weight assigned to collision probability
    k_s::Float64=1.0, # weight assigned to relate lateral and longitudinal squared jerk
    k_v::Float64=0.0, # weight assigned to the squared terminal desired speed deviation
    #k_τ::Float64=0.0, # weight assinged to terminal time?
    #k_ξ₁::Float64=0.0, # weight assigned to terminal reference difference

    # TODO(tim): include other factors?
    )

    #=
    Assign a scalar cost to a given trajectory
    =#

    # todo: compute this
    J_s = 0.0
    J_d = 0.0
    # for link in trajdef

    #     τ = link.n_pdset_frames * sec_per_frame

    #     sdot₁::Float64,
    #     sddot₁::Float64,
    #     d₁::Float64,
    #     ddot₁::Float64,
    #     dddot₁::Float64,

    #     d₂ = link.d # final lateral offset
    #     ddot₂ = link.v * sin(link.ϕ) # final lateral velocity
    #     dddot₂ = link.a * sin(link.ψ) # final lateral accel

    #     sdot₂ = link.v * cos(link.ϕ) # final longitudinal velocity
    #     sddot₂ = link.a * cos(link.ψ) # final longitudinal accel

    #     poly_s = get_quartic_coefficients(0.0, sdot₁, sddot₁, sdot₂, sddot₂, τ)
    #     poly_d = get_quintic_coefficients(d₁, ddot₁, dddot₁, d₂, ddot₂, dddot₂, τ)
    #     J_s += calc_integrated_squared_jerk(poly_s)
    #     J_d += calc_integrated_squared_jerk(poly_d)
    # end 

    Δv = trajdef[end].v - desired_speed
    Δv = Δv*Δv

    k_c*collision_prob + (J_d + k_s*J_s) + k_v*Δv
end

function generate_candidate_trajectories(
    basics::FeatureExtractBasicsPdSet,
    policy::RiskEstimationPolicy,
    active_carid::Integer,
    validfind::Integer,
    )

    candidate_trajectories = Vector{TrajDefLink}[]

    pdset, sn = basics.pdset, basics.sn
    horizon = policy.horizon

    carind = carid2ind(pdset, active_carid, validfind)
    start_speed = get_speed(pdset, carind, validfind)
    start_inertial = get_inertial(pdset, carind, validfind)

    cur_lanetag = get(pdset, :lanetag, carind, validfind)
    lane = get_lane(sn, cur_lanetag)
    proj = project_point_to_streetmap(start_inertial.x, start_inertial.y, sn)
    extind = proj.extind
    closest_node = closest_node_to_extind(lane, extind)

    lanetags = [cur_lanetag]
    if closest_node.n_lanes_left > 0
        push!(lanetags, get_neighbor_lanetag_left(sn, lane, closest_node))
    end
    if closest_node.n_lanes_right > 0
        push!(lanetags, get_neighbor_lanetag_right(sn, lane, closest_node))
    end

    for lanetag in lanetags
        for speed_delta in policy.speed_deltas
            push!(candidate_trajectories, [TrajDefLink(horizon, lanetag, 0.0, start_speed+speed_delta)])
            if lanetag != cur_lanetag
                half_horizon1 = ifloor(horizon/2)
                half_horizon2 = iceil(horizon/2)
                push!(candidate_trajectories, [TrajDefLink(half_horizon1, lanetag, 0.0, start_speed+speed_delta/2),
                                               TrajDefLink(half_horizon2, lanetag, 0.0, start_speed+speed_delta)])
            end
        end
        
    end

    candidate_trajectories
end
function calc_collision_risk_monte_carlo!(
    basics::FeatureExtractBasicsPdSet,
    policy::RiskEstimationPolicy,
    extracted_trajdefs::Vector{ExtractedTrajdef},
    active_carid::Integer,
    validfind::Integer,
    )

    pdset, sn = basics.pdset, basics.sn

    n_othercars = get_num_cars_in_frame(pdset, validfind)-1
    n_candidatetrajs = length(extracted_trajdefs)

    # allocate pdset for sim 
    # TODO(tim): allocate one for *horizon*, but copy the past
    pdset_for_sim = deepcopy(pdset)

    max_validfind = maximum([validfind + get_num_pdset_frames(extracted) for extracted in extracted_trajdefs])::Integer
    if max_validfind > nvalidfinds(pdset_for_sim)
        expand!(pdset_for_sim, max_validfind - nvalidfinds(pdset_for_sim))
    end

    basics_for_sim = FeatureExtractBasicsPdSet(pdset_for_sim, sn)

    behavior_pairs = (AbstractVehicleBehavior, Int)[]
    for carid in get_carids(pdset_for_sim)
        if carid != active_carid
            push!(behavior_pairs, (policy.human_behavior,carid))
        end
    end

    start_points = calc_start_points(pdset, behavior_pairs, validfind)
    collision_risk = Array(Float64, n_candidatetrajs)
    for (i,extracted) in enumerate(extracted_trajdefs)

        insert!(pdset_for_sim, extracted)

        horizon = get_num_pdset_frames(extracted)-1

        # TODO(tim): make this terminate early
        # TODO(tim): parallelize?
        collision_risk[i] = calc_collision_risk_monte_carlo!(
                                    basics_for_sim, behavior_pairs, 
                                    validfind, validfind+horizon-1,
                                    nsimulations=policy.nsimulations)
    end

    collision_risk
end
function calc_costs(
    policy::RiskEstimationPolicy,
    trajectories::Vector{Vector{TrajDefLink}},
    collision_risks::Vector{Float64},
    )
    
    costs = Array(Float64, length(trajectories))
    for (i,trajdef) in enumerate(trajectories)
        costs[i] = calc_cost(trajdef, collision_risks[i], policy.desired_speed)
    end
    costs
end

function propagate!(
    basics::FeatureExtractBasicsPdSet,
    policy::RiskEstimationPolicy,
    active_carid::Int,
    validfind::Int,
    pdset_frames_per_sim_frame::Int
    )

    #=
    1 - generate candidate trajectories
    2 - simulate candidate trajectories to estimate risk
    3 - select the highest-scoring trajectory
    4 - execute the highest-scoring trajectory
    =#

    ###########################################################
    # 1 - generate candidate trajectories
    
    candidate_trajectories = generate_candidate_trajectories(basics, policy, active_carid, validfind)
    extracted_trajdefs = extract_trajdefs(basics, candidate_trajectories, active_carid, validfind)

    ###########################################################
    # 2 - simulate candidate trajectories to estimate risk

    collision_risk = calc_collision_risk_monte_carlo!(basics, policy, extracted_trajdefs, 
                                                      active_carid, validfind)

    ###########################################################
    # 3 - select the highest-scoring trajectory

    best_trajectory_index = 1
    best_cost = Inf
    for (i,trajdef) in enumerate(candidate_trajectories)
        cost = calc_cost(trajdef, collision_risk[i], policy.desired_speed)
        if cost < best_cost
            best_trajectory_index = i
            best_cost = cost
        end
    end

    ###########################################################
    # 4 - execute the highest-scoring trajectory by writing directly into basics.pdset
    #    TODO(tim): maybe have 
    #         ActionProbabilityModel <: AbstractVehicleBehavior - is sampled for propagation
    #                  DrivingPolicy <: AbstractVehicleBehavior - directly overwrites pdset

    insert!(basics.pdset, extracted_trajdefs[best_trajectory_index], validfind+1, validfind+pdset_frames_per_sim_frame)
end

function evaluate_policy(
    basics::FeatureExtractBasicsPdSet,
    policy::RiskEstimationPolicy,
    active_carid::Integer,
    validfind_start::Integer,
    validfind_end::Integer,
    )

    for validfind in validfind_start : N_FRAMES_PER_SIM_FRAME : validfind_end-1

        propagate!(basics, policy, active_carid, validfind, N_FRAMES_PER_SIM_FRAME)

        if has_intersection(basics.pdset, validfind, min(validfind+N_FRAMES_PER_SIM_FRAME, validfind_end))
            return true
        end
    end
    
    return false
end
function evaluate_policy(
    scenario::Scenario,
    active_carid::Integer,
    policy::RiskEstimationPolicy,
    nsimulations::Integer
    )
    
    pdset = create_scenario_pdset(scenario)
    sn = scenario.sn
    basics = FeatureExtractBasicsPdSet(pdset, sn)

    collision_count = 0
    for simulation in 1 : nsimulations
        println("sim ", simulation)
        it_has_intersection = evaluate_policy(basics, policy, active_carid, scenario.history, nframeinds(basics.pdset))
        collision_count += it_has_intersection
        if it_has_intersection
            println("\tIT HAD INTERSECTION!")
        end
    end
    collision_count / nsimulations
end