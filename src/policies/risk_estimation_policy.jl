export 
    RiskEstimationPolicy,
    PolicyEvaluationResults,

    calc_integrated_squared_jerk,
    calc_cost,
    calc_costs,

    parallel_eval,

    generate_candidate_trajectories,
    calc_collision_risk_monte_carlo!,
    calc_costs,
    evaluate_policy,

    get_speed_deltas

type RiskEstimationPolicy <: AbstractVehicleBehavior

    trajectory_durations       :: Vector{Uint}    # various trajectory framecounts
    speed_deltas               :: Vector{Float64} # various speed deltas that we will check
    generate_follow_trajectory :: Bool            # wether to generate a policy to follow lead vehicle
                                                  # if there is one
    trailing_distance :: Float64      # distance behind lead car to follow at [m]
    k_relative_speed  :: Float64      # scalar affecting trailing distance [-], > 0


    nsimulations   :: Uint            # number of simulations per risk estimation
    history        :: Uint            # number of pdset frames in the past to record
    horizon        :: Uint            # number of pdset frames to predict forward
    desired_speed  :: Float64         # desired speed [m/s]
    col_method     :: Symbol          # collision check method ∈ [:OBB, :AABB, :Circle]
    sec_per_frame  :: Float64         # number of seconds per frame to use in prediction simulation
    human_behavior :: AbstractVehicleBehavior # behavior to assign to humans in sim

    k_c            :: Float64         # weight assigned to collision probability
    k_s            :: Float64         # weight assigned to relate lateral and longitudinal squared jerk
    k_v            :: Float64         # weight assigned to the squared terminal desired speed deviation

    function RiskEstimationPolicy{F<:Real, I<:Integer}(
        human_behavior::AbstractVehicleBehavior = VehicleBehaviorGaussian(0.001,0.1);

        speed_deltas::Vector{F} = [-2.235*2, 0.0, 2.235*2],
        generate_follow_trajectory::Bool = true,
        trailing_distance::Real = 40.0,
        k_relative_speed::Real = 0.0,

        nsimulations::Integer = 100,
        history::Integer = 2*DEFAULT_FRAME_PER_SEC,
        horizon::Integer = 8*DEFAULT_FRAME_PER_SEC,
        desired_speed::Real = 29.06,
        sec_per_frame::Real = DEFAULT_SEC_PER_FRAME,
        col_method::Symbol = :OBB,

        trajectory_durations::Vector{I} = [div(horizon,2), horizon],

        k_c::Real = 100.0,
        k_s::Real =   0.5,
        k_v::Real =   0.01,
        )

        retval = new()
        retval.trajectory_durations = convert(Vector{Uint}, trajectory_durations)
        retval.speed_deltas   = convert(Vector{Float64}, speed_deltas)
        retval.generate_follow_trajectory = generate_follow_trajectory

        retval.nsimulations   = nsimulations
        retval.horizon        = history
        retval.horizon        = horizon
        retval.desired_speed  = desired_speed
        retval.col_method     = col_method
        retval.sec_per_frame  = sec_per_frame
        retval.human_behavior = human_behavior

        retval.k_c            = k_c
        retval.k_s            = k_s
        retval.k_v            = k_v

        retval
    end
end

function get_speed_deltas(delta_count::Int, delta_jump::Float64)
    if delta_count == 0
        [0.0]
    else
        [-delta_count : delta_count].*delta_jump
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
    extracted_polynomial_factored_trajectories::ExtractedPolynomialFactoredTrajectories,
    collision_prob::Real,
    policy::RiskEstimationPolicy;
    )

    #=
    Assign a scalar cost to a given trajectory
        A lower cost is better
    =#

    k_c = policy.k_c
    k_s = policy.k_s
    k_v = policy.k_v

    J_s = 0.0
    J_d = 0.0
    # for i in 1:length(extracted_polynomial_factored_trajectories)
    #     poly = extracted_polynomial_factored_trajectories.trajs[i]
    #     τ = extracted_polynomial_factored_trajectories.durations[i]

    #     J_s += calc_integrated_squared_jerk(poly.s, τ)
    #     J_d += calc_integrated_squared_jerk(poly.d, τ)
    # end 

    sdot_f = p₂(extracted_polynomial_factored_trajectories.trajs[end].s, extracted_polynomial_factored_trajectories.durations[end])
    ddot_f = p₂(extracted_polynomial_factored_trajectories.trajs[end].d, extracted_polynomial_factored_trajectories.durations[end])
    v_f = sqrt(sdot_f*sdot_f + ddot_f*ddot_f)

    Δv = v_f - policy.desired_speed
    Δv = Δv*Δv

    k_c*collision_prob + ((1.0-k_s)*J_d + k_s*J_s) + k_v*Δv
end

function parallel_eval(tup::(PrimaryDataset, StreetNetwork, Vector{TrajDefLink}, Int, Int, Int, AbstractVehicleBehavior, Float64))

    pdset_for_sim  = tup[1]
    sn             = tup[2]
    links          = tup[3]
    active_carid   = tup[4]
    validfind      = tup[5]
    nsimulations   = tup[6]
    human_behavior = tup[7]
    sec_per_frame  = tup[8]

    behavior_pairs = (AbstractVehicleBehavior, Int)[]
    for carid in get_carids(pdset_for_sim)
        if carid != active_carid
            push!(behavior_pairs, (human_behavior,carid))
        end
    end

    basics_for_sim = FeatureExtractBasicsPdSet(pdset_for_sim, sn)
    trajdef = TrajDef(pdset_for_sim, sn, active_carid, validfind)
    append!(trajdef.links, links)

    frameind_start = validfind2frameind(pdset_for_sim, validfind)
    extracted, extracted_polynomial_factored_trajectories = extract_trajdef(pdset_for_sim, sn, trajdef, active_carid, frameind_start, sec_per_frame)

    insert!(pdset_for_sim, extracted)
    horizon = get_num_pdset_frames(links)-1
    calc_collision_risk_monte_carlo!(
        basics_for_sim, behavior_pairs, 
        validfind, validfind+horizon-1,
        nsimulations=nsimulations)
end

function generate_candidate_trajectories(
    basics::FeatureExtractBasicsPdSet,
    policy::RiskEstimationPolicy,
    active_carid::Integer,
    validfind::Integer,
    )

    #=
    Generate a set of candidate trajectories as a list of TrajDefLink lists
    All trajectories are assumed to begin at the current position of active_carid
    
    Returns a Vector{Vector{TrajDefLink}}

    Trajectories of duration policy.horizon will be generated
    Sequence length is dictated by policy.trajectory_durations
    Any sequences longer than that will be truncated to policy.horizon
    Any sequences shorter than that will be extended to policy.horizon assuming constant speed
    =#

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
    closest_node_index = closest_node_to_extind(sn, lane, extind)
    closest_node = sn.nodes[closest_node_index]

    lanetags = [cur_lanetag]
    if closest_node.n_lanes_left > 0
        push!(lanetags, get_neighbor_lanetag_left(sn, lane, closest_node))
    end
    if closest_node.n_lanes_right > 0
        push!(lanetags, get_neighbor_lanetag_right(sn, lane, closest_node))
    end

    for lanetag in lanetags
        for speed_delta in policy.speed_deltas
            for duration in policy.trajectory_durations
                push!(candidate_trajectories, [TrajDefLinkTargetSpeed(duration, lanetag, 0.0, start_speed+speed_delta)])
                if duration < horizon
                    remaining_frames = horizon - duration
                    push!(candidate_trajectories[end], TrajDefLinkTargetSpeed(remaining_frames, lanetag, 0.0, start_speed+speed_delta))
                end
            end
        end
    end

    # TODO(tim): genearte follow trajectories for left and right lanes?
    if policy.generate_follow_trajectory
        # check whether there is a lead vehicle
        carind = carid2ind(pdset, active_carid, validfind)
        f_ind_front = get(INDFRONT, basics, carind, validfind)
        if f_ind_front != NA_ALIAS
            ind_front = int(f_ind_front)

            lanetag = get(pdset, :lanetag, ind_front, validfind)::LaneTag

            frameind = validfind2frameind(pdset, validfind)

            push!(candidate_trajectories, [TrajDefLinkTargetPosition(pdset, frameind,
                                                                     horizon, policy.sec_per_frame, ind_front, lanetag, 
                                                                     0.0, policy.trailing_distance, policy.k_relative_speed)])
        end
    end

    candidate_trajectories
end

function calc_collision_risk_monte_carlo!(
    basics::FeatureExtractBasicsPdSet,
    policy::RiskEstimationPolicy,
    candidate_trajectories::Vector{Vector{TrajDefLink}},
    active_carid::Integer,
    validfind::Integer,
    sec_per_frame::Float64,
    )

    pdset, sn = basics.pdset, basics.sn

    n_othercars = get_num_cars_in_frame(pdset, validfind)-1
    n_candidatetrajs = length(candidate_trajectories)

    # allocate pdset for sim 
    # TODO(tim): allocate one for *horizon*, but copy the past
    pdset_for_sim = deepcopy(pdset)

    max_validfind = maximum([validfind + get_num_pdset_frames(links) for links in candidate_trajectories])::Int
    if max_validfind > nvalidfinds(pdset_for_sim)
        expand!(pdset_for_sim, max_validfind - nvalidfinds(pdset_for_sim))
    end

    nsimulations = policy.nsimulations
    human_behavior = policy.human_behavior

    tups = Array((PrimaryDataset, StreetNetwork, Vector{TrajDefLink}, Int, Int, Int, AbstractVehicleBehavior, Float64), length(candidate_trajectories))
    for (i,links) in enumerate(candidate_trajectories)
        tups[i] = (pdset_for_sim, sn, links, active_carid, validfind, nsimulations, human_behavior, sec_per_frame)
    end

    collision_risks = convert(Vector{Float64}, pmap(parallel_eval, tups))

    collision_risks
end
function calc_costs(
    policy::RiskEstimationPolicy,
    extracted_trajs::Vector{ExtractedPolynomialFactoredTrajectories},
    collision_risks::Vector{Float64},
    )
    
    costs = Array(Float64, length(extracted_trajs))
    for (i,extracted_polynomial_factored_trajectories) in enumerate(extracted_trajs)
        costs[i] = calc_cost(extracted_polynomial_factored_trajectories, collision_risks[i], policy)
    end
    costs
end

function propagate!(
    basics::FeatureExtractBasicsPdSet,
    policy::RiskEstimationPolicy,
    active_carid::Int,
    validfind::Int,
    pdset_frames_per_sim_frame::Int,
    sec_per_frame::Float64=DEFAULT_SEC_PER_FRAME
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
    extracted_trajdefs, extracted_polies = extract_trajdefs(basics, candidate_trajectories, active_carid, validfind)

    ###########################################################
    # 2 - simulate candidate trajectories to estimate risk

    sec_per_frame = calc_sec_per_frame(basics.pdset)
    collision_risk = calc_collision_risk_monte_carlo!(basics, policy, candidate_trajectories, 
                                                      active_carid, validfind, sec_per_frame)

    ###########################################################
    # 3 - select the highest-scoring trajectory

    best_trajectory_index = 1
    best_cost = Inf
    for (i,extracted_polynomial_factored_trajectories) in enumerate(extracted_polies)
        cost = calc_cost(extracted_polynomial_factored_trajectories, collision_risk[i], policy)
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

immutable PolicyEvaluationResults
    nruns::Int             # number of runs conducted to judge performance
    ncollisions::Int       # number of collisions during policy evaluation
    time_per_tick::Float64 # average time per policy tick
    performance::Float64   # overall policy performance measure
end
immutable SinglePolicyRunEvalResults
    had_collision::Bool    # whether a collision occurred
    total_tick_time::Float64 # average time per policy tick
    nticks::Int
end

function _evaluate_policy(
    basics::FeatureExtractBasicsPdSet,
    policy::RiskEstimationPolicy,
    active_carid::Integer,
    validfind_start::Integer,
    validfind_end::Integer,
    )
    
    nticks = 0
    total_tick_time = 0.0
    for validfind in validfind_start : N_FRAMES_PER_SIM_FRAME : validfind_end-1

        starttime = time()    
        propagate!(basics, policy, active_carid, validfind, N_FRAMES_PER_SIM_FRAME)
        total_tick_time += time() - starttime
        nticks += 1

        if has_intersection(basics.pdset, validfind, min(validfind+N_FRAMES_PER_SIM_FRAME, validfind_end))
            return SinglePolicyRunEvalResults(true, total_tick_time, nticks)
        end
    end
    
    SinglePolicyRunEvalResults(false, total_tick_time, nticks)
end
function evaluate_policy(
    scenario::Scenario,
    active_carid::Integer,
    policy::RiskEstimationPolicy,
    nruns::Integer;

    r_collision_frequency::Float64 = -100.0, # weighting for collision frequency
    r_time_per_tick::Float64       =   -1.0, # weighting for time per tick

    verbosity::Int = 0
    )
    
    pdset = create_scenario_pdset(scenario)
    sn = scenario.sn
    basics = FeatureExtractBasicsPdSet(pdset, sn)

    ncollisions = 0
    nticks = 0
    total_tick_time = 0.0

    for simulation in 1 : nruns
        verbosity ≤ 0 || println("policy evaluation run ", simulation, " / ", nruns)
        eval_results = _evaluate_policy(basics, policy, active_carid, scenario.history, nframeinds(basics.pdset))
        ncollisions += eval_results.had_collision
        total_tick_time += eval_results.total_tick_time
        nticks += eval_results.nticks
    end

    collision_frequency = ncollisions/nruns
    time_per_tick = total_tick_time/nticks
    performance = r_collision_frequency * collision_frequency +
                  r_time_per_tick * time_per_tick

    PolicyEvaluationResults(nruns, ncollisions, time_per_tick, performance)
end