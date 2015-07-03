export
    VehicleBehaviorEM,
    ModelSimParams,

    calc_probability_distribution_over_assignments!,
    calc_log_probability_of_assignment

import Discretizers: encode, decode
import Graphs: topological_sort_by_dfs, in_degree, in_neighbors


# The vehicle always selects actions using the given encounter model
type ModelSimParams
    sampling_scheme  :: AbstractSampleMethod
    smoothing        :: Symbol # :none, :SMA, :WMA
    smoothing_counts :: Int    # number of previous counts to use

    function ModelSimParams(
        sampling_scheme :: AbstractSampleMethod = SAMPLE_UNIFORM,
        smoothing       :: Symbol = :none,
        smoothing_counts :: Int = 1
        )

        @assert(smoothing_counts > 0)
        new(sampling_scheme, smoothing, smoothing_counts)
    end
end
type VehicleBehaviorEM <: AbstractVehicleBehavior
    em :: EM
    indicators :: Vector{AbstractFeature}
    symbol_lat :: Symbol
    symbol_lon :: Symbol

    simparams_lat :: ModelSimParams
    simparams_lon :: ModelSimParams

    function VehicleBehaviorEM(
        em            :: EM,
        simparams_lat :: ModelSimParams,
        simparams_lon :: ModelSimParams
        )

        retval = new()
        retval.em = em
        retval.indicators = get_indicators(em)

        targets = get_targets(em)
        retval.symbol_lat = symbol(get_target_lat(em, targets))
        retval.symbol_lon = symbol(get_target_lon(em, targets))

        retval.simparams_lat = simparams_lat
        retval.simparams_lon = simparams_lon

        retval
    end
end

function select_action(
    basics    :: FeatureExtractBasics,
    behavior  :: VehicleBehaviorEM,
    carind    :: Int,
    frameind  :: Int
    )

    # propagate the simulation by one step for the given vehicle

    em = behavior.em
    symbol_lat = behavior.symbol_lat
    symbol_lon = behavior.symbol_lon
    
    simparams_lat = behavior.simparams_lat
    simparams_lon = behavior.simparams_lon
    samplemethod_lat = simparams_lat.sampling_scheme
    samplemethod_lon = simparams_lon.sampling_scheme
    smoothing_lat = simparams_lat.smoothing
    smoothing_lon = simparams_lon.smoothing
    smoothcounts_lat = simparams_lat.smoothing_counts
    smoothcounts_lon = simparams_lon.smoothing_counts

    bmap_lat = em.binmaps[findfirst(em.BN.names, symbol_lat)]
    bmap_lon = em.binmaps[findfirst(em.BN.names, symbol_lon)]

    logindexbase = calc_logindexbase(carind)

    observations = observe(basics, carind, frameind, behavior.indicators)
    assignment = encode(observations, em)
    assignment, logPs = sample_and_logP!(em, assignment)

    logPa = logPs[symbol_lon]
    logPω = logPs[symbol_lat]

    bin_lat = assignment[symbol_lat]
    bin_lon = assignment[symbol_lon]

    action_lat = decode(bmap_lat, bin_lat, samplemethod_lat)
    action_lon = decode(bmap_lon, bin_lon, samplemethod_lon)

    a = get_input_acceleration(symbol_lon, action_lon, basics.simlog, frameind, logindexbase)
    ω = get_input_turnrate(    symbol_lat, action_lat, basics.simlog, frameind, logindexbase)

    if smoothing_lat == :SMA
        ω = calc_sequential_moving_average(basics.simlog, sub2ind(size(basics.simlog), frameind, logindexbase + LOG_COL_T), smoothcounts_lat)
    elseif smoothing_lat == :WMA
        ω = calc_weighted_moving_average(basics.simlog, sub2ind(size(basics.simlog), frameind, logindexbase + LOG_COL_T), smoothcounts_lat)
    end

    if smoothing_lon == :SMA
        a = calc_sequential_moving_average(basics.simlog, sub2ind(size(basics.simlog), frameind, logindexbase + LOG_COL_A), smoothcounts_lon)
    elseif smoothing_lon == :WMA
        a = calc_weighted_moving_average(basics.simlog, sub2ind(size(basics.simlog), frameind, logindexbase + LOG_COL_A), smoothcounts_lon)
    end

    _record_frame_values!(basics.simlog, frameind, logindexbase, 
                          bin_lat=bin_lat, bin_lon=bin_lon,
                          action_lat=action_lat, action_lon=action_lon,
                          logPa=logPa, logPω=logPω)

    (a, ω)
end
function calc_action_loglikelihood(
    basics::FeatureExtractBasics,
    behavior::VehicleBehaviorEM,
    carind::Int,
    frameind::Int
    )

    #=
    Compute the log-likelihood of the action taken during a single frame
    given the VehicleBehaviorEM

    Must:
       - pull the actual acceleration and turnrate for the frame
       - reverse the smoothing
       - determine what the lateral and longitudinal actions were
       - compute the pdf(sample) for lat and lon
    =#

    logindexbase = calc_logindexbase(carind)

    em = behavior.em
    symbol_lat = behavior.symbol_lat
    symbol_lon = behavior.symbol_lon
    bmap_lat = em.binmaps[findfirst(em.BN.names, symbol_lat)]
    bmap_lon = em.binmaps[findfirst(em.BN.names, symbol_lon)]

    simparams_lat = behavior.simparams_lat
    simparams_lon = behavior.simparams_lon
    samplemethod_lat = simparams_lat.sampling_scheme
    samplemethod_lon = simparams_lon.sampling_scheme
    smoothing_lat = simparams_lat.smoothing
    smoothing_lon = simparams_lon.smoothing
    smoothcounts_lat = simparams_lat.smoothing_counts
    smoothcounts_lon = simparams_lon.smoothing_counts

    a = basics.simlog[frameind, logindexbase + LOG_COL_A]
    ω = basics.simlog[frameind, logindexbase + LOG_COL_T]

    if behavior.simparams_lat.smoothing == :SMA
        ω = _reverse_smoothing_sequential_moving_average(basics.simlog, sub2ind(size(basics.simlog), frameind, logindexbase + LOG_COL_T), smoothcounts_lat)
    elseif behavior.simparams_lat.smoothing == :WMA
        ω = _reverse_smoothing_weighted_moving_average(basics.simlog, sub2ind(size(basics.simlog), frameind, logindexbase + LOG_COL_T), smoothcounts_lat)
    end

    if behavior.simparams_lon.smoothing == :SMA
        a = _reverse_smoothing_sequential_moving_average(basics.simlog, sub2ind(size(basics.simlog), frameind, logindexbase + LOG_COL_A), smoothcounts_lon)
    elseif behavior.simparams_lon.smoothing == :WMA
        a = _reverse_smoothing_weighted_moving_average(basics.simlog, sub2ind(size(basics.simlog), frameind, logindexbase + LOG_COL_A), smoothcounts_lon)
    end

    action_lat = _infer_action_lat_from_input_turnrate(symbol_lat, ω, basics.simlog, frameind, logindexbase)
    action_lon = _infer_action_lon_from_input_acceleration(symbol_lon, a, basics.simlog, frameind, logindexbase)

    if min(bmap_lat) ≤ action_lat ≤ max(bmap_lat) &&
        min(bmap_lon) ≤ action_lon ≤ max(bmap_lon)


        bin_lat = encode(bmap_lat, action_lat)
        bin_lon = encode(bmap_lon, action_lon)

        observations = observe(basics, carind, frameind, behavior.indicators)
        assignment = encode(observations, em)

        binprobs_lat = Array(Float64, nlabels(bmap_lat))
        binprobs_lon = Array(Float64, nlabels(bmap_lon))

        if is_parent(em, symbol_lon, symbol_lat) # lon -> lat
            calc_probability_distribution_over_assignments!(binprobs_lon, em, assignment, symbol_lon)
            fill!(binprobs_lat, 0.0)
            temp = Array(Float64, nlabels(bmap_lon))
            for (i,p) in enumerate(binprobs_lon)
                assignment[symbol_lon] = i
                binprobs_lat += calc_probability_distribution_over_assignments!(temp, em, assignment, symbol_lat) .* p
            end
        elseif is_parent(em, symbol_lat, symbol_lon) # lat -> lon
            calc_probability_distribution_over_assignments!(binprobs_lat, em, assignment, symbol_lat)
            fill!(binprobs_lon, 0.0)
            temp = Array(Float64, nlabels(bmap_lat))
            for (i,p) in enumerate(binprobs_lat)
                assignment[symbol_lat] = i
                binprobs_lon += calc_probability_distribution_over_assignments!(temp, em, assignment, symbol_lon) .* p
            end
        else
            calc_probability_distribution_over_assignments!(binprobs_lat, em, assignment, symbol_lat)
            calc_probability_distribution_over_assignments!(binprobs_lon, em, assignment, symbol_lon)
        end

        P_bin_lat = binprobs_lat[bin_lat]
        P_bin_lon = binprobs_lon[bin_lon]

        p_within_bin_lat = calc_probability_for_uniform_sample_from_bin(P_bin_lat, bmap_lat, bin_lat)
        p_within_bin_lon = calc_probability_for_uniform_sample_from_bin(P_bin_lon, bmap_lon, bin_lon)

        return log(p_within_bin_lat * p_within_bin_lon)
    else
        -Inf
    end
end

function observe(
    basics::FeatureExtractBasics,
    carind::Int,
    frameind::Int,
    features::Vector{AbstractFeature}
    )

    observations = Dict{Symbol,Any}()
    for f in features
        val = get(f, basics, carind, frameind)
        observations[symbol(f)] = val
    end
    observations
end
function encode(observations::Dict{Symbol, Any}, em::EM)
    # take each observation and bin it appropriately for the EM
    # returns a Dict{Symbol,Int}

    assignment = Dict{Symbol,Int}()
    for i = 1 : length(em.istarget)
        if !em.istarget[i]
            sym = em.BN.names[i]
            if isnan(observations[sym])
                error("$sym is NaN!")
            end
            assignment[sym] = encode(em.binmaps[i], observations[sym])
        end
    end
    assignment
end
function sample!(em::EM, assignment::Dict{Symbol, Int})
    # run through nodes in topological order, building the instantiation vector as we go
    # nodes we already know we use
    # nodes we do not know we sample from
    # modifies assignment to include new sampled symbols

    # TODO(tim): precompute and store ordering
    ordering = topological_sort_by_dfs(em.BN.dag)
    for name in em.BN.names[ordering]
        if !haskey(assignment, name)
            assignment[name] = BayesNets.rand(BayesNets.cpd(em.BN, name), assignment)
        end
    end
    assignment
end
function sample_and_logP!(em::EM, assignment::Dict{Symbol, Int})
    
    logPs = Dict{Symbol, Float64}()

    # TODO(tim): precompute and store ordering?
    ordering = topological_sort_by_dfs(em.BN.dag)
    for name in em.BN.names[ordering]
        if !haskey(assignment, name)
            cpd = BayesNets.cpd(em.BN, name)

            p = cpd.parameterFunction(assignment)
            n = length(p)
            i = 1
            c = p[1]
            u = rand()
            while c < u && i < n
                c += p[i += 1]
            end
            assignment[name] = cpd.domain[i]
            logPs[name] = log(p[i])
        end
    end

    assignment, logPs
end
function calc_log_probability_of_assignment(em::EM, assignment::Dict{Symbol, Int}, symb::Symbol)
    # returns the discrete log probability of the given bin assignment
    cpd = BayesNets.cpd(em.BN, symb)
    bin = assignment[sym]
    p = cpd.parameterFunction(assignment)
    log(p[bin])
end
function calc_probability_distribution_over_assignments!(
    dest::Vector{Float64},
    em::EM,
    assignment::Dict{Symbol, Int},
    target::Symbol
    )

    # NOTE (tim): cpd.parameterFunction(assignment) returns the actual probability vector, not a copy
    cpd = BayesNets.cpd(em.BN, target)
    copy!(dest, cpd.parameterFunction(assignment))
end

function get_input_acceleration(sym::Symbol, action_lon::Float64, simlog::Matrix{Float64}, frameind::Int, logindexbase::Int)

    if sym == :f_accel_250ms || sym == :f_accel_500ms
        return action_lon
    elseif sym == :f_des_speed_250ms || sym == :f_des_speed_500ms
        Δv_des = action_lon 
        Kp = Features.KP_DESIRED_SPEED
        return Δv_des*Kp
    else
        error("unknown longitudinal target $sym")
    end
end
function get_input_turnrate(sym::Symbol, action_lat::Float64, simlog::Matrix{Float64}, frameind::Int, logindexbase::Int)

    if sym == :f_turnrate_250ms || sym == :f_turnrate_500ms
        return action_lat
    elseif sym == :f_des_angle_250ms || sym == :f_des_angle_500ms
        phi_des = action_lat
        ϕ = simlog[frameind, logindexbase + LOG_COL_ϕ]
        return (phi_des - ϕ)*Features.KP_DESIRED_ANGLE
    else
        error("unknown lateral target $sym")
    end
end

function _infer_action_lon_from_input_acceleration(sym::Symbol, accel::Float64, simlog::Matrix{Float64}, frameind::Int, logindexbase::Int)

    if sym == :f_accel_250ms || sym == :f_accel_500ms
        return accel
    elseif sym == :f_des_speed_250ms || sym == :f_des_speed_500ms
        return accel/Features.KP_DESIRED_SPEED
    else
        error("unknown longitudinal target $sym")
    end
end
function _infer_action_lat_from_input_turnrate(sym::Symbol, turnrate::Float64, simlog::Matrix{Float64}, frameind::Int, logindexbase::Int)

    if sym == :f_turnrate_250ms || sym == :f_turnrate_500ms
        return turnrate
    elseif sym == :f_des_angle_250ms || sym == :f_des_angle_500ms
        ϕ = simlog[frameind, logindexbase + LOG_COL_ϕ]
        return (turnrate / Features.KP_DESIRED_ANGLE) + ϕ
    else
        error("unknown lateral target $sym")
    end
end

function draw_action(emsample::Dict{Symbol, Int}, em::EM)
    # construct an action from the emsample
    # create a dictionary mapping target features to specific values (float64, probably)

    action_dict = Dict{Symbol,Any}()
    for i = 1 : length(em.istarget)
        if em.istarget[i]
            sym = em.BN.names[i]
            action_dict[sym] = decode(em.binmaps[i], emsample[sym])
        end
    end
    action_dict
end