export  SimParams,
       
        simulate!,
        tick!,
        propagate!,

        init_log!,
        init_logs!,

        isdone,

        calc_probability_distribution_over_assignments,
        calc_log_probability_of_assignment


import Discretizers: encode, decode
import Graphs: topological_sort_by_dfs, in_degree, in_neighbors
using  DataArrays


immutable SimParams
	sec_per_frame :: Float64
	n_euler_steps :: Int
	extracted_feature_cache :: ExtractedFeatureCache

	function SimParams(
		sec_per_frame :: Float64 = 0.25,
		n_euler_steps :: Int     = 10,
		extracted_feature_cache :: ExtractedFeatureCache = ExtractedFeatureCache()
		)

		@assert(sec_per_frame > 0.0)
		@assert(n_euler_steps > 0)

		new(sec_per_frame, n_euler_steps, extracted_feature_cache)
	end
end

function simulate!{B<:AbstractVehicleBehavior}(
	simlog        :: Matrix{Float64}, # initialized appropriately
	behaviors     :: Vector{B},
	road          :: StraightRoadway,
	frameind      :: Int, # starting index within simlog
	params        :: SimParams = SimParams(),
	)

	numcars = get_ncars(simlog)
	@assert(length(behaviors) == numcars)

	while !isdone(simlog, frameind)

		for (carind,behavior) in enumerate(behaviors)
			tick!(simlog, carind, behavior, road, frameind, params)
		end

		frameind += 1
	end

	simlog
end
function simulate!{B<:AbstractVehicleBehavior}(
    simlogs::Vector{Matrix{Float64}},
    behaviors::Vector{B},
    road::StraightRoadway,
    history::Int,
    simparams :: SimParams
    )

    for simlog in simlogs
        simulate!(simlog, behaviors[1:get_ncars(simlog)], road, history, simparams)
    end
    simlogs
end

function tick!(
	simlog    :: Matrix{Float64}, # initialized appropriately
	carind    :: Int,
	behavior  :: VehicleBehaviorNone,
	road      :: StraightRoadway,
	frameind  :: Int,
	params    :: SimParams = SimParams()
	)

	simlog
end
function tick!(
	simlog    :: Matrix{Float64}, # initialized appropriately
	carind    :: Int,
	behavior  :: VehicleBehaviorDriveStraight,
	road      :: StraightRoadway,
	frameind  :: Int,
	params    :: SimParams = SimParams()
	)

	sec_per_frame = params.sec_per_frame
	n_euler_steps = params.n_euler_steps
	δt            = sec_per_frame / n_euler_steps

	a = 0.0
	ω = 0.0

	logPa = 0.0
	logPω = 0.0

	logindexbase = calc_logindexbase(carind)

	propagate!(simlog, frameind, logindexbase, a, ω, logPa, logPω, EM_ID_UNKNOWN, n_euler_steps, δt)
end
function tick!(
	simlog    :: Matrix{Float64}, # initialized appropriately
	carind    :: Int,
	behavior  :: VehicleBehaviorEM,
	road      :: StraightRoadway,
	frameind  :: Int,
	params    :: SimParams = SimParams()
	)

	# propagate the simulation by one step for the given vehicle

	em = behavior.em
	symbol_lat = behavior.symbol_lat
	symbol_lon = behavior.symbol_lon

	sec_per_frame = params.sec_per_frame
	n_euler_steps = params.n_euler_steps
	δt            = sec_per_frame / n_euler_steps
	extracted_feature_cache = params.extracted_feature_cache
	
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

	observations = observe(simlog, road, carind, frameind, behavior.indicators, sec_per_frame, extracted_feature_cache)
	assignment   = encode(observations, em)
	assignment, logPs  = sample_and_logP!(em, assignment)

	action_lat = decode(bmap_lat, assignment[symbol_lat], samplemethod_lat)
	action_lon = decode(bmap_lon, assignment[symbol_lon], samplemethod_lon)

	a = get_input_acceleration(symbol_lon, action_lon, simlog, frameind, logindexbase)
	ω = get_input_turnrate(    symbol_lat, action_lat, simlog, frameind, logindexbase)

	logPa = logPs[symbol_lon]
	logPω = logPs[symbol_lat]

	if smoothing_lat == :SMA
		n_frames = frameind > smoothcounts_lat ? smoothcounts_lat : frameind
		lo = frameind-(n_frames-1)
		ω = (sum(simlog[lo:frameind-1, logindexbase + LOG_COL_T]) + ω) / n_frames
	elseif smoothing_lat == :WMA
		n_frames = frameind > smoothcounts_lat ? smoothcounts_lat : frameind
		lo = frameind-(n_frames-1)
		ω = (sum(simlog[lo:frameind-1, logindexbase + LOG_COL_T] .* [1:n_frames-1]) + n_frames*ω) / (0.5n_frames*(n_frames+1))
	end

	if smoothing_lon == :SMA
		n_frames = frameind > smoothcounts_lon ? smoothcounts_lon : frameind
		lo = frameind-(n_frames-1)
		a = (sum(simlog[lo:frameind-1, logindexbase + LOG_COL_A])+a) / n_frames
	elseif smoothing_lon == :WMA
		n_frames = frameind > smoothcounts_lon ? smoothcounts_lon : frameind
		lo = frameind-(n_frames-1)
		a = (sum(simlog[lo:frameind-1, logindexbase + LOG_COL_A] .* [1:n_frames-1]) + n_frames*a) / (0.5n_frames*(n_frames+1))
	end

	propagate!(simlog, frameind, logindexbase, a, ω, logPa, logPω, EM_ID_UNKNOWN, n_euler_steps, δt)
end
function tick!(
	simlog    :: Matrix{Float64}, # initialized appropriately
	carind    :: Int,
	behavior_orig  :: VehicleBehaviorEMOriginal,
	road      :: StraightRoadway,
	frameind  :: Int,
	params    :: SimParams = SimParams()
	)

	# propagate the simulation by one step for the given vehicle
	# chose the accel from the bin which is already given in the simlog

	behavior = behavior_orig.behavior
	em = behavior.em
	symbol_lat = behavior.symbol_lat
	symbol_lon = behavior.symbol_lon

	sec_per_frame = params.sec_per_frame
	n_euler_steps = params.n_euler_steps
	δt            = sec_per_frame / n_euler_steps
	extracted_feature_cache = params.extracted_feature_cache
	
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

	observations = observe(simlog, road, carind, frameind, behavior.indicators, sec_per_frame, extracted_feature_cache)
	assignment   = encode(observations, em)

	bin_lat = encode(bmap_lat, simlog[frameind, logindexbase + LOG_COL_T])
	bin_lon = encode(bmap_lon, simlog[frameind, logindexbase + LOG_COL_A])

	action_lat = decode(bmap_lat, bin_lat, samplemethod_lat)
	action_lon = decode(bmap_lon, bin_lon, samplemethod_lon)

	a = get_input_acceleration(symbol_lon, action_lon, simlog, frameind, logindexbase)
	ω = get_input_turnrate(    symbol_lat, action_lat, simlog, frameind, logindexbase)

	logPa = calc_log_probability_of_assignment(em, assignment, symbol_lon)
	logPω = calc_log_probability_of_assignment(em, assignment, symbol_lat)

	if smoothing_lat == :SMA
		n_frames = frameind > smoothcounts_lat ? smoothcounts_lat : frameind
		lo = frameind-(n_frames-1)
		ω = (sum(simlog[lo:frameind-1, logindexbase + LOG_COL_T]) + ω) / n_frames
	elseif smoothing_lat == :WMA
		n_frames = frameind > smoothcounts_lat ? smoothcounts_lat : frameind
		lo = frameind-(n_frames-1)
		ω = (sum(simlog[lo:frameind-1, logindexbase + LOG_COL_T] .* [1:n_frames-1]) + n_frames*ω) / (0.5n_frames*(n_frames+1))
	end

	if smoothing_lon == :SMA
		n_frames = frameind > smoothcounts_lon ? smoothcounts_lon : frameind
		lo = frameind-(n_frames-1)
		a = (sum(simlog[lo:frameind-1, logindexbase + LOG_COL_A])+a) / n_frames
	elseif smoothing_lon == :WMA
		n_frames = frameind > smoothcounts_lon ? smoothcounts_lon : frameind
		lo = frameind-(n_frames-1)
		a = (sum(simlog[lo:frameind-1, logindexbase + LOG_COL_A] .* [1:n_frames-1]) + n_frames*a) / (0.5n_frames*(n_frames+1))
	end

	propagate!(simlog, frameind, logindexbase, a, ω, logPa, logPω, EM_ID_UNKNOWN, n_euler_steps, δt)
end

function propagate!(
	simlog        :: Matrix{Float64},
	frameind      :: Int,
	logindexbase  :: Int,
	a             :: Float64,
	ω             :: Float64,
	logPa         :: Float64,
	logPω         :: Float64,
	em_id         :: Int,
	n_euler_steps :: Int,
	δt            :: Float64,
	)

	# run physics on the given car at time frameind
	# place results in log for that car in frameind + 1
	

	simlog[frameind, logindexbase + LOG_COL_A] = a
	simlog[frameind, logindexbase + LOG_COL_T] = ω
	simlog[frameind, logindexbase + LOG_COL_logprobweight_A] = logPa
	simlog[frameind, logindexbase + LOG_COL_logprobweight_T] = logPω
	simlog[frameind, logindexbase + LOG_COL_em] = float64(em_id)

	x = simlog[frameind, logindexbase + LOG_COL_X]
	y = simlog[frameind, logindexbase + LOG_COL_Y]
	ϕ = simlog[frameind, logindexbase + LOG_COL_ϕ]
	v = simlog[frameind, logindexbase + LOG_COL_V]

	for j = 1 : n_euler_steps
		v += a*δt
		ϕ += ω*δt
		x += v*cos(ϕ)*δt
		y += v*sin(ϕ)*δt
	end

	simlog[frameind+1, logindexbase + LOG_COL_X] = x
	simlog[frameind+1, logindexbase + LOG_COL_Y] = y
	simlog[frameind+1, logindexbase + LOG_COL_ϕ] = ϕ
	simlog[frameind+1, logindexbase + LOG_COL_V] = v

	simlog
end
function propagate!(
	simlog        :: Matrix{Float64},
	frameind      :: Int,
	logindexbase  :: Int,
	a             :: Float64,
	ω             :: Float64,
	logPa         :: Float64,
	logPω         :: Float64,
	em_id         :: Int,
	params        :: SimParams	
	)
	
	sec_per_frame = params.sec_per_frame
	n_euler_steps = params.n_euler_steps
	δt            = sec_per_frame / n_euler_steps

	propagate!(simlog, frameind, logindexbase, a, ω, logPa, logPω, em_id, n_euler_steps, δt)
end
function propagate!(
	simlog        :: Matrix{Float64},
	frameind      :: Int,
	a             :: Float64,
	ω             :: Float64,
	logPa         :: Float64,
	logPω         :: Float64,
	em_id         :: Int,
	params        :: SimParams	
	)
	
	sec_per_frame = params.sec_per_frame
	n_euler_steps = params.n_euler_steps
	δt            = sec_per_frame / n_euler_steps

	for carind in get_ncars(simlog)
		propagate!(simlog, frameind, calc_logindexbase(carind), a, ω, logPa, logPω, em_id, n_euler_steps, δt)
	end

	simlog
end

init_log!(simlog::Matrix{Float64}, carind::Int, ::VehicleBehaviorNone, trace::VehicleTrace, startframe::Int) =
    fill_log_with_trace_complete!(simlog, trace, carind, startframe)
init_log!(simlog::Matrix{Float64}, carind::Int, ::VehicleBehaviorEM, trace::VehicleTrace, startframe::Int) =
    fill_log_with_trace_partial!(simlog, trace, carind, startframe)
init_log!(simlog::Matrix{Float64}, carind::Int, ::VehicleBehaviorSS, trace::VehicleTrace, startframe::Int) =
    fill_log_with_trace_partial!(simlog, trace, carind, startframe)

function init_log!{B<:AbstractVehicleBehavior}(
    simlog     :: Matrix{Float64},
    behaviors  :: Vector{B},
    traces     :: Vector{VehicleTrace},
    startframe :: Int
    )
    
    num_cars = get_ncars(simlog)
    @assert(num_cars == length(behaviors))
    @assert(num_cars == length(traces))

    for carind = 1 : num_cars
        behavior = behaviors[carind]
        trace = traces[carind]
        init_log!(simlog, carind, behavior, trace, startframe)
    end

    simlog
end
function init_logs!{B<:AbstractVehicleBehavior}(
    simlogs::Vector{Matrix{Float64}},
    tracesets::Vector{Vector{VehicleTrace}},
    behaviors::Vector{B},
    history::Int
    )
    
    @assert(length(simlogs) == length(tracesets))
    for (simlog, traces) in zip(simlogs, tracesets)
        init_log!(simlog, behaviors[1:get_ncars(simlog)], traces, history)
    end
    simlogs
end

isdone(simlog::Matrix{Float64}, frameind::Int) = frameind >= size(simlog, 1)

function get_input_acceleration(sym::Symbol, action_lon::Float64, log::Matrix{Float64}, frameind::Int, baseind::Int)

	if sym == :f_accel_250ms || sym == :f_accel_500ms
		return action_lon
	elseif sym == :f_des_speed_250ms || sym == :f_des_speed_500ms
		Δv_des = action_lon 
		Kp = 0.2
		return Δv_des*Kp
	else
		error("unknown longitudinal target $sym")
	end
end
function get_input_turnrate(sym::Symbol, action_lat::Float64, log::Matrix{Float64}, frameind::Int, baseind::Int)

	if sym == :f_turnrate_250ms || sym == :f_turnrate_500ms
		return action_lat
	elseif sym == :f_des_angle_250ms || sym == :f_des_angle_500ms
		phi_des = action_lat
		ϕ = log[frameind, baseind + LOG_COL_ϕ]
		return (phi_des - ϕ)*Features.KP_DESIRED_ANGLE
	else
		error("unknown lateral target $sym")
	end
end

function observe(
	log::Matrix{Float64},
	road::StraightRoadway,
	carind::Int,
	frameind::Int,
	features::Vector{AbstractFeature},
	timestep::Float64,
	extracted_feature_cache :: ExtractedFeatureCache
	)

	observations = Dict{Symbol,Any}()
	for f in features
		val = get(f, log, road, timestep, carind, frameind, extracted_feature_cache)
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
	cpd = BayesNets.cpd(em.BN, symb)
	bin = assignment[sym]
	p = cpd.parameterFunction(assignment)
	log(p[bin])
end
function calc_probability_distribution_over_assignments(em::EM, assignment::Dict{Symbol, Int}, symb::Symbol)
	cpd = BayesNets.cpd(em.BN, symb)
	return cpd.parameterFunction(assignment)
end

function action_prior(observations::Dict{Symbol, Any}, road::StraightRoadway, params::SimParams)

	# car avoids driving off roadway
	# if off-roadway, car heads back to road
	# car tries to drive at the speed limit

	action_dict = Dict{Symbol, Any}()

	yaw   = observations[:yaw]
	velFx = observations[:velFx]
	d_cl = observations[:d_cl]

	# select acceleration
	action_dict[params.accel_sym] = clamp(-(velFx-29.0)/Δt, -1.0, 1.0)
	

	# select turnrate
	# if d_cl < -road.lanewidth*0.6
	# 	# car is off the road on the right side
	# 	# proportional control to get yaw to 0.01
	# 	turnrate = 0.25*(0.01-yaw)/Δt
	# elseif d_cl > road.lanewidth*0.6
	# 	# car is off the road on the left side
	# 	# proportional control to get yaw to -0.01
	# 	turnrate = 0.25*(-0.01-yaw)/Δt
	# else
		# the car is on the road
		# proportional control towards the nearest centerlane
		yaw_des = clamp(-d_cl*0.01, -0.01, 0.01)
		turnrate_towards = 0.25*(yaw_des-yaw)/Δt

		yaw_des = 0.0
		turnrate_along = clamp((yaw_des-yaw)/Δt, -0.01, 0.01)
	# end

	# select delta y
	action_dict[:f_turnrate_towards] = turnrate_towards
	action_dict[:f_turnrate_along] = turnrate_along

	action_dict
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

