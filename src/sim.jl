export SimParams
export simulate!, propagate!, compute_log_metrics, SimParams
export set_sec_per_frame

import Discretizers: encode, decode
import Graphs: topological_sort_by_dfs, in_degree, in_neighbors
using  DataArrays


immutable SimParams
	sec_per_frame :: Float64
	n_euler_steps :: Int
	n_frames      :: Int

	function SimParams(
		sec_per_frame :: Float64 = 0.125,
		n_euler_steps :: Int     = 10,
		n_frames      :: Int     = 100
		)

		@assert(sec_per_frame > 0.0)
		@assert(n_euler_steps > 0)
		@assert(n_frames      > 0)

		new(sec_per_frame, n_euler_steps, n_frames)
	end
end

function simulate!(
	simlog        :: Matrix{Float64}, # initialized appropriately
	behaviors     :: Vector{AbstractVehicleBehavior},
	road          :: StraightRoadway,
	frameind      :: Int, # starting index within simlog
	params        :: SimParams = SimParams()
	)

	numcars = ncars(simlog)
	@assert(nframes(simlog) == params.n_frames)
	@assert(length(behaviors) == numcars)

	while !isdone(simlog, frameind)

		clearmeta()

		for (carind,behavior) in enumerate(behaviors)
			tick!(simlog, carind, behavior, road, frameind, params)
		end

		frameind += 1
	end

	simlog
end

# function simulate!(
# 	simlog        :: Matrix{Float64},
# 	road          :: StraightRoadway,
# 	em            :: EM,
# 	params        :: SimParams = SimParams();
# 	frameind      :: Int = 1
# 	)
	
# 	numcars = ncars(simlog)
# 	@assert(nframes(simlog) == params.n_frames)

# 	sec_per_frame = params.sec_per_frame
# 	n_euler_steps = params.n_euler_steps
# 	δt            = sec_per_frame / n_euler_steps

# 	symbol_lat = params.symbol_lat
# 	symbol_lon = params.symbol_lon
# 	sample_lat = params.sampling_lat
# 	sample_lon = params.sampling_lon
# 	samplemethod_lat = sample_lat.sampling_scheme
# 	samplemethod_lon = sample_lon.sampling_scheme
# 	smoothing_lat = sample_lat.smoothing
# 	smoothing_lon = sample_lon.smoothing
# 	smoothcounts_lat = sample_lat.smoothing_counts
# 	smoothcounts_lon = sample_lon.smoothing_counts

# 	bmap_lat = em.binmaps[findfirst(em.BN.names, symbol_lat)]
# 	bmap_lon = em.binmaps[findfirst(em.BN.names, symbol_lon)]

# 	indicators = get_indicators(em)

# 	while !isdone(simlog, frameind)
# 		clearmeta()

# 		for carind in 1 : numcars
# 			base = logindexbase(carind)

# 			a, ω = 0.0, 0.0

# 			if carind == 1

# 				# TODO(tim): pre-allocate observations?
# 				observations = observe(simlog, road, carind, frameind, indicators, sec_per_frame)
# 				assignment   = encode(observations, em)
# 				assignment, logPs  = sample_and_logP!(em, assignment)

# 				action_lat = decode(samplemethod_lat, bmap_lat, assignment[symbol_lat], observations, sec_per_frame)
# 				action_lon = decode(samplemethod_lon, bmap_lon, assignment[symbol_lon], observations, sec_per_frame)

# 				a = get_input_acceleration(symbol_lon, action_lon, simlog, frameind, base)
# 				ω = get_input_turnrate(    symbol_lat, action_lat, simlog, frameind, base)

# 				logPa = logPs[symbol_lon]
# 				logPω = logPs[symbol_lat]

# 				if smoothing_lat == :SMA
# 					n_frames = frameind > smoothcounts_lat ? smoothcounts_lat : frameind
# 					lo = frameind-(n_frames-1)
# 					ω = (sum(log[lo:frameind-1, base + LOG_COL_T]) + ω) / n_frames
# 				elseif smoothing_lat == :WMA
# 					n_frames = frameind > smoothcounts_lat ? smoothcounts_lat : frameind
# 					lo = frameind-(n_frames-1)
# 					ω = (sum(log[lo:frameind-1, base + LOG_COL_T] .* [1:n_frames-1]) + n_frames*ω) / (0.5n_frames*(n_frames+1))
# 				end

# 				if smoothing_lon == :SMA
# 					n_frames = frameind > smoothcounts_lon ? smoothcounts_lon : frameind
# 					lo = frameind-(n_frames-1)
# 					a = (sum(log[lo:frameind-1, base + LOG_COL_A])+a) / n_frames
# 				elseif smoothing_lon == :WMA
# 					n_frames = frameind > smoothcounts_lon ? smoothcounts_lon : frameind
# 					lo = frameind-(n_frames-1)
# 					a = (sum(log[lo:frameind-1, base + LOG_COL_A] .* [1:n_frames-1]) + n_frames*a) / (0.5n_frames*(n_frames+1))
# 				end
# 			end

# 			propagate!(simlog, frameind, base, a, ω, logPa, logPω, EM_ID_UNKNOWN, n_euler_steps, δt)
# 		end

# 		frameind += 1
# 	end

# 	simlog
# end
# function simulate!(
# 	log              :: Matrix{Float64},
# 	road             :: StraightRoadway,
# 	scenarioselector :: ScenarioSelector,
# 	params           :: SimParams = SimParams();
# 	frameind         :: Int = 1
# 	)
	
# 	numcars = ncars(log)
# 	@assert(nframes(log) == params.n_frames)

# 	sec_per_frame = params.sec_per_frame
# 	n_euler_steps = params.n_euler_steps
# 	δt            = sec_per_frame / n_euler_steps

# 	symbol_lat = params.symbol_lat
# 	symbol_lon = params.symbol_lon
# 	sample_lat = params.sampling_lat
# 	sample_lon = params.sampling_lon
# 	samplemethod_lat = sample_lat.sampling_scheme
# 	samplemethod_lon = sample_lon.sampling_scheme
# 	smoothing_lat = sample_lat.smoothing
# 	smoothing_lon = sample_lon.smoothing
# 	smoothcounts_lat = sample_lat.smoothing_counts
# 	smoothcounts_lon = sample_lon.smoothing_counts

# 	while !isdone(log, frameind)
		
# 		clearmeta()

# 		for carind in 1 : numcars
# 			base = logindexbase(carind)

# 			# if carind == 1

# 				em_id = select_encounter_model(scenarioselector, log, road, carind, frameind, sec_per_frame)
# 				em = get(scenarioselector, em_id)

# 				indicators = get_indicators(em)
# 				bmap_lat = em.binmaps[findfirst(em.BN.names, symbol_lat)]
# 				bmap_lon = em.binmaps[findfirst(em.BN.names, symbol_lon)]

# 				observations = observe(log, road, carind, frameind, indicators, sec_per_frame)
# 				assignment   = encode(observations, em)
# 				assignment, logPs  = sample_and_logP!(em, assignment)

# 				action_lat = decode(samplemethod_lat, bmap_lat, assignment[symbol_lat], observations, sec_per_frame)
# 				action_lon = decode(samplemethod_lon, bmap_lon, assignment[symbol_lon], observations, sec_per_frame)

# 				a = get_input_acceleration(symbol_lon, action_lon, log, frameind, base)
# 				ω = get_input_turnrate(    symbol_lat, action_lat, log, frameind, base)

# 				logPa = logPs[symbol_lon]
# 				logPω = logPs[symbol_lat]

# 				if smoothing_lat == :SMA
# 					n_frames = frameind > smoothcounts_lat ? smoothcounts_lat : frameind
# 					lo = frameind-(n_frames-1)
# 					ω = (sum(log[lo:frameind-1, base + LOG_COL_T]) + ω) / n_frames
# 				elseif smoothing_lat == :WMA
# 					n_frames = frameind > smoothcounts_lat ? smoothcounts_lat : frameind
# 					lo = frameind-(n_frames-1)
# 					ω = (sum(log[lo:frameind-1, base + LOG_COL_T] .* [1:n_frames-1]) + n_frames*ω) / (0.5n_frames*(n_frames+1))
# 				end

# 				if smoothing_lon == :SMA
# 					n_frames = frameind > smoothcounts_lon ? smoothcounts_lon : frameind
# 					lo = frameind-(n_frames-1)
# 					a = (sum(log[lo:frameind-1, base + LOG_COL_A])+a) / n_frames
# 				elseif smoothing_lon == :WMA
# 					n_frames = frameind > smoothcounts_lon ? smoothcounts_lon : frameind
# 					lo = frameind-(n_frames-1)
# 					a = (sum(log[lo:frameind-1, base + LOG_COL_A] .* [1:n_frames-1]) + n_frames*a) / (0.5n_frames*(n_frames+1))
# 				end

# 				propagate!(log, frameind, base, a, ω, logPa, logPω, em_id, n_euler_steps, δt)
# 			# else
# 			# 	log[frameind, base + LOG_COL_A] = 0.0
# 			# 	log[frameind, base + LOG_COL_T] = 0.0
# 			# 	propagate!(log, frameind, base, 0.0, 0.0, n_euler_steps, δt, params)				
# 			# end
# 		end

# 		frameind += 1
# 	end

# 	log
# end
# function simulate!(
# 	scene         :: Scene,
# 	road          :: StraightRoadway,
# 	em            :: Any,
# 	params        :: SimParams = SimParams();
# 	frameind      :: Int = 1
# 	)
	
# 	runlog = create_log(length(scene), params.n_frames)
#     init_log!(runlog, scene, params)
#     simulate!(runlog, road, em, params, frameind=2)
	
# 	runlog
# end

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
	
	sample_lat = behavior.sampling_lat
	sample_lon = behavior.sampling_lon
	samplemethod_lat = sample_lat.sampling_scheme
	samplemethod_lon = sample_lon.sampling_scheme
	smoothing_lat = sample_lat.smoothing
	smoothing_lon = sample_lon.smoothing
	smoothcounts_lat = sample_lat.smoothing_counts
	smoothcounts_lon = sample_lon.smoothing_counts

	bmap_lat = em.binmaps[findfirst(em.BN.names, symbol_lat)]
	bmap_lon = em.binmaps[findfirst(em.BN.names, symbol_lon)]

	base = logindexbase(carind)

	a, ω = 0.0, 0.0

	observations = observe(simlog, road, carind, frameind, behavior.indicators, sec_per_frame)
	assignment   = encode(observations, em)
	assignment, logPs  = sample_and_logP!(em, assignment)

	action_lat = decode(bmap_lat, assignment[symbol_lat], samplemethod_lat)
	action_lon = decode(bmap_lon, assignment[symbol_lon], samplemethod_lon)

	a = get_input_acceleration(symbol_lon, action_lon, simlog, frameind, base)
	ω = get_input_turnrate(    symbol_lat, action_lat, simlog, frameind, base)

	logPa = logPs[symbol_lon]
	logPω = logPs[symbol_lat]

	if smoothing_lat == :SMA
		n_frames = frameind > smoothcounts_lat ? smoothcounts_lat : frameind
		lo = frameind-(n_frames-1)
		ω = (sum(simlog[lo:frameind-1, base + LOG_COL_T]) + ω) / n_frames
	elseif smoothing_lat == :WMA
		n_frames = frameind > smoothcounts_lat ? smoothcounts_lat : frameind
		lo = frameind-(n_frames-1)
		ω = (sum(simlog[lo:frameind-1, base + LOG_COL_T] .* [1:n_frames-1]) + n_frames*ω) / (0.5n_frames*(n_frames+1))
	end

	if smoothing_lon == :SMA
		n_frames = frameind > smoothcounts_lon ? smoothcounts_lon : frameind
		lo = frameind-(n_frames-1)
		a = (sum(simlog[lo:frameind-1, base + LOG_COL_A])+a) / n_frames
	elseif smoothing_lon == :WMA
		n_frames = frameind > smoothcounts_lon ? smoothcounts_lon : frameind
		lo = frameind-(n_frames-1)
		a = (sum(simlog[lo:frameind-1, base + LOG_COL_A] .* [1:n_frames-1]) + n_frames*a) / (0.5n_frames*(n_frames+1))
	end

	propagate!(simlog, frameind, base, a, ω, logPa, logPω, EM_ID_UNKNOWN, n_euler_steps, δt)
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
		Kp = 0.2
		return (phi_des - ϕ)*Kp
	else
		error("unknown lateral target $sym")
	end
end

function propagate!(
	simlog        :: Matrix{Float64},
	frameind      :: Int,
	baseind       :: Int,
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
	

	simlog[frameind, baseind + LOG_COL_A] = a
	simlog[frameind, baseind + LOG_COL_T] = ω
	simlog[frameind, baseind + LOG_COL_logprobweight_A] = logPa
	simlog[frameind, baseind + LOG_COL_logprobweight_T] = logPω
	simlog[frameind, baseind + LOG_COL_em] = float64(em_id)

	x = simlog[frameind, baseind + LOG_COL_X]
	y = simlog[frameind, baseind + LOG_COL_Y]
	ϕ = simlog[frameind, baseind + LOG_COL_ϕ]
	v = simlog[frameind, baseind + LOG_COL_V]

	for j = 1 : n_euler_steps
		v += a*δt
		ϕ += ω*δt
		x += v*cos(ϕ)*δt
		y += v*sin(ϕ)*δt
	end

	simlog[frameind+1, baseind + LOG_COL_X] = x
	simlog[frameind+1, baseind + LOG_COL_Y] = y
	simlog[frameind+1, baseind + LOG_COL_ϕ] = ϕ
	simlog[frameind+1, baseind + LOG_COL_V] = v

	simlog
end
function propagate!(
	simlog        :: Matrix{Float64},
	frameind      :: Int,
	carind        :: Int,
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

	propagate!(simlog, frameind, logindexbase(carind), a, ω, logPa, logPω, em_id, n_euler_steps, δt)
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

	for carind in ncars(simlog)
		propagate!(simlog, frameind, logindexbase(carind), a, ω, logPa, logPω, em_id, n_euler_steps, δt)
	end

	simlog
end
# function propagate!(
# 	simlog        :: Matrix{Float64},
# 	frameind      :: Int,
# 	em            :: EM,
# 	road          :: StraightRoadway,
# 	params        :: SimParams = SimParams();
# 	em_id         :: Int = EM_ID_UNKNOWN
# 	)

# 	sec_per_frame = params.sec_per_frame
# 	n_euler_steps = params.n_euler_steps
# 	δt            = sec_per_frame / n_euler_steps

# 	symbol_lat = params.symbol_lat
# 	symbol_lon = params.symbol_lon
# 	sample_lat = params.sampling_lat
# 	sample_lon = params.sampling_lon
# 	samplemethod_lat = sample_lat.sampling_scheme
# 	samplemethod_lon = sample_lon.sampling_scheme
# 	smoothing_lat = sample_lat.smoothing
# 	smoothing_lon = sample_lon.smoothing
# 	smoothcounts_lat = sample_lat.smoothing_counts
# 	smoothcounts_lon = sample_lon.smoothing_counts

# 	bmap_lat = em.binmaps[findfirst(em.BN.names, symbol_lat)]
# 	bmap_lon = em.binmaps[findfirst(em.BN.names, symbol_lon)]

# 	indicators = get_indicators(em)

# 	clearmeta()

# 	for carind in 1 : ncars(log)
# 		base = logindexbase(carind)

# 		# TODO(tim): pre-allocate observations?
# 		observations = observe(log, road, carind, frameind, indicators, sec_per_frame)
# 		assignment   = encode(observations, em)
# 		assignment, logPs  = sample_and_logP!(em, assignment)

# 		action_lat = decode(samplemethod_lat, bmap_lat, assignment[symbol_lat], observations, sec_per_frame)
# 		action_lon = decode(samplemethod_lon, bmap_lon, assignment[symbol_lon], observations, sec_per_frame)

# 		a = get_input_acceleration(symbol_lon, action_lon, log, frameind, base)
# 		ω = get_input_turnrate(    symbol_lat, action_lat, log, frameind, base)

# 		logPa = logPs[symbol_lon]
# 		logPω = logPs[symbol_lat]

# 		if smoothing_lat == :SMA
# 			n_frames = frameind > smoothcounts_lat ? smoothcounts_lat : frameind
# 			lo = frameind-(n_frames-1)
# 			ω = (sum(log[lo:frameind-1, base + LOG_COL_T]) + ω) / n_frames
# 		elseif smoothing_lat == :WMA
# 			n_frames = frameind > smoothcounts_lat ? smoothcounts_lat : frameind
# 			lo = frameind-(n_frames-1)
# 			ω = (sum(log[lo:frameind-1, base + LOG_COL_T] .* [1:n_frames-1]) + n_frames*ω) / (0.5n_frames*(n_frames+1))
# 		end

# 		if smoothing_lon == :SMA
# 			n_frames = frameind > smoothcounts_lon ? smoothcounts_lon : frameind
# 			lo = frameind-(n_frames-1)
# 			a = (sum(log[lo:frameind-1, base + LOG_COL_A])+a) / n_frames
# 		elseif smoothing_lon == :WMA
# 			n_frames = frameind > smoothcounts_lon ? smoothcounts_lon : frameind
# 			lo = frameind-(n_frames-1)
# 			a = (sum(log[lo:frameind-1, base + LOG_COL_A] .* [1:n_frames-1]) + n_frames*a) / (0.5n_frames*(n_frames+1))
# 		end

# 		propagate!(log, frameind, base, a, ω, logPa, logPω, EM_ID_UNKNOWN, n_euler_steps, δt)
# 	end
# end
# function propagate!(
# 	log           :: Matrix{Float64},
# 	frameind      :: Int,
# 	carind        :: Int,
# 	em            :: EM,
# 	road          :: StraightRoadway,
# 	params        :: SimParams = SimParams();
# 	em_id         :: Int = EM_ID_UNKNOWN
# 	)

# 	sec_per_frame = params.sec_per_frame
# 	n_euler_steps = params.n_euler_steps
# 	δt            = sec_per_frame / n_euler_steps

# 	symbol_lat = params.symbol_lat
# 	symbol_lon = params.symbol_lon
# 	sample_lat = params.sampling_lat
# 	sample_lon = params.sampling_lon
# 	samplemethod_lat = sample_lat.sampling_scheme
# 	samplemethod_lon = sample_lon.sampling_scheme
# 	smoothing_lat = sample_lat.smoothing
# 	smoothing_lon = sample_lon.smoothing
# 	smoothcounts_lat = sample_lat.smoothing_counts
# 	smoothcounts_lon = sample_lon.smoothing_counts

# 	bmap_lat = em.binmaps[findfirst(em.BN.names, symbol_lat)]
# 	bmap_lon = em.binmaps[findfirst(em.BN.names, symbol_lon)]

# 	indicators = get_indicators(em)

# 	clearmeta()

# 	base = logindexbase(carind)

# 	# TODO(tim): pre-allocate observations?
# 	observations = observe(log, road, carind, frameind, indicators, sec_per_frame)
# 	assignment   = encode(observations, em)
# 	assignment, logPs  = sample_and_logP!(em, assignment)

# 	action_lat = decode(samplemethod_lat, bmap_lat, assignment[symbol_lat], observations, sec_per_frame)
# 	action_lon = decode(samplemethod_lon, bmap_lon, assignment[symbol_lon], observations, sec_per_frame)

# 	a = get_input_acceleration(symbol_lon, action_lon, log, frameind, base)
# 	ω = get_input_turnrate(    symbol_lat, action_lat, log, frameind, base)

# 	logPa = logPs[symbol_lon]
# 	logPω = logPs[symbol_lat]

# 	if smoothing_lat == :SMA
# 		n_frames = frameind > smoothcounts_lat ? smoothcounts_lat : frameind
# 		lo = frameind-(n_frames-1)
# 		ω = (sum(log[lo:frameind-1, base + LOG_COL_T]) + ω) / n_frames
# 	elseif smoothing_lat == :WMA
# 		n_frames = frameind > smoothcounts_lat ? smoothcounts_lat : frameind
# 		lo = frameind-(n_frames-1)
# 		ω = (sum(log[lo:frameind-1, base + LOG_COL_T] .* [1:n_frames-1]) + n_frames*ω) / (0.5n_frames*(n_frames+1))
# 	end

# 	if smoothing_lon == :SMA
# 		n_frames = frameind > smoothcounts_lon ? smoothcounts_lon : frameind
# 		lo = frameind-(n_frames-1)
# 		a = (sum(log[lo:frameind-1, base + LOG_COL_A])+a) / n_frames
# 	elseif smoothing_lon == :WMA
# 		n_frames = frameind > smoothcounts_lon ? smoothcounts_lon : frameind
# 		lo = frameind-(n_frames-1)
# 		a = (sum(log[lo:frameind-1, base + LOG_COL_A] .* [1:n_frames-1]) + n_frames*a) / (0.5n_frames*(n_frames+1))
# 	end

# 	propagate!(log, frameind, base, a, ω, logPa, logPω, EM_ID_UNKNOWN, n_euler_steps, δt)
# end


function observe(log::Matrix{Float64}, road::StraightRoadway, carind::Int, frameind::Int, features::Vector{AbstractFeature}, timestep::Float64)
	observations = Dict{Symbol,Any}()
	for f in features
		val = get(f, log, road, timestep, carind, frameind)
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

# function compute_log_metrics(log::Matrix{Float64}, road::StraightRoadway, params::SimParams)

# 	Δt = params.sec_per_frame

# 	n = size(log,1)

# 	has_collision_ego = false # whether ego car collides with another car
# 	n_lanechanges_ego = 0 # whether ego car makes a lange change
# 	elapsed_time = size(log, 1) * Δt
# 	has_other_cars = size(log,2) > LOG_NCOLS_PER_CAR

# 	mean_speed_ego = mean(log[:,LOG_COL_V]) # mean ego speed
# 	mean_centerline_offset_ego = 0.0
# 	std_speed_ego  = std(log[:,LOG_COL_V])  # stdev of ego speed
# 	time_of_first_offroad = Inf
# 	n_frames_offroad_ego = 0

# 	arr_v_x = (log[2:end,LOG_COL_X] - log[1:end-1,LOG_COL_X]) ./ Δt
# 	arr_v_y = (log[2:end,LOG_COL_Y] - log[1:end-1,LOG_COL_Y]) ./ Δt
# 	arr_a_x = (arr_v_x[2:end] - arr_v_x[1:end-1]) ./ Δt
# 	arr_a_y = (arr_v_y[2:end] - arr_v_y[1:end-1]) ./ Δt
# 	abs_arr_j_x = abs((arr_a_x[2:end] - arr_a_x[1:end-1]) ./ Δt)
# 	abs_arr_j_y = abs((arr_a_y[2:end] - arr_a_y[1:end-1]) ./ Δt)
# 	abs_jerk_mean_x = mean(abs_arr_j_x)
# 	abs_jerk_std_x = stdm(abs_arr_j_x, abs_jerk_mean_x)
# 	abs_jerk_mean_y = mean(abs_arr_j_y)
# 	abs_jerk_std_y = stdm(abs_arr_j_y, abs_jerk_mean_y)

# 	numcars = ncars(log)
# 	if numcars > 1
# 		mean_headway = mean(log[:,LOG_NCOLS_PER_CAR+LOG_COL_X] - log[:,LOG_COL_X])
# 		mean_timegap = mean((log[:,LOG_NCOLS_PER_CAR+LOG_COL_X] - log[:,LOG_COL_X]) ./ log[:,LOG_COL_V])
# 	else
# 		mean_headway = 0.0
# 		mean_timegap = 0.0
# 	end

# 	lane_centers = lanecenters(road) # [0,lw,2lw,...]

# 	for i = 1 : n
# 		# check for lange change (by ego car)
# 		lane_dists = abs(lane_centers .- log[i,  LOG_COL_Y])
# 		cl_cur = indmin(lane_dists)
# 		mean_centerline_offset_ego += lane_dists[cl_cur]

# 		if i > 1
# 			cl_old = indmin(abs(lane_centers .- log[i-1,LOG_COL_Y]))
# 			if cl_old != cl_cur
# 				n_lanechanges_ego += 1
# 			end
# 		end

# 		if !onroad(log[i, LOG_COL_Y], road)
# 			n_frames_offroad_ego += 1
# 			time_of_first_offroad = min(i*Δt, time_of_first_offroad)
# 		end

# 		# check for collision
# 		if has_other_cars
# 			# TODO(tim): make work for more than 1 other car
# 			dx = log[i,LOG_COL_X] - log[i,LOG_COL_X+LOG_NCOLS_PER_CAR]
# 			dy = log[i,LOG_COL_Y] - log[i,LOG_COL_Y+LOG_NCOLS_PER_CAR]
# 			if abs(dx) < CAR_LENGTH && abs(dy) < CAR_WIDTH
# 				has_collision_ego = true
# 				# NOTE(tim): anything after a collision is invalid - break here
# 				break
# 			end
# 		end
# 	end

# 	mean_centerline_offset_ego /= n

# 	[
# 	 :has_collision_ego=>has_collision_ego,
# 	 :n_lanechanges_ego=>n_lanechanges_ego,
# 	 :mean_speed_ego=>mean_speed_ego,
# 	 :mean_centerline_offset_ego=>mean_centerline_offset_ego,
# 	 :std_speed_ego=>std_speed_ego,
# 	 :n_sec_offroad_ego=>n_frames_offroad_ego * Δt,
# 	 :elapsed_time=>elapsed_time,
# 	 :time_of_first_offroad=>time_of_first_offroad,
# 	 :went_offroad=>time_of_first_offroad!=Inf,
# 	 :jerk_mean_x=>abs_jerk_mean_x,
# 	 :jerk_std_x=>abs_jerk_std_x,
# 	 :jerk_mean_y=>abs_jerk_mean_y,
# 	 :jerk_std_y=>abs_jerk_std_y,
# 	 :final_x=>log[end,LOG_COL_X],
# 	 :final_y=>log[end,LOG_COL_Y],
# 	 :initial_speed=>log[2,LOG_COL_V],
# 	 :mean_headway=>mean_headway,
# 	 :mean_timegap=>mean_timegap,
# 	 :logPA=>sum(log[:,LOG_COL_logprobweight_A]),
# 	 :logPT=>sum(log[:,LOG_COL_logprobweight_T]),
# 	 :percent_freeflow=>sum(log[:,LOG_COL_em] == EM_ID_FREEFLOW) / n,
# 	 :percent_carfollow=>sum(log[:,LOG_COL_em] == EM_ID_CARFOLLOW) / n,
# 	 :percent_lanechange=>sum(log[:,LOG_COL_em] == EM_ID_LANECHANGE) / n
# 	]::Dict{Symbol, Any}
# end

# export save_metrics, calc_metric, aggregate_metrics, 
# 	   print_results_human_readable,
# 	   print_results_csv_readable

# function save_metrics(metrics::Dict{Symbol, Any}, outfile::String)
# 	open(outfile, "w") do fout
# 		for (sym,val) in metrics
# 			println(fout, string(sym), ": ", val)
# 		end
# 	end
# end
# function calc_metric(sym::Symbol, ::Type{Int}, metrics)
# 	counts = Dict{Int,Int}()
# 	for i = 1 : length(metrics)
# 		counts[metrics[i][sym]] = get(counts, metrics[i][sym], 0) + 1
# 	end

# 	t_arr = [metrics[i][:elapsed_time] for i in 1 : length(metrics)]
# 	tot_time = sum(t_arr)

# 	arr = [metrics[i][sym] for i in 1 : length(metrics)]
# 	ave = mean(arr)
# 	stdev = stdm(arr, ave)
# 	weighted_ave = sum([metrics[i][sym] * metrics[i][:elapsed_time] for i in 1 : length(metrics)]) / tot_time

# 	(ave, stdev, weighted_ave)
# end
# function calc_metric(sym::Symbol, ::Type{Bool}, metrics)
# 	n_true = sum([metrics[i][sym] for i in 1 : length(metrics)])
# 	ave_time_to_true = sum([metrics[i][sym] ? metrics[i][:elapsed_time] : 0.0 for i in 1 : length(metrics)]) / n_true

# 	t_arr = [metrics[i][:elapsed_time] for i in 1 : length(metrics)]
# 	tot_time = sum(t_arr)

# 	odds_true_per_run = n_true / length(metrics)
# 	odds_true_per_sec = n_true / tot_time

# 	(odds_true_per_run, odds_true_per_sec, ave_time_to_true)
# end
# function calc_metric(sym::Symbol, ::Type{Float64}, metrics, use_abs=false)
# 	if use_abs
# 		arr = convert(Vector{Float64}, [abs(metrics[i][sym]) for i in 1 : length(metrics)])
# 	else
# 		arr = convert(Vector{Float64}, [metrics[i][sym] for i in 1 : length(metrics)])
# 	end
# 	inds = find(a->a!=Inf, arr)
# 	arr = arr[inds]
# 	ave = mean(arr)
# 	stdev = stdm(arr, ave)
# 	(ave, stdev)
# end

# function aggregate_metrics(
# 	initial_scene :: Scene,
# 	road          :: StraightRoadway,
# 	em            :: EM,
# 	params        :: SimParams,
# 	n_simulations :: Int,
# 	runlog        :: Matrix{Float64} = create_log(length(initial_scene), params.n_frames)
# 	)

# 	@assert(n_simulations > 0)

# 	metrics = Array(Dict{Symbol, Any}, n_simulations)

# 	# TODO(tim): parallelize
# 	for isim = 1 : n_simulations
# 		simulate!(initial_scene, road, em, params, runlog)
# 		metrics[isim] = compute_log_metrics(runlog, road, params)
# 	end
# 	metrics
# end
# function aggregate_metrics(
# 	initial_scene :: Function,
# 	road          :: StraightRoadway,
# 	em            :: EM,
# 	params        :: SimParams,
# 	n_simulations :: Int,
# 	runlog        :: Matrix{Float64} = create_log(length(initial_scene()), params.n_frames)
# 	)

# 	@assert(n_simulations > 0)

# 	metrics = Array(Dict{Symbol, Any}, n_simulations)

# 	# TODO(tim): parallelize
# 	for isim = 1 : n_simulations
# 		simulate!(initial_scene(), road, em, params, runlog)
# 		metrics[isim] = compute_log_metrics(runlog, road, params)
# 	end
# 	metrics
# end
# function aggregate_metrics(
# 	scenes        :: Vector{Scene},
# 	road          :: StraightRoadway,
# 	em            :: EM,
# 	params        :: SimParams,
# 	runlog        :: Matrix{Float64} = create_log(length(scenes[1]), params.n_frames)
# 	)

# 	n_simulations = length(scenes)

# 	@assert(n_simulations > 0)

# 	metrics = Array(Dict{Symbol, Any}, n_simulations)

# 	# TODO(tim): parallelize
# 	for isim = 1 : n_simulations
# 		simulate!(scenes[isim], road, em, params, runlog)
# 		metrics[isim] = compute_log_metrics(runlog, road, params)
# 	end
# 	metrics
# end
# function aggregate_metrics(
# 	scenes        :: Vector{Scene},
# 	road          :: StraightRoadway,
# 	ss            :: ScenarioSelector,
# 	params        :: SimParams,
# 	runlog        :: Matrix{Float64} = create_log(length(scenes[1]), params.n_frames)
# 	)

# 	n_simulations = length(scenes)

# 	@assert(n_simulations > 0)

# 	metrics = Array(Dict{Symbol, Any}, n_simulations)

# 	# TODO(tim): parallelize
# 	for isim = 1 : n_simulations
# 		drivelog = simulate!(scenes[isim], road, ss, params) # runlog
# 		metrics[isim] = compute_log_metrics(drivelog, road, params)
# 	end
# 	metrics
# end
# function print_results_human_readable(params::SimParams, aggmetrics::Vector{Dict{Symbol, Any}})
# 	println("GLOBAL")
# 	println("\tsec_per_frame: ", params.sec_per_frame)
# 	println("\tn_euler_steps: ", params.n_euler_steps)
# 	println("\tn_frames:      ", params.n_frames)
# 	println("\ttot_time:      ", params.n_frames * params.sec_per_frame)
# 	println("\tsymbol_lat:    ", params.symbol_lat)
# 	println("\tsymbol_lon:    ", params.symbol_lon)
# 	println("")
# 	println("ACCELERATION")
# 	println("\tbinning:      ", typeof(params.sampling_lon.sampling_scheme))
# 	print("\tsmoothing:    ", params.sampling_lon.smoothing)
# 	if params.sampling_lon.smoothing != :none
# 	 	print(" {", params.sampling_lon.smoothing_counts, "}")
# 	end
# 	println("\n")
# 	println("TURNRATE")
# 	println("\tbinning:      ", typeof(params.sampling_lat.sampling_scheme))
# 	print("\tsmoothing:    ", params.sampling_lat.smoothing)
# 	if params.sampling_lat.smoothing != :none
# 	 	print(" {", params.sampling_lat.smoothing_counts, "}")
# 	end
# 	println("\n")
# 	println("RESULTS")
# 	println("\tmean centerline offset: ", calc_metric(:mean_centerline_offset_ego, Float64, aggmetrics, true))
# 	println("\tego speed:              ", calc_metric(:mean_speed_ego, Float64, aggmetrics))
# 	println("\toffroad rate:           ", calc_metric(:went_offroad, Bool, aggmetrics))
# 	println("\ttime to offroad:        ", calc_metric(:time_of_first_offroad, Float64, aggmetrics))
# 	println("\tlane change rate:       ", calc_metric(:n_lanechanges_ego, Float64, aggmetrics))
# 	println("\tjerk mean x:            ", calc_metric(:jerk_mean_x, Float64, aggmetrics))
# 	println("\tjerk std x:             ", calc_metric(:jerk_std_x, Float64, aggmetrics))
# 	println("\tjerk mean y:            ", calc_metric(:jerk_mean_y, Float64, aggmetrics))
# 	println("\tjerk std y:             ", calc_metric(:jerk_std_y, Float64, aggmetrics))
# 	println("\tmean headway:           ", calc_metric(:mean_headway, Float64, aggmetrics))
# 	println("\tmean timegap:           ", calc_metric(:mean_timegap, Float64, aggmetrics))
# end
# function print_results_csv_readable(io::IO, params::SimParams, aggmetrics::Vector{Dict{Symbol, Any}})
# 	mean_centerline_offset_ego = calc_metric(:mean_centerline_offset_ego, Float64, aggmetrics, true)
# 	mean_speed_ego = calc_metric(:mean_speed_ego, Float64, aggmetrics)
# 	went_offroad = calc_metric(:went_offroad, Bool, aggmetrics)
# 	time_of_first_offroad = calc_metric(:time_of_first_offroad, Float64, aggmetrics)
# 	n_lanechanges_ego = calc_metric(:n_lanechanges_ego, Float64, aggmetrics)
# 	jerk_mean_x = calc_metric(:jerk_mean_x, Float64, aggmetrics)
# 	jerk_std_x = calc_metric(:jerk_std_x, Float64, aggmetrics)
# 	jerk_mean_y = calc_metric(:jerk_mean_y, Float64, aggmetrics)
# 	jerk_std_y = calc_metric(:jerk_std_y, Float64, aggmetrics)
# 	mean_headway = calc_metric(:mean_headway, Float64, aggmetrics)
# 	mean_timegap = calc_metric(:mean_timegap, Float64, aggmetrics)

# 	str_smoothing_lat = params.sampling_lon.smoothing == :none ? "none" :
# 						@sprintf("%s {%d}", string(params.sampling_lon.smoothing), params.sampling_lon.smoothing_counts)
# 	str_smoothing_lon = params.sampling_lat.smoothing == :none ? "none" :
# 						@sprintf("%s {%d}", string(params.sampling_lat.smoothing), params.sampling_lat.smoothing_counts)

# 	@printf(io, "%.0f, %.3f, ", params.n_frames, params.sec_per_frame)
# 	@printf(io, "%s, %s, ", string(typeof(params.sampling_lat.sampling_scheme)), str_smoothing_lat)
# 	@printf(io, "%s, %s, ", string(typeof(params.sampling_lon.sampling_scheme)), str_smoothing_lon)
# 	@printf(io, "%.3f pm %.3f, %.2f pm %.3f, %.3f, %.2f pm %.2f, %.4f pm %.4f, %.3f pm %.2f, %.2f pm %.2f, %.3f pm %.2f, %.2f pm %.2f, %.2f pm %.2f, %.3f pm %.3f",
# 		mean_centerline_offset_ego[1], mean_centerline_offset_ego[2],
# 		mean_speed_ego[1], mean_speed_ego[2],
# 		went_offroad[1],
# 		time_of_first_offroad[1], time_of_first_offroad[2],
# 		n_lanechanges_ego[1], n_lanechanges_ego[2],
# 		jerk_mean_x[1], jerk_mean_x[2],
# 		jerk_std_x[1], jerk_std_x[2],
# 		jerk_mean_y[1], jerk_mean_y[2],
# 		jerk_std_y[1], jerk_std_y[2],
# 		mean_headway[1], mean_headway[2],
# 		mean_timegap[1], mean_timegap[2],
# 		)
# end
# print_results_csv_readable(params::SimParams, aggmetrics::Vector{Dict{Symbol, Any}}) = print_results_csv_readable(STDOUT, params, aggmetrics)
