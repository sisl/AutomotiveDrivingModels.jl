export SimParams, SamplingParams
export simulate!, compute_log_metrics, SimParams
export set_sec_per_frame
export SampleMethod, SampleUniform, SampleBinCenter, SampleUniformZeroBin, 
       SampleTurnrateTowardsLaneCenter, SampleTurnrateAlongLane

import BinMaps: encode, decode
import Graphs: topological_sort_by_dfs, in_degree, in_neighbors
using  DataArrays

abstract SampleMethod
immutable SampleUniform <: SampleMethod end # always draw uniform random from bin
immutable SampleBinCenter <: SampleMethod end # always draw from bin center
immutable SampleUniformZeroBin <: SampleMethod end # draw 0.0 if bin contains zero, otherwise uniform random
type SampleTurnrateTowardsLaneCenter <: SampleMethod

	# if bin contains ω_des use it
	# otherwise sample uniform random

	# C = centerline_scalar
	# κ = proportional_scalar
	# ψ_max = max_abs_desired_yaw
	# ψ_des = clamp(-d_cl*C, ±ψ_max)
	# ω_des = κ*(ψ_des - ψ)/Δt

	max_abs_desired_yaw :: Float64 # [rad]
	centerline_scalar   :: Float64 # [rad / m]
	proportional_scalar :: Float64 # [1/s]

	function SampleTurnrateTowardsLaneCenter()
		self = new()
		self.max_abs_desired_yaw = 0.01
		self.centerline_scalar = 0.01
		self.proportional_scalar = 0.25
		self
	end
end
type SampleTurnrateAlongLane <: SampleMethod

	# if bin contains ω_des use it
	# otherwise sample uniform random

	# κ = proportional_scalar
	# ψ_des = 0
	# ω_max = max_abs_omega
	# ω_des = κ*(ψ_des - ψ)/Δt

	max_abs_omega       :: Float64 # [rad]
	proportional_scalar :: Float64 # [1/s]

	function SampleTurnrateAlongLane()
		self = new()
		self.max_abs_omega = 0.01
		self.proportional_scalar = 1.0
		self
	end
end

type SamplingParams
	sampling_scheme  :: SampleMethod
	smoothing        :: Symbol # :none, :SMA, :WMA
	smoothing_counts :: Int    # number of previous counts to use

	function SamplingParams(
		sampling_scheme :: SampleMethod = SampleUniform(),
		smoothing :: Symbol = :none,
		smoothing_counts :: Int = 0
		)
		@assert(smoothing_counts ≥ 0)
		new(sampling_scheme, smoothing, smoothing_counts)
	end
end
immutable SimParams
	symbol_lat    :: Symbol
	symbol_lon    :: Symbol
	sampling_lat  :: SamplingParams
	sampling_lon  :: SamplingParams
	sec_per_frame :: Float64
	n_euler_steps :: Int
	n_frames      :: Int

	function SimParams(
		symbol_lat    :: Symbol,
		symbol_lon    :: Symbol,
		sampling_lat  :: SamplingParams = SamplingParams(),
		sampling_lon  :: SamplingParams = SamplingParams(),
		sec_per_frame :: Float64 = 0.125,
		n_euler_steps :: Int     = 10,
		n_frames      :: Int     = 100
		)

		@assert(sec_per_frame > 0.0)
		@assert(n_euler_steps > 0)
		@assert(n_frames      > 0)

		new(symbol_lat, symbol_lon, sampling_lat, sampling_lon,
			sec_per_frame, n_euler_steps, n_frames)
	end
	function SimParams(
		symbol_lat    :: Symbol,
		symbol_lon    :: Symbol;
		sampling_lat  :: SamplingParams = SamplingParams(),
		sampling_lon  :: SamplingParams = SamplingParams(),
		sec_per_frame :: Float64 = 0.125,
		n_euler_steps :: Int     = 10,
		n_frames      :: Int     = 100
		)

		@assert(sec_per_frame > 0.0)
		@assert(n_euler_steps > 0)
		@assert(n_frames      > 0)

		new(symbol_lat, symbol_lon, sampling_lat, sampling_lon,
			sec_per_frame, n_euler_steps, n_frames)
	end
end

function simulate!(
	initial_scene :: Scene,
	road          :: StraightRoadway,
	em            :: EM,
	params        :: SimParams = SimParams(),
	log           :: Matrix{Float64} = create_log(length(initial_scene), params.n_frames)
	)
	
	numcars = ncars(log)
	@assert(numcars == length(initial_scene))
	@assert(nframes(log) == params.n_frames)
	init_log!(log, initial_scene)

	sec_per_frame = params.sec_per_frame
	n_euler_steps = params.n_euler_steps
	δt            = sec_per_frame / n_euler_steps

	symbol_lat = params.symbol_lat
	symbol_lon = params.symbol_lon
	sample_lat = params.sampling_lat
	sample_lon = params.sampling_lon
	samplemethod_lat = sample_lat.sampling_scheme
	samplemethod_lon = sample_lon.sampling_scheme
	bmap_lat = em.binmaps[findfirst(em.BN.names, symbol_lat)]
	bmap_lon = em.binmaps[findfirst(em.BN.names, symbol_lon)]

	# NOTE(tim): propagate a single step in order to get a non-empty history
	frameind = 1
	for carind in 1 : numcars
		propagate!(log, carind, frameind, 0.0, 0.0,
				   n_euler_steps, δt, params)
	end

	indicators = get_indicators(em)
	if isa(samplemethod_lat, SampleTurnrateTowardsLaneCenter)
		error("avoid this")
		# for f in [YAW, D_CL]
		# 	if !in(f, indicators)
		# 		push!(indicators, f)
		# 	end
		# end
	elseif isa(samplemethod_lat, SampleTurnrateAlongLane)
		error("avoid this")
		# if !in(YAW, indicators)
		# 	push!(indicators, YAW)
		# end
	end

	while !isdone(log, frameind)
		frameind += 1
		clearmeta()

		for carind in 1 : numcars
			base = logindexbase(carind)

			# TODO(tim): pre-allocate observations?
			observations = observe(log, road, carind, frameind, indicators, sec_per_frame)
			assignment   = encode(observations, em)
			sample!(em, assignment)

			action_lat = decode(samplemethod_lat, bmap_lat, assignment[symbol_lat], observations, sec_per_frame)
			action_lon = decode(samplemethod_lon, bmap_lon, assignment[symbol_lon], observations, sec_per_frame)

			a = get_input_acceleration(symbol_lon, action_lon, log, frameind, base)
			ω = get_input_turnrate(    symbol_lat, action_lat, log, frameind, base)

			log[frameind, base + LOG_COL_A] = a
			log[frameind, base + LOG_COL_T] = ω

			# TODO(tim): put this into a function
			if sample_lat == :SMA
				n_frames = frameind > sample_lat.smoothing_counts ? sample_lat.smoothing_counts : frameind
				lo = frameind-(n_frames-1)
				log[frameind, base + LOG_COL_A] = a = sum(log[lo:frameind, logindexbase(carind) + LOG_COL_A]) / n_frames
			elseif sample_lat == :WMA
				n_frames = frameind > sample_lat.smoothing_counts ? sample_lat.smoothing_counts : frameind
				lo = frameind-(n_frames-1)
				log[frameind, base + LOG_COL_A] = a = (sum(log[lo:frameind, logindexbase(carind) + LOG_COL_A] .* [1:n_frames]) + a) / (0.5n_frames*(n_frames+1))
			end

			if sample_lon == :SMA
				n_frames = frameind > sample_lon.smoothing_counts ? sample_lon.smoothing_counts : frameind
				lo = frameind-(n_frames-1)
				log[frameind, base + LOG_COL_T] = ω = sum(log[lo:frameind, logindexbase(carind) + LOG_COL_T]) / n_frames
			elseif sample_lon == :WMA
				n_frames = frameind > sample_lon.smoothing_counts ? sample_lon.smoothing_counts : frameind
				lo = frameind-(n_frames-1)
				log[frameind, base + LOG_COL_T] = ω = sum(log[lo:frameind, logindexbase(carind) + LOG_COL_T] .* [1:n_frames]) / (0.5n_frames*(n_frames+1))
			end

			propagate!(log, frameind, base, a, ω, n_euler_steps, δt, params)
		end
	end

	log
end

function isdone(log::Matrix{Float64}, frameind::Int)
	if frameind >= size(log, 1)-1
		return true
	end
	false
end
function init_log!(log::Matrix{Float64}, initial_scene::Scene)
	fill!(log, 0.0)
	add_to_log!(log, initial_scene, 1)
end
function add_to_log!(log::Matrix{Float64}, s::Scene, frameind::Int)
	ind = 0
	for (i,car) in enumerate(s)
		log[frameind, ind+LOG_COL_X] = car.pos.x
		log[frameind, ind+LOG_COL_Y] = car.pos.y
		log[frameind, ind+LOG_COL_ϕ] = car.pos.ϕ
		log[frameind, ind+LOG_COL_V] = car.speed
		ind += LOG_NCOLS_PER_CAR
	end
end

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
	log           :: Matrix{Float64},
	frameind      :: Int,
	baseind       :: Int,
	a             :: Float64,
	ω             :: Float64,
	n_euler_steps :: Int,
	δt            :: Float64,
	params        :: SimParams
	)
	# run physics on the given car at time frameind
	# place results in log for that car in frameind + 1

	# const Kp = 0.2

	# a = action[params.accel_sym]    # [m/s²]
	# ω = action[params.turnrate_sym] # [rad/s]
	# phi_des = action[params.turnrate_sym]
	
	
	x = log[frameind, baseind + LOG_COL_X]
	y = log[frameind, baseind + LOG_COL_Y]
	ϕ = log[frameind, baseind + LOG_COL_ϕ]
	v = log[frameind, baseind + LOG_COL_V]

	for j = 1 : n_euler_steps
		# ω = (phi_des - ϕ)*Kp
		v += a*δt
		ϕ += ω*δt
		x += v*cos(ϕ)*δt
		y += v*sin(ϕ)*δt
	end

	log[frameind+1, baseind + LOG_COL_X] = x
	log[frameind+1, baseind + LOG_COL_Y] = y
	log[frameind+1, baseind + LOG_COL_ϕ] = ϕ
	log[frameind+1, baseind + LOG_COL_V] = v
end
function observe(log::Matrix{Float64}, road::StraightRoadway, carind::Int, frameind::Int, features::Vector{AbstractFeature}, timestep::Float64)
	observations = Dict{Symbol,Any}()
	for f in features
		val = get(f, log, road, timestep, carind, frameind)
		if !isinf(val)
			observations[symbol(f)] = val
		else
			observations[symbol(f)] = NA
		end
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

decode{T,S}(s::SampleMethod, bmap::BinMap{T,S}, x::S, observations::Dict{Symbol, Any}, Δt::Float64) = error("unsupporterd sample method $s")
decode{T,S}(::SampleUniform, bmap::BinMap{T,S}, x::S, observations::Dict{Symbol, Any}, Δt::Float64) = decode(bmap, x)
function decode{T,S}(::SampleBinCenter, bmap::BinMap{T,S}, x::S, observations::Dict{Symbol, Any}, Δt::Float64)
	ind = bmap.bin2i[x]
	lo  = bmap.binedges[ind]
	hi  = bmap.binedges[ind+1]
	0.5(hi - lo)
end
function decode{T,S}(::SampleUniformZeroBin, bmap::BinMap{T,S}, x::S, observations::Dict{Symbol, Any}, Δt::Float64)
	ind = bmap.bin2i[x]
	lo  = bmap.binedges[ind]
	hi  = bmap.binedges[ind+1]
	if lo ≤ 0.0 ≤ hi
		0.0
	else
		lo + (hi-lo)*rand()
	end
end
function decode{T,S}(samplemethod::SampleTurnrateTowardsLaneCenter, bmap::BinMap{T,S}, x::S, observations::Dict{Symbol, Any}, Δt::Float64)
	d_cl = observations[:d_cl]
	ψ = observations[:yaw]

	C = samplemethod.centerline_scalar
	κ = samplemethod.proportional_scalar
	ψ_max = samplemethod.max_abs_desired_yaw
	ψ_des = clamp(-d_cl*C, -ψ_max, ψ_max)
	ω_des = κ*(ψ_des - ψ)/Δt

	ind = bmap.bin2i[x]
	lo  = bmap.binedges[ind]
	hi  = bmap.binedges[ind+1]
	if lo ≤ ω_des ≤ hi
		ω_des
	else
		lo + (hi-lo)*rand()
	end
end
function decode{T,S}(samplemethod::SampleTurnrateAlongLane, bmap::BinMap{T,S}, x::S, observations::Dict{Symbol, Any}, Δt::Float64)
	
	ψ = observations[:yaw]

	κ = samplemethod.proportional_scalar
	ω_max = samplemethod.max_abs_omega
	ω_des = clamp(-ψ*κ/Δt, -ω_max, ω_max)

	ind = bmap.bin2i[x]
	lo  = bmap.binedges[ind]
	hi  = bmap.binedges[ind+1]
	if lo ≤ ω_des ≤ hi
		ω_des
	else
		lo + (hi-lo)*rand()
	end
end

decode{T,S}(s::SampleMethod, bmap::DataBinMap{T,S}, x::S, ::Dict{Symbol, Any}, Δt::Float64) = error("unsupporterd sample method $s")
# decode{T,S}(s::SampleMethod, bmap::DataBinMap{T,S}, x::NAtype, ::Dict{Symbol, Any}, Δt::Float64) = decode(bmap, x)
decode{T,S}(::SampleUniform, bmap::DataBinMap{T,S}, x::S, ::Dict{Symbol, Any}, Δt::Float64) = decode(bmap, x)
function decode{T,S}(::SampleBinCenter, bmap::DataBinMap{T,S}, x::S, ::Dict{Symbol, Any}, Δt::Float64)
	ind = bmap.bin2i[x]
	lo  = bmap.binedges[ind]
	hi  = bmap.binedges[ind+1]
	0.5(hi - lo)
end
function decode{T,S}(::SampleUniformZeroBin, bmap::DataBinMap{T,S}, x::S, ::Dict{Symbol, Any}, Δt::Float64)
	ind = bmap.bin2i[x]
	lo  = bmap.binedges[ind]
	hi  = bmap.binedges[ind+1]
	if lo ≤ 0.0 ≤ hi
		0.0
	else
		lo + (hi-lo)*rand()
	end
end
function decode{T,S}(samplemethod::SampleTurnrateTowardsLaneCenter, bmap::DataBinMap{T,S}, x::S, observations::Dict{Symbol, Any}, Δt::Float64)
	
	d_cl = observations[:d_cl]
	ψ = observations[:yaw]

	C = samplemethod.centerline_scalar
	κ = samplemethod.proportional_scalar
	ψ_max = samplemethod.max_abs_desired_yaw
	ψ_des = clamp(-d_cl*C, -ψ_max, ψ_max)
	ω_des = κ*(ψ_des - ψ)/Δt

	ind = bmap.bin2i[x]
	lo  = bmap.binedges[ind]
	hi  = bmap.binedges[ind+1]
	if lo ≤ ω_des ≤ hi
		ω_des
	else
		lo + (hi-lo)*rand()
	end
end
function decode{T,S}(samplemethod::SampleTurnrateAlongLane, bmap::DataBinMap{T,S}, x::S, observations::Dict{Symbol, Any}, Δt::Float64)
	
	ψ = observations[:yaw]

	κ = samplemethod.proportional_scalar
	ω_max = samplemethod.max_abs_omega
	ω_des = clamp(-ψ*κ/Δt, -ω_max, ω_max)

	ind = bmap.bin2i[x]
	lo  = bmap.binedges[ind]
	hi  = bmap.binedges[ind+1]
	if lo ≤ ω_des ≤ hi
		ω_des
	else
		lo + (hi-lo)*rand()
	end
end

function compute_log_metrics(log::Matrix{Float64}, road::StraightRoadway, params::SimParams)

	Δt = params.sec_per_frame

	has_collision_ego = false # whether ego car collides with another car
	n_lanechanges_ego = 0 # whether ego car makes a lange change
	elapsed_time = size(log, 1) * Δt
	has_other_cars = size(log,2) > LOG_NCOLS_PER_CAR

	df_ego = log[1]
	mean_speed_ego = mean(log[:,LOG_COL_V]) # mean ego speed
	mean_centerline_offset_ego = 0.0
	std_speed_ego  = std(log[:,LOG_COL_V])  # stdev of ego speed
	time_of_first_offroad = Inf
	n_frames_offroad_ego = 0

	arr_v_x = (log[2:end,LOG_COL_X] - log[1:end-1,LOG_COL_X]) ./ Δt
	arr_v_y = (log[2:end,LOG_COL_Y] - log[1:end-1,LOG_COL_Y]) ./ Δt
	arr_a_x = (arr_v_x[2:end] - arr_v_x[1:end-1]) ./ Δt
	arr_a_y = (arr_v_y[2:end] - arr_v_y[1:end-1]) ./ Δt
	abs_arr_j_x = abs((arr_a_x[2:end] - arr_a_x[1:end-1]) ./ Δt)
	abs_arr_j_y = abs((arr_a_y[2:end] - arr_a_y[1:end-1]) ./ Δt)
	abs_jerk_mean_x = mean(abs_arr_j_x)
	abs_jerk_std_x = stdm(abs_arr_j_x, abs_jerk_mean_x)
	abs_jerk_mean_y = mean(abs_arr_j_y)
	abs_jerk_std_y = stdm(abs_arr_j_y, abs_jerk_mean_y)

	lane_centers = lanecenters(road) # [0,lw,2lw,...]

	for i = 1 : size(log,1)
		# check for lange change (by ego car)
		lane_dists = abs(lane_centers .- log[i,  LOG_COL_Y])
		cl_cur = indmin(lane_dists)
		mean_centerline_offset_ego += lane_dists[cl_cur]

		if i > 1
			cl_old = indmin(abs(lane_centers .- log[i-1,LOG_COL_Y]))
			if cl_old != cl_cur
				n_lanechanges_ego += 1
			end
		end

		if !onroad(log[i, LOG_COL_Y], road)
			n_frames_offroad_ego += 1
			time_of_first_offroad = min(i*Δt, time_of_first_offroad)
		end

		# check for collision
		if has_other_cars
			# TODO(tim): make work for more than 1 other car
			dx = log[i,LOG_COL_X] - log[i,LOG_COL_X+LOG_NCOLS_PER_CAR]
			dy = log[i,LOG_COL_Y] - log[i,LOG_COL_Y+LOG_NCOLS_PER_CAR]
			if abs(dx) < CAR_LENGTH && abs(dy) < CAR_WIDTH
				has_collision_ego = true
				# NOTE(tim): anything after a collision is invalid - break here
				break
			end
		end
	end

	mean_centerline_offset_ego /= size(log,1)

	[
	 :has_collision_ego=>has_collision_ego,
	 :n_lanechanges_ego=>n_lanechanges_ego,
	 :mean_speed_ego=>mean_speed_ego,
	 :mean_centerline_offset_ego=>mean_centerline_offset_ego,
	 :std_speed_ego=>std_speed_ego,
	 :n_sec_offroad_ego=>n_frames_offroad_ego * Δt,
	 :elapsed_time=>elapsed_time,
	 :time_of_first_offroad=>time_of_first_offroad,
	 :went_offroad=>time_of_first_offroad!=Inf,
	 :jerk_mean_x=>abs_jerk_mean_x,
	 :jerk_std_x=>abs_jerk_std_x,
	 :jerk_mean_y=>abs_jerk_mean_y,
	 :jerk_std_y=>abs_jerk_std_y,
	 :final_x=>log[end,LOG_COL_X],
	 :final_y=>log[end,LOG_COL_Y],
	 :initial_speed=>log[2,LOG_COL_V]
	]::Dict{Symbol, Any}
end

export save_metrics, calc_metric, aggregate_metrics, 
	   print_results_human_readable,
	   print_results_csv_readable

function save_metrics(metrics::Dict{Symbol, Any}, outfile::String)
	open(outfile, "w") do fout
		for (sym,val) in metrics
			println(fout, string(sym), ": ", val)
		end
	end
end
function calc_metric(sym::Symbol, ::Type{Int}, metrics)
	counts = Dict{Int,Int}()
	for i = 1 : length(metrics)
		counts[metrics[i][sym]] = get(counts, metrics[i][sym], 0) + 1
	end

	t_arr = [metrics[i][:elapsed_time] for i in 1 : length(metrics)]
	tot_time = sum(t_arr)

	arr = [metrics[i][sym] for i in 1 : length(metrics)]
	ave = mean(arr)
	stdev = stdm(arr, ave)
	weighted_ave = sum([metrics[i][sym] * metrics[i][:elapsed_time] for i in 1 : length(metrics)]) / tot_time

	(ave, stdev, weighted_ave)
end
function calc_metric(sym::Symbol, ::Type{Bool}, metrics)
	n_true = sum([metrics[i][sym] for i in 1 : length(metrics)])
	ave_time_to_true = sum([metrics[i][sym] ? metrics[i][:elapsed_time] : 0.0 for i in 1 : length(metrics)]) / n_true

	t_arr = [metrics[i][:elapsed_time] for i in 1 : length(metrics)]
	tot_time = sum(t_arr)

	odds_true_per_run = n_true / length(metrics)
	odds_true_per_sec = n_true / tot_time

	(odds_true_per_run, odds_true_per_sec, ave_time_to_true)
end
function calc_metric(sym::Symbol, ::Type{Float64}, metrics, use_abs=false)
	if use_abs
		arr = convert(Vector{Float64}, [abs(metrics[i][sym]) for i in 1 : length(metrics)])
	else
		arr = convert(Vector{Float64}, [metrics[i][sym] for i in 1 : length(metrics)])
	end
	inds = find(a->a!=Inf, arr)
	arr = arr[inds]
	ave = mean(arr)
	stdev = stdm(arr, ave)
	(ave, stdev)
end
function aggregate_metrics(
	initial_scene :: Scene,
	road          :: StraightRoadway,
	em            :: EM,
	params        :: SimParams,
	n_simulations :: Int,
	runlog        :: Matrix{Float64} = create_log(length(initial_scene), params.n_frames)
	)

	@assert(n_simulations > 0)

	metrics = Array(Dict{Symbol, Any}, n_simulations)

	# TODO(tim): parallelize
	for isim = 1 : n_simulations
		simulate!(initial_scene, road, em, params, runlog)
		metrics[isim] = compute_log_metrics(runlog, road, params)
	end
	metrics
end
function print_results_human_readable(params::SimParams, aggmetrics::Vector{Dict{Symbol, Any}})
	println("GLOBAL")
	println("\tsec_per_frame: ", params.sec_per_frame)
	println("\tn_euler_steps: ", params.n_euler_steps)
	println("\tn_frames:      ", params.n_frames)
	println("\ttot_time:      ", params.n_frames * params.sec_per_frame)
	println("\tsymbol_lat:    ", params.symbol_lat)
	println("\tsymbol_lon:    ", params.symbol_lon)
	println("")
	println("ACCELERATION")
	println("\tbinning:      ", typeof(params.sampling_lon.sampling_scheme))
	print("\tsmoothing:    ", params.sampling_lon.smoothing)
	if params.sampling_lon.smoothing != :none
	 	print(" {", params.sampling_lon.smoothing_counts, "}")
	end
	println("\n")
	println("TURNRATE")
	println("\tbinning:      ", typeof(params.sampling_lat.sampling_scheme))
	print("\tsmoothing:    ", params.sampling_lat.smoothing)
	if params.sampling_lat.smoothing != :none
	 	print(" {", params.sampling_lat.smoothing_counts, "}")
	end
	println("\n")
	println("RESULTS")
	println("\tmean centerline offset: ", calc_metric(:mean_centerline_offset_ego, Float64, aggmetrics, true))
	println("\tego speed:              ", calc_metric(:mean_speed_ego, Float64, aggmetrics))
	println("\toffroad rate:           ", calc_metric(:went_offroad, Bool, aggmetrics))
	println("\ttime to offroad:        ", calc_metric(:time_of_first_offroad, Float64, aggmetrics))
	println("\tlane change rate:       ", calc_metric(:n_lanechanges_ego, Float64, aggmetrics))
	println("\tjerk mean x:            ", calc_metric(:jerk_mean_x, Float64, aggmetrics))
	println("\tjerk std x:             ", calc_metric(:jerk_std_x, Float64, aggmetrics))
	println("\tjerk mean y:            ", calc_metric(:jerk_mean_y, Float64, aggmetrics))
	println("\tjerk std y:             ", calc_metric(:jerk_std_y, Float64, aggmetrics))
end
function print_results_csv_readable(io::IO, params::SimParams, aggmetrics::Vector{Dict{Symbol, Any}})
	mean_centerline_offset_ego = calc_metric(:mean_centerline_offset_ego, Float64, aggmetrics, true)
	mean_speed_ego = calc_metric(:mean_speed_ego, Float64, aggmetrics)
	went_offroad = calc_metric(:went_offroad, Bool, aggmetrics)
	time_of_first_offroad = calc_metric(:time_of_first_offroad, Float64, aggmetrics)
	n_lanechanges_ego = calc_metric(:n_lanechanges_ego, Float64, aggmetrics)
	jerk_mean_x = calc_metric(:jerk_mean_x, Float64, aggmetrics)
	jerk_std_x = calc_metric(:jerk_std_x, Float64, aggmetrics)
	jerk_mean_y = calc_metric(:jerk_mean_y, Float64, aggmetrics)
	jerk_std_y = calc_metric(:jerk_std_y, Float64, aggmetrics)

	str_smoothing_a = params.sampling_lon.smoothing == :none ? "none" :
						@sprintf("%s {%d}", string(params.sampling_lon.smoothing), params.sampling_lon.smoothing_counts)
	str_smoothing_ϕ = params.sampling_lat.smoothing == :none ? "none" :
						@sprintf("%s {%d}", string(params.sampling_lat.smoothing), params.sampling_lat.smoothing_counts)

	@printf(io, "%.0f, %.3f, ", params.n_frames, params.sec_per_frame)
	@printf(io, "%s, %s, ", string(typeof(params.sampling_a.sampling_scheme)), str_smoothing_a)
	@printf(io, "%s, %s, ", string(typeof(params.sampling_ϕ.sampling_scheme)), str_smoothing_ϕ)
	@printf(io, "%.3f pm %.3f, %.2f pm %.3f, %.3f, %.2f pm %.2f, %.4f pm %.4f, %.3f pm %.2f, %.2f pm %.2f, %.3f pm %.2f, %.2f pm %.2f\n",
		mean_centerline_offset_ego[1], mean_centerline_offset_ego[2],
		mean_speed_ego[1], mean_speed_ego[2],
		went_offroad[1],
		time_of_first_offroad[1], time_of_first_offroad[2],
		n_lanechanges_ego[1], n_lanechanges_ego[2],
		jerk_mean_x[1], jerk_mean_x[2],
		jerk_std_x[1], jerk_std_x[2],
		jerk_mean_y[1], jerk_mean_y[2],
		jerk_std_y[1], jerk_std_y[2])
end
print_results_csv_readable(params::SimParams, aggmetrics::Vector{Dict{Symbol, Any}}) = print_results_csv_readable(STDOUT, params, aggmetrics)
