export SimParams, SamplingParams
export simulate!, compute_log_metrics, SimParams
export SEC_PER_FRAME, set_sec_per_frame

import BinMaps: encode
import Graphs: topological_sort_by_dfs, in_degree, in_neighbors

# TODO: change sampling scheme to a class object and use decode(bmap, scheme, x)

immutable SamplingParams
	sampling_scheme  :: Symbol # :vanilla, :zero, :towards (or :along for ϕ)
	smoothing        :: Symbol # :none, :SMA, :WMA
	smoothing_counts :: Int    # number of previous counts to use

	function SamplingParams(
		sampling_scheme :: Symbol = :vanilla,
		smoothing :: Symbol = :none,
		smoothing_counts :: Int = 0
		)
		@assert(smoothing_counts ≥ 0)
		new(sampling_scheme, smoothing, smoothing_counts)
	end
end
immutable SimParams
	sampling_a    :: SamplingParams
	sampling_ϕ    :: SamplingParams
	sec_per_frame :: Float64
	n_euler_steps :: Int
	n_frames      :: Int

	function SimParams(
		sampling_a :: SamplingParams = SamplingParams(),
		sampling_ϕ :: SamplingParams = SamplingParams(),
		sec_per_frame :: Float64     = 0.125,
		n_euler_steps :: Int         = 10,
		n_frames      :: Int         = 100
		)

		@assert(sec_per_frame > 0.0)
		@assert(n_euler_steps > 0)
		@assert(n_frames      > 0)
		new(sampling_a, sampling_ϕ, sec_per_frame, n_euler_steps, n_frames)
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

	indicators = get_indicators(em)
	frameind = 1

	n_euler_steps = params.n_euler_steps
	δt = params.sec_per_frame / n_euler_steps

	bmap_a = em.binmaps[findfirst(em.BN.names, :f_accel_250ms)]
	bmap_ϕ = em.binmaps[findfirst(em.BN.names, :f_turnrate_250ms)]

	# NOTE(tim): propagate a single step in order to get a non-empty history
	for carind in 1 : numcars
		propagate!(log, carind, frameind, 
			[:f_accel_250ms=>0.0, :f_turnrate_250ms=>0.0],
			n_euler_steps, δt)
	end

	while !isdone(log, frameind)
		frameind += 1
		clearmeta()

		for carind in 1 : numcars
			base = logindexbase(carind)

			# TODO(tim): pre-allocate observations?
			observations = observe(log, road, carind, frameind, indicators, params.sec_per_frame)
			assignment = encode(observations, em)
			sample!(em, assignment)
			action = draw_action(assignment, em)

			# handle acceleration first
			# TODO(tim): clean this up
			# action_p = action_prior(observations, road)
			# if params.sample_scheme_a == :zero
			# 	ind = bmap_a.bin2i[assignment[:f_accel_250ms]]
			# 	lo  = bmap_a.binedges[ind]
			# 	hi  = bmap_a.binedges[ind+1]
			# 	if lo < 0.0 < hi
			# 		action[:f_accel_250ms] = 0.0
			# 	end
			# elseif params.sample_scheme_a == :towards
			# 	ind = bmap_a.bin2i[assignment[:f_accel_250ms]]
			# 	lo  = bmap_a.binedges[ind]
			# 	hi  = bmap_a.binedges[ind+1]
			# 	need = action_p[:f_accel_250ms]
			# 	if lo < need < hi
			# 		action[:f_accel_250ms] = need
			# 	end
			# end


			log[frameind, base + LOG_COL_A] = action[:f_accel_250ms]

			# TODO(tim): moving into a function
			# if params.smoothing_a == :SMA
			# 	n_frames = frameind > params.smoothing_param_a ? params.smoothing_param_a : frameind
			# 	lo = frameind-(n_frames-1)
			# 	action[:f_accel_250ms] = sum(log[lo:frameind, logindexbase(carind) + LOG_COL_A]) / n_frames
			# elseif params.smoothing_a == :WMA
			# 	n_frames = frameind > params.smoothing_param_a ? params.smoothing_param_a : frameind
			# 	lo = frameind-(n_frames-1)
			# 	action[:f_accel_250ms] = sum(log[lo:frameind, logindexbase(carind) + LOG_COL_A] .* [1:n_frames]) / (0.5n_frames*(n_frames+1))
			# end

			# handle turnrate
			# if params.sample_scheme_ϕ == :zero
			# 	ind = bmap_ϕ.bin2i[assignment[:f_turnrate_250ms]]
			# 	lo  = bmap_ϕ.binedges[ind]
			# 	hi  = bmap_ϕ.binedges[ind+1]
			# 	if lo < 0.0 < hi
			# 		action[:f_turnrate_250ms] = 0.0
			# 	end
			# elseif params.sample_scheme_ϕ == :towards
			# 	ind = bmap_ϕ.bin2i[assignment[:f_turnrate_250ms]]
			# 	lo  = bmap_ϕ.binedges[ind]
			# 	hi  = bmap_ϕ.binedges[ind+1]
			# 	need = action_p[:f_turnrate_250ms_towards]
			# 	if lo < need < hi
			# 		action[:f_turnrate_250ms] = need
			# 	end
			# elseif params.sample_scheme_ϕ == :along
			# 	ind = bmap_ϕ.bin2i[assignment[:f_turnrate_250ms]]
			# 	lo  = bmap_ϕ.binedges[ind]
			# 	hi  = bmap_ϕ.binedges[ind+1]
			# 	need = action_p[:f_turnrate_250ms_along]
			# 	if lo < need < hi
			# 		action[:f_turnrate_250ms] = need
			# 	end
			# end

			log[frameind, base + LOG_COL_T] = action[:f_turnrate_250ms]

			# if params.smoothing_ϕ == :SMA
			# 	n_frames = frameind > params.smoothing_param_ϕ ? params.smoothing_param_ϕ : frameind
			# 	lo = frameind-(n_frames-1)
			# 	action[:f_turnrate_250ms] = sum(log[lo:frameind, logindexbase(carind) + LOG_COL_T]) / n_frames
			# elseif params.smoothing_ϕ == :WMA
			# 	n_frames = frameind > params.smoothing_param_ϕ ? params.smoothing_param_ϕ : frameind
			# 	lo = frameind-(n_frames-1)
			# 	action[:f_turnrate_250ms] = sum(log[lo:frameind, logindexbase(carind) + LOG_COL_T] .* [1:n_frames]) / (0.5n_frames*(n_frames+1))
			# end

			propagate!(log, carind, frameind, action, n_euler_steps, δt)
		end
	end

	# metrics = compute_log_metrics(log, road)
	# metrics
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

function propagate!{S<:Any}(
	log::Matrix{Float64},
	carind::Int,
	frameind::Int,
	action::Dict{Symbol,S},
	n_euler_steps :: Int,
	δt :: Float64
	)
	# run physics on the given car at time frameind
	# place results in log for that car in frameind + 1

	a = action[:f_accel_250ms]    # [m/s²]
	ω = action[:f_turnrate_250ms] # [rad/s]
	
	b = logindexbase(carind)
	x = log[frameind, b + LOG_COL_X]
	y = log[frameind, b + LOG_COL_Y]
	ϕ = log[frameind, b + LOG_COL_ϕ]
	v = log[frameind, b + LOG_COL_V]

	for j = 1 : n_euler_steps
		v += a*δt
		ϕ += ω*δt
		x += v*cos(ϕ)*δt
		y += v*sin(ϕ)*δt
	end

	log[frameind+1, b + LOG_COL_X] = x
	log[frameind+1, b + LOG_COL_Y] = y
	log[frameind+1, b + LOG_COL_ϕ] = ϕ
	log[frameind+1, b + LOG_COL_V] = v
end
function observe(log::Matrix{Float64}, road::StraightRoadway, carind::Int, frameind::Int, features::Vector{AbstractFeature}, timestep::Float64)
	observations = Dict{Symbol,Any}()
	for f in features
		observations[symbol(f)] = get(f, log, road, timestep, carind, frameind)
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
	# nodes we already know (hasley(conditions, nodesym)) we use
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
function action_prior(observations::Dict{Symbol, Any}, road::StraightRoadway)

	# car avoids driving off roadway
	# if off-roadway, car heads back to road
	# car tries to drive at the speed limit

	action_dict = Dict{Symbol, Any}()

	yaw   = observations[:yaw]
	velFx = observations[:velFx]
	d_cl = observations[:d_cl]

	# select acceleration
	action_dict[:f_accel_250ms] = clamp(-(velFx-29.0)/Δt, -1.0, 1.0)
	

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
	action_dict[:f_turnrate_250ms_towards] = turnrate_towards
	action_dict[:f_turnrate_250ms_along] = turnrate_along

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
	]::Dict{Symbol, Any}
end