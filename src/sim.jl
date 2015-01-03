export simulate!, compute_log_metrics, SimParams

import BinMaps: encode
import Graphs: topological_sort_by_dfs, in_degree, in_neighbors

const PRIOR_VARS = [YAW,VELFX,D_CL]

type SimParams
	sample_scheme_a :: Symbol # :vanilla, :zero, or :towards
	sample_scheme_ϕ :: Symbol # :vanilla, :zero, :towards, or :along
	prior_threshold_a :: Int  # min number of samples before prior to allow a prior override
	prior_threshold_ϕ :: Int
	smoothing_a :: Symbol # :none, :SMA, or :WMA
	smoothing_param_a :: Int # number of previous counts to use
	smoothing_ϕ :: Symbol
	smoothing_param_ϕ :: Int

	SimParams() = new(:vanilla, :vanilla, 0, 0, :none, 0, :none, 0)
end

function simulate!(
	initial_scene :: Scene,
	road          :: Roadway,
	log           :: Matrix{Float64},
	em            :: EM,
	# choose_from_prior_accel, 
	# choose_from_prior_turnrate,
	params        :: SimParams = SimParams()
	)
	
	frameind = 1
	ncars = CarEM.ncars(log)
	@assert(ncars == length(initial_scene))
	indicators = unique(append!(get_indicators(em), PRIOR_VARS))

	init_log!(log, initial_scene) # zero out, then write in the scene into the first frame

	bmap_a = em.binmaps[findfirst(em.BN.names, :f_accel_250ms)]
	bmap_ϕ = em.binmaps[findfirst(em.BN.names, :f_turnrate_250ms)]

	# propagate a single step in order to get a nice history
	for carind in 1 : ncars
		propagate!(log, carind, frameind, [:f_accel_250ms=>0.0, :f_turnrate_250ms=>0.0])
	end

	n_prior_samples_a = 0
	n_prior_samples_ϕ = 0

	parents_accel = Array(Symbol, in_degree(1, em.BN.dag))
	for (i,neighbor) in enumerate(in_neighbors(1, em.BN.dag))
		parents_accel[i] = em.BN.names[neighbor]
	end

	parents_turnrate = Array(Symbol, in_degree(2, em.BN.dag))
	for (i,neighbor) in enumerate(in_neighbors(2, em.BN.dag))
		parents_turnrate[i] = em.BN.names[neighbor]
	end

	while !isdone(log, frameind)
		frameind += 1
		clearmeta()

		for carind in 1 : ncars
			base = logindexbase(carind)

			observations = observe(log, road, carind, frameind, indicators) # TODO(tim): pre-allocate observations?
			assignment = encode(observations, em)

			# println(observations)

			sample!(em, assignment)
			action = draw_action(assignment, em)
			action_p = action_prior(observations, road)

			# handle acceleration first
			if params.sample_scheme_a == :zero
				ind = bmap_a.bin2i[assignment[:f_accel_250ms]]
				lo  = bmap_a.binedges[ind]
				hi  = bmap_a.binedges[ind+1]
				if lo < 0.0 < hi
					action[:f_accel_250ms] = 0.0
				end
			elseif params.sample_scheme_a == :towards
				ind = bmap_a.bin2i[assignment[:f_accel_250ms]]
				lo  = bmap_a.binedges[ind]
				hi  = bmap_a.binedges[ind+1]
				need = action_p[:f_accel_250ms]
				if lo < need < hi
					action[:f_accel_250ms] = need
				end
			end

			# assignment_accel = [sym=>assignment[sym] for sym in parents_accel]
			# if in(assignment_accel, choose_from_prior_accel)
			# 	action[:f_accel_250ms] = action_p[:f_accel_250ms]
			# 	n_prior_samples_a += 1
			# end

			log[frameind, base + LOG_COL_A] = action[:f_accel_250ms]

			if params.smoothing_a == :SMA
				n_frames = frameind > params.smoothing_param_a ? params.smoothing_param_a : frameind
				lo = frameind-(n_frames-1)
				action[:f_accel_250ms] = sum(log[lo:frameind, logindexbase(carind) + LOG_COL_A]) / n_frames
			elseif params.smoothing_a == :WMA
				n_frames = frameind > params.smoothing_param_a ? params.smoothing_param_a : frameind
				lo = frameind-(n_frames-1)
				action[:f_accel_250ms] = sum(log[lo:frameind, logindexbase(carind) + LOG_COL_A] .* [1:n_frames]) / (0.5n_frames*(n_frames+1))
			end

			# handle turnrate
			if params.sample_scheme_ϕ == :zero
				ind = bmap_ϕ.bin2i[assignment[:f_turnrate_250ms]]
				lo  = bmap_ϕ.binedges[ind]
				hi  = bmap_ϕ.binedges[ind+1]
				if lo < 0.0 < hi
					action[:f_turnrate_250ms] = 0.0
				end
			elseif params.sample_scheme_ϕ == :towards
				ind = bmap_ϕ.bin2i[assignment[:f_turnrate_250ms]]
				lo  = bmap_ϕ.binedges[ind]
				hi  = bmap_ϕ.binedges[ind+1]
				need = action_p[:f_turnrate_250ms_towards]
				if lo < need < hi
					action[:f_turnrate_250ms] = need
				end
			elseif params.sample_scheme_ϕ == :along
				ind = bmap_ϕ.bin2i[assignment[:f_turnrate_250ms]]
				lo  = bmap_ϕ.binedges[ind]
				hi  = bmap_ϕ.binedges[ind+1]
				need = action_p[:f_turnrate_250ms_along]
				if lo < need < hi
					action[:f_turnrate_250ms] = need
				end
			end

			# assignment_turnrate = [sym=>assignment[sym] for sym in parents_turnrate]
			# if in(assignment_turnrate, choose_from_prior_turnrate)
			# 	action[:f_turnrate_250ms] = action_p[:f_turnrate_250ms_towards]
			# 	n_prior_samples_ϕ += 1
			# end

			log[frameind, base + LOG_COL_T] = action[:f_turnrate_250ms]

			if params.smoothing_ϕ == :SMA
				n_frames = frameind > params.smoothing_param_ϕ ? params.smoothing_param_ϕ : frameind
				lo = frameind-(n_frames-1)
				action[:f_turnrate_250ms] = sum(log[lo:frameind, logindexbase(carind) + LOG_COL_T]) / n_frames
			elseif params.smoothing_ϕ == :WMA
				n_frames = frameind > params.smoothing_param_ϕ ? params.smoothing_param_ϕ : frameind
				lo = frameind-(n_frames-1)
				action[:f_turnrate_250ms] = sum(log[lo:frameind, logindexbase(carind) + LOG_COL_T] .* [1:n_frames]) / (0.5n_frames*(n_frames+1))
			end

			propagate!(log, carind, frameind, action)
		end
	end

	metrics = compute_log_metrics(log, road)
	metrics[:n_prior_samples_a] = n_prior_samples_a
	metrics[:n_prior_samples_ϕ] = n_prior_samples_ϕ
	metrics
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

function propagate!{S<:Any}(log::Matrix{Float64}, carind::Int, frameind::Int, action::Dict{Symbol,S})
	# run physics on the given car at time frameind
	# place results in log for that car in frameind + 1

	a = action[:f_accel_250ms]    # [m/s²]
	ω = action[:f_turnrate_250ms] # [rad/s]
	
	b = logindexbase(carind)
	x = log[frameind, b + LOG_COL_X]
	y = log[frameind, b + LOG_COL_Y]
	ϕ = log[frameind, b + LOG_COL_ϕ]
	v = log[frameind, b + LOG_COL_V]

	for j = 1 : N_EULER_STEPS
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
function observe(log::Matrix{Float64}, road::Roadway, carind::Int, frameind::Int, features::Vector{AbstractFeature})
	observations = Dict{Symbol,Any}()
	for f in features
		observations[symbol(f)] = get(f, log, road, carind, frameind)
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

			# if name == :f_turnrate_250ms
			# 	bin_probs = BayesNets.cpd(em.BN, name).parameterFunction(assignment)
			# 	@printf("[")
			# 	for p in bin_probs
			# 		@printf("%.3f, ", p)
			# 	end
			# 	@printf("] %d\n", assignment[name])
			# end
		end
	end
	assignment
end
function action_prior(observations::Dict{Symbol, Any}, road::Roadway)

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

function compute_log_metrics(log::Matrix{Float64}, road::Roadway)

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