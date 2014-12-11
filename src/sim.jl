export simulate, compute_log_metrics

const N_EULER_STEPS = 10
const Δt = 0.25 # [sec]
const δt = Δt / N_EULER_STEPS

function simulate(
	initial_scene :: Scene,
	road          :: Roadway,
	log           :: Matrix{Float64},
	em            :: EM
	)
	
	frameind = 1
	ncars = ncars(log)
	@assert(ncars == length(scene))

	init_log!(log, initial_scene) # zero out, then write in the scene into the first frame

	# propagate a single step in order to get a nice history
	for carind in 1 : ncars
		propagate!(log, carind, frameind, [:f_accel_250ms=>0.0, :f_deltaY_250ms=>0.0])
	end

	while !isdone(log, frameind)
		frameind += 1
		clearmeta()

		for carind in 1 : ncars
			observations = observe(log, carind, frameind, indicators) # TODO(tim): pre-allocate observations?
			assignment = encode(observations, em)
			
			if false # use_prior(assignment, em)
				action = action_prior(observations)
			else
				sample!(em, assignment)
				action = draw_action(assignment, em)
			end

			propagate!(log, carind, frameind, action)
		end
	end

	# TODO(tim): return metrics?
end

function isdone(log::Matrix{Float64}, frameind::Int)
	if frameind >= size(log, 1)
		return true
	end
	false
end
function init_log!(log::Matrix{Float64}, initial_scene::Scene)
	fill!(log, 0.0)
	add_to_log!(log, s, 1)
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

	log[frameind, b + LOG_COL_X] = x
	log[frameind, b + LOG_COL_Y] = y
	log[frameind, b + LOG_COL_ϕ] = ϕ
	log[frameind, b + LOG_COL_V] = v
end
function observe(log::Matrix{Float64}, carind::Int, frameind::Int, features::Vector{AbstractFeature})
	observations = Dict{Symbol,Any}()
	for f in features
		observations[symbol(f)] = get(f, log, ROAD, carind, frameind)
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

	ordering = topological_sort_by_dfs(em.BN.dag)
	for name in em.BN.names[ordering]
		if !haskey(assignment, name)
			assignment[name] = BayesNets.rand(BayesNets.cpd(em.BN, name), assignment)
		end
	end
	assignment
end
function action_prior(observations::Dict{Symbol, Any})

	# car avoids driving off roadway
	# if off-roadway, car heads back to road
	# car tries to drive at the speed limit

	action_dict = Dict{Symbol, Any}()

	yaw   = observations[:yaw]
	posFy = observations[:posFy]
	speed = observations[:speed]
	d_cl = observations[:d_cl]

	# select acceleration
	action_dict[:f_accel_250ms] = -(speed-29.0)/Δt
	

	# select turnrate
	if posFy < -ROAD.lanewidth
		# car is off the road on the right side
		# proportional control to get yaw to 0.01
		turnrate = 0.25*(0.01-yaw)/Δt
	elseif (ROAD.nlanes - 0.5)*ROAD.lanewidth < posFy
		# car is off the road on the left side
		# proportional control to get yaw to -0.01
		turnrate = 0.25*(-0.01-yaw)/Δt
	else
		# the car is on the road
		# proportional control towards the nearest centerlane
		yaw_des = -d_cl*0.01
		turnrate = 0.25*(yaw_des-yaw)/Δt
	end

	# select delta y
	action_dict[:f_turnrate_250ms] = turnrate
	# action_dict[:f_deltaY_250ms] = 

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

function compute_log_metrics(log::Matrix{Float64})

	has_collision_ego = false # whether ego car collides with another car
	n_lanechanges_ego = 0 # whether ego car makes a lange change
	elapsed_time = size(log, 1) / Δt
	has_other_cars = size(log,2) > LOG_NCOLS_PER_CAR

	df_ego = log[1]
	mean_speed_ego = mean(log[:,LOG_COL_V]) # mean ego speed
	std_speed_ego  = std(log[:,LOG_COL_V])  # stdev of ego speed
	n_frames_offroad_ego = 0

	lane_centers = [0:(ROAD_NLANES-1)].*ROAD_LANE_WIDTH # [0,lw,2lw,...]

	for i = 1 : size(log,1)
		# check for lange change (by ego car)
		if i > 1
			cl_old = indmin(abs(lane_centers .- log[i-1,LOG_COL_Y]))
			cl_new = indmin(abs(lane_centers .- log[i,  LOG_COL_Y]))
			if cl_old != cl_new
				n_lanechanges_ego += 1
			end
		end

		if !onroad(log[i, LOG_COL_Y], ROAD)
			n_frames_offroad_ego += 1
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

	[
	 :has_collision_ego=>has_collision_ego,
	 :n_lanechanges_ego=>n_lanechanges_ego,
	 :mean_speed_ego=>mean_speed_ego,
	 :std_speed_ego=>std_speed_ego,
	 :n_sec_offroad_ego=>n_frames_offroad_ego / Δt,
	 :elapsed_time=>elapsed_time
	]::Dict{Symbol, Any}
end