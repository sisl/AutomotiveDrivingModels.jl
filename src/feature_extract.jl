meta = Dict{(Int,Symbol), Float64}() # (carid, fsym) -> value

clearmeta() = empty!(meta)
function get(F::AbstractFeature, scene::Vector{Scene}, road::Roadway, carind::Int)

	key = (carind, symbol(F))
	if haskey(meta, key)
		return meta[key]
	end

	value = _get(F, scene, road, carind)::Float64
	meta[key] = value
	value
end
function _get(F::AbstractFeature, scene::Vector{Scene}, road::Roadway, carind::Int)
	warn(@sprintf("get not implemented for feature %s", string(symbol(F))))
	return 0.0
end

function get(F::Features.Feature_VelFx, scenes::Vector{Scene}, road::Roadway, carind::Int)
	car = scenes[1][carind]
	car.speed * cos(car.pos.ϕ)
end
function get(F::Features.Feature_VelFy, scenes::Vector{Scene}, road::Roadway, carind::Int)
	car = scenes[1][carind]
	car.speed * sin(car.pos.ϕ)
end
function _get(F::Features.Feature_CL, scenes::Vector{Scene}, road::Roadway, carind::Int)
	car = scenes[1][carind]
	# generate the lane centerlines
	posFy = car.pos.y
	nlanes = road.nlanes
	lanewidth = road.lanewidth
	lane_centers = [0:(nlanes-1)].*lanewidth # [0,lw,2lw,...]
	float64(indmin(abs(lane_centers .- posFy)))
end
function _get(F::Features.Feature_D_CL, scenes::Vector{Scene}, road::Roadway, carind::Int)
	car = scenes[1][carind]
	lane = get(CL, scenes, carind)
	car.pos.y - (lane-1)*road.lanewidth
end
function _get(F::Features.Feature_N_LANE_L, scenes::Vector{Scene}, road::Roadway, carind::Int)
	lane = get(CL, scenes, carind)
	float64(road.nlanes - lane)
end
		 _get(F::Features.Feature_N_LANE_R, scenes::Vector{Scene}, road::Roadway, carind::Int) = get(CL, scenes, carind) + 1.0
function _get(F::Features.Feature_TurnRate, scenes::Vector{Scene}, road::Roadway, carind::Int)
	if length(scenes) <= 1
		return NA_ALIAS
	end

	curr = scenes[1][carind].pos.ϕ
	past = scenes[2][carind].pos.ϕ
	Features.deltaangle(curr, past) / Δt
end
function _get(F::Features.Feature_AccFx, scenes::Vector{Scene}, road::Roadway, carind::Int)
	if length(scenes) <= 1
		return NA_ALIAS
	end

	curr = get(VELFX, scenes, carind)
	past = get(VELFX, scenes[2:end], carind)
	(curr - past) / Δt
end
function _get(F::Features.Feature_AccFy, scenes::Vector{Scene}, road::Roadway, carind::Int)
	if length(scenes) <= 1
		return NA_ALIAS
	end

	curr = get(VELFY, scenes, carind)
	past = get(VELFY, [scenes[2]], carind)
	(curr - past) / Δt
end
function _get(F::Features.Feature_IndFront, scenes::Vector{Scene}, road::Roadway, carind::Int)
	
	s = scenes[1]
	car = s[carind]
	mylane = get(CL, scenes, road, carind)
	myFx = car.pos.x
	myFy = car.pos.y

	frontcar_ind = 0
	frontcar_dist = Inf
	for i = 1 : length(s)
		if i == carind
			continue
		end

		dy = s[i].pos.y - myFy
		dlane = get(CL, scenes, road, i) - mylane
		if isapprox(dlane, 0.0) || dy < Features.THRESHOLD_DY_CONSIDERED_IN_FRONT
			dx = s[i].pos.x - myFx
			if 0 < dx < frontcar_dist
				frontcar_dist, frontcar_ind = dx, i
			end
		end

	end

	if frontcar_ind != 0
		# compute extra stats
		meta[(carind, :d_x_front)] = frontcar_dist
		return float64(frontcar_ind)
	end

	meta[(carind, :d_x_front)] = Features.NA_ALIAS
	return Features.NA_ALIAS
end
function _get(F::Features.Feature_D_X_FRONT, scenes::Vector{Scene}, road::Roadway, carind::Int)
	get(INDFRONT, scenes, road, carind)
	meta[(carind, :d_x_front)]
end
function _get(F::Features.Feature_IndRear, scenes::Vector{Scene}, road::Roadway, carind::Int)
	
	s = scenes[1]
	car = s[carind]
	mylane = get(CL, scenes, road, carind)
	myFx = car.pos.x
	myFy = car.pos.y

	rearcar_ind = 0
	rearcar_dist = Inf
	for i = 1 : length(s)
		if i == carind
			continue
		end

		dy = s[i].pos.y - myFy
		dlane = get(CL, scenes, road, i) - mylane
		if isapprox(dlane, 0.0) || dy < Features.THRESHOLD_DY_CONSIDERED_IN_FRONT
			dx = myFx - s[i].pos.x
			if 0 < dx < rearcar_dist
				rearcar_dist, rearcar_ind = dx, i
			end
		end

	end

	if rearcar_ind != 0
		# compute extra stats
		meta[(carind, :d_x_rear)] = rearcar_dist
		return float64(rearcar_ind)
	end

	meta[(carind, :d_x_rear)] = Features.NA_ALIAS
	return Features.NA_ALIAS
end
function _get(F::Features.Feature_D_X_REAR, scenes::Vector{Scene}, road::Roadway, carind::Int)
	get(INDREAR, scenes, road, carind)
	meta[(carind, :d_x_rear)]
end