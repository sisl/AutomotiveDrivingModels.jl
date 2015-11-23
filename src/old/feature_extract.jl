export
		ExtractedFeatureCache,
		FeatureExtractBasics,

		get,
		get_unimplemented_indicators,
		observe


type ExtractedFeatureCache
	dict :: Dict{(Int, Int, Symbol), (Float64, Int)} # (carid, frameind, feature symbol) -> (value, runid)
	# where runid is an Int associated with a particular run
    ExtractedFeatureCache() = new(Dict{(Int, Int, Symbol), (Float64, Int)}())
end
function Base.setindex!(cache::ExtractedFeatureCache, val::(Float64, Int), key::(Int, Int, Symbol))
	cache.dict[key] = val
end
function Base.getindex(cache::ExtractedFeatureCache, key::(Int, Int, Symbol))
	cache.dict[key]
end

type FeatureExtractBasics
	simlog  :: Matrix{Float64}
	road :: StraightRoadway
	sec_per_frame :: Float64
	cache :: ExtractedFeatureCache
	runid :: Int
end
function Base.setindex!(basics::FeatureExtractBasics, val::Float64, key::(Int, Int, Symbol))
	basics.cache[key] = (val, basics.runid)
end
function Base.getindex(basics::FeatureExtractBasics, key::(Int, Int, Symbol))
	basics.cache[key][1]
end

# TODO(tim): should not use this in parallel because meta() is a single global value

unimplemented = false
clear_unimp() = global unimplemented = false

function get(F::AbstractFeature, basics::FeatureExtractBasics, carind::Int, frameind::Int)

	cache = basics.cache

	key = (carind, frameind, symbol(F))
	value, runid = get(cache.dict, key, (NaN, 0))
	if runid != basics.runid
		value = _get(F, basics, carind, frameind)::Float64
		cache.dict[key] = (value, basics.runid)
	end

	!isnan(value) || error("value from feature $(symbol(F)) was NaN!")

	value
end
function _get(F::AbstractFeature, basics::FeatureExtractBasics, carind::Int, frameind::Int)
	warn(@sprintf("get not implemented for feature %s", string(symbol(F))))
	global unimplemented = true
	return NaN
end

function observe(
    basics::FeatureExtractBasics,
    carind::Int,
    frameind::Int,
    features::Vector{AbstractFeature}
    )

    observations = Dict{Symbol,Float64}()
    for f in features
        val = get(f, basics, carind, frameind)::Float64
        observations[symbol(f)] = val
    end
    observations
end

# NOTE(tim):  get() is used for fast lookups
#            _get() is used for caching calculations
function get(::Features.Feature_PosFx, basics::FeatureExtractBasics, carind::Int, frameind::Int)

	basics.simlog[frameind, calc_logindexbase(carind) + LOG_COL_X]
end
function get(::Features.Feature_PosFy, basics::FeatureExtractBasics, carind::Int, frameind::Int)

	basics.simlog[frameind, calc_logindexbase(carind) + LOG_COL_Y]
end
function get(::Features.Feature_Yaw, basics::FeatureExtractBasics, carind::Int, frameind::Int)
	basics.simlog[frameind, calc_logindexbase(carind) + LOG_COL_ϕ]
end
function get(::Features.Feature_Speed, basics::FeatureExtractBasics, carind::Int, frameind::Int)
	basics.simlog[frameind, calc_logindexbase(carind) + LOG_COL_V]
end
function get(::Features.Feature_Delta_Speed_Limit, basics::FeatureExtractBasics, carind::Int, frameind::Int)
	Features.SPEED_LIMIT - basics.simlog[frameind, calc_logindexbase(carind) + LOG_COL_V] # TODO(tim): make this less arbitrary?
end
function get(::Features.Feature_VelFx, basics::FeatureExtractBasics, carind::Int, frameind::Int)
	baseind = calc_logindexbase(carind)
	v = basics.simlog[frameind, baseind + LOG_COL_V]
	ϕ = basics.simlog[frameind, baseind + LOG_COL_ϕ]
	v * cos(ϕ)
end
function get(::Features.Feature_VelFy, basics::FeatureExtractBasics, carind::Int, frameind::Int)
	baseind = calc_logindexbase(carind)
	v = basics.simlog[frameind, baseind + LOG_COL_V]
	ϕ = basics.simlog[frameind, baseind + LOG_COL_ϕ]
	v * sin(ϕ)
end
function _get(::Features.Feature_SceneVelFx, basics::FeatureExtractBasics, carind::Int, frameind::Int)

	ncars = get_ncars(basics.simlog)
	@assert(ncars > 0)
	total = 0.0
	for carind = 1 : ncars
		# println("vel: ", get(VELFX, basics, carind, frameind))
		total += get(VELFX, basics, carind, frameind)
	end
	total / ncars
end
function _get(::Features.Feature_TurnRate, basics::FeatureExtractBasics, carind::Int, frameind::Int)
	if frameind <= 1
		return NA_ALIAS
	end

	indϕ = calc_logindexbase(carind) + LOG_COL_ϕ
	curr = basics.simlog[frameind, indϕ]
	past = basics.simlog[frameind-1, indϕ]
	Features.deltaangle(curr, past) / basics.sec_per_frame
end

# function _get(::Features.Feature_CL, basics::FeatureExtractBasics, carind::Int, frameind::Int)
# 	posFy = get(POSFY, basics, carind, frameind)
# 	# generate the lane centerlines
# 	nlanes = basics.road.nlanes
# 	lanewidth = basics.road.lanewidth
# 	lane_centers = [0:(nlanes-1)].*lanewidth # [0,lw,2lw,...]
# 	Float64(indmin(abs(lane_centers .- posFy)))
# end
function _get(::Features.Feature_D_CL, basics::FeatureExtractBasics, carind::Int, frameind::Int)
	# y - cl_y
	lane = get(CL, basics, carind, frameind)
	posFy = get(POSFY, basics, carind, frameind)
	posFy - (lane-1)*basics.road.lanewidth
end
function _get(::Features.Feature_D_ML, basics::FeatureExtractBasics, carind::Int, frameind::Int)
	d_cl = get(D_CL, basics, carind, frameind)
	if d_cl > 0.5basics.road.lanewidth
		return NA_ALIAS
	end
	d_cl - 0.5basics.road.lanewidth
end
function _get(::Features.Feature_D_MR, basics::FeatureExtractBasics, carind::Int, frameind::Int)
	d_cl = get(D_CL, basics, carind, frameind)
	if d_cl < -0.5basics.road.lanewidth
		return NA_ALIAS
	end
	0.5basics.road.lanewidth - d_cl
end
function  get(::Features.Feature_TimeToCrossing_Left, basics::FeatureExtractBasics, carind::Int, frameind::Int)
	d_ml = get(D_ML, basics, carind, frameind)
	velFy = get(VELFY, basics, carind, frameind)
	if d_ml > 0.0 && velFy > 0.0
		min(d_ml / velFy, Features.THRESHOLD_TIME_TO_CROSSING)
	else
		NA_ALIAS
	end
end
function  get(::Features.Feature_TimeToCrossing_Right, basics::FeatureExtractBasics, carind::Int, frameind::Int)
	d_mr = get(D_MR, basics, carind, frameind)
	velFy = get(VELFY, basics, carind, frameind)
	if d_mr > 0.0 && velFy < 0.0
		min(-d_mr / velFy, Features.THRESHOLD_TIME_TO_CROSSING)
	else
		NA_ALIAS
	end
end
function _get(::Features.Feature_EstimatedTimeToLaneCrossing, basics::FeatureExtractBasics, carind::Int, frameind::Int)
	ttcr_left = get(TIMETOCROSSING_LEFT, basics, carind, frameind)
	if !isinf(ttcr_left)
		return ttcr_left
	end
	get(TIMETOCROSSING_RIGHT, basics, carind, frameind)
end
function get(::Features.Feature_TimeSinceLaneCrossing, basics::FeatureExtractBasics, carind::Int, frameind::Int)

	lane_start = get(CL, basics, carind, frameind)

	cur_frameind = frameind

	for pastframeind = frameind - 1 : -1 : 1
		Δt = (frameind - pastframeind) * basics.sec_per_frame
		if Δt > Features.THRESHOLD_TIMESINCELANECROSSING
			return Features.THRESHOLD_TIMESINCELANECROSSING
		end

		past_lane = get(CL, basics, carind, frameind)
		if past_lane != lane_start
			return Δt
		end
	end
	return Features.THRESHOLD_TIMESINCELANECROSSING
end
function get(::Features.Feature_D_Merge, basics::FeatureExtractBasics, carind::Int, frameind::Int)

	100.0 # NOTE(tim): make this less arbitrary - saturation point for StreetMap d_merge
end
function get(::Features.Feature_D_Split, basics::FeatureExtractBasics, carind::Int, frameind::Int)

	100.0 # NOTE(tim): make this less arbitrary - saturation point for StreetMap d_split
end

function _get(::Features.Feature_N_LANE_L, basics::FeatureExtractBasics, carind::Int, frameind::Int)
	lane = get(CL, basics, carind, frameind)
	Float64(basics.road.nlanes - lane)
end
function _get(::Features.Feature_HAS_LANE_L, basics::FeatureExtractBasics, carind::Int, frameind::Int)

	Float64(_get(N_LANE_L, basics, carind, frameind) > 0.0)
end
function _get(::Features.Feature_N_LANE_R, basics::FeatureExtractBasics, carind::Int, frameind::Int)

	get(CL, basics, carind, frameind) - 1.0
end
function _get(::Features.Feature_HAS_LANE_R, basics::FeatureExtractBasics, carind::Int, frameind::Int)

	Float64(_get(N_LANE_R, basics, carind, frameind) > 0.0)
end
function _get(::Features.Feature_Acc, basics::FeatureExtractBasics, carind::Int, frameind::Int)
	if frameind <= 1
		return 0.0
	end

	curr = get(SPEED, basics, carind, frameind)
	past = get(SPEED, basics, carind, frameind-1)
	(curr - past) / basics.sec_per_frame
end
function _get(::Features.Feature_AccFx, basics::FeatureExtractBasics, carind::Int, frameind::Int)
	if frameind <= 1
		return 0.0
	end

	curr = get(VELFX, basics, carind, frameind)
	past = get(VELFX, basics, carind, frameind-1)
	(curr - past) / basics.sec_per_frame
end
function _get(::Features.Feature_AccFy, basics::FeatureExtractBasics, carind::Int, frameind::Int)
	if frameind <= 1
		return 0.0
	end

	curr = get(VELFY, basics, carind, frameind)
	past = get(VELFY, basics, carind, frameind-1)
	(curr - past) / basics.sec_per_frame
end

function get(::Features.Feature_A_REQ_StayInLane, basics::FeatureExtractBasics, carind::Int, frameind::Int)
	velFy = get(VELFY, basics, carind, frameind)
	d_mr = get(D_MR, basics, carind, frameind)
	if d_mr > 0.0
		return -min(0.5velFy*velFy / d_mr, Features.THRESHOLD_A_REQ)
	end
	d_ml = get(D_ML, basics, carind, frameind)
	d_ml > 0.0 ?  -min(0.5velFy*velFy / d_ml, Features.THRESHOLD_A_REQ) : NA_ALIAS
end

# FRONT
	function _get(::Features.Feature_IndFront, basics::FeatureExtractBasics, carind::Int, frameind::Int)

		baseind = calc_logindexbase(carind)
		mylane = get(CL, basics, carind, frameind)
		myFx = basics.simlog[frameind, baseind+LOG_COL_X]
		myFy = basics.simlog[frameind, baseind+LOG_COL_Y]

		frontcar_ind = 0
		frontcar_dist = Inf
		for i = 1 : get_ncars(basics.simlog)
			if i == carind
				continue
			end

			baseind2 = calc_logindexbase(i)
			dy = abs(basics.simlog[frameind, baseind2+LOG_COL_Y] - myFy)
			dlane = get(CL, basics, i, frameind) - mylane
			if isapprox(dlane, 0.0) || dy < Features.THRESHOLD_DY_CONSIDERED_IN_FRONT
				dx = basics.simlog[frameind, baseind2+LOG_COL_X] - myFx
				if 0 < dx < frontcar_dist
					frontcar_dist, frontcar_ind = dx, i
				end
			end
		end

		if frontcar_ind != 0
			@assert(frontcar_ind != carind)

			# compute extra stats
			myϕ  = basics.simlog[frameind, baseind+LOG_COL_ϕ]
			myv  = basics.simlog[frameind, baseind+LOG_COL_V]
			myVx = myv * cos(myϕ)
			myVy = myv * sin(myϕ)

			baseind2 = calc_logindexbase(frontcar_ind)
			othFy = basics.simlog[frameind, baseind2+LOG_COL_Y]
			othϕ  = basics.simlog[frameind, baseind2+LOG_COL_ϕ]
			othv  = basics.simlog[frameind, baseind2+LOG_COL_V]
			othVx = othv * cos(othϕ)
			othVy = othv * sin(othϕ)

			basics[(carind, frameind, :has_front)] = 1.0
			basics[(carind, frameind, :d_x_front)] = frontcar_dist
			basics[(carind, frameind, :d_y_front)] = othFy - myFy
			basics[(carind, frameind, :v_x_front)] = othVx - myVx
			basics[(carind, frameind, :v_y_front)] = othVy - myVy
			return Float64(frontcar_ind)
		end

		basics[(carind, frameind, :has_front)] = 0.0
		basics[(carind, frameind, :d_x_front)] = Features.NA_ALIAS
		basics[(carind, frameind, :d_y_front)] = Features.NA_ALIAS
		basics[(carind, frameind, :v_x_front)] = Features.NA_ALIAS
		basics[(carind, frameind, :v_y_front)] = Features.NA_ALIAS
		return Features.NA_ALIAS
	end
	function _get(::Features.Feature_HAS_FRONT, basics::FeatureExtractBasics, carind::Int, frameind::Int)
		get(INDFRONT, basics, carind, frameind)
		basics[(carind, frameind, :has_front)]
	end
	function _get(::Features.Feature_D_X_FRONT, basics::FeatureExtractBasics, carind::Int, frameind::Int)
		get(INDFRONT, basics, carind, frameind)
		basics[(carind, frameind, :d_x_front)]
	end
	function _get(::Features.Feature_D_Y_FRONT, basics::FeatureExtractBasics, carind::Int, frameind::Int)
		get(INDFRONT, basics, carind, frameind)
		basics[(carind, frameind, :d_y_front)]
	end
	function _get(::Features.Feature_V_X_FRONT, basics::FeatureExtractBasics, carind::Int, frameind::Int)
		get(INDFRONT, basics, carind, frameind)
		basics[(carind, frameind, :v_x_front)]
	end
	function _get(::Features.Feature_V_Y_FRONT, basics::FeatureExtractBasics, carind::Int, frameind::Int)
		get(INDFRONT, basics, carind, frameind)
		basics[(carind, frameind, :v_y_front)]
	end
	function _get(::Features.Feature_TTC_X_FRONT, basics::FeatureExtractBasics, carind::Int, frameind::Int)

		ind_front = get(INDFRONT, basics, carind, frameind)
		if ind_front == NA_ALIAS
			return NA_ALIAS
		end

		dx = get(D_X_FRONT, basics, carind, frameind) # distance between cars
		dv = get(V_X_FRONT, basics, carind, frameind) # v_front - v_back

		if dv >= 0.0 # they are pulling away; we are good
			return NA_ALIAS
		end

		min(-dx / dv, Features.THRESHOLD_TIME_TO_COLLISION)
	end
	function _get(::Features.Feature_A_REQ_FRONT, basics::FeatureExtractBasics, carind::Int, frameind::Int)

		ind_front = get(INDFRONT, basics, carind, frameind)
		if ind_front == NA_ALIAS
			return NA_ALIAS
		end

		dx = get(D_X_FRONT, basics, carind, frameind) # distance between cars
		dv = get(V_X_FRONT, basics, carind, frameind) # v_front - v_back

		if dv >= 0.0 # they are pulling away; we are good
			return NA_ALIAS
		end

		-min(dv*dv / (2dx), Features.THRESHOLD_A_REQ)
	end
	function _get(::Features.Feature_TimeGap_X_FRONT, basics::FeatureExtractBasics, carind::Int, frameind::Int)

		ind_front = get(INDFRONT, basics, carind, frameind)
		if ind_front == NA_ALIAS
			return Features.THRESHOLD_TIMEGAP
		end

		dx = get(D_X_FRONT, basics, carind, frameind) # distance between cars
		 v = get(VELFX,     basics, carind, frameind)

		if v <= 0.0
			return Features.THRESHOLD_TIMEGAP
		end

		min(dx / v, Features.THRESHOLD_TIMEGAP)
	end

# REAR
	function _get(::Features.Feature_IndRear, basics::FeatureExtractBasics, carind::Int, frameind::Int)

		baseind = calc_logindexbase(carind)
		mylane = get(CL, basics, carind, frameind)
		myFx = basics.simlog[frameind, baseind+LOG_COL_X]
		myFy = basics.simlog[frameind, baseind+LOG_COL_Y]

		rearcar_ind = 0
		rearcar_dist = Inf
		for i = 1 : get_ncars(basics.simlog)
			if i == carind
				continue
			end

			baseind2 = calc_logindexbase(i)
			dy = abs(basics.simlog[frameind, baseind2+LOG_COL_Y] - myFy)
			dlane = get(CL, basics, i, frameind) - mylane
			if isapprox(dlane, 0.0) || dy < Features.THRESHOLD_DY_CONSIDERED_IN_FRONT
				dx = basics.simlog[frameind, baseind2+LOG_COL_X] - myFx
				if rearcar_dist < dx < 0
					rearcar_dist, rearcar_ind = dx, i
				end
			end
		end

		if rearcar_ind != 0
			@assert(rearcar_ind != carind)

			# compute extra stats
			myϕ  = basics.simlog[frameind, baseind+LOG_COL_ϕ]
			myv  = basics.simlog[frameind, baseind+LOG_COL_V]
			myVx = myv * cos(myϕ)
			myVy = myv * sin(myϕ)

			baseind2 = calc_logindexbase(rearcar_ind)
			othϕ = basics.simlog[frameind, baseind2+LOG_COL_ϕ]
			othv = basics.simlog[frameind, baseind2+LOG_COL_V]
			othVx = othv * cos(othϕ)
			othVy = othv * sin(othϕ)
			othFy = basics.simlog[frameind, baseind2+LOG_COL_Y]

			basics[(carind, frameind, :has_rear)] = 1.0
			basics[(carind, frameind, :d_x_rear)] = abs(rearcar_dist)
			basics[(carind, frameind, :d_y_rear)] = othFy - myFy
			basics[(carind, frameind, :v_x_rear)] = othVx - myVx
			basics[(carind, frameind, :v_y_rear)] = othVy - myVy
			return Float64(rearcar_ind)
		end

		basics[(carind, frameind, :has_rear)] = 0.0
		basics[(carind, frameind, :d_x_rear)] = Features.NA_ALIAS
		basics[(carind, frameind, :d_y_rear)] = Features.NA_ALIAS
		basics[(carind, frameind, :v_x_rear)] = Features.NA_ALIAS
		basics[(carind, frameind, :v_y_rear)] = Features.NA_ALIAS
		return Features.NA_ALIAS
	end
	function _get(::Features.Feature_HAS_REAR, basics::FeatureExtractBasics, carind::Int, frameind::Int)
		get(INDREAR, basics, carind, frameind)
		basics[(carind, frameind, :has_rear)]
	end
	function _get(::Features.Feature_D_X_REAR, basics::FeatureExtractBasics, carind::Int, frameind::Int)
		get(INDREAR, basics, carind, frameind)
		basics[(carind, frameind, :d_x_rear)]
	end
	function _get(::Features.Feature_D_Y_REAR, basics::FeatureExtractBasics, carind::Int, frameind::Int)
		get(INDREAR, basics, carind, frameind)
		basics[(carind, frameind, :d_y_rear)]
	end
	function _get(::Features.Feature_V_X_REAR, basics::FeatureExtractBasics, carind::Int, frameind::Int)
		get(INDREAR, basics, carind, frameind)
		basics[(carind, frameind, :v_x_rear)]
	end
	function _get(::Features.Feature_V_Y_REAR, basics::FeatureExtractBasics, carind::Int, frameind::Int)
		get(INDREAR, basics, carind, frameind)
		basics[(carind, frameind, :v_y_rear)]
	end
	function _get(::Features.Feature_TTC_X_REAR, basics::FeatureExtractBasics, carind::Int, frameind::Int)

		ind_rear = get(INDREAR, basics, carind, frameind)
		if ind_rear == NA_ALIAS
			return NA_ALIAS
		end

		dx = get(D_X_FRONT, basics, carind, frameind) # distance between cars
		dv = get(V_X_FRONT, basics, carind, frameind) # v_front - v_back

		if dv <= 0.0
			return NA_ALIAS
		end

		if dv <= 0.0
			return NA_ALIAS
		end

		min(dx / dv, THRESHOLD_TIME_TO_COLLISION)
	end
	function _get( ::Features.Feature_A_REQ_REAR, basics::FeatureExtractBasics, carind::Int, frameind::Int)

		ind_rear = get(INDREAR, basics, carind, frameind)
		if ind_rear == NA_ALIAS
			return NA_ALIAS
		end

		dx = get(D_X_REAR, basics, carind, frameind) # distance between cars
		dv = get(V_X_REAR, basics, carind, frameind) # v_rear - v_back

		if dv <= 0.0 # they are pulling away; we are good
			return NA_ALIAS
		end

		min(dv*dv / (2dx), Features.THRESHOLD_A_REQ)
	end
	function _get(::Features.Feature_Timegap_X_REAR, basics::FeatureExtractBasics, carind::Int, frameind::Int)

		ind_rear = get(INDREAR, basics, carind, frameind)
		if ind_rear == NA_ALIAS
			return Features.THRESHOLD_TIMEGAP
		end

		dx = get(D_X_REAR, basics, carind, frameind) # distance between cars
		 v = get(VELFX,    basics, carind, frameind)

		if v <= 0.0
			return Features.THRESHOLD_TIMEGAP
		end

		min(dx / v, Features.THRESHOLD_TIMEGAP)
	end

# LEFT
	function _get(::Features.Feature_IndLeft, basics::FeatureExtractBasics, carind::Int, frameind::Int)

		baseind = calc_logindexbase(carind)
		mylane = get(CL, basics, carind, frameind)
		myFx = basics.simlog[frameind, baseind+LOG_COL_X]
		myFy = basics.simlog[frameind, baseind+LOG_COL_Y]
		myϕ  = basics.simlog[frameind, baseind+LOG_COL_ϕ]
		myV  = basics.simlog[frameind, baseind+LOG_COL_V]
		myVx = myV * cos(myϕ)
		myVy = myV * sin(myϕ)

		d_y_left = 0.0
		v_x_left = 0.0
		v_y_left = 0.0
		yaw_left = 0.0

		leftcar_ind = 0
		leftcar_dist = Inf
		for i = 1 : get_ncars(basics.simlog)
			if i == carind
				continue
			end

			baseind2 = calc_logindexbase(i)
			dy = abs(basics.simlog[frameind, baseind2+LOG_COL_Y] - myFy)
			their_lane = get(CL, basics, i, frameind) - mylane
			if isapprox(their_lane, mylane+1)
				dx = basics.simlog[frameind, baseind2+LOG_COL_X] - myFx
				if abs(dx) < abs(leftcar_dist)
					leftcar_ind = i
					leftcar_dist = dx
					d_y_left = basics.simlog[frameind, baseind2+LOG_COL_Y] - myFy
					v_left   = basics.simlog[frameind, baseind2+LOG_COL_V]
					yaw_left = basics.simlog[frameind, baseind2+LOG_COL_ϕ]
					v_x_left = v_left * cos(yaw_left) - myVx
					v_y_left = v_left * sin(yaw_left) - myVy
				end
			end
		end

		if leftcar_ind != 0
			@assert(leftcar_ind != carind)

			basics[(carind, frameind, :d_x_left)] = leftcar_dist
			basics[(carind, frameind, :d_y_left)] = d_y_left
			basics[(carind, frameind, :v_x_left)] = v_x_left
			basics[(carind, frameind, :v_y_left)] = v_y_left
			basics[(carind, frameind, :yaw_left)] = yaw_left
			return Float64(leftcar_ind)
		end

		basics[(carind, frameind, :d_x_left)] = Features.NA_ALIAS
		basics[(carind, frameind, :d_y_left)] = Features.NA_ALIAS
		basics[(carind, frameind, :v_x_left)] = Features.NA_ALIAS
		basics[(carind, frameind, :v_y_left)] = Features.NA_ALIAS
		basics[(carind, frameind, :yaw_left)] = Features.NA_ALIAS
		return Features.NA_ALIAS
	end
	function _get(::Features.Feature_D_X_LEFT, basics::FeatureExtractBasics, carind::Int, frameind::Int)
		get(INDLEFT, basics, carind, frameind)
		basics[(carind, frameind, :d_x_left)]
	end
	function _get(::Features.Feature_D_Y_LEFT, basics::FeatureExtractBasics, carind::Int, frameind::Int)
		get(INDLEFT, basics, carind, frameind)
		basics[(carind, frameind, :d_y_left)]
	end
	function _get(::Features.Feature_V_X_LEFT, basics::FeatureExtractBasics, carind::Int, frameind::Int)
		get(INDLEFT, basics, carind, frameind)
		basics[(carind, frameind, :v_x_left)]
	end
	function _get(::Features.Feature_V_Y_LEFT, basics::FeatureExtractBasics, carind::Int, frameind::Int)
		get(INDLEFT, basics, carind, frameind)
		basics[(carind, frameind, :v_y_left)]
	end
	function _get(::Features.Feature_YAW_LEFT, basics::FeatureExtractBasics, carind::Int, frameind::Int)
		get(INDLEFT, basics, carind, frameind)
		basics[(carind, frameind, :yaw_left)]
	end

# RIGHT
	function _get(::Features.Feature_IndRight, basics::FeatureExtractBasics, carind::Int, frameind::Int)

		baseind = calc_logindexbase(carind)
		mylane = get(CL, basics, carind, frameind)
		myFx = basics.simlog[frameind, baseind+LOG_COL_X]
		myFy = basics.simlog[frameind, baseind+LOG_COL_Y]
		myϕ  = basics.simlog[frameind, baseind+LOG_COL_ϕ]
		myV  = basics.simlog[frameind, baseind+LOG_COL_V]
		myVx = myV * cos(myϕ)
		myVy = myV * sin(myϕ)

		d_y_right = 0.0
		v_x_right = 0.0
		v_y_right = 0.0
		yaw_right = 0.0

		rightcar_ind = 0
		rightcar_dist = Inf
		for i = 1 : get_ncars(basics.simlog)
			if i == carind
				continue
			end

			baseind2 = calc_logindexbase(i)
			dy = abs(basics.simlog[frameind, baseind2+LOG_COL_Y] - myFy)
			their_lane = get(CL, basics, i, frameind) - mylane
			if isapprox(their_lane, mylane-1)
				dx = basics.simlog[frameind, baseind2+LOG_COL_X] - myFx
				if abs(dx) < abs(rightcar_dist)
					rightcar_ind = i
					rightcar_dist = dx
					d_y_right = basics.simlog[frameind, baseind2+LOG_COL_Y] - myFy
					v_right   = basics.simlog[frameind, baseind2+LOG_COL_V]
					yaw_right = basics.simlog[frameind, baseind2+LOG_COL_ϕ]
					v_x_right = v_right * cos(yaw_right) - myVx
					v_y_right = v_right * sin(yaw_right) - myVy
				end
			end
		end

		if rightcar_ind != 0
			@assert(rightcar_ind != carind)

			basics[(carind, frameind, :d_x_right)] = rightcar_dist
			basics[(carind, frameind, :d_y_right)] = d_y_right
			basics[(carind, frameind, :v_x_right)] = v_x_right
			basics[(carind, frameind, :v_y_right)] = v_y_right
			basics[(carind, frameind, :yaw_right)] = yaw_right
			return Float64(rightcar_ind)
		end

		basics[(carind, frameind, :d_x_right)] = Features.NA_ALIAS
		basics[(carind, frameind, :d_y_right)] = Features.NA_ALIAS
		basics[(carind, frameind, :v_x_right)] = Features.NA_ALIAS
		basics[(carind, frameind, :v_y_right)] = Features.NA_ALIAS
		basics[(carind, frameind, :yaw_right)] = Features.NA_ALIAS
		return Features.NA_ALIAS
	end
	function _get(::Features.Feature_D_X_RIGHT, basics::FeatureExtractBasics, carind::Int, frameind::Int)
		get(INDRIGHT, basics, carind, frameind)
		basics[(carind, frameind, :d_x_right)]
	end
	function _get(::Features.Feature_D_Y_RIGHT, basics::FeatureExtractBasics, carind::Int, frameind::Int)
		get(INDRIGHT, basics, carind, frameind)
		basics[(carind, frameind, :d_y_right)]
	end
	function _get(::Features.Feature_V_X_RIGHT, basics::FeatureExtractBasics, carind::Int, frameind::Int)
		get(INDRIGHT, basics, carind, frameind)
		basics[(carind, frameind, :v_x_right)]
	end
	function _get(::Features.Feature_V_Y_RIGHT, basics::FeatureExtractBasics, carind::Int, frameind::Int)
		get(INDRIGHT, basics, carind, frameind)
		basics[(carind, frameind, :v_y_right)]
	end
	function _get(::Features.Feature_YAW_RIGHT, basics::FeatureExtractBasics, carind::Int, frameind::Int)
		get(INDRIGHT, basics, carind, frameind)
		basics[(carind, frameind, :yaw_right)]
	end
	function _get(::Features.Feature_A_REQ_RIGHT, basics::FeatureExtractBasics, carind::Int, frameind::Int)
		get(INDRIGHT, basics, carind, frameind)
		basics[(carind, frameind, :yaw_right)]

		ind_right = get(INDRIGHT, basics, carind, frameind)
		if ind_right == NA_ALIAS
			return NA_ALIAS
		end

		dx = get(D_X_RIGHT, basics, carind, frameind) # distance between cars
		dv = get(V_X_RIGHT, basics, carind, frameind) # v_other - v_me

		if (dx > 0.0 && dv > 0.0) || (dx < 0.0 && dv < 0.0)
			return NA_ALIAS
		end

		min(dv*dv / (2*abs(dx)), Features.THRESHOLD_A_REQ)
	end

function get_past_feature(F::AbstractFeature, basics::FeatureExtractBasics, carind::Int, frameind::Int, sec_past::Float64)
	#NOTE(tim): this function will not overflow past frame 1

	frames_back = int(ceil(sec_past / basics.sec_per_frame))
	past_frame = max(frameind-frames_back, 1)
	get(F, basics, carind, past_frame)
end
get(::Features.Feature_PastTurnrate250ms, basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_past_feature(TURNRATE, basics, carind, frameind, 0.25)
get(::Features.Feature_PastTurnrate500ms, basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_past_feature(TURNRATE, basics, carind, frameind, 0.5)
get(::Features.Feature_PastTurnrate750ms, basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_past_feature(TURNRATE, basics, carind, frameind, 0.75)
get(::Features.Feature_PastTurnrate1s,    basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_past_feature(TURNRATE, basics, carind, frameind, 1.0)

get(::Features.Feature_PastAcc250ms, basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_past_feature(ACC, basics, carind, frameind,  0.25)
get(::Features.Feature_PastAcc500ms, basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_past_feature(ACC, basics, carind, frameind,  0.5)
get(::Features.Feature_PastAcc750ms, basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_past_feature(ACC, basics, carind, frameind,  0.75)
get(::Features.Feature_PastAcc1s,    basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_past_feature(ACC, basics, carind, frameind, 1.0)

get(::Features.Feature_PastVelFy250ms, basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_past_feature(VELFY, basics, carind, frameind,  0.25)
get(::Features.Feature_PastVelFy500ms, basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_past_feature(VELFY, basics, carind, frameind,  0.5)
get(::Features.Feature_PastVelFy750ms, basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_past_feature(VELFY, basics, carind, frameind,  0.75)
get(::Features.Feature_PastVelFy1s,    basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_past_feature(VELFY, basics, carind, frameind, 1.0)

get(::Features.Feature_PastD_CL250ms, basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_past_feature(D_CL, basics, carind, frameind,  0.25)
get(::Features.Feature_PastD_CL500ms, basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_past_feature(D_CL, basics, carind, frameind,  0.5)
get(::Features.Feature_PastD_CL750ms, basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_past_feature(D_CL, basics, carind, frameind,  0.75)
get(::Features.Feature_PastD_CL1s,    basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_past_feature(D_CL, basics, carind, frameind, 1.0)

function _get(::Features.Feature_Time_Consecutive_Brake, basics::FeatureExtractBasics, carind::Int, frameind::Int)
	# scan backward until the car is no longer braking
	# NOTE(tim): returns positive time values

	const THRESHOLD_BRAKING = -0.05 # [m/s²]

	if frameind == 1
		basics[(carind, frameind, :time_consecutive_accel)] = 0.0
		return 0.0
	end

	cur_accel = get(ACC, basics, carind, frameind)
	if cur_accel > THRESHOLD_BRAKING
		return 0.0
	end

	basics[(carind, frameind, :time_consecutive_accel)] = 0.0

	past_frameind = frameind
	completed = false
	while !completed
		past_frameind -= 1
		if past_frameind > 1
			past_accel = get(ACC, basics, carind, past_frameind)
			if past_accel > THRESHOLD_BRAKING
				return basics.sec_per_frame * (frameind - past_frameind - 1)
			end

			Δt = basics.sec_per_frame * (frameind - past_frameind)
			if Δt > Features.THRESHOLD_TIMECONSECUTIVEACCEL
				return Features.THRESHOLD_TIMECONSECUTIVEACCEL
			end
		else
			return basics.sec_per_frame * (frameind - past_frameind - 1)
		end
	end

	error("INVALID CODEPATH")
	return 0.0
end
function _get(::Features.Feature_Time_Consecutive_Accel, basics::FeatureExtractBasics, carind::Int, frameind::Int)
	# scan backward until the car is no longer braking
	# NOTE(tim): returns positive time values

	const THRESHOLD_ACCELERATION = 0.05 # [m/s²]

	if frameind == 1
		basics[(carind, frameind, :time_consecutive_brake)] = 0.0
		return 0.0 # default
	end

	cur_accel = get(ACC, basics, carind, frameind)
	if cur_accel < THRESHOLD_ACCELERATION
		return 0.0
	end

	basics[(carind, frameind, :time_consecutive_brake)] = 0.0

	past_frameind = frameind
	completed = false
	while !completed
		past_frameind -= 1
		if past_frameind > 1

			past_accel = get(ACC, basics, carind, past_frameind)
			if past_accel < THRESHOLD_ACCELERATION
				return basics.sec_per_frame * (frameind - past_frameind - 1)
			end

			Δt = basics.sec_per_frame * (frameind - past_frameind)
			if Δt > Features.THRESHOLD_TIMECONSECUTIVEACCEL
				return Features.THRESHOLD_TIMECONSECUTIVEACCEL
			end
		else
			return basics.sec_per_frame * (frameind - past_frameind - 1)
		end
	end

	error("INVALID CODEPATH")
	return 0.0
end
function _get(::Features.Feature_Time_Consecutive_Throttle, basics::FeatureExtractBasics, carind::Int, frameind::Int)
	t_consec_accel = get(TIME_CONSECUTIVE_ACCEL, basics, carind, frameind)
	if t_consec_accel > 0
		return t_consec_accel
	end
	-get(TIME_CONSECUTIVE_BRAKE, basics, carind, frameind)
end

function get_max_accFx(basics::FeatureExtractBasics, carind::Int, frameind::Int, sec_past::Float64)

	frames_back = int(ceil(sec_past / basics.sec_per_frame))

	if frameind <= frames_back
		frames_back = frameind-1
	end

	retval = 0.0
	val = get(VELFX, basics, carind, frameind)
	for i = 1 : frames_back-1
		val2 = get(VELFX, basics, carind, frameind-i)
		accFx = (val2 - val) / basics.sec_per_frame
		if abs(accFx) > retval
			retval = accFx
		end
		val = val2
	end

	retval
end
get(::Features.Feature_MaxAccFx500ms, basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_max_accFx(basics, carind, frameind, 0.5)
get(::Features.Feature_MaxAccFx750ms, basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_max_accFx(basics, carind, frameind, 0.75)
get(::Features.Feature_MaxAccFx1s,    basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_max_accFx(basics, carind, frameind, 1.0)
get(::Features.Feature_MaxAccFx1500ms,basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_max_accFx(basics, carind, frameind, 1.5)
get(::Features.Feature_MaxAccFx2s,    basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_max_accFx(basics, carind, frameind, 2.0)
get(::Features.Feature_MaxAccFx2500ms,basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_max_accFx(basics, carind, frameind, 2.5)
get(::Features.Feature_MaxAccFx3s,    basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_max_accFx(basics, carind, frameind, 3.0)
get(::Features.Feature_MaxAccFx4s,    basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_max_accFx(basics, carind, frameind, 4.0)

function get_max_accFy(basics::FeatureExtractBasics, carind::Int, frameind::Int, sec_past::Float64)

	frames_back = int(ceil(sec_past / basics.sec_per_frame))

	if frameind <= frames_back
		frames_back = frameind - 1
	end

	retval = 0.0
	val = get(VELFY, basics, carind, frameind)
	for i = 1 : frames_back-1
		val2 = get(VELFY, basics, carind, frameind-i)
		accFy = (val2 - val) / basics.sec_per_frame
		if abs(accFy) > retval
			retval = accFy
		end
		val = val2
	end

	retval
end
get(::Features.Feature_MaxAccFy500ms, basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_max_accFy(basics, carind, frameind, 0.5)
get(::Features.Feature_MaxAccFy750ms, basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_max_accFy(basics, carind, frameind, 0.75)
get(::Features.Feature_MaxAccFy1s,    basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_max_accFy(basics, carind, frameind, 1.0)
get(::Features.Feature_MaxAccFy1500ms,basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_max_accFy(basics, carind, frameind, 1.5)
get(::Features.Feature_MaxAccFy2s,    basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_max_accFy(basics, carind, frameind, 2.0)
get(::Features.Feature_MaxAccFy2500ms,basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_max_accFy(basics, carind, frameind, 2.5)
get(::Features.Feature_MaxAccFy3s,    basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_max_accFy(basics, carind, frameind, 3.0)
get(::Features.Feature_MaxAccFy4s,    basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_max_accFy(basics, carind, frameind, 4.0)


function get_max_turnrate(basics::FeatureExtractBasics, carind::Int, frameind::Int, sec_past::Float64)

	frames_back = int(ceil(sec_past / basics.sec_per_frame))

	if frameind <= frames_back
		frames_back = frameind - 1
	end

	indϕ = calc_logindexbase(carind) + LOG_COL_ϕ
	retval = 0.0
	val = basics.simlog[frameind, indϕ]
	for i = 1 : frames_back-1
		val2 = basics.simlog[frameind-i, indϕ]
		turnrate = Features.deltaangle(val, val2) / basics.sec_per_frame
		if abs(turnrate) > retval
			retval = turnrate
		end
		val = val2
	end

	retval
end
get(::Features.Feature_MaxTurnRate500ms, basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_max_turnrate(basics, carind, frameind, 0.5)
get(::Features.Feature_MaxTurnRate750ms, basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_max_turnrate(basics, carind, frameind, 0.75)
get(::Features.Feature_MaxTurnRate1s,    basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_max_turnrate(basics, carind, frameind, 1.0)
get(::Features.Feature_MaxTurnRate1500ms,basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_max_turnrate(basics, carind, frameind, 1.5)
get(::Features.Feature_MaxTurnRate2s,    basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_max_turnrate(basics, carind, frameind, 2.0)
get(::Features.Feature_MaxTurnRate2500ms,basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_max_turnrate(basics, carind, frameind, 2.5)
get(::Features.Feature_MaxTurnRate3s,    basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_max_turnrate(basics, carind, frameind, 3.0)
get(::Features.Feature_MaxTurnRate4s,    basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_max_turnrate(basics, carind, frameind, 4.0)

function get_mean_accFx(basics::FeatureExtractBasics, carind::Int, frameind::Int, sec_past::Float64)

	frames_back = int(ceil(sec_past / basics.sec_per_frame ))

	if frameind < frames_back
		frames_back = frameind-1
	end

	retval = 0.0
	val = get(VELFX, basics, carind, frameind)
	for i = 1 : frames_back-1
		val2 = get(VELFX, basics, carind, frameind-i)
		retval += (val2 - val) / basics.sec_per_frame
		val = val2
	end

	retval / frames_back
end
get(::Features.Feature_MeanAccFx500ms, basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_mean_accFx(basics, carind, frameind, 0.5)
get(::Features.Feature_MeanAccFx750ms, basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_mean_accFx(basics, carind, frameind, 0.75)
get(::Features.Feature_MeanAccFx1s, basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_mean_accFx(basics, carind, frameind, 1.0)
get(::Features.Feature_MeanAccFx1500ms,basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_mean_accFx(basics, carind, frameind, 1.5)
get(::Features.Feature_MeanAccFx2s,    basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_mean_accFx(basics, carind, frameind, 2.0)
get(::Features.Feature_MeanAccFx2500ms,basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_mean_accFx(basics, carind, frameind, 2.5)
get(::Features.Feature_MeanAccFx3s,    basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_mean_accFx(basics, carind, frameind, 3.0)
get(::Features.Feature_MeanAccFx4s,    basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_mean_accFx(basics, carind, frameind, 4.0)

function get_mean_accFy(basics::FeatureExtractBasics, carind::Int, frameind::Int, sec_past::Float64)

	frames_back = int(ceil(sec_past / basics.sec_per_frame))

	if frameind < frames_back
		frames_back = frameind-1
	end

	retval = 0.0
	val = get(VELFY, basics, carind, frameind)
	for i = 1 : frames_back-1
		val2 = get(VELFY, basics, carind, frameind-i)
		retval += (val2 - val) / basics.sec_per_frame
		val = val2
	end

	retval / frames_back
end
get(::Features.Feature_MeanAccFy500ms, basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_mean_accFy(basics, carind, frameind, 0.5)
get(::Features.Feature_MeanAccFy750ms, basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_mean_accFy(basics, carind, frameind, 0.75)
get(::Features.Feature_MeanAccFy1s, basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_mean_accFy(basics, carind, frameind, 1.0)
get(::Features.Feature_MeanAccFy1500ms,basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_mean_accFy(basics, carind, frameind, 1.5)
get(::Features.Feature_MeanAccFy2s,    basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_mean_accFy(basics, carind, frameind, 2.0)
get(::Features.Feature_MeanAccFy2500ms,basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_mean_accFy(basics, carind, frameind, 2.5)
get(::Features.Feature_MeanAccFy3s,    basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_mean_accFy(basics, carind, frameind, 3.0)
get(::Features.Feature_MeanAccFy4s,    basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_mean_accFy(basics, carind, frameind, 4.0)

function get_mean_turnrate(basics::FeatureExtractBasics, carind::Int, frameind::Int, sec_past::Float64)

	frames_back = int(ceil(sec_past / basics.sec_per_frame))

	if frameind < frames_back
		frames_back = frameind-1
	end

	indϕ = calc_logindexbase(carind) + LOG_COL_ϕ
	retval = 0.0
	val = basics.simlog[frameind, indϕ]
	for i = 1 : frames_back-1
		val2 = basics.simlog[frameind-i, indϕ]
		retval += Features.deltaangle(val, val2) / basics.sec_per_frame
		val = val2
	end

	retval / frames_back
end
get(::Features.Feature_MeanTurnRate500ms, basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_mean_turnrate(basics, carind, frameind, 0.5)
get(::Features.Feature_MeanTurnRate750ms, basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_mean_turnrate(basics, carind, frameind, 0.75)
get(::Features.Feature_MeanTurnRate1s,    basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_mean_turnrate(basics, carind, frameind, 1.0)
get(::Features.Feature_MeanTurnRate1500ms,basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_mean_turnrate(basics, carind, frameind, 1.5)
get(::Features.Feature_MeanTurnRate2s,    basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_mean_turnrate(basics, carind, frameind, 2.0)
get(::Features.Feature_MeanTurnRate2500ms,basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_mean_turnrate(basics, carind, frameind, 2.5)
get(::Features.Feature_MeanTurnRate3s,    basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_mean_turnrate(basics, carind, frameind, 3.0)
get(::Features.Feature_MeanTurnRate4s,    basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_mean_turnrate(basics, carind, frameind, 4.0)

function _get_std_feature(arr::Vector{Float64}, nan_return_value::Float64 = 0.0)
	retval = std(arr)
	if isnan(retval)
		return nan_return_value
	end
	retval
end
function get_std_accFx(basics::FeatureExtractBasics, carind::Int, frameind::Int, sec_past::Float64)

	frames_back = int(ceil(sec_past / basics.sec_per_frame))

	if frameind < frames_back
		frames_back = frameind-1
	end

	if frames_back < 2
		return 0.0
	end

	retval = 0.0
	arr = zeros(Float64, frames_back)
	val = get(VELFX, basics, carind, frameind)
	for i = 1 : frames_back-1
		val2 = get(VELFX, basics, carind, frameind-i)
		arr[i] = (val2 - val) / basics.sec_per_frame
		val = val2
	end

	_get_std_feature(arr)
end
get(::Features.Feature_StdAccFx500ms, basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_std_accFx(basics, carind, frameind, 0.5)
get(::Features.Feature_StdAccFx750ms, basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_std_accFx(basics, carind, frameind, 0.75)
get(::Features.Feature_StdAccFx1s,    basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_std_accFx(basics, carind, frameind, 1.0)
get(::Features.Feature_StdAccFx1500ms,basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_std_accFx(basics, carind, frameind, 1.5)
get(::Features.Feature_StdAccFx2s,    basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_std_accFx(basics, carind, frameind, 2.0)
get(::Features.Feature_StdAccFx2500ms,basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_std_accFx(basics, carind, frameind, 2.5)
get(::Features.Feature_StdAccFx3s,    basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_std_accFx(basics, carind, frameind, 3.0)
get(::Features.Feature_StdAccFx4s,    basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_std_accFx(basics, carind, frameind, 4.0)

function get_std_accFy(basics::FeatureExtractBasics, carind::Int, frameind::Int, sec_past::Float64)

	frames_back = int(ceil(sec_past / basics.sec_per_frame))

	if frameind+1 <= frames_back
		frames_back = frameind-1
	end

	if frames_back < 2
		return 0.0
	end

	retval = 0.0
	arr = zeros(Float64, frames_back)
	val = get(VELFY, basics, carind, frameind)
	for i = 1 : frames_back-1
		val2 = get(VELFY, basics, carind, frameind-i)
		arr[i] = (val2 - val) / basics.sec_per_frame
		val = val2
	end

	_get_std_feature(arr)
end
get(::Features.Feature_StdAccFy250ms, basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_std_accFy(basics, carind, frameind, 0.25)
get(::Features.Feature_StdAccFy500ms, basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_std_accFy(basics, carind, frameind, 0.5)
get(::Features.Feature_StdAccFy750ms, basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_std_accFy(basics, carind, frameind, 0.75)
get(::Features.Feature_StdAccFy1s,    basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_std_accFy(basics, carind, frameind, 1.0)
get(::Features.Feature_StdAccFy1500ms,basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_std_accFy(basics, carind, frameind, 1.5)
get(::Features.Feature_StdAccFy2s,    basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_std_accFy(basics, carind, frameind, 2.0)
get(::Features.Feature_StdAccFy2500ms,basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_std_accFy(basics, carind, frameind, 2.5)
get(::Features.Feature_StdAccFy3s,    basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_std_accFy(basics, carind, frameind, 3.0)
get(::Features.Feature_StdAccFy4s,    basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_std_accFy(basics, carind, frameind, 4.0)


function get_std_turnrate(basics::FeatureExtractBasics, carind::Int, frameind::Int, sec_past::Float64)

	frames_back = int(ceil(sec_past / basics.sec_per_frame))

	if frameind < frames_back
		frames_back = frameind-1
	end

	if frames_back < 2
		return 0.0
	end

	indϕ = calc_logindexbase(carind) + LOG_COL_ϕ
	retval = 0.0
	arr = zeros(Float64, frames_back)
	val = get(TURNRATE, basics, carind, frameind)
	for i = 1 : frames_back-1
		val2 = get(TURNRATE, basics, carind, frameind-i)
		arr[i] = (val2 - val) / basics.sec_per_frame
		val = val2
	end

	_get_std_feature(arr)
end
get(::Features.Feature_StdTurnRate250ms, basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_std_turnrate(basics, carind, frameind, 0.25)
get(::Features.Feature_StdTurnRate500ms, basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_std_turnrate(basics, carind, frameind, 0.5)
get(::Features.Feature_StdTurnRate750ms, basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_std_turnrate(basics, carind, frameind, 0.75)
get(::Features.Feature_StdTurnRate1s,    basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_std_turnrate(basics, carind, frameind, 1.0)
get(::Features.Feature_StdTurnRate1500ms,basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_std_turnrate(basics, carind, frameind, 1.5)
get(::Features.Feature_StdTurnRate2s,    basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_std_turnrate(basics, carind, frameind, 2.0)
get(::Features.Feature_StdTurnRate2500ms,basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_std_turnrate(basics, carind, frameind, 2.5)
get(::Features.Feature_StdTurnRate3s,    basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_std_turnrate(basics, carind, frameind, 3.0)
get(::Features.Feature_StdTurnRate4s,    basics::FeatureExtractBasics, carind::Int, frameind::Int) =
	get_std_turnrate(basics, carind, frameind, 4.0)