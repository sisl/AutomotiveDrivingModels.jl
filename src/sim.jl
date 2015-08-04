export  SimParams,
		DEFAULT_SIM_PARAMS,
       
        simulate!,
        propagate!,
        record_frame_values,

        isdone



immutable SimParams
	sec_per_frame :: Float64
	n_euler_steps :: Int
	extracted_feature_cache :: ExtractedFeatureCache

	function SimParams(
		sec_per_frame :: Float64 = DEFAULT_SEC_PER_FRAME,
		n_euler_steps :: Int     = 10,
		extracted_feature_cache :: ExtractedFeatureCache = ExtractedFeatureCache()
		)

		@assert(sec_per_frame > 0.0)
		@assert(n_euler_steps > 0)

		new(sec_per_frame, n_euler_steps, extracted_feature_cache)
	end
end
const DEFAULT_SIM_PARAMS = SimParams()

function simulate!{B<:AbstractVehicleBehavior}(
	simlog        :: Matrix{Float64}, # initialized appropriately
	behaviors     :: Vector{B},
	road          :: StraightRoadway,
	frameind      :: Int, # starting index within simlog
	params        :: SimParams = DEFAULT_SIM_PARAMS;
	runid         :: Int = rand(Int)
	)
	
	basics = FeatureExtractBasics(simlog, road, params.sec_per_frame, params.extracted_feature_cache, runid)
	n_euler_steps = params.n_euler_steps
	δt = basics.sec_per_frame / n_euler_steps

	numcars = get_ncars(simlog)
	@assert(length(behaviors) == numcars)

	while !isdone(simlog, frameind)

		for (carind,behavior) in enumerate(behaviors)
			if !isa(behavior, VehicleBehaviorNone)
				a, ω = select_action(basics, behavior, carind, frameind)
				logindexbase = calc_logindexbase(carind)
				propagate!(basics.simlog, frameind, logindexbase, a, ω, n_euler_steps, δt)
			end
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
    simparams :: SimParams = DEFAULT_SIM_PARAMS
    )

    for (i,simlog) in enumerate(simlogs)
        simulate!(simlog, behaviors[1:get_ncars(simlog)], road, history, simparams, runid = i)
    end
    simlogs
end

function record_frame_values!(
	simlog::Matrix{Float64},
	frameind::Int,
	logindexbase::Int;
	bin_lat::Int = 0,
	bin_lon::Int = 0,
	action_lat::Float64 = 0.0, # before smoothing
	action_lon::Float64 = 0.0, # before smoothing
	logPa::Float64 = 0.0,
	logPω::Float64 = 0.0,
	em_id::Int = 0
	)

	#=
	Record extra values to the log for the given vehicle
	=#

	simlog[frameind, logindexbase + LOG_COL_BIN_LAT] = bin_lat
	simlog[frameind, logindexbase + LOG_COL_BIN_LON] = bin_lon
	simlog[frameind, logindexbase + LOG_COL_LAT_BEFORE_SMOOTHING] = action_lat
	simlog[frameind, logindexbase + LOG_COL_LON_BEFORE_SMOOTHING] = action_lon
	simlog[frameind, logindexbase + LOG_COL_logprobweight_A] = logPa
	simlog[frameind, logindexbase + LOG_COL_logprobweight_T] = logPω
	simlog[frameind, logindexbase + LOG_COL_em] = float64(em_id)


	simlog
end

function propagate!(
	simlog        :: Matrix{Float64},
	frameind      :: Int,
	logindexbase  :: Int,
	a             :: Float64,
	ω             :: Float64,
	n_euler_steps :: Int,
	δt            :: Float64,
	)

	# run physics on the given car at time frameind
	# place results in log for that car in frameind + 1

	simlog[frameind, logindexbase + LOG_COL_A] = a
	simlog[frameind, logindexbase + LOG_COL_T] = ω

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
	params        :: SimParams	
	)
	
	sec_per_frame = params.sec_per_frame
	n_euler_steps = params.n_euler_steps
	δt            = sec_per_frame / n_euler_steps

	propagate!(simlog, frameind, logindexbase, a, ω, n_euler_steps, δt)
end
function propagate!(
	simlog        :: Matrix{Float64},
	frameind      :: Int,
	a             :: Float64,
	ω             :: Float64,
	params        :: SimParams	
	)
	
	sec_per_frame = params.sec_per_frame
	n_euler_steps = params.n_euler_steps
	δt            = sec_per_frame / n_euler_steps

	for carind in get_ncars(simlog)
		propagate!(simlog, frameind, calc_logindexbase(carind), a, ω, n_euler_steps, δt)
	end

	simlog
end

isdone(simlog::Matrix{Float64}, frameind::Int) = frameind >= size(simlog, 1)

function calc_sequential_moving_average(
	vec         :: AbstractArray{Float64}, # vector of values to smooth on
	index_start :: Int,                    # the present index; value must be already populated
	history     :: Int                     # the number of values to smooth over, (≥ 1)
	)

	# Sequential Moving Average: the average of the past n results

	@assert(history ≥ 1)

	clamped_history = min(history, index_start)
	index_low = index_start - clamped_history + 1

	retval = 0.0
	for i = index_low : index_start
		retval += vec[i]
	end
	retval / clamped_history
end
function calc_weighted_moving_average(
	vec         :: AbstractArray{Float64}, # vector of values to smooth on
	index_start :: Int,                    # the present index; value must be already populated
	history     :: Int                     # the number of values to smooth over, (≥ 1)
	)

	# Weighted Moving Average: the average of the past n results weighted linearly
	# ex: (3×f₁ + 2×f₂ + 1×f₃) / (3 + 2 + 1)

	@assert(history ≥ 1)

	clamped_history = min(history, index_start)
	index_low = index_start - clamped_history + 1

	retval = 0.0
	for i = index_low : index_start
		retval += vec[i] * (i - index_low + 1)
	end
	retval / (0.5clamped_history*(clamped_history+1))
end

function _reverse_smoothing_sequential_moving_average(
	vec::AbstractArray{Float64}, # vector of values originally smoothed on; 
	                             # with the most recent value having been overwritten with the smoothed value
	index_start::Int, # the present index; value must be already populated
	history::Int # the number of values to smooth over, (≥ 1)
	)

	# If the SMA is (f₁ + f₂ + f₃ + ...) / n
	# the reverse value is f₁ = n⋅SMA - f₂ - f₃ - ...

	@assert(history ≥ 1)

	clamped_history = min(history, index_start)
	index_low = index_start - clamped_history + 1

	smoothed_result = vec[index_start]

	retval = clamped_history * smoothed_result
	for i = index_low : index_start-1
		retval -= vec[i]
	end
	retval
end
function _reverse_smoothing_weighted_moving_average(
	vec::AbstractArray{Float64}, # vector of values originally smoothed on; 
	                             # with the most recent value having been overwritten with the smoothed value
	index_start::Int, # the present index; value must be already populated
	history::Int # the number of values to smooth over, (≥ 1)
	)

	# If the WMA is (3×f₁ + 2×f₂ + 1×f₃) / (3 + 2 + 1)
	# the reverse value is f₁ = [WMA * (3 + 2 + 1) - 2×f₂ - 1×f₃] / 3

	@assert(history ≥ 1)

	clamped_history = min(history, index_start)
	index_low = index_start - clamped_history + 1

	smoothed_result = vec[index_start]

	retval = (0.5clamped_history*(clamped_history+1)) * smoothed_result
	for i = index_low : index_start-1
		retval -= vec[i] * (i - index_low + 1)
	end
	retval / clamped_history
end

