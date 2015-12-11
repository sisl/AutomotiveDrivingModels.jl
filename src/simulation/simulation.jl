export  SimParams,
		DEFAULT_SIM_PARAMS,

        simulate!,
        simulate_but_terminate_if_collision!,
        propagate!,

        get_input_acceleration,
		get_input_turnrate,

        calc_sequential_moving_average,
		calc_weighted_moving_average

immutable SimParams
	sec_per_frame :: Float64
	n_euler_steps :: Int

	function SimParams(
		sec_per_frame :: Float64 = DEFAULT_SEC_PER_FRAME,
		n_euler_steps :: Int     = 10,
		)

		@assert(sec_per_frame > 0.0)
		@assert(n_euler_steps > 0)

		new(sec_per_frame, n_euler_steps)
	end
end
const DEFAULT_SIM_PARAMS = SimParams()

function simulate!(
    runlog          :: RunLog,
    sn              :: StreetNetwork,
    behavior        :: AbstractVehicleBehavior,
    id              :: UInt,
    frame_start     :: Int,
    frame_end       :: Int;
    pdset_frames_per_sim_frame::Int = N_FRAMES_PER_SIM_FRAME,
    n_euler_steps   :: Int = 2
    )

    for frame in frame_start : pdset_frames_per_sim_frame : frame_end-1

        colset = id2colset(runlog, id, frame)
        @assert(colset != COLSET_NULL)

        action_lat, action_lon = select_action(behavior, runlog, sn, colset, frame)
        propagate!(runlog, sn, frame, colset, action_lat, action_lon,
                   pdset_frames_per_sim_frame, n_euler_steps)
    end

    runlog
end

function simulate!(
    basics          :: FeatureExtractBasicsPdSet,
    behavior        :: AbstractVehicleBehavior,
    carid           :: Int,
    validfind_start :: Int,
    validfind_end   :: Int;
    pdset_frames_per_sim_frame::Int=N_FRAMES_PER_SIM_FRAME,
    n_euler_steps   :: Int = 2
    )

    for validfind in validfind_start : pdset_frames_per_sim_frame : validfind_end-1
        carind = carid2ind(basics.pdset, carid, validfind)
        action_lat, action_lon = select_action(basics, behavior, carind, validfind)
        propagate!(basics.pdset, basics.sn, validfind, carid, action_lat, action_lon,
                   pdset_frames_per_sim_frame, n_euler_steps)
    end

    basics
end
function simulate!(
    basics          :: FeatureExtractBasicsPdSet,
    behavior_pairs  :: Vector{Tuple{AbstractVehicleBehavior,Int}}, # (behavior, carid)
    validfind_start :: Int,
    validfind_end   :: Int;
    pdset_frames_per_sim_frame::Int=N_FRAMES_PER_SIM_FRAME,
    n_euler_steps   :: Int = 2
    )

    for validfind in validfind_start : pdset_frames_per_sim_frame: validfind_end-1
        for (behavior,carid) in behavior_pairs
            if !isa(behavior, VehicleBehaviorNone)
                carind = carid2ind(basics.pdset, carid, validfind)
                action_lat, action_lon = select_action(basics, behavior, carind, validfind)
                propagate!(basics.pdset, basics.sn, validfind, carid, action_lat, action_lon,
                          pdset_frames_per_sim_frame, n_euler_steps)
            end
        end
    end

    basics
end

function simulate_but_terminate_if_collision!(
    basics          :: FeatureExtractBasicsPdSet,
    behavior_pairs  :: Vector{Tuple{AbstractVehicleBehavior,Int}}, # (behavior, carid)
    validfind_start :: Int,
    validfind_end   :: Int;

    pdset_frames_per_sim_frame::Int=N_FRAMES_PER_SIM_FRAME,
    n_euler_steps   :: Int = 2,
    carA            :: Vehicle = Vehicle(),
    carB            :: Vehicle = Vehicle(),
    cornersCarA     :: FourCorners=FourCorners(),
    cornersCarB     :: FourCorners=FourCorners(),
    )

    # Returns whether or not a collison occurred
    # NOTE(tim): assumes no collision in current frame

    for validfind in validfind_start : pdset_frames_per_sim_frame: validfind_end-1
        for (behavior,carid) in behavior_pairs
            if !isa(behavior, VehicleBehaviorNone)
                action_lat, action_lon = select_action(basics, behavior, carid, validfind)
                propagate!(basics.pdset, basics.sn, validfind, carid, action_lat, action_lon,
                          pdset_frames_per_sim_frame, n_euler_steps)
            end
        end

        if has_intersection(basics.pdset, validfind+1, validfind+pdset_frames_per_sim_frame,
                            carA, carB, cornersCarA, cornersCarB)
            return true
        end
    end

    false
end

# TODO(tim): make this a functuion on the target feature?
get_input_acceleration(action_lon::Float64) = action_lon
function get_input_turnrate(action_lat::Float64, ϕ::Float64)
    phi_des = action_lat
    (phi_des - ϕ)*Features.KP_DESIRED_ANGLE
end

function _propagate_one_pdset_frame!(
    pdset         :: PrimaryDataset,
    sn            :: StreetNetwork,
    validfind     :: Int,
    carid         :: Int,
    action_lat    :: Float64,
    action_lon    :: Float64,
    n_euler_steps :: Int,
    )

    validfind_fut = jumpframe(pdset, validfind, 1)
    @assert(validfind_fut != 0)

    Δt = Trajdata.get_elapsed_time(pdset, validfind, validfind_fut)
    δt = Δt / n_euler_steps

    carind = carid2ind(pdset, carid, validfind)

    if !Trajdata.idinframe(pdset, carid, validfind_fut)
        add_car_to_validfind!(pdset, carid, validfind_fut)
    end
    carind_fut = carid2ind(pdset, carid, validfind_fut)

    inertial = get_inertial(pdset, carind, validfind)
    x = inertial.x
    y = inertial.y
    θ = inertial.θ

    s = d = 0.0
    ϕ = get(pdset, :posFyaw, carind, validfind)
    v = get_speed(pdset, carind, validfind)

    for j = 1 : n_euler_steps

        a = get_input_acceleration(action_lon)
        ω = get_input_turnrate(action_lat, ϕ)

        v += a*δt
        θ += ω*δt
        x += v*cos(θ)*δt
        y += v*sin(θ)*δt

        proj = project_point_to_streetmap(x, y, sn)
        @assert(proj.successful)

        ptG = proj.footpoint
        s, d, ϕ = pt_to_frenet_xyy(ptG, x, y, θ)
    end

    proj = project_point_to_streetmap(x, y, sn)
    @assert(proj.successful)

    ptG = proj.footpoint
    lane = proj.lane
    seg = get_segment(sn, lane.id)
    d_end = distance_to_lane_end(sn, seg, lane.id.lane, proj.extind)

    if carind == CARIND_EGO
        frameind_fut = validfind2frameind(pdset, validfind_fut)
        sete!(pdset, :posGx, frameind_fut, x)
        sete!(pdset, :posGy, frameind_fut, y)
        sete!(pdset, :posGyaw, frameind_fut, θ)

        sete!(pdset, :posFx, frameind_fut, ptG.s) # this should basically never be used
        sete!(pdset, :posFy, frameind_fut, NaN) # this should also basically never be used
        sete!(pdset, :posFyaw, frameind_fut, ϕ)

        sete!(pdset, :velFx, frameind_fut, v*cos(ϕ)) # vel along the lane
        sete!(pdset, :velFy, frameind_fut, v*sin(ϕ)) # vel perpendicular to lane

        sete!(pdset, :lanetag, frameind_fut, proj.lane.id)
        sete!(pdset, :curvature, frameind_fut, ptG.k)
        sete!(pdset, :d_cl, frameind_fut, d)

        laneid = proj.lane.id.lane
        d_merge = distance_to_lane_merge(sn, seg, laneid, proj.extind)
        d_split = distance_to_lane_split(sn, seg, laneid, proj.extind)
        sete!(pdset, :d_merge, frameind_fut, isinf(d_merge) ? NA : d_merge)
        sete!(pdset, :d_split, frameind_fut, isinf(d_split) ? NA : d_split)

        nll, nlr = StreetNetworks.num_lanes_on_sides(sn, seg, laneid, proj.extind)
        @assert(nll ≥ 0)
        @assert(nlr ≥ 0)
        sete!(pdset, :nll, frameind_fut, nll)
        sete!(pdset, :nlr, frameind_fut, nlr)

        lane_width_left, lane_width_right = marker_distances(sn, seg, laneid, proj.extind)
        sete!(pdset, :d_mr, frameind_fut, (d <  lane_width_left)  ?  lane_width_left - d  : Inf)
        sete!(pdset, :d_ml, frameind_fut, (d > -lane_width_right) ?  d - lane_width_right : Inf)
    else

        setc!(pdset, :posGx, carind_fut, validfind_fut, x)
        setc!(pdset, :posGy, carind_fut, validfind_fut, y)
        setc!(pdset, :posGyaw, carind_fut, validfind_fut, θ)

        setc!(pdset, :posFx, carind_fut, validfind_fut, ptG.s) # this should basically never be used
        setc!(pdset, :posFy, carind_fut, validfind_fut, NaN) # this should also basically never be used
        setc!(pdset, :posFyaw, carind_fut, validfind_fut, ϕ)

        setc!(pdset, :velFx, carind_fut, validfind_fut, v*cos(ϕ)) # vel along the lane
        setc!(pdset, :velFy, carind_fut, validfind_fut, v*sin(ϕ)) # vel perpendicular to lane

        setc!(pdset, :lanetag,   carind_fut, validfind_fut, proj.lane.id)
        setc!(pdset, :curvature, carind_fut, validfind_fut, ptG.k)
        setc!(pdset, :d_cl,      carind_fut, validfind_fut, d)

        laneid = proj.lane.id.lane
        d_merge = distance_to_lane_merge(sn, seg, laneid, proj.extind)
        d_split = distance_to_lane_split(sn, seg, laneid, proj.extind)
        setc!(pdset, :d_merge, carind_fut, validfind_fut, isinf(d_merge) ? NA : d_merge)
        setc!(pdset, :d_split, carind_fut, validfind_fut, isinf(d_split) ? NA : d_split)

        nll, nlr = num_lanes_on_sides(sn, seg, laneid, proj.extind)
        @assert(nll ≥ 0)
        @assert(nlr ≥ 0)
        setc!(pdset, :nll, carind_fut, validfind_fut, nll)
        setc!(pdset, :nlr, carind_fut, validfind_fut, nlr)

        lane_width_left, lane_width_right = marker_distances(sn, seg, laneid, proj.extind)
        setc!(pdset, :d_mr, carind_fut, validfind_fut, (d <  lane_width_left)  ?  lane_width_left - d  : Inf)
        setc!(pdset, :d_ml, carind_fut, validfind_fut, (d > -lane_width_right) ?  d - lane_width_right : Inf)

        setc!(pdset, :id,        carind_fut, validfind_fut, carid)
        setc!(pdset, :t_inview,  carind_fut, validfind_fut, getc(pdset, :t_inview, carind, validfind) + Δt)
        # NOTE(tim): `trajind` is deprecated
    end

    pdset
end
function propagate!(
    pdset         :: PrimaryDataset,
    sn            :: StreetNetwork,
    validfind     :: Int,
    carid         :: Int,
    action_lat    :: Float64,
    action_lon    :: Float64,
    pdset_frames_per_sim_frame :: Int,
    n_euler_steps :: Int,
    )

    # println("\n")
    # println(pdset.df_ego[validfind, :])
    # for i = 0 : pdset_frames_per_sim_frame
    #     carind = carid2ind(pdset, carid, validfind+i)
    #     velFx = get(pdset, "velFx", carind, validfind+i)
    #     velFy = get(pdset, "velFy", carind, validfind+i)
    #     v = hypot(velFx, velFy)
    #     println("$i) v = ", v)
    # end

    for jump in 0 : pdset_frames_per_sim_frame-1
        validfind_fut = jumpframe(pdset, validfind, jump)
        @assert(validfind_fut != 0)
        _propagate_one_pdset_frame!(pdset, sn, validfind_fut, carid, action_lat, action_lon, n_euler_steps)
    end

    # carind = carid2ind(pdset, carid, validfind)
    # action_lat_after = Features._get(FUTUREDESIREDANGLE_250MS, pdset, sn, carind, validfind)
    # action_lon_after = Features._get(FUTUREACCELERATION_250MS, pdset, sn, carind, validfind)

    # println("\n")
    # for i = 0 : pdset_frames_per_sim_frame
    #     carind = carid2ind(pdset, carid, validfind+i)
    #     velFx = get(pdset, "velFx", carind, validfind+i)
    #     velFy = get(pdset, "velFy", carind, validfind+i)
    #     v = hypot(velFx, velFy)
    #     println("$i) v = ", v)
    # end

    # @printf("before: %10.6f %10.6f\n", action_lat, action_lon)
    # @printf("after:  %10.6f %10.6f\n", action_lat_after, action_lon_after)
    # println(pdset.df_ego[validfind, :])
    # println(pdset.df_ego[validfind+5, :])
    # println("\n")
    # exit()

    pdset
end

function _propagate_one_runlog_frame!(
    runlog        :: RunLog,
    sn            :: StreetNetwork,
    frame         :: Int,
    colset        :: UInt,
    action_lat    :: Float64,
    action_lon    :: Float64,
    n_euler_steps :: Int,
    )

    frame_fut = frame + 1
    Δt = RunLogs.get_elapsed_time(runlog, frame, frame_fut)
    δt = Δt / n_euler_steps

    inertial = get(runlog, colset, frame, :inertial)::VecSE2
    x = inertial.x
    y = inertial.y
    θ = inertial.θ

    s = d = 0.0
    ϕ = ϕₒ = (get(runlog, colset, frame, :frenet)::VecSE2).θ

    rates = get(runlog, colset, frame, :ratesB)::VecSE2
    v = sqrt(rates.x*rates.x + rates.y*rates.y)

    for i = 1 : n_euler_steps

        a = get_input_acceleration(action_lon)
        ω = get_input_turnrate(action_lat, ϕ)

        v += a*δt
        θ += ω*δt
        x += v*cos(θ)*δt
        y += v*sin(θ)*δt

        proj = project_point_to_streetmap(x, y, sn)
        @assert(proj.successful)
        s, d, ϕ = pt_to_frenet_xyy(proj.footpoint, x, y, θ)
    end

    proj = project_point_to_streetmap(x, y, sn)
    @assert(proj.successful)
    s, d, ϕ = pt_to_frenet_xyy(proj.footpoint, x, y, θ)

    colset_fut = get(runlog, colset, frame, :next_colset)::UInt
    if colset_fut == NULL
        # automatically insert car into future frame if necessary
        id = colset2id(runlog, colset, frame)
        colset_fut = get_first_vacant_colset!(runlog, id, frame_fut)
    end

    RunLogs.set!(runlog, colset_fut, frame_fut,
         get(runlog, colset, frame, :id)::UInt,
         VecSE2(x, y, θ), # inertial
         VecSE2(s, d, ϕ), # frenet
         VecSE2(v*cos(θ), v*sin(θ), (ϕ - ϕₒ)/Δt), # ratesB
         proj.extind,
         proj.footpoint,
         proj.lane.id,
         COLSET_NULL,
         COLSET_NULL,
         get(runlog, colset, frame, :behavior)::UInt16
    )

    RunLogs.set!(runlog, colset_fut, frame_fut, :colset_front,
        RunLogs.calc_front_vehicle_colset(runlog, sn, colset_fut, frame_fut))
    RunLogs.set!(runlog, colset_fut, frame_fut, :colset_rear,
        RunLogs.calc_rear_vehicle_colset(runlog, sn, colset_fut, frame_fut))

    runlog
end
function propagate!(
    runlog        :: RunLog,
    sn            :: StreetNetwork,
    frame         :: Int,
    colset        :: UInt,
    action_lat    :: Float64,
    action_lon    :: Float64,
    pdset_frames_per_sim_frame :: Int,
    n_euler_steps :: Int,
    )

    for jump in 0 : pdset_frames_per_sim_frame-1
        frame_fut = frame + jump
        _propagate_one_runlog_frame!(runlog, sn, frame_fut, colset, action_lat, action_lon, n_euler_steps)
    end

    runlog
end

# TODO(tim): move to elsewhere?
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

