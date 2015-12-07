module PrimaryDataExtractor

using DataFrames
using HDF5, JLD

using AutomotiveDrivingModels.Vec
using AutomotiveDrivingModels.CommonTypes
using AutomotiveDrivingModels.Curves
using AutomotiveDrivingModels.StreetNetworks
using AutomotiveDrivingModels.Trajdata
using AutomotiveDrivingModels.RunLogs

include(Pkg.dir("AutomotiveDrivingModels", "src", "io", "filesystem_utils.jl"))

export
    PrimaryDataExtractionParams,
    extract_runlogs,
    gen_primary_data,
    gen_primary_data_no_smoothing

type PrimaryDataExtractionParams

    resample_rate :: Float64 # seconds between re-sampled samples (>0)
    padding_size  :: Int     # number of samples to append to either end in smoothing (≥0)

    ransac_n_iter :: Int     # number of RANSAC iterations (>0)
    ransac_window_width :: Int # [samples] (>0)
    ransac_window_overlap :: Int # [samples] (≥0)
    ransac_n_inliers_for_first_fit :: Int # (>1)

    threshold_percent_outliers_warn  :: Float64 # will warn user if higher percent outliers than this
    threshold_percent_outliers_error :: Float64 # will throw an error if higher percent outliers than this

    buffer_frames                     :: Int     # number of frames to discard on either end of a drive log (≥0)
    min_frames_ego_on_freeway         :: Int     # min frames ego must be on freeway
    threshold_lane_angle_ego          :: Float64 # [rad]
    threshold_lane_lateral_offset_ego :: Float64 # [m]
    threshold_proj_sqdist_ego         :: Float64 # [m²]

    threshold_other_frame_gap         :: Int     # threshold for splitting other cars into continous segments. Minimum # of frames we can interpolate over
    threshold_other_segment_length    :: Int     # minimum length for a continuous segment to be included for other vehicle
    threshold_percent_outliers_toss   :: Float64 # if we have a higher percent outlier than this, toss the segment

    threshold_lane_lateral_offset_other :: Float64 # [m]
    threshold_proj_sqdist_other         :: Float64 # [m²]
    threshold_lane_angle_other          :: Float64 # [rad]
    threshold_other_from_lane_ends      :: Float64 # [m] distance from lane start & end that we want to be

    frameinds :: Vector{Int} # if empty, use all of them; otherwise use [x₁:x₂, x₃:x₄, ...]
    default_control_status :: UInt16

    function PrimaryDataExtractionParams(mode::Symbol=:BoschRaw)
        self = new()

        self.resample_rate = 1/20.0 # exactly 20 Hz # TODO(tim): merge this appropriately
        self.frameinds = Int[]

        if mode == :BoschRaw
            self.padding_size  = 50
            self.ransac_n_iter = 50
            self.ransac_window_width = 20
            self.ransac_window_overlap = 5
            self.ransac_n_inliers_for_first_fit = 5
            self.threshold_percent_outliers_warn = 0.5
            self.threshold_percent_outliers_error = 2.5
            self.buffer_frames = 5
            self.min_frames_ego_on_freeway = 1200
            self.threshold_lane_angle_ego = deg2rad(30)
            self.threshold_lane_lateral_offset_ego = 2.5
            self.threshold_proj_sqdist_ego = self.threshold_lane_lateral_offset_ego * self.threshold_lane_lateral_offset_ego
            self.threshold_other_frame_gap = 5
            self.threshold_other_segment_length = 20
            self.threshold_percent_outliers_toss = 25.0
            self.threshold_lane_lateral_offset_other = 2.2
            self.threshold_proj_sqdist_other = self.threshold_lane_lateral_offset_other * self.threshold_lane_lateral_offset_other
            self.threshold_lane_angle_other = deg2rad(45)
            self.threshold_other_from_lane_ends = 1.0
            self.default_control_status = ControlStatus.UNKNOWN
        elseif mode == :PerfectSim
            self.padding_size  = 1

            # NOTE(tim): do not do ransac
            self.ransac_n_iter = 1
            self.ransac_window_width = 3
            self.ransac_window_overlap = 0
            self.ransac_n_inliers_for_first_fit = 3

            self.threshold_percent_outliers_warn = 0.5
            self.threshold_percent_outliers_error = 2.5
            self.buffer_frames = 1 # NOTE(tim): no frames removed
            self.min_frames_ego_on_freeway = 0
            self.threshold_lane_angle_ego = deg2rad(45)
            self.threshold_lane_lateral_offset_ego = 2.5
            self.threshold_proj_sqdist_ego = self.threshold_lane_lateral_offset_ego * self.threshold_lane_lateral_offset_ego
            self.threshold_other_frame_gap = 5
            self.threshold_other_segment_length = 20
            self.threshold_percent_outliers_toss = 25.0
            self.threshold_lane_lateral_offset_other = 2.2
            self.threshold_proj_sqdist_other = self.threshold_lane_lateral_offset_other * self.threshold_lane_lateral_offset_other
            self.threshold_lane_angle_other = deg2rad(45)
            self.threshold_other_from_lane_ends = 0.0
            self.default_control_status = ControlStatus.VIRES_AUTO
        end

        self
    end
end

function quat2euler{T <: Real}( quat::Array{T,1} )
    # convert a quaternion to roll-pitch-yaw

    d = norm(quat)
    w = quat[1]/d
    x = quat[2]/d
    y = quat[3]/d
    z = quat[4]/d

    roll  = atan2(y*z+w*x, 0.5-x^2-y^2)
    pitch = asin(-2*(x*z + w*y))
    yaw   = atan2(x*y+w*z, 0.5-y^2-z^2)

    return (roll, pitch, yaw)
end
function pull_samples_from_set!(to,from)
    n = length(to)
    m = length(from)
    @assert(n <= m)
    i = 1
    while i ≤ n
        candidate = from[rand(1:m)]
        if !in(candidate, to[1:i-1])
            to[i] = candidate
            i += 1
        end
    end
    return to
end
function fit_poly(
    x_arr :: AbstractVector{Float64},
    y_arr :: AbstractVector{Float64},
    )

    m = length(x_arr)
    @assert(length(y_arr) == m)
    X = [ones(Float64, m)  x_arr  x_arr.^2]
    Xp = X'
    return (Xp*X)\(Xp)*y_arr
end
function fit_error(
    x_arr :: AbstractVector{Float64},
    y_arr :: AbstractVector{Float64},
    coeffs:: AbstractVector{Float64}
    )

    #NOTE(tim): computes sum squares vertical distance from poly
    c1 = coeffs[1]
    c2 = coeffs[2]
    c3 = coeffs[3]

    m = length(x_arr)
    @assert(length(y_arr) == m)

    tot = 0.0
    for i = 1 : m
        x, y = x_arr[i], y_arr[i]
        δ = y - (c1 + x*(c2 + x*c3))
        tot += δ*δ
    end
    tot
end
function distace_to_polynomial(x::Float64, y::Float64, coeffs::AbstractVector{Float64})
    c1 = coeffs[1]
    c2 = coeffs[2]
    c3 = coeffs[3]
    abs(y - (c1 + x*(c2 + x*c3)))
end

immutable RANSAC_Result
    coefficients :: Tuple{Float64,Float64,Float64}
    n_iterations :: Int
    n_inliers    :: Int
    n_total_data :: Int
    fit_error    :: Float64
    outliers     :: Set{Int}
end
function RANSAC_polynomial(
    x_arr :: AbstractVector{Float64},
    y_arr :: AbstractVector{Float64},
    n_iter :: Integer,
    datum_fit_threshold :: Real, # threshold value for whether a data point fits model (abs distance from fit)
    n_inliers_for_fit :: Integer # number of inliers required to assert a model fits well
    )

    mean_x = mean(x_arr)
    x_arr .-= mean_x

    iterations = 0
    bestfit = [0.0,0.0,0.0] # [a0,a1,a2]
    besterr = Inf
    ninliers = 0

    m = length(x_arr)
    @assert(m == length(y_arr))
    maybeinliers = Array(Int, 3)
    all_pts = collect(1:m)

    while iterations < n_iter
        pull_samples_from_set!(maybeinliers, all_pts)
        maybemodel = fit_poly(x_arr[maybeinliers], y_arr[maybeinliers])
        allinliers = falses(m)
        for i = 1 : m
            if !in(i, maybeinliers)
                δ = distace_to_polynomial(x_arr[i], y_arr[i], maybemodel)
                allinliers[i] = (δ < datum_fit_threshold)
            else
                allinliers[i] = true # include pre-selected
            end
        end
        n_new_inliers = sum(allinliers)
        if n_new_inliers ≥ n_inliers_for_fit
            # this implies we may have found a good model
            # now test how good it is
            bettermodel = fit_poly(x_arr[allinliers], y_arr[allinliers])
            thiserr = fit_error(x_arr[allinliers], y_arr[allinliers], bettermodel) / sum(allinliers)
            if n_new_inliers > ninliers || (n_new_inliers == ninliers && thiserr < besterr)
                ninliers, bestfit, besterr = n_new_inliers, bettermodel, thiserr
            end
        end
        iterations += 1
    end

    coefficients = tuple(bestfit...)
    n_iterations = iterations
    n_total_data = m
    fiterror = besterr
    outliers = Set(filter(index->distace_to_polynomial(x_arr[index], y_arr[index], bestfit) > datum_fit_threshold, 1:m))
    n_inliers = m - length(outliers)
    RANSAC_Result(coefficients, n_iterations, n_inliers, n_total_data, fiterror, outliers)
end
function sliding_window_RANSAC(
    t_arr :: AbstractVector{Float64}, # time, sorted smallest to highest
    y_arr :: AbstractVector{Float64},
    n_iter :: Integer, # number of iterations per
    datum_fit_threshold :: Real, # threshold value for whether a data point fits model (abs distance from fit)
    n_inliers_for_fit :: Integer, # number of inliers required to assert a model fits well
    window_n_samples :: Integer, # number of samples in window
    window_n_overlap :: Integer # number of samples overlapped between windows
    )

    n = length(t_arr)
    @assert(length(y_arr) == n)
    @assert(window_n_samples > 0)
    @assert(window_n_overlap ≥ 0)

    outliers = Set{Int}()
    advancement = window_n_samples - window_n_overlap

    window_lo = 1 - advancement
    window_hi = 0
    while window_hi < n
        window_lo += advancement
        window_hi = min(n, window_lo+window_n_samples-1)
        res = RANSAC_polynomial(t_arr[window_lo:window_hi], y_arr[window_lo:window_hi], n_iter, datum_fit_threshold, n_inliers_for_fit)
        union!(outliers, collect(res.outliers)+window_lo-1)
    end

    outliers
end

function make_angle_continuous!( arr::Vector{Float64} )
    # take an array of angles in [0,2pi] and unwind it without wraparound
    # (jumps from 0 -> 2pi become 0 -> < 0 instead)
    del = 0.0
    add = [0.0, 2pi, -2pi]
    for i = 2 : length(arr)
        δ = arr[i] + del - arr[i-1]
        del += add[indmin(abs(δ + add))]
        arr[i] += del
    end
    arr
end
function make_angle_continuous!( arr::DataVector{Float64} )
    # take an array of angles in [0,2pi] and unwind it without wraparound
    # (jumps from 0 -> 2pi become 0 -> < 0 instead)

    del = 0.0
    add = [0.0, 2pi, -2pi]
    for i = 2 : length(arr)
        if !isna(arr[i]) && !isna(arr[i-1])
            δ = arr[i] + del - arr[i-1]
            del += add[indmin(abs(δ + add))]
            arr[i] += del
        end
    end
    arr
end
function mod2pi_neg_pi_to_pi( θ::Float64 )
    θ = mod2pi(θ)
    if θ > π
        θ -= 2π
    end
    θ
end
function mod2pi_neg_pi_to_pi!( arr::Vector{Float64} )
    for i = 1 : length(arr)
        arr[i] = mod2pi_neg_pi_to_pi(arr[i])
    end
    arr
end

function pad_linear(arr::Vector{Float64}, n_samples_each_side::Int)
    # NOTE(tim): extends both ends of arr using a two-point linear approximation

    n = length(arr)

    @assert(n > 1)
    @assert(n_samples_each_side ≥ 0)

    slope_start = arr[2]   - arr[1]
    slope_end   = arr[end] - arr[end-1]

    retval = Array(Float64, n + 2n_samples_each_side)
    retval[n_samples_each_side+1:n_samples_each_side+n] = arr

    ind_start = n_samples_each_side
    ind_end   = n + n_samples_each_side + 1
    for i = 1 : n_samples_each_side
        retval[ind_start] = retval[ind_start+1] - slope_start
        retval[ind_end]   = retval[ind_end-1]   + slope_end
        ind_start -= 1
        ind_end += 1
    end

    retval
end
function remove_pad(arr::Vector, n_samples_each_side::Int)
    n = length(arr)

    @assert(n ≥ 2n_samples_each_side)
    @assert(n_samples_each_side ≥ 0)

    lo = n_samples_each_side + 1
    hi = n - n_samples_each_side
    arr[lo:hi]
end

function smooth(x_arr::Vector{Float64}, y_arr::Vector{Float64}, variance::Float64, outliers::Set{Int}=Set{Int}())

    @assert(variance > 0.0)
    @assert(length(x_arr) == length(y_arr))
    @assert(length(x_arr) >  length(outliers))

    G = x -> exp(-x*x/(2*variance)) / sqrt(2pi*variance)
    threshold = 1e-9

    N = length(x_arr)
    y_arr2 = Array(Float64, N)
    for (i, x) in enumerate(x_arr)

        i_left  = i - 1 # number of indeces to the left
        i_right = N - i # number of indeces to the right
        i_lo,i_hi    = i_left < i_right ? (1,2i-1) : (2i-N,N)

        if in(i,outliers)
            w_tot = 0.0
            v_tot = 0.0
        else
            w_tot = G(x - x_arr[i])
            v_tot = w_tot*y_arr[i]
        end

        for ind = i-1 : -1 : i_lo
            w = G(x-x_arr[ind])
            if w < threshold
                break
            end
            if !in(ind, outliers)
                w_tot += w
                v_tot += w*y_arr[ind]
            end
        end
        for ind = i+1 : i_hi
            w = G(x-x_arr[ind])
            if w < threshold
                break
            end
            if !in(ind, outliers)
                w_tot += w
                v_tot += w*y_arr[ind]
            end
        end
        y_arr2[i] = v_tot / w_tot
    end
    y_arr2
end
function smooth(x_arr::Vector{Float64}, y_arr::Vector{Float64}, x_arr2::Vector{Float64}, variance::Float64, outliers::Set{Int}=Set{Int}())

    @assert(variance > 0.0)
    @assert(length(x_arr) == length(y_arr))
    @assert(length(x_arr) >  length(outliers))

    G = x -> exp(-x*x/(2*variance)) / sqrt(2pi*variance)
    threshold = 1e-9

    N = length(x_arr)
    y_arr2 = Array(Float64, length(x_arr2))
    for (i, x) in enumerate(x_arr2)

        ind_closest = indmin([abs(x-x0) for x0 in x_arr])

        i_left  = ind_closest - 1 # number of indeces to the left
        i_right = N - ind_closest # number of indeces to the right
        i_lo,i_hi    = i_left < i_right ? (1,2ind_closest-1) : (2ind_closest-N,N)

        if in(ind_closest, outliers)
            w_tot = v_tot = 0.0
        else
            w_tot = G(x - x_arr[ind_closest])
            v_tot = w_tot*y_arr[ind_closest]
        end

        for ind = ind_closest-1 : -1 : i_lo
            w = G(x-x_arr[ind])
            if w < threshold
                break
            end
            if !in(ind, outliers)
                w_tot += w
                v_tot += w*y_arr[ind]
            end
        end
        for ind = ind_closest+1 : i_hi
            w = G(x-x_arr[ind])
            if w < threshold
                break
            end
            if !in(ind, outliers)
                w_tot += w
                v_tot += w*y_arr[ind]
            end
        end
        y_arr2[i] = v_tot / w_tot
    end
    y_arr2
end
function _resample_snap_to_closest{R<:Any}(x_arr::Vector{Float64}, y_arr::DataVector{R}, x_arr2::Vector{Float64})

    n = length(x_arr)
    @assert(length(y_arr) == n)
    m = length(x_arr2)

    retval = Array(R, m)

    x_ind = 1
    x = (x_arr[x_ind] + x_arr[x_ind+1])/2

    for i = 1 : m
        x2 = x_arr2[i]

        while x2 > x && x_ind < n
            x_ind += 1
            x = (x_arr[x_ind] + x_arr[min(x_ind+1,n)])/2
        end

        retval[i] = y_arr[x_ind]
    end

    retval
end

function continuous_segments(arr::AbstractVector{Bool})

    # INPUT: boolean or bit array
    # OUTPUT: set of continuous segments
    #
    # ex: [T,T,T,F,F,T,T]
    #     [(1,3)(6,7)]

    ind = 1
    N = length(arr)
    segmentset = Tuple{Int,Int}[] # (frameind_lo, frameind_hi)
    while ind <= N
        curseg_1 = findnext(arr, true, ind)
        if curseg_1 == 0
            break
        end

        ind = curseg_1 + 1
        curseg_2 = findnext(arr, false, ind)

        if curseg_2 == 0
            curseg_2 = N
        else
            ind = curseg_2
            curseg_2 -= 1
        end

        push!(segmentset, (curseg_1, curseg_2))
    end
    segmentset
end
function near_continuous_segments(arr::Vector{Int}, tol::Int)
    # this assumes frames are evenly spaced
    # arr :: list of integer values, sorted in increasing order
    # tol :: tolerance in jump between segments
    # outputs list of segments by index of arr

    # example:
    #   arr = [1,2,3,5,10,11,12]
    #   tol = 2
    #   output: [(1,4),(5,7)]

    segmentset = Tuple{Int,Int}[] # (lo,hi)
    prev_val, prev_ind = arr[1], 1
    curseg_1 = 1
    for (ind,val) in enumerate(arr)
        Δ = val - prev_val
        @assert(Δ >= 0)
        if Δ > tol # start a new segment
            push!(segmentset, (curseg_1, prev_ind))
            curseg_1 = ind
        end
        prev_ind = ind
        prev_val = val
    end

    push!(segmentset, (curseg_1, length(arr)))
    segmentset
end

function encompasing_indeces{I<:Integer}(inds::Vector{I}, sample_time_A::Vector{Float64}, sample_time_B::Vector{Float64})
    # given a set of indeces (in order, ascending) from sample_time_A (in order, ascending)
    # find the set of indeces in sample_time_B that encompass inds

    keep = falses(length(sample_time_B))

    Ni = length(inds)
    Na = length(sample_time_A)
    Nb = length(sample_time_B)

    indB = 0

    ia = 1
    while ia ≤ Ni
        ind = convert(Int, inds[ia])
        t_lo = sample_time_A[ind]

        while ia < Ni && inds[ia+1] == ind+1
            ia += 1
            ind += 1
        end

        t_hi = sample_time_A[ind]
        ia += 1

        # next project [t_lo ↔ t_hi] onto sample_time_B

        while indB < Nb && sample_time_B[indB+1] ≤ t_lo
            indB += 1
        end

        keep[indB] = true

        while indB < Nb && sample_time_B[indB+1] ≤ t_hi
            keep[indB += 1] = true
        end

        if !isapprox(t_hi, sample_time_B[indB]) && indB < Nb
            keep[indB+=1] = true
        end
    end

    retval = Array(I, sum(keep))
    index = 0
    for (i,k) in enumerate(keep)
        if k
            retval[index+=1] = i
        end
    end

    retval
end

function load_trajdata(csvfile::AbstractString)

    file = open(csvfile, "r")
    lines = readlines(file)
    close(file)

    n_cols = length(matchall(r",", lines[1]))+1

    temp_name = tempname()*".csv"

    file = open(temp_name, "w")
    for (i,line) in enumerate(lines)

        line = replace(line, "None", "Inf")

        cols = length(matchall(r",", lines[i]))+1
        @printf(file, "%s", lines[i][1:end-1]*(","^(n_cols-cols))*"\n" )
    end
    close(file)

    df = readtable(temp_name)
    rm(temp_name)

    # rename the columns
    # ----------------------------------------
    colnames = names(df)
    rename_scheme = ((:entry, :frame), (:timings, :time), (:control_node_status, :control_status), (:global_position_x, :posGx),
        (:global_position_y, :posGy), (:global_position_z, :posGz), (:global_rotation_w, :quatw), (:global_rotation_x, :quatx),
        (:global_rotation_y, :quaty), (:global_rotation_z, :quatz), (:odom_velocity_x, :velEx), (:odom_velocity_y, :velEy),
        (:odom_velocity_z, :velEz), (:odom_acceleration_x, :accEx), (:odom_acceleration_y, :accEy), (:odom_acceleration_z, :accEz))
    for (source, target) in rename_scheme
        if in(source, colnames)
            rename!(df, source, target)
        end
    end

    carind = 0
    while haskey(df, symbol(@sprintf("car_id%d", carind)))

        rename!(df, symbol(@sprintf("car_id%d",carind)), symbol(@sprintf("id_%d",    carind)))
        rename!(df, symbol(@sprintf("ego_x%d", carind)), symbol(@sprintf("posEx_%d", carind)))
        rename!(df, symbol(@sprintf("ego_y%d", carind)), symbol(@sprintf("posEy_%d", carind)))
        rename!(df, symbol(@sprintf("v_x%d",   carind)), symbol(@sprintf("velEx_%d", carind)))
        rename!(df, symbol(@sprintf("v_y%d",   carind)), symbol(@sprintf("velEy_%d", carind)))

        carind += 1
    end
    maxcarind = carind - 1

    # replace quatx, quaty, quatz with rollG, pitchG, yawG
    # ----------------------------------------
    rpy = zeros(size(df,1), 3)
    for i = 1 : size(df,1)
        quat = [df[:quatw][i], df[:quatx][i], df[:quaty][i], df[:quatz][i]]
        rpy[i,:] = [quat2euler(quat)...]
    end
    df[:quatx] = rpy[:,1]
    df[:quaty] = rpy[:,2]
    df[:quatz] = rpy[:,3]
    rename!(df, :quatx, :rollG)
    rename!(df, :quaty, :pitchG)
    rename!(df, :quatz, :yawG)
    delete!(df, :quatw)

    # add a column for every id seen
    # for each frame, list the car index it corresponds to or 0 if it is not in the frame
    # ----------------------------------------
    idset = Int[]
    for i = 1 : size(df,1)
        for carind = 0 : maxcarind
            id = df[symbol(@sprintf("id_%d", carind))][i]
            if !isa(id, NAtype) && !in( id, idset )
                push!(idset, id)
            end
        end
    end
    for id in sort!(idset)
        df[symbol(@sprintf("has_%d", id))] = -1*ones(size(df,1))
    end
    for frame = 1 : size(df,1)
        for carind = 0 : maxcarind
            carid = df[symbol(@sprintf("id_%d", carind))][frame]
            if !isa(carid, NAtype)
                df[symbol(@sprintf("has_%d", carid))][frame] = carind
            end
        end
    end

    df
end

function _assert_valid_primarydata_extraction_params(params::PrimaryDataExtractionParams)
    @assert(params.resample_rate > 0.0)
    @assert(params.padding_size > 0)
    @assert(params.ransac_n_iter > 0)
    @assert(params.ransac_window_width > 0)
    @assert(params.ransac_window_overlap ≥ 0)
    @assert(params.ransac_n_inliers_for_first_fit > 1)
    @assert(params.threshold_percent_outliers_warn ≥ 0.0)
    @assert(params.threshold_percent_outliers_error ≥ params.threshold_percent_outliers_warn)
    @assert(params.buffer_frames ≥ 0)
    @assert(params.threshold_lane_angle_ego ≥ 0.0)
    @assert(params.threshold_lane_lateral_offset_ego ≥ 0.0)
    @assert(params.threshold_proj_sqdist_ego ≥ 0.0)
    @assert(params.threshold_other_frame_gap ≥ 0)
    @assert(params.threshold_other_segment_length > 0)
    @assert(params.threshold_percent_outliers_toss > 0.0)
    @assert(params.threshold_lane_lateral_offset_other ≥ 0.0)
    @assert(params.threshold_proj_sqdist_other ≥ 0.0)
    @assert(params.threshold_lane_angle_other > 0.0)
    @assert(params.threshold_other_from_lane_ends ≥ 0.0)
end

# import PyPlot
function gen_primary_data(trajdata::DataFrame, sn::StreetNetwork, params::PrimaryDataExtractionParams)

    _assert_valid_primarydata_extraction_params(params)

    # initial ego smoothing and outlier removal
    # ----------------------------------------

    n_initial_samples  = size(trajdata,1)

    arr_time  = convert(Array{Float64}, trajdata[:time]) .- trajdata[1, :time]::Float64

    n_resamples = floor(Int, (arr_time[end] - arr_time[1]) / params.resample_rate)
    arr_time_resampled = collect(0:n_resamples-1)*params.resample_rate + arr_time[1]
    arr_time_padded = pad_linear(arr_time, params.padding_size)
    arr_time_resampled_padded = pad_linear(arr_time_resampled, params.padding_size)

    # println("n_initial_samples: ", n_initial_samples)
    # println("initial samplerate [Hz]: ", (n_initial_samples-1)/(arr_time[end] - arr_time[1]))
    # println("                    [s]: ", (arr_time[end] - arr_time[1])/(n_initial_samples-1))
    # println("target samplerate [Hz]:  ", 1/params.resample_rate)
    # println("                   [s]:  ", params.resample_rate)
    # println("n_resamples: ", n_resamples)

    trajdata_smoothed = DataFrame()
    trajdata[:yawG] = make_angle_continuous!(convert(Array{Float64}, trajdata[:yawG]))
    for (variable, RANSAC_fit_threshold, smoothing_variance) in
        [(:posGx,  Inf,  0.5),
         (:posGy,  Inf,  0.5),
         (:yawG,   0.01, 0.5),
         (:velEx,  0.1,  0.5),
         (:velEy,  0.05, 0.5)]

        arr_orig_padded = pad_linear(convert(Array{Float64}, trajdata[variable]), params.padding_size)

        if isinf(RANSAC_fit_threshold) || params.ransac_n_inliers_for_first_fit ≤ 3
            outliers = Set{Int}()
        else
            outliers = sliding_window_RANSAC(arr_time_padded, arr_orig_padded, params.ransac_n_iter,
                            RANSAC_fit_threshold, params.ransac_n_inliers_for_first_fit,
                            params.ransac_window_width, params.ransac_window_overlap)
        end

        percent_outliers = 100.0*length(outliers) / n_initial_samples
        percent_outliers < params.threshold_percent_outliers_error || error("TOO MANY OUTLIERS ($percent_outliers \%) FOR VARIABLE $variable")
        percent_outliers < params.threshold_percent_outliers_warn  ||  warn("TOO MANY OUTLIERS ($percent_outliers \%) FOR VARIABLE $variable")

        arr_smoothed_padded = smooth(arr_time_padded, arr_orig_padded, arr_time_resampled_padded, smoothing_variance, outliers)
        trajdata_smoothed[variable] = remove_pad(arr_smoothed_padded, params.padding_size)
    end

    ##
    # Plot before and after
    ##
    # fig = PyPlot.figure(facecolor="white")
    # ax = fig[:add_subplot](111)
    # ax[:plot](convert(Vector{Float64}, trajdata[:posGx]), convert(Vector{Float64}, trajdata[:posGy]), color=blue)
    # ax[:plot](convert(Vector{Float64}, trajdata_smoothed[:posGx]), convert(Vector{Float64}, trajdata_smoothed[:posGy]), color=red)
    # ax[:set_xlabel]("posGx [m]")
    # ax[:set_ylabel]("posGy [m]")

    # fig = PyPlot.figure(facecolor="white")
    # ax = fig[:add_subplot](111)
    # ax[:plot](arr_time, convert(Vector{Float64}, trajdata[:velEx]), color=blue)
    # ax[:plot](arr_time, convert(Vector{Float64}, trajdata_smoothed[:velEx]), color=red)
    # ax[:set_xlabel]("time [s]")
    # ax[:set_ylabel]("velEx [m/s]")

    # fig = PyPlot.figure(facecolor="white")
    # ax = fig[:add_subplot](111)
    # ax[:plot](arr_time, convert(Vector{Float64}, trajdata[:velEy]), color=blue)
    # ax[:plot](arr_time, convert(Vector{Float64}, trajdata_smoothed[:velEy]), color=red)
    # ax[:set_xlabel]("time [s]")
    # ax[:set_ylabel]("velEy [m/s]")

    # println("yawG: ", extrema(convert(Vector{Float64}, trajdata[:yawG])))
    # println("velEx: ", extrema(convert(Vector{Float64}, trajdata[:velEx])))
    # println("velEy: ", extrema(convert(Vector{Float64}, trajdata[:velEy])))
    # println("")

    # initialize the DataFrame
    # ----------------------------------------

    df_ego = DataFrame(
        time           = arr_time_resampled,              # time
        frame          = collect(1:n_resamples),                 # frame
        control_status = DataArray(Int, n_resamples),     # trajdata[:control_status], # enum identifier of control status (5==AUTO)
        posGx          = trajdata_smoothed[:posGx],       # easting in the global frame
        posGy          = trajdata_smoothed[:posGy],       # northing in the global frame
        posGyaw        = trajdata_smoothed[:yawG],        # heading in the global frame
        posFx          = DataArray(Float64, n_resamples), # longitudinal distance along RHS lane centerline
        posFy          = DataArray(Float64, n_resamples), # lateral distance from RHS lane center
        posFyaw        = DataArray(Float64, n_resamples), # heading angle (average between two closest lanes)
        velFx          = DataArray(Float64, n_resamples), # longitudinal speed in lane
        velFy          = DataArray(Float64, n_resamples), # lateral speed in lane
        lanetag        = DataArray(LaneTag, n_resamples), # current lane definition
        nll            = DataArray(Int8,    n_resamples), # number of lanes to the left
        nlr            = DataArray(Int8,    n_resamples), # number of lanes to the right
        curvature      = DataArray(Float64, n_resamples), # local lane curvature
        d_cl           = DataArray(Float64, n_resamples), # lateral distance between center of car and closest centerline (true)
        d_ml           = DataArray(Float64, n_resamples), # lateral distance from left lane marker
        d_mr           = DataArray(Float64, n_resamples), # lateral distance from right lane marker
        d_merge        = DataArray(Float64, n_resamples), # distance along the road until the next merge
        d_split        = DataArray(Float64, n_resamples)  # distance along the road until the next split
    )

    # lane frame projection and feature extraction
    # --------------------------------------------

    ego_car_on_freeway = falses(n_resamples)

    if :control_status in names(trajdata)
        df_ego[:,:control_status] = _resample_snap_to_closest(arr_time, trajdata[:control_status], arr_time_resampled)
    else
        fill!(df_ego[:control_status], params.default_control_status)
    end

    for frameind = params.buffer_frames : (n_resamples-params.buffer_frames)

        posGx   = trajdata_smoothed[frameind, :posGx]
        posGy   = trajdata_smoothed[frameind, :posGy]
        posGyaw = trajdata_smoothed[frameind, :yawG ]

        proj = project_point_to_streetmap(posGx, posGy, sn)
        if proj.successful && proj.sqdist < params.threshold_proj_sqdist_ego
            ptG = proj.footpoint
            s,d,θ = pt_to_frenet_xyy(ptG, posGx, posGy, posGyaw)

            df_ego[frameind, :posFx] = s
            df_ego[frameind, :posFy] = d
            df_ego[frameind, :posFyaw] = θ

            meets_lane_lateral_offset_criterion = abs(d) < params.threshold_lane_lateral_offset_ego
            meets_lane_angle_criterion = abs(θ) < params.threshold_lane_angle_ego
            ego_car_on_freeway[frameind] = meets_lane_lateral_offset_criterion && meets_lane_angle_criterion

            println((posGx, posGy, posGyaw))
            println("($s $d $θ)  ", meets_lane_lateral_offset_criterion, "  ", meets_lane_angle_criterion)

            if ego_car_on_freeway[frameind]

                # extract specifics
                speed = hypot(trajdata_smoothed[frameind, :velEx], trajdata_smoothed[frameind, :velEy])
                df_ego[frameind, :velFx] = speed * cos(θ) # vel along the lane
                df_ego[frameind, :velFy] = speed * sin(θ) # vel perpendicular to lane

                df_ego[frameind,:lanetag] = proj.lane.id
                df_ego[frameind,:curvature] = ptG.k

                df_ego[frameind,:d_cl   ] = d::Float64

                laneid = proj.lane.id.lane
                seg = get_segment(sn, proj.lane.id)
                d_merge = distance_to_lane_merge(sn, seg, laneid, proj.extind)
                d_split = distance_to_lane_split(sn, seg, laneid, proj.extind)
                df_ego[frameind, :d_merge]  =isinf(d_merge)  ? NA : d_merge
                df_ego[frameind, :d_split] = isinf(d_split) ? NA : d_split

                nll, nlr = StreetNetworks.num_lanes_on_sides(sn, seg, laneid, proj.extind)
                @assert(nll >= 0)
                @assert(nlr >= 0)
                df_ego[frameind,:nll    ] = nll # number of lanes to the left
                df_ego[frameind,:nlr    ] = nlr # number of lanes to the right

                lane_width_left, lane_width_right = marker_distances(sn, seg, laneid, proj.extind)
                df_ego[frameind, :d_mr] = (d <  lane_width_left)  ?  lane_width_left - d  : Inf
                df_ego[frameind, :d_ml] = (d > -lane_width_right) ?  d - lane_width_right : Inf
            end
        end
    end

    # println("posFyaw: ", extrema(convert(Vector{Float64}, dropna(df_ego[:posFyaw]))))
    # println("velFx: ", extrema(convert(Vector{Float64}, dropna(df_ego[:velFx]))))
    # println("velFy: ", extrema(convert(Vector{Float64}, dropna(df_ego[:velFy]))))
    # println("")

    if !isempty(params.frameinds)

        orig_index = 1
        t_lo = arr_time[params.frameinds[orig_index]]
        t_hi = arr_time[params.frameinds[orig_index+1]]

        for sample_index = 1 : n_resamples
            t = arr_time_resampled[sample_index]

            if t > t_hi
                orig_index += 2
                if orig_index < length(params.frameinds)
                    t_lo = arr_time[params.frameinds[orig_index]]
                    t_hi = arr_time[params.frameinds[orig_index+1]]
                else
                    t_lo = Inf
                    t_hi = Inf
                end
            end

            if t < t_lo
                ego_car_on_freeway[sample_index] = false
            end
        end
    end
    # println("first: ", findfirst(ego_car_on_freeway))
    # println("last: ", length(ego_car_on_freeway)+1-findfirst(i->ego_car_on_freeway[i], length(ego_car_on_freeway):-1:1))

    # fix frames_on_freeway with params.valid_frameinds
    # if !isempty(params.valid_frameinds)
    #   len_start = sum(ego_car_on_freeway)
    #   validfinds_index = 1
    #   lo = params.valid_frameinds[validfinds_index]
    #   hi = params.valid_frameinds[validfinds_index+1]
    #   validfind = 0
    #   for frameind = 1 : n_resamples
    #       if ego_car_on_freeway[frameind]
    #           validfind += 1
    #           if validfind < lo
    #               ego_car_on_freeway[frameind] = false
    #           elseif validfind ≥ hi
    #               validfinds_index += 2
    #               if validfinds_index < length(params.valid_frameinds)+1
    #                   lo = params.valid_frameinds[validfinds_index]
    #                   hi = params.valid_frameinds[validfinds_index+1]
    #               else
    #                   lo = len_start + 1
    #                   hi = len_start + 2
    #               end
    #           end
    #       end
    #   end
    # end

    const n_frames_on_freeway = sum(ego_car_on_freeway)
    if n_frames_on_freeway < params.min_frames_ego_on_freeway
        print_with_color(:red, "EGO IS ONLY ON FREEWAY FOR $n_frames_on_freeway FRAMES!\n")
        return NaN
    end

    println(n_frames_on_freeway)

    # run post-smoothing
    # --------------------------------------------

    arr_time_on_freeway = pad_linear(arr_time_resampled[ego_car_on_freeway], params.padding_size)

    df_ego[ego_car_on_freeway, :posFyaw] = make_angle_continuous!(convert(Vector{Float64}, df_ego[ego_car_on_freeway, :posFyaw]))
    for (variable, smoothing_variance) in
        [(:posFyaw,   0.2),
         (:velFx,     0.2),
         (:velFy,     0.2),
         (:curvature, 0.2)]

        arr_orig = pad_linear(convert(Vector{Float64}, df_ego[ego_car_on_freeway, variable]), params.padding_size)
        arr_smoothed = smooth(arr_time_on_freeway, arr_orig, smoothing_variance)
        df_ego[ego_car_on_freeway, variable] = remove_pad(arr_smoothed, params.padding_size)
    end
    df_ego[ego_car_on_freeway, :posFyaw] = mod2pi_neg_pi_to_pi!(convert(Vector{Float64}, df_ego[ego_car_on_freeway, :posFyaw]))

    # println("posFyaw: ", extrema(convert(Vector{Float64}, dropna(df_ego[:posFyaw]))))
    # println("velFx: ", extrema(convert(Vector{Float64}, dropna(df_ego[:velFx]))))
    # println("velFy: ", extrema(convert(Vector{Float64}, dropna(df_ego[:velFy]))))

    # Other Car Extraction
    #  - for each continuous freeway segment
    #      - for each car id
    #            - identify frames where it too is on the freeway
    #            - pull data for all frames where it is available
    #            - maintain list of missing frames
    #            - run smoothing / interpolation same as ego
    #            - map to frenet
    # --------------------------------------------

    const N = size(df_ego, 1)

    const maxncars = get_max_num_cars(trajdata)
    const frameinds = collect(1:N)
    const freeway_frameinds = frameinds[ego_car_on_freeway]
    const freeway_frameinds_raw = encompasing_indeces(freeway_frameinds, arr_time_resampled, arr_time)
    const frameind2validfind_arr = begin
                                    arr = fill(-1, N)
                                    for (validfind,frameind) in enumerate(freeway_frameinds)
                                        arr[frameind] = validfind
                                    end
                                    arr
                                end

    # --------------------------------------------------
    # df_other contains values for each validfind indexed by carind
    # to obtain values from a carid one needs to go through dict_other_idmap, and mat_other_indmap

    df_other = DataFrame()
    add_symbol! = (str,ind,typ)->df_other[symbol(@sprintf("%s_%d",str,ind))] = DataArray(typ, n_frames_on_freeway)
    add_slot!   = (cind)->begin
                                add_symbol!("posGx",     cind, Float64)
                                add_symbol!("posGy",     cind, Float64)
                                add_symbol!("posGyaw",   cind, Float64)
                                add_symbol!("posFx",     cind, Float64)
                                add_symbol!("posFy",     cind, Float64)
                                add_symbol!("posFyaw",   cind, Float64)
                                add_symbol!("velFx",     cind, Float64)
                                add_symbol!("velFy",     cind, Float64)
                                add_symbol!("lanetag",   cind, LaneTag)
                                add_symbol!("nlr",       cind, Int8)
                                add_symbol!("nll",       cind, Int8)
                                add_symbol!("curvature", cind, Float64)
                                add_symbol!("d_cl",      cind, Float64)
                                add_symbol!("d_mr",      cind, Float64)
                                add_symbol!("d_ml",      cind, Float64)
                                add_symbol!("d_merge",   cind, Float64)
                                add_symbol!("d_split",   cind, Float64)
                                add_symbol!("id",        cind, UInt32)
                                add_symbol!("t_inview",  cind, Float64)
                                add_symbol!("trajind",   cind, UInt32)
                              end
    for cind = 0 : maxncars # NOTE: indexing starts from 0, like in Trajdata
        add_slot!(cind)
    end

    # --------------------------------------------------

    carids = get_carids(trajdata) # a Set{Int} of carids
    delete!(carids, CARID_EGO)

    dict_other_idmap = Dict{UInt32,UInt16}() # dict carid -> matind,  index for mat_other_indmap
    mat_other_indmap = fill(Int16(-1), n_frames_on_freeway, length(carids)) # [validfind,matind] -> carind, -1 if does not exist

    local_setc! = (str,carind,vind,value) -> df_other[vind, symbol(@sprintf("%s_%d", str, carind))] = value
    next_available_carind = fill(Int32(-1), n_frames_on_freeway)

    # --------------------------------------------------

    # println("1:", N)
    # println(extrema(freeway_frameinds))
    # println(extrema(freeway_frameinds_raw))

    for (matind, carid) in enumerate(carids)

        tic()
        print(carid, ": ", matind, " / ", length(carids), "  ")

        # assign a matind to this carid
        dict_other_idmap[carid] = matind

        # all freeway_frameinds where the car exists
        car_frameinds_raw = filter(frame->carid_exists(trajdata, carid, frame), freeway_frameinds_raw)
        if isempty(car_frameinds_raw)
            toc()
            continue
        end
        # println("car_frameinds_raw: ", extrema(car_frameinds_raw))
        # println(car_frameinds_raw)

        segment_count = 0
        data_arr_index = 0

        for (lo,hi) in near_continuous_segments(car_frameinds_raw, params.threshold_other_frame_gap)

            # println("\tseg: ($lo, $hi), [$(car_frameinds_raw[lo]), $(car_frameinds_raw[hi])]")

            segment_frameinds = collect(car_frameinds_raw[lo] : car_frameinds_raw[hi]) # array of all frames for this segment
            n_frames_in_seg = length(segment_frameinds)

            # println("n_frames_in_seg: ", n_frames_in_seg)

            # ------------------------------------------
            # enforce minimum segment length
            if n_frames_in_seg < params.threshold_other_segment_length
                print_with_color(:red, "skipping due to insufficient length ($n_frames_in_seg < $(params.threshold_other_segment_length))\n")
                continue
            end

            # ------------------------------------------
            # run smoothing + interpolation

            # whether car exists in frame
            car_exists = falses(n_frames_in_seg)
            car_exists[car_frameinds_raw[lo:hi]-car_frameinds_raw[lo]+1] = true
            n_frames_exist = sum(car_exists)

            # println(map(b->b ? '1' : '0', car_exists))

            # map index to carind (-1 if no exist)
            carinds_raw = map(i->Trajdata.carid2ind_or_negative_one_otherwise(trajdata, carid, segment_frameinds[i]), 1:n_frames_in_seg)

            time_obs = arr_time[car_frameinds_raw[lo:hi]] # actual measured time

            time_resampled_ind_lo = findfirst(i->arr_time_resampled[i] ≥ time_obs[1], 1:n_resamples)
            time_resampled_ind_hi = findnext(i->arr_time_resampled[i+1] > time_obs[end], 1:n_resamples-1, time_resampled_ind_lo)
            @assert(time_resampled_ind_lo != 0 && time_resampled_ind_hi != 0)
            time_resampled = arr_time_resampled[time_resampled_ind_lo:time_resampled_ind_hi]
            smoothed_frameinds = df_ego[time_resampled_ind_lo:time_resampled_ind_hi, :frame]

            # println(time_resampled_ind_lo, "  ", arr_time_resampled[time_resampled_ind_lo-1], " < ", time_obs[1], " ≤ ", arr_time_resampled[time_resampled_ind_lo])
            # println(time_resampled_ind_hi, "  ", arr_time_resampled[time_resampled_ind_hi], " ≤ ", time_obs[end], " < ", arr_time_resampled[time_resampled_ind_hi+1])
            # println(size(time_resampled))

            time_obs_padded = pad_linear(time_obs, params.padding_size)
            time_resampled_padded = pad_linear(time_resampled, params.padding_size)

            # TODO(tim): can this be optimized with pre-allocation outside of the loop?
            # NOTE(tim): this assumes zero sideslip
            data_obs = DataFrame()
            for sym in (:posGx, :posGy, :yawG, :velBx)
                data_obs[sym] = DataArray(Float64, n_frames_exist)
            end

            total = 0
            for (i,frameind) in enumerate(segment_frameinds)

                if car_exists[i]
                    total += 1
                    carind = carinds_raw[i]
                    @assert(carind != -1)

                    posEx = getc(trajdata, "posEx", carind, frameind)
                    posEy = getc(trajdata, "posEy", carind, frameind)
                    velEx = getc(trajdata, "velEx", carind, frameind) # NOTE(tim): velocity in the ego frame but pre-compensated for ego velocity
                    velEy = getc(trajdata, "velEy", carind, frameind)

                    posGx_ego = trajdata[frameind, :posGx]
                    posGy_ego = trajdata[frameind, :posGy]
                    yawG_ego  = trajdata[frameind, :yawG]

                    # posG = body2inertial(VecE2(posGx_ego, posGy_ego), VecSE2(posEx, posEy, yawG_ego))

                    # data_obs[total, :posGx] = posG.x
                    # data_obs[total, :posGy] = posG.y

                    # velG = body2inertial(VecE2(0,0), VecSE2(velEx, velEy, yawG_ego))

                    # if abs(velG) > 3.0
                    #     yawG = atan2(velG)
                    # else
                    #     yawG = yawG_ego # to fix problem with very low velocities
                    # end
                    # data_obs[total, :velBx] = abs(velG)
                    # data_obs[total, :yawG]  = yawG

                    posGx, posGy = Trajdata.ego2global(posGx_ego, posGy_ego, yawG_ego, posEx, posEy)

                    data_obs[total, :posGx] = posGx
                    data_obs[total, :posGy] = posGy

                    velGx, velGy = Trajdata.ego2global(0.0, 0.0, yawG_ego, velEx, velEy)

                    if hypot(velGx, velGy) > 3.0
                        yawG = atan2(velGy, velGx)
                    else
                        yawG = yawG_ego # to fix problem with very low velocities
                    end
                    data_obs[total, :velBx] = hypot(velGx, velGy)
                    data_obs[total, :yawG]  = yawG
                end
            end
            @assert(total == size(data_obs, 1))
            data_obs[:yawG] = make_angle_continuous!(convert(Vector{Float64}, data_obs[:yawG]))

            data_smoothed = DataFrame()
            should_toss_due_to_outliers = false
            for (variable, RANSAC_fit_threshold, smoothing_variance) in
                [(:posGx,  0.5,  0.5), # TODO(tim): tune these
                 (:posGy,  0.5,  0.5),
                 (:yawG,   0.05, 0.5),
                 (:velBx,  2.0,  0.5)] # NOTE(tim): many vehicles are sensitive here... need to investigate

                arr_orig_padded = pad_linear(convert(Vector{Float64}, data_obs[variable]), params.padding_size)

                # if length(arr_orig_padded) != length(time_obs_padded)
                    # println(length(arr_orig_padded),  "  ", length(time_obs_padded))
                # end

                if  params.ransac_n_inliers_for_first_fit ≤ 3
                    outliers = Set{Int}()
                else
                    outliers = sliding_window_RANSAC(time_obs_padded, arr_orig_padded, params.ransac_n_iter,
                                        RANSAC_fit_threshold, params.ransac_n_inliers_for_first_fit,
                                        params.ransac_window_width, params.ransac_window_overlap)
                end

                percent_outliers = 100.0*length(outliers) / n_frames_in_seg
                if percent_outliers > params.threshold_percent_outliers_toss
                    should_toss_due_to_outliers = true
                    print_with_color(:red, "skipping due to high outlier percentage in $variable ($percent_outliers > $(params.threshold_percent_outliers_toss))\n")
                    break
                end

                # println(size(time_obs_padded), "  ", size(arr_orig_padded), "   ", size(time_resampled_padded))
                # println(time_obs_padded[1], " ≤ ", time_resampled_padded[1], "     ", time_resampled_padded[end] , " ≤ ", time_obs_padded[end])
                # println(sort!(collect(outliers)))

                arr_smoothed_padded = smooth(time_obs_padded, arr_orig_padded, time_resampled_padded, smoothing_variance, outliers)
                data_smoothed[variable] = remove_pad(arr_smoothed_padded, params.padding_size)
            end
            if should_toss_due_to_outliers
                continue
            end

            inds_to_keep = find(frame->ego_car_on_freeway[frame], smoothed_frameinds)
            smoothed_frameinds = smoothed_frameinds[inds_to_keep]
            data_smoothed = data_smoothed[inds_to_keep, :]

            if size(data_smoothed, 1) < params.threshold_other_segment_length
                print_with_color(:red, "skipping due to insufficient length after smoothing ($n_frames_in_seg < $(params.threshold_other_segment_length))\n")
                continue
            end

            # ------------------------------------------
            # map to frenet frame & extract values

            segment_count += 1
            for (i,frameind) in enumerate(smoothed_frameinds)

                @assert(ego_car_on_freeway[frameind])

                posGx   = data_smoothed[i, :posGx]
                posGy   = data_smoothed[i, :posGy]
                posGyaw = data_smoothed[i, :yawG ]

                proj = project_point_to_streetmap(posGx, posGy, sn)
                if proj.successful && proj.sqdist < params.threshold_proj_sqdist_other

                    ptG = proj.footpoint
                    s,d,θ = pt_to_frenet_xyy(ptG, posGx, posGy, posGyaw)

                    laneid = convert(Int, proj.lane.id.lane)
                    seg = get_segment(sn, proj.lane.id)
                    d_end = distance_to_lane_end(sn, seg, laneid, proj.extind)

                    meets_lane_lateral_offset_criterion = abs(d) < params.threshold_lane_lateral_offset_other
                    meets_lane_angle_criterion = abs(θ) < params.threshold_lane_angle_other
                    meets_lane_end_criterion = d_end > params.threshold_other_from_lane_ends

                    # if meets_lane_lateral_offset_criterion && meets_lane_angle_criterion && meets_lane_end_criterion

                        validfind = frameind2validfind_arr[frameind]
                        carind = (next_available_carind[validfind] += 1)

                        # need to add another car slot
                        if carind > maxncars
                            maxncars += 1
                            add_slot!(maxncars)
                            carind = maxncars
                        end

                        #set mat_other_indmap to point to the carind for this car for all valid frames
                        mat_other_indmap[validfind, matind] = carind

                        local_setc!("posGx", carind, validfind, posGx)
                        local_setc!("posGy", carind, validfind, posGy)
                        local_setc!("posGyaw", carind, validfind, posGyaw)
                        local_setc!("posFx", carind, validfind, s)
                        local_setc!("posFy", carind, validfind, d)
                        local_setc!("posFyaw", carind, validfind, θ)

                        # extract specifics
                        speed = data_smoothed[i, :velBx]
                        local_setc!("velFx",     carind, validfind, speed * cos(θ)) # vel along the lane
                        local_setc!("velFy",     carind, validfind, speed * sin(θ)) # vel perpendicular to lane
                        local_setc!("lanetag",   carind, validfind, proj.lane.id::LaneTag)
                        local_setc!("curvature", carind, validfind, ptG.k)
                        local_setc!("d_cl",      carind, validfind, d::Float64)

                        d_merge = distance_to_lane_merge(sn, seg, laneid, proj.extind)
                        d_split = distance_to_lane_split(sn, seg, laneid, proj.extind)
                        local_setc!("d_merge", carind, validfind, isinf(d_merge) ? NA : d_merge)
                        local_setc!("d_split", carind, validfind, isinf(d_split) ? NA : d_split)

                        nll, nlr = StreetNetworks.num_lanes_on_sides(sn, seg, laneid, proj.extind)
                        @assert(nll >= 0)
                        @assert(nlr >= 0)
                        local_setc!("nll", carind, validfind, nll)
                        local_setc!("nlr", carind, validfind, nlr)

                        lane_width_left, lane_width_right = marker_distances(sn, seg, laneid, proj.extind)
                        local_setc!("d_mr", carind, validfind, (d <  lane_width_left)  ?  lane_width_left - d  : Inf)
                        local_setc!("d_ml", carind, validfind, (d > -lane_width_right) ?  d - lane_width_right : Inf)

                        local_setc!("id",        carind, validfind, carid)
                        local_setc!("t_inview",  carind, validfind, time_resampled[end] - time_resampled[i])
                        local_setc!("trajind",   carind, validfind, segment_count)
                    # else

                    #   print_with_color(:red, "skipping frame due to failed criteria:\nmeets lane lateral offset criterion: $meets_lane_lateral_offset_criterion\nmeets lane angle criterion: $meets_lane_angle_criterion\nmeets lane end criterion: $meets_lane_end_criterion\n")
                    #   end
                    # end
                end
            end
        end

        toc()
    end


    # println("posFyaw: ", extrema(convert(Vector{Float64}, dropna(df_ego[:posFyaw]))))
    # println("velFx: ", extrema(convert(Vector{Float64}, dropna(df_ego[:velFx]))))
    # println("velFy: ", extrema(convert(Vector{Float64}, dropna(df_ego[:velFy]))))

    dict_trajmat = Dict{UInt32,DataFrame}()
    pdset = PrimaryDataset(df_ego, df_other, dict_trajmat, dict_other_idmap, mat_other_indmap, ego_car_on_freeway)
end
function gen_primary_data_no_smoothing(trajdata::DataFrame, sn::StreetNetwork, params::PrimaryDataExtractionParams;
    carids::Union{Void, AbstractVector{Int}} = nothing, # set of carids to pull
    )

    _assert_valid_primarydata_extraction_params(params)

    nframes  = size(trajdata,1)
    ego_car_on_freeway = trues(nframes) # assume all are on freeway

    # initialize the DataFrame
    # ----------------------------------------

    df_ego = DataFrame(
        time           = (trajdata[:time] .- trajdata[1, :time])::DataArray{Float64}, # time
        frame          = collect(1:nframes),                 # frame
        control_status = DataArray(Int, nframes),     # trajdata[:control_status], # enum identifier of control status (5==AUTO)
        posGx          = trajdata[:posGx],         # easting in the global frame
        posGy          = trajdata[:posGy],         # northing in the global frame
        posGyaw        = trajdata[:yawG],          # heading in the global frame
        posFx          = DataArray(Float64, nframes), # longitudinal distance along RHS lane centerline
        posFy          = DataArray(Float64, nframes), # lateral distance from RHS lane center
        posFyaw        = DataArray(Float64, nframes), # heading angle (average between two closest lanes)
        velFx          = DataArray(Float64, nframes), # longitudinal speed in lane
        velFy          = DataArray(Float64, nframes), # lateral speed in lane
        lanetag        = DataArray(LaneTag, nframes), # current lane definition
        nll            = DataArray(Int8,    nframes), # number of lanes to the left
        nlr            = DataArray(Int8,    nframes), # number of lanes to the right
        curvature      = DataArray(Float64, nframes), # local lane curvature
        d_cl           = DataArray(Float64, nframes), # lateral distance between center of car and closest centerline (true)
        d_ml           = DataArray(Float64, nframes), # lateral distance from left lane marker
        d_mr           = DataArray(Float64, nframes), # lateral distance from right lane marker
        d_merge        = DataArray(Float64, nframes), # distance along the road until the next merge
        d_split        = DataArray(Float64, nframes)  # distance along the road until the next split
    )

    # lane frame projection and feature extraction
    # --------------------------------------------

    if :control_status in names(trajdata)
        df_ego[:,:control_status] = trajdata[:control_status]
    else
        fill!(df_ego[:control_status], params.default_control_status)
    end

    for frameind = 1 : nframes

        posGx   = trajdata[frameind, :posGx]
        posGy   = trajdata[frameind, :posGy]
        posGyaw = trajdata[frameind, :yawG ]

        proj = project_point_to_streetmap(posGx, posGy, sn)
        @assert(proj.successful)

        ptG = proj.footpoint
        s,d,θ = pt_to_frenet_xyy(ptG, posGx, posGy, posGyaw)

        df_ego[frameind, :posFx] = s
        df_ego[frameind, :posFy] = d
        df_ego[frameind, :posFyaw] = θ

        # println((posGx, posGy, posGyaw))
        # println("($s $d $θ)  ")

        # extract specifics
        speed = hypot(trajdata[frameind, :velEx], trajdata[frameind, :velEy])
        df_ego[frameind, :velFx] = speed * cos(θ) # vel along the lane
        df_ego[frameind, :velFy] = speed * sin(θ) # vel perpendicular to lane

        df_ego[frameind, :lanetag] = proj.lane.id
        df_ego[frameind, :curvature] = ptG.k
        df_ego[frameind, :d_cl] = d::Float64

        seg = get_segment(sn, proj.lane.id)
        laneid = proj.lane.id.lane
        d_merge = distance_to_lane_merge(sn, seg, laneid, proj.extind)
        d_split = distance_to_lane_split(sn, seg, laneid, proj.extind)
        df_ego[frameind, :d_merge] = isinf(d_merge)  ? NA : d_merge
        df_ego[frameind, :d_split] = isinf(d_split) ? NA : d_split

        nll, nlr = StreetNetworks.num_lanes_on_sides(sn, seg, laneid, proj.extind)
        @assert(nll ≥ 0)
        @assert(nlr ≥ 0)
        df_ego[frameind,:nll    ] = nll # number of lanes to the left
        df_ego[frameind,:nlr    ] = nlr # number of lanes to the right

        lane_width_left, lane_width_right = marker_distances(sn, seg, laneid, proj.extind)
        df_ego[frameind, :d_mr] = (d <  lane_width_left)  ?  lane_width_left - d  : Inf
        df_ego[frameind, :d_ml] = (d > -lane_width_right) ?  d - lane_width_right : Inf
    end

    # Other Car Extraction
    #  - for each continuous freeway segment
    #      - for each car id
    #            - identify frames where it too is on the freeway
    #            - pull data for all frames where it is available
    #            - maintain list of missing frames
    #            - map to frenet
    # --------------------------------------------

    const maxncars = get_max_num_cars(trajdata)

    # --------------------------------------------------
    # df_other contains values for each validfind indexed by carind
    # to obtain values from a carid one needs to go through dict_other_idmap, and mat_other_indmap

    df_other = DataFrame()
    add_symbol! = (str,ind,typ)->df_other[symbol(@sprintf("%s_%d",str,ind))] = DataArray(typ, nframes)
    add_slot!   = (cind)->begin
                                add_symbol!("posGx",     cind, Float64)
                                add_symbol!("posGy",     cind, Float64)
                                add_symbol!("posGyaw",   cind, Float64)
                                add_symbol!("posFx",     cind, Float64)
                                add_symbol!("posFy",     cind, Float64)
                                add_symbol!("posFyaw",   cind, Float64)
                                add_symbol!("velFx",     cind, Float64)
                                add_symbol!("velFy",     cind, Float64)
                                add_symbol!("lanetag",   cind, LaneTag)
                                add_symbol!("nlr",       cind, Int8)
                                add_symbol!("nll",       cind, Int8)
                                add_symbol!("curvature", cind, Float64)
                                add_symbol!("d_cl",      cind, Float64)
                                add_symbol!("d_mr",      cind, Float64)
                                add_symbol!("d_ml",      cind, Float64)
                                add_symbol!("d_merge",   cind, Float64)
                                add_symbol!("d_split",   cind, Float64)
                                add_symbol!("id",        cind, UInt32)
                                add_symbol!("t_inview",  cind, Float64)
                                add_symbol!("trajind",   cind, UInt32)
                              end
    for cind = 0 : maxncars # NOTE: indexing starts from 0, like in Trajdata
        add_slot!(cind)
    end

    # --------------------------------------------------

    if isa(carids, Void)
        carids = get_carids(trajdata) # a Set{Int} of carids
        delete!(carids, CARID_EGO)
    end

    dict_other_idmap = Dict{UInt32,UInt16}() # dict carid -> matind,  index for mat_other_indmap
    mat_other_indmap = fill(Int16(-1), nframes, length(carids)) # [validfind,matind] -> carind, -1 if does not exist

    function local_setc!(str::AbstractString, carind::Integer, validfind::Integer, value::Any)
        df_other[validfind, symbol(@sprintf("%s_%d", str, carind))] = value
    end
    next_available_carind = fill(Int32(-1), nframes)

    # --------------------------------------------------

    for (matind, carid) in enumerate(carids)

        # tic()
        # print(carid, ": ", matind, " / ", length(carids), "  ")

        # assign a matind to this carid
        dict_other_idmap[carid] = matind

        # all freeway_frameinds where the car exists
        car_frameinds_raw = filter(frame->carid_exists(trajdata, carid, frame), 1:nframes)
        if isempty(car_frameinds_raw)
            # toc()
            continue
        end
        # println("car_frameinds_raw: ", extrema(car_frameinds_raw))
        # println(car_frameinds_raw)

        segment_count = 0
        data_arr_index = 0

        for (lo,hi) in near_continuous_segments(car_frameinds_raw, params.threshold_other_frame_gap)

            # println("\tseg: ($lo, $hi), [$(car_frameinds_raw[lo]), $(car_frameinds_raw[hi])]")

            segment_frameinds = [car_frameinds_raw[lo] : car_frameinds_raw[hi]] # array of all frames for this segment
            n_frames_in_seg = length(segment_frameinds)

            # println("n_frames_in_seg: ", n_frames_in_seg)

            # ------------------------------------------
            # enforce minimum segment length
            if n_frames_in_seg < params.threshold_other_segment_length
                # print_with_color(:red, "skipping due to insufficient length ($n_frames_in_seg < $(params.threshold_other_segment_length))\n")
                continue
            end

            # TODO(tim): can this be optimized with pre-allocation outside of the loop?
            # NOTE(tim): this assumes zero sideslip
            data_obs = DataFrame(
                            posGx = DataArray(Float64, n_frames_in_seg),
                            posGy = DataArray(Float64, n_frames_in_seg),
                            yawG  = DataArray(Float64, n_frames_in_seg),
                            velBx = DataArray(Float64, n_frames_in_seg),
                            exists = falses(n_frames_in_seg),
                        )

            car_exists = falses(n_frames_in_seg)
            car_exists[car_frameinds_raw[lo:hi]-car_frameinds_raw[lo]+1] = true
            carinds_raw = map(i->Trajdata.carid2ind_or_negative_one_otherwise(trajdata, carid, segment_frameinds[i]), 1:n_frames_in_seg)

            total = 0
            for (i,frameind) in enumerate(segment_frameinds)

                if car_exists[i]
                    total += 1
                    carind = carinds_raw[i]
                    @assert(carind != -1)

                    posEx = getc(trajdata, "posEx", carind, frameind)
                    posEy = getc(trajdata, "posEy", carind, frameind)
                    velEx = getc(trajdata, "velEx", carind, frameind) # NOTE(tim): velocity in the ego frame but pre-compensated for ego velocity
                    velEy = getc(trajdata, "velEy", carind, frameind)

                    posGx_ego = trajdata[frameind, :posGx]
                    posGy_ego = trajdata[frameind, :posGy]
                    yawG_ego  = trajdata[frameind, :yawG]

                    # # posGx, posGy = Trajdata.ego2global(posGx_ego, posGy_ego, yawG_ego, posEx, posEy)
                    # posG = body2inertial(VecE2(posGx_ego, posGy_ego), VecSE2(posEx, posEy, yawG_ego))

                    # data_obs[total, :posGx] = posG.x
                    # data_obs[total, :posGy] = posG.y

                    # # velGx, velGy = Trajdata.ego2global(0.0, 0.0, yawG_ego, velEx, velEy)
                    # velG = body2inertial(VecE2(0,0), VecSE2(velEx, velEy, yawG_ego))

                    # if abs(velG) > 3.0
                    #     yawG = atan2(velG)
                    # else
                    #     yawG = yawG_ego # to fix problem with very low velocities
                    # end
                    # data_obs[total, :velBx] = abs(velG)
                    # data_obs[total, :yawG]  = yawG
                    # data_obs[total, :exists] = true

                    posGx, posGy = Trajdata.ego2global(posGx_ego, posGy_ego, yawG_ego, posEx, posEy)

                    data_obs[total, :posGx] = posGx
                    data_obs[total, :posGy] = posGy

                    velGx, velGy = Trajdata.ego2global(0.0, 0.0, yawG_ego, velEx, velEy)

                    if hypot(velGx, velGy) > 3.0
                        yawG = atan2(velGy, velGx)
                    else
                        yawG = yawG_ego # to fix problem with very low velocities
                    end
                    data_obs[total, :velBx] = hypot(velGx, velGy)
                    data_obs[total, :yawG]  = yawG
                    data_obs[total, :exists] = true
                end
            end
            # @assert(total == size(data_obs, 1))
            data_obs[:yawG] = make_angle_continuous!(data_obs[:yawG])

            # ------------------------------------------
            # map to frenet frame & extract values

            segment_count += 1
            for frameind in 1 : n_frames_in_seg

                if data_obs[frameind, :exists]

                    posGx   = data_obs[frameind, :posGx]
                    posGy   = data_obs[frameind, :posGy]
                    posGyaw = data_obs[frameind, :yawG ]

                    proj = project_point_to_streetmap(posGx, posGy, sn)
                    @assert(proj.successful)

                    ptG = proj.footpoint
                    s, d, θ = pt_to_frenet_xyy(ptG, posGx, posGy, posGyaw)

                    laneid = proj.lane.id.lane
                    seg = get_segment(sn, proj.lane.id)
                    d_end = distance_to_lane_end(sn, seg, laneid, proj.extind)

                    validfind = frameind
                    carind = (next_available_carind[validfind] += 1)

                    # need to add another car slot
                    if carind > maxncars
                        maxncars += 1
                        add_slot!(maxncars)
                        carind = maxncars
                    end

                    #set mat_other_indmap to point to the carind for this car for all valid frames
                    mat_other_indmap[validfind, matind] = carind

                    local_setc!("posGx",   carind, validfind, posGx)
                    local_setc!("posGy",   carind, validfind, posGy)
                    local_setc!("posGyaw", carind, validfind, posGyaw)
                    local_setc!("posFx",   carind, validfind, s)
                    local_setc!("posFy",   carind, validfind, d)
                    local_setc!("posFyaw", carind, validfind, θ)

                    # extract specifics
                    speed = data_obs[frameind, :velBx]
                    local_setc!("velFx",     carind, validfind, speed * cos(θ)) # vel along the lane
                    local_setc!("velFy",     carind, validfind, speed * sin(θ)) # vel perpendicular to lane
                    local_setc!("lanetag",   carind, validfind, proj.lane.id)
                    local_setc!("curvature", carind, validfind, ptG.k)
                    local_setc!("d_cl",      carind, validfind, d::Float64)

                    d_merge = distance_to_lane_merge(sn, seg, laneid, proj.extind)
                    d_split = distance_to_lane_split(sn, seg, laneid, proj.extind)
                    local_setc!("d_merge", carind, validfind, isinf(d_merge) ? NA : d_merge)
                    local_setc!("d_split", carind, validfind, isinf(d_split) ? NA : d_split)

                    nll, nlr = StreetNetworks.num_lanes_on_sides(sn, seg, laneid, proj.extind)
                    @assert(nll >= 0)
                    @assert(nlr >= 0)
                    local_setc!("nll", carind, validfind, nll)
                    local_setc!("nlr", carind, validfind, nlr)

                    lane_width_left, lane_width_right = marker_distances(sn, seg, laneid, proj.extind)
                    local_setc!("d_mr", carind, validfind, (d <  lane_width_left)  ?  lane_width_left - d  : Inf)
                    local_setc!("d_ml", carind, validfind, (d > -lane_width_right) ?  d - lane_width_right : Inf)

                    local_setc!("id",        carind, validfind, carid)
                    local_setc!("t_inview",  carind, validfind, trajdata[end, :time] - trajdata[frameind, :time])
                    local_setc!("trajind",   carind, validfind, segment_count)
                end
            end
        end

        # toc()
    end

    println("PrimaryDataset: 1")

    dict_trajmat = Dict{UInt32,DataFrame}()
    retval = PrimaryDataset(df_ego, df_other, dict_trajmat, dict_other_idmap, mat_other_indmap, ego_car_on_freeway)
    println("PrimaryDataset: 2")
    retval
end

function extract_runlogs(trajdata::DataFrame, sn::StreetNetwork, params::PrimaryDataExtractionParams, runlogheader::RunLogHeader)

    _assert_valid_primarydata_extraction_params(params)

    # initial ego smoothing and outlier removal
    # -----------------------------------------

    n_initial_samples  = size(trajdata,1)

    arr_time  = convert(Array{Float64}, trajdata[:time]) .- trajdata[1, :time]::Float64

    n_resamples = floor(Int, (arr_time[end] - arr_time[1]) / params.resample_rate)
    arr_time_resampled = collect(0:n_resamples-1)*params.resample_rate + arr_time[1]
    arr_time_padded = pad_linear(arr_time, params.padding_size)
    arr_time_resampled_padded = pad_linear(arr_time_resampled, params.padding_size)

    trajdata_smoothed = DataFrame()
    trajdata[:yawG] = make_angle_continuous!(convert(Array{Float64}, trajdata[:yawG]))
    for (variable, RANSAC_fit_threshold, smoothing_variance) in
        [(:posGx,  Inf,  0.5),
         (:posGy,  Inf,  0.5),
         (:yawG,   0.01, 0.5),
         (:velEx,  0.1,  0.5),
         (:velEy,  0.05, 0.5)]

        arr_orig_padded = pad_linear(convert(Array{Float64}, trajdata[variable]), params.padding_size)

        if isinf(RANSAC_fit_threshold) || params.ransac_n_inliers_for_first_fit ≤ 3
            outliers = Set{Int}()
        else
            outliers = sliding_window_RANSAC(arr_time_padded, arr_orig_padded, params.ransac_n_iter,
                            RANSAC_fit_threshold, params.ransac_n_inliers_for_first_fit,
                            params.ransac_window_width, params.ransac_window_overlap)
        end

        percent_outliers = 100.0*length(outliers) / n_initial_samples
        percent_outliers < params.threshold_percent_outliers_error || error("TOO MANY OUTLIERS ($percent_outliers \%) FOR VARIABLE $variable")
        percent_outliers < params.threshold_percent_outliers_warn  ||  warn("TOO MANY OUTLIERS ($percent_outliers \%) FOR VARIABLE $variable")

        arr_smoothed_padded = smooth(arr_time_padded, arr_orig_padded, arr_time_resampled_padded, smoothing_variance, outliers)
        trajdata_smoothed[variable] = remove_pad(arr_smoothed_padded, params.padding_size)
    end

    # lane frame projection and determine ego_car_on_freeway
    # --------------------------------------------

    ego_car_on_freeway = falses(n_resamples)
    projections = Array(TilePoint2DProjectionResult, n_resamples)

    for frame in params.buffer_frames : (n_resamples-params.buffer_frames)

        posGx   = trajdata_smoothed[frame, :posGx]
        posGy   = trajdata_smoothed[frame, :posGy]
        posGyaw = trajdata_smoothed[frame, :yawG ]

        proj = project_point_to_streetmap(posGx, posGy, sn)
        projections[frame] = proj

        if proj.successful && proj.sqdist < params.threshold_proj_sqdist_ego
            ptG = proj.footpoint
            s,d,θ = pt_to_frenet_xyy(ptG, posGx, posGy, posGyaw)

            meets_lane_lateral_offset_criterion = abs(d) < params.threshold_lane_lateral_offset_ego
            meets_lane_angle_criterion = abs(θ) < params.threshold_lane_angle_ego
            ego_car_on_freeway[frame] = meets_lane_lateral_offset_criterion && meets_lane_angle_criterion
        end
    end

    # adjust ego_car_on_freeway based on given frameinds
    # ----------------------------------------
    if !isempty(params.frameinds)

        orig_index = 1
        t_lo = arr_time[params.frameinds[orig_index]]
        t_hi = arr_time[params.frameinds[orig_index+1]]

        for sample_index = 1 : n_resamples
            t = arr_time_resampled[sample_index]

            if t > t_hi
                orig_index += 2
                if orig_index < length(params.frameinds)
                    t_lo = arr_time[params.frameinds[orig_index]]
                    t_hi = arr_time[params.frameinds[orig_index+1]]
                else
                    t_lo = Inf
                    t_hi = Inf
                end
            end

            if t < t_lo
                ego_car_on_freeway[sample_index] = false
            end
        end
    end

    # extract a RunLog for each continuous segment
    # --------------------------------------------

    segments = continuous_segments(ego_car_on_freeway)
    n_runlogs = length(segments)
    retval = Array(RunLog, n_runlogs)
    for i in 1 : n_runlogs
        frame_lo, frame_hi = segments[i]
        retval[i] = _extract_runlog(trajdata, trajdata_smoothed,
                                    sn, params, runlogheader,
                                    frame_lo, frame_hi,
                                    arr_time, arr_time_resampled,
                                    projections)
    end

    retval
end

function _extract_runlog(
    trajdata::DataFrame,
    trajdata_smoothed::DataFrame,
    sn::StreetNetwork,
    params::PrimaryDataExtractionParams,
    runlogheader::RunLogHeader,
    frame_lo::Int,
    frame_hi::Int,
    arr_time::AbstractVector{Float64},
    arr_time_resampled::AbstractVector{Float64},
    projections::Vector{TilePoint2DProjectionResult},
    )

    _assert_valid_primarydata_extraction_params(params)
    @assert(frame_hi > frame_lo)
    @assert(frame_lo > 0)
    @assert(frame_hi ≤ length(projections))

    # initialize the RunLog
    # ----------------------------------------

    nframes = frame_hi - frame_lo + 1

    runlog = RunLog(nframes, deepcopy(runlogheader), 1)
    push_agent!(runlog, AgentDefinition(AgentClass.CAR, ID_EGO))

    colset_ego = one(UInt)
    RunLogs.set!(runlog, 1:nframes, :time, arr_time_resampled[frame_lo:frame_hi])

    if :control_status in names(trajdata)
        resampled_control_status = _resample_snap_to_closest(arr_time, trajdata[:control_status], arr_time_resampled)
        RunLogs.set!(runlog, 1:nframes, :environment, resampled_control_status[frame_lo:frame_hi])
    else
        RunLogs.set!(runlog, 1:nframes, :environment, repeated(params.default_control_status, nframes))
    end

    # ego feature extraction
    # --------------------------------------------

    for (frame_new, frame_old) in enumerate(frame_lo : frame_hi)

        inertial = VecSE2(trajdata_smoothed[frame_old, :posGx],
                          trajdata_smoothed[frame_old, :posGy],
                          mod2pi_neg_pi_to_pi(trajdata_smoothed[frame_old, :yawG ]))

        proj = projections[frame_old]
        @assert(proj.successful)
        @assert(proj.sqdist < params.threshold_proj_sqdist_ego)

        extind = proj.extind
        footpoint = proj.footpoint
        lanetag = proj.lane.id
        frenet = VecSE2(pt_to_frenet_xyy(footpoint, inertial.x, inertial.y, inertial.θ)...)

        turnrate = NaN
        if frame_new > 1
            turnrate = (inertial.θ - trajdata_smoothed[frame_old-1, :yawG]) / RunLogs.get_elapsed_time(runlog, frame_new-1, frame_new)
        elseif frame_old < frame_hi
            turnrate = (trajdata_smoothed[frame_old+1, :yawG] - inertial.θ) / RunLogs.get_elapsed_time(runlog, frame_new, frame_new+1)
        end

        ratesB = VecSE2(trajdata_smoothed[frame_old, :velEx],
                        trajdata_smoothed[frame_old, :velEy],
                        turnrate)

        RunLogs.set!(runlog, colset_ego, frame_new, ID_EGO,
             inertial, frenet, ratesB, extind, footpoint, lanetag)
    end

    # Other Car Extraction
    #  - for each car id
    #    - identify frames where it too is on the freeway
    #    - pull data for all frames where it is available
    #    - maintain list of missing frames
    #    - run smoothing / interpolation same as ego
    #    - map to frenet
    # --------------------------------------------

    const freeway_frameinds = collect(frame_lo:frame_hi)
    const freeway_frameinds_raw = encompasing_indeces(freeway_frameinds, arr_time_resampled, arr_time)

    const CARID_EGO_ORIG = -1
    trajdata_carids = get_carids(trajdata) # a Set{Int} of carids
    delete!(trajdata_carids, CARID_EGO_ORIG)

    for (i,trajdata_carid) in enumerate(trajdata_carids)

        tic()
        println(trajdata_carid, ": ", i, " / ", length(trajdata_carids))

        carid_new = convert(UInt, CARID_EGO + i)

        # all frames where the car exists
        car_frameinds_raw = filter(frame->carid_exists(trajdata, trajdata_carid, frame), freeway_frameinds_raw)
        if isempty(car_frameinds_raw)
            toc()
            continue
        end

        data_arr_index = 0

        for (lo,hi) in near_continuous_segments(car_frameinds_raw, params.threshold_other_frame_gap)

            segment_frameinds = collect(car_frameinds_raw[lo] : car_frameinds_raw[hi]) # array of all frames for this segment
            n_frames_in_seg = length(segment_frameinds)

            # ------------------------------------------
            # enforce minimum segment length
            if n_frames_in_seg < params.threshold_other_segment_length
                print_with_color(:red, "skipping due to insufficient length ($n_frames_in_seg < $(params.threshold_other_segment_length))\n")
                continue
            end

            # ------------------------------------------
            # run smoothing + interpolation

            # whether car exists in frame
            car_exists = falses(n_frames_in_seg)
            car_exists[car_frameinds_raw[lo:hi]-car_frameinds_raw[lo]+1] = true
            n_frames_exist = sum(car_exists)

            # map index to carind (-1 if no exist)
            carinds_raw = map(i->Trajdata.carid2ind_or_negative_one_otherwise(trajdata, trajdata_carid, segment_frameinds[i]), 1:n_frames_in_seg)
            time_obs = arr_time[car_frameinds_raw[lo:hi]] # actual measured time

            time_resampled_ind_lo = findfirst(i->arr_time_resampled[i] ≥ time_obs[1], 1:length(arr_time_resampled))
            time_resampled_ind_hi = time_resampled_ind_lo + findfirst(i->arr_time_resampled[i+1] > time_obs[end], (time_resampled_ind_lo+1):(length(arr_time_resampled)-1))
            @assert(time_resampled_ind_lo != 0 && time_resampled_ind_hi != 0)
            time_resampled = arr_time_resampled[time_resampled_ind_lo:time_resampled_ind_hi]
            smoothed_frameinds = collect(time_resampled_ind_lo:time_resampled_ind_hi)

            time_obs_padded = pad_linear(time_obs, params.padding_size)
            time_resampled_padded = pad_linear(time_resampled, params.padding_size)

            # TODO(tim): can this be optimized with pre-allocation outside of the loop?
            # NOTE(tim): this assumes zero sideslip
            data_obs = DataFrame(
                posGx = DataArray(Float64, n_frames_exist),
                posGy = DataArray(Float64, n_frames_exist),
                yawG  = DataArray(Float64, n_frames_exist),
                velBx = DataArray(Float64, n_frames_exist)
                )

            total = 0
            for (i,frameind) in enumerate(segment_frameinds)

                if car_exists[i]
                    total += 1
                    carind = carinds_raw[i]
                    @assert(carind != -1)

                    posEx = getc(trajdata, "posEx", carind, frameind)
                    posEy = getc(trajdata, "posEy", carind, frameind)
                    velEx = getc(trajdata, "velEx", carind, frameind) # NOTE(tim): velocity in the ego frame but pre-compensated for ego velocity
                    velEy = getc(trajdata, "velEy", carind, frameind)

                    posGx_ego = trajdata[frameind, :posGx]
                    posGy_ego = trajdata[frameind, :posGy]
                    yawG_ego  = trajdata[frameind, :yawG]

                    posGx, posGy = Trajdata.ego2global(posGx_ego, posGy_ego, yawG_ego, posEx, posEy)

                    data_obs[total, :posGx] = posGx
                    data_obs[total, :posGy] = posGy

                    velGx, velGy = Trajdata.ego2global(0.0, 0.0, yawG_ego, velEx, velEy)

                    if hypot(velGx, velGy) > 3.0
                        yawG = atan2(velGy, velGx)
                    else
                        yawG = yawG_ego # to fix problem with very low velocities
                    end
                    data_obs[total, :velBx] = hypot(velGx, velGy)
                    data_obs[total, :yawG]  = yawG
                end
            end
            @assert(total == size(data_obs, 1))
            data_obs[:yawG] = make_angle_continuous!(convert(Vector{Float64}, data_obs[:yawG]))

            data_smoothed = DataFrame()
            should_toss_due_to_outliers = false
            for (variable, RANSAC_fit_threshold, smoothing_variance) in
                [(:posGx,  0.5,  0.5), # TODO(tim): tune these
                 (:posGy,  0.5,  0.5),
                 (:yawG,   0.05, 0.5),
                 (:velBx,  2.0,  0.5)] # NOTE(tim): many vehicles are sensitive here... need to investigate

                arr_orig_padded = pad_linear(convert(Vector{Float64}, data_obs[variable]), params.padding_size)

                if  params.ransac_n_inliers_for_first_fit ≤ 3
                    outliers = Set{Int}()
                else
                    outliers = sliding_window_RANSAC(time_obs_padded, arr_orig_padded, params.ransac_n_iter,
                                        RANSAC_fit_threshold, params.ransac_n_inliers_for_first_fit,
                                        params.ransac_window_width, params.ransac_window_overlap)
                end

                percent_outliers = 100.0*length(outliers) / n_frames_in_seg
                if percent_outliers > params.threshold_percent_outliers_toss
                    should_toss_due_to_outliers = true
                    print_with_color(:red, "skipping due to high outlier percentage in $variable ($percent_outliers > $(params.threshold_percent_outliers_toss))\n")
                    break
                end

                arr_smoothed_padded = smooth(time_obs_padded, arr_orig_padded, time_resampled_padded, smoothing_variance, outliers)
                data_smoothed[variable] = remove_pad(arr_smoothed_padded, params.padding_size)
            end
            if should_toss_due_to_outliers
                continue
            end

            inds_to_keep = find(frame->frame_lo ≤ frame ≤ frame_hi, smoothed_frameinds)
            smoothed_frameinds = smoothed_frameinds[inds_to_keep]
            data_smoothed = data_smoothed[inds_to_keep, :]

            if size(data_smoothed, 1) < params.threshold_other_segment_length
                print_with_color(:red, "skipping due to insufficient length after smoothing ($n_frames_in_seg < $(params.threshold_other_segment_length))\n")
                continue
            end

            # ------------------------------------------
            # map to frenet frame & extract values

            for (i,frame_old) in enumerate(smoothed_frameinds)

                frame_new = frame_old - frame_lo + 1

                inertial = VecSE2(data_smoothed[i, :posGx],
                                  data_smoothed[i, :posGy],
                                  mod2pi_neg_pi_to_pi(data_smoothed[i, :yawG ]))

                proj = project_point_to_streetmap(inertial.x, inertial.y, sn)
                if proj.successful && proj.sqdist < params.threshold_proj_sqdist_other

                    extind = proj.extind
                    footpoint = proj.footpoint
                    lanetag = proj.lane.id
                    frenet = VecSE2(pt_to_frenet_xyy(footpoint, inertial.x, inertial.y, inertial.θ)...)

                    velBx = data_smoothed[i, :velBx]
                    ratesB = VecSE2(velBx*cos(inertial.θ),
                                    velBx*sin(inertial.θ),
                                    NaN) # NOTE(tim): turnrate will be computed in another pass

                    laneid = convert(Int, proj.lane.id.lane)
                    seg = get_segment(sn, proj.lane.id)
                    d_end = distance_to_lane_end(sn, seg, laneid, extind)

                    meets_lane_lateral_offset_criterion = abs(frenet.y) < params.threshold_lane_lateral_offset_other
                    meets_lane_angle_criterion = abs(frenet.θ) < params.threshold_lane_angle_other
                    meets_lane_end_criterion = d_end > params.threshold_other_from_lane_ends

                    if  meets_lane_lateral_offset_criterion &&
                        meets_lane_angle_criterion &&
                        meets_lane_end_criterion

                        colset = get_first_vacant_colset!(runlog, carid_new, frame_new)
                        RunLogs.set!(runlog, colset, frame_new, carid_new,
                             inertial, frenet, ratesB, extind, footpoint, lanetag)
                    end
                end
            end

            # second pass to compute turnrate
            for (i,frame_old) in enumerate(smoothed_frameinds)
                frame_new = frame_old - frame_lo + 1
                if idinframe(runlong, carid_new, frame_new)

                    turnrate = 0.0
                    if frame_new > 1 && idinframe(runlog, carid_new, frame_new-1)
                        turnrate = _estimate_turnrate(runlog, carid_new, frame_new-1, frame_new)
                    elseif frame_old < frame_hi && idinframe(runlog, carid_new, frame_new+1)
                        turnrate = _estimate_turnrate(runlog, carid_new, frame_new, frame_new+1)
                    end

                    colset = id2colset(runlog, carid_new, frame_new)
                    ratesB = get(runlog, colset, :ratesB)::VecSE2
                    ratesB = VecSE2(ratesB.x, ratesB.y, turnrate)
                    set!(runlog, colset, :ratesB, ratesB)
                end
            end
        end

        toc()
    end

    # now that the full scene set is computed, calc front and rear
    _calc_front_and_rear!(runlog, sn)

    runlog
end

function _estimate_turnrate(runlog::RunLog, id::UInt, frame1::Integer, frame2::Integer)
    colset₁ = id2colset(runlog, id, frame1)
    colset₂ = id2colset(runlog, id, frame2)
    θ₁ = get(runlog, colset₁, frame1, :inertial).θ
    θ₂ = get(runlog, colset₂, frame2, :inertial).θ
    @assert(!isnan(θ₁))
    @assert(!isnan(θ₂))
    (θ₂ - θ₁)/get_elapsed_time(runlog, frame1, frame2)
end

function _calc_front_vehicle_colset(runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Int)

    #=
    It turns out that defining the closest vehicle in front is tricky
      ex: a vehicle in neighboring lane that is moving towards your lane is likely
          to be what you pay attention to

    This merely finds the closest vehicle in the same lane based on footpoint and lanetag
    Quits once it reaches a max distance
    =#

    best_colset = COLSET_NULL
    best_dist   = Inf # [m]
    max_dist    = 1000.0 # [m]

    active_lanetag = get(runlog, colset, frame, :lanetag)::LaneTag
    active_lane = get_lane(sn, active_lanetag)
    footpoint_s_host = (get(runlog, colset, frame, :footpoint)::CurvePt).s
    dist = -footpoint_s_host # [m] dist along curve from host inertial to base of footpoint

    # walk forwards along the lanetag until we find a car in it or reach max dist
    while true
        for i in 1 : nactors(runlog, frame)
            test_colset = convert(UInt, i)

            if test_colset == colset
                continue
            end

            lanetag_target = get(runlog, test_colset, frame, :lanetag)::LaneTag
            if lanetag == active_lanetag

                footpoint_s_target = (get(runlog, test_colset, frame, :footpoint)::CurvePt).s
                if footpoint_s_target > footpoint_s_host

                    cand_dist = dist + footpoint_s_target - footpoint_s_host
                    if cand_dist < best_dist
                        best_dist = cand_dist
                        best_colset = test_colset
                    end
                end
            end
        end

        if isinf(best_dest) || dist > max_dist || !has_next_lane(sn, active_lane)
            break
        end

        dist += active_lane.curve.s[end] # move full curve length
        active_lane = next_lane(sn, active_lane)
        active_lanetag = active_lane.id
        footpoint_s_host = 0.0 # move to base of curve
    end

    best_colset
end
function _calc_rear_vehicle_colset(runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Int)

    best_colset = COLSET_NULL
    best_dist   = Inf # [m]
    max_dist    = 1000.0 # [m]

    active_lanetag = get(runlog, colset, frame, :lanetag)::LaneTag
    active_lane = get_lane(sn, active_lanetag)
    footpoint_s_host = (get(runlog, colset, frame, :footpoint)::CurvePt).s
    dist = -footpoint_s_host # [m] dist along curve from host inertial to base of footpoint

    # walk forwards along the lanetag until we find a car in it or reach max dist
    while true
        for i in 1 : nactors(runlog, frame)
            test_colset = convert(UInt, i)

            if test_colset == colset
                continue
            end

            lanetag_target = get(runlog, test_colset, frame, :lanetag)::LaneTag
            if lanetag == active_lanetag

                footpoint_s_target = (get(runlog, test_colset, frame, :footpoint)::CurvePt).s
                if footpoint_s_target < footpoint_s_host

                    cand_dist = dist + footpoint_s_host - footpoint_s_target
                    if cand_dist < best_dist
                        best_dist = cand_dist
                        best_colset = test_colset
                    end
                end
            end
        end

        if isinf(best_dest) || dist > max_dist || !has_prev_lane(sn, active_lane)
            break
        end

        active_lane = prev_lane(sn, active_lane)
        active_lanetag = active_lane.id
        dist += active_lane.curve.s[end] # move full length
        footpoint_s_host = active_lane.curve.s[end] # move to end of curve
    end

    best_colset
end
function _calc_front_and_rear!(runlog::RunLog, sn::StreetNetwork)
    #=
    Run through populated runlog and calc front and rear for each vehicle in each frame
    =#

    for frame in 1 : nframes(runlog)
        for i in 1 : nactors(runlog, frame)
            colset = convert(UInt, i)
            set!(runlog, colset, frame, :front, _calc_front_vehicle_colset(runlog, sn, colset, frame)::UInt)
            set!(runlog, colset, frame, :rear, _calc_rear_vehicle_colset(runlog, sn, colset, frame)::UInt)
        end
    end

    runlog
end

end # module
