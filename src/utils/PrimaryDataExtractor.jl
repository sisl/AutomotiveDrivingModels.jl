module PrimaryDataExtractor

using DataFrames
using HDF5, JLD

using AutomotiveDrivingModels.Vec
using AutomotiveDrivingModels.CommonTypes
using AutomotiveDrivingModels.Curves
using AutomotiveDrivingModels.StreetNetworks
using AutomotiveDrivingModels.Trajdata
using AutomotiveDrivingModels.RunLogs
using AutomotiveDrivingModels.Features

import AutomotiveDrivingModels: CSVFileSet

include(Pkg.dir("AutomotiveDrivingModels", "src", "io", "filesystem_utils.jl"))

export
    PrimaryDataExtractionParams,
    load_header_trajdata_and_streetmap,
    extract_runlogs,
    gen_primary_data,
    gen_primary_data_no_smoothing,
    trajdata_csv_to_header_file

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
    csvfileset::Union{CSVFileSet,Void} # used to set the behavior attribute (otherwise it is extrated automatically)

    verbosity::Int

    function PrimaryDataExtractionParams(mode::Symbol=:BoschRaw)
        self = new()

        self.resample_rate = 1/20.0 # exactly 20 Hz # TODO(tim): merge this appropriately
        self.frameinds = Int[]
        self.csvfileset = nothing
        self.verbosity = 0

        self.threshold_percent_outliers_warn = 0.5
        self.threshold_percent_outliers_error = 100.0 # 14.0
        self.threshold_lane_lateral_offset_ego = 2.5
        self.threshold_proj_sqdist_ego = self.threshold_lane_lateral_offset_ego * self.threshold_lane_lateral_offset_ego
        self.threshold_other_frame_gap = 5
        self.threshold_other_segment_length = 20
        self.threshold_percent_outliers_toss = 25.0
        self.threshold_lane_lateral_offset_other = 2.2
        self.threshold_proj_sqdist_other = self.threshold_lane_lateral_offset_other * self.threshold_lane_lateral_offset_other
        self.threshold_lane_angle_other = deg2rad(45)

        if mode == :BoschRaw
            self.padding_size  = 50

            self.ransac_n_iter = 50
            self.ransac_window_width = 20
            self.ransac_window_overlap = 5
            self.ransac_n_inliers_for_first_fit = 5
            self.buffer_frames = 5
            self.min_frames_ego_on_freeway = 1200
            self.threshold_lane_angle_ego = deg2rad(30)
            self.threshold_other_from_lane_ends = 1.0
            self.default_control_status = ControlStatus.UNKNOWN
        elseif mode == :PerfectSim
            self.padding_size  = 1
            self.ransac_n_iter = 1 # NOTE(tim): do not do ransac
            self.ransac_window_width = 3 # NOTE(tim): do not do ransac
            self.ransac_window_overlap = 0 # NOTE(tim): do not do ransac
            self.ransac_n_inliers_for_first_fit = 3 # NOTE(tim): do not do ransac
            self.buffer_frames = 1 # NOTE(tim): no frames removed
            self.min_frames_ego_on_freeway = 0
            self.threshold_lane_angle_ego = deg2rad(45)
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
function _resample_snap_to_closest{R<:Any}(x_arr::Vector{Float64}, y_arr::AbstractVector{R}, x_arr2::Vector{Float64})

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

trajdata_csv_to_header_file(csvfile::AbstractString) = splitext(csvfile)[1] * "_header.txt"
function load_header_trajdata_and_streetmap(csvfile::AbstractString)

    header_file = trajdata_csv_to_header_file(csvfile)
    header_file_io = open(header_file, "r")
    @assert(isfile(header_file))
    header = RunLogHeader(header_file_io)
    close(header_file_io)

    streetnet_file =joinpath(STREETMAP_BASE, "streetmap_" * header.map_name*".jld")
    sn = JLD.load(streetnet_file, "streetmap")

    csvfile_io = open(csvfile)
    trajdata = PrimaryDataExtractor.load_trajdata(csvfile_io, header)
    close(csvfile_io)

    (header, trajdata, sn)
end

function load_trajdata(csvfile::AbstractString)

    file = open(csvfile, "r")
    lines = readlines(file)
    close(file)

    n_cols = length(matchall(r",", lines[1]))+1

    temp_name = tempname()*".csv"

    # reexport with enough commas so we can read it in properly
    # ----------------------------------------
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
        (:odom_velocity_z, :velEz), (:odom_acceleration_x, :accEx), (:odom_acceleration_y, :accEy), (:odom_acceleration_z, :accEz)) # , (:heading, :yawG)
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
        QUAT_ENU = Quat(df[i, :quatx], df[i, :quaty], df[i, :quatz], df[i, :quatw])
        rpy_convert = convert(RPY, QUAT_ENU)
        rpy[i,1] = rpy_convert.r
        rpy[i,2] = rpy_convert.p
        rpy[i,3] = rpy_convert.y
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
function load_trajdata(io::IO, header::RunLogHeader)

    lines = readlines(io)

    n_cols = length(matchall(r",", lines[1]))+1

    temp_name = tempname()*".csv"

    # reexport with enough commas so we can read it in properly
    # ----------------------------------------
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
        (:odom_velocity_z, :velEz), (:odom_acceleration_x, :accEx), (:odom_acceleration_y, :accEy), (:odom_acceleration_z, :accEz)) # , (:heading, :yawG)
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
    if isa(header.quat_frame, Type{UTM})
        for i = 1 : size(df,1)
            quat_utm = Quat(df[i, :quatx], df[i, :quaty], df[i, :quatz], df[i, :quatw])
            rpy_convert = convert(RPY, quat_utm)
            rpy[i,1] = rpy_convert.r
            rpy[i,2] = rpy_convert.p
            rpy[i,3] = rpy_convert.y
        end
    elseif isa(header.quat_frame, Type{ECEF})
        for i = 1 : size(df,1)
            #=
            The newer runlogs have the quaternion in ECEF
            They are rectified here
            =#

            POS_UTM = UTM(df[i, :posGx], df[i, :posGy], df[i, :posGz], 10) # TODO(tim): remove default zone of 10
            QUAT_ECEF = Quat(df[i, :quatx], df[i, :quaty], df[i, :quatz], df[i, :quatw])
            POS_LLA = convert(LatLonAlt, POS_UTM)
            R = convert(Matrix{Float64}, QUAT_ECEF)
            fp_ecef = VecE3(R * [10.0,0.0,0.0])
            POS_ECEF = convert(ECEF, POS_LLA)
            FP_ECEF = ECEF(fp_ecef.x + POS_ECEF.x, fp_ecef.y+POS_ECEF.y, fp_ecef.z+POS_ECEF.z)
            POS_LLA = convert(LatLonAlt, POS_UTM)
            FP_LLA = convert(LatLonAlt, FP_ECEF)
            FP_UTM = convert(UTM, FP_LLA)
            AXIS_UTM = FP_UTM - POS_UTM
            yaw = atan2(AXIS_UTM.n, AXIS_UTM.e)

            rpy[i,1] = NaN
            rpy[i,2] = NaN
            rpy[i,3] = yaw
        end
    else
        error("unrecognized header quat_frame type $(header.quat_frame)")
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
        percent_outliers < params.threshold_percent_outliers_warn  || params.verbosity == 0 || warn("TOO MANY OUTLIERS ($percent_outliers \%) FOR VARIABLE $variable")

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
    df_ego[ego_car_on_freeway, :posFyaw] = _angle_to_range!(convert(Vector{Float64}, df_ego[ego_car_on_freeway, :posFyaw]), -π)

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

    carids = Trajdata.get_carids(trajdata) # a Set{Int} of carids
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
        carids = Trajdata.get_carids(trajdata) # a Set{Int} of carids
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


function extract_runlogs(
    trajdata::DataFrame,
    sn::StreetNetwork,
    params::PrimaryDataExtractionParams,
    runlogheader::RunLogHeader
    )

    _assert_valid_primarydata_extraction_params(params)

    # by default extract all frames
    if isempty(params.frameinds)
        params.frameinds = [1,size(trajdata,1)]
    end

    retval = RunLog[]
    for i in 1 : 2 : length(params.frameinds)
        frameind_lo = params.frameinds[i]
        frameind_hi = params.frameinds[i+1]
        if frameind_hi < frameind_lo + 100
            warn("Runlog with frameind range $frameind_lo : $frameind_hi is not worth extracting")
        else
            append!(retval, _extract_runlogs(trajdata[frameind_lo:frameind_hi,:], sn, params, runlogheader))
        end
    end
    retval
end
function _extract_runlogs(
    trajdata::DataFrame,
    sn::StreetNetwork,
    params::PrimaryDataExtractionParams,
    runlogheader::RunLogHeader
    )

    # initial ego smoothing and outlier removal
    # -----------------------------------------

    n_initial_samples  = size(trajdata,1)

    arr_time = convert(Array{Float64}, trajdata[:time]) .- trajdata[1, :time]::Float64

    n_resamples = floor(Int, (arr_time[end] - arr_time[1]) / params.resample_rate)
    arr_time_resampled = collect(0:n_resamples-1)*params.resample_rate + arr_time[1]
    arr_time_padded = pad_linear(arr_time, params.padding_size)
    arr_time_resampled_padded = pad_linear(arr_time_resampled, params.padding_size)

    # add new columns to trajdata for posGx and posGy
    # to avoid the distortion obtained by smoothing ego first

    max_carind = get_max_carind(trajdata)
    for carind in 0 : max_carind
        trajdata[symbol(@sprintf("posGx_%d", carind))] = fill(NaN, nrow(trajdata))
        trajdata[symbol(@sprintf("posGy_%d", carind))] = fill(NaN, nrow(trajdata))
        trajdata[symbol(@sprintf("velGx_%d", carind))] = fill(NaN, nrow(trajdata))
        trajdata[symbol(@sprintf("velGy_%d", carind))] = fill(NaN, nrow(trajdata))
    end

    for frame in 1 : nrow(trajdata)
        for carind in 0 : max_carind
            if carind_exists(trajdata, carind, frame)

                posEx_oth = getc(trajdata, "posEx", carind, frame)
                posEy_oth = getc(trajdata, "posEy", carind, frame)

                # NOTE(tim): velocity in the ego frame but pre-compensated for ego velocity
                velEx_oth = getc(trajdata, "velEx", carind, frame)
                velEy_oth = getc(trajdata, "velEy", carind, frame)

                posGx_ego = trajdata[frame, :posGx]
                posGy_ego = trajdata[frame, :posGy]
                yawG_ego  = trajdata[frame, :yawG]

                posGx_oth, posGy_oth = Trajdata.ego2global(posGx_ego, posGy_ego, yawG_ego,
                                                           posEx_oth, posEy_oth)
                velGx_oth, velGy_oth = Trajdata.ego2global(0.0, 0.0, yawG_ego, velEx_oth, velEy_oth)

                trajdata[frame, symbol(@sprintf("posGx_%d", carind))] = posGx_oth
                trajdata[frame, symbol(@sprintf("posGy_%d", carind))] = posGy_oth
                trajdata[frame, symbol(@sprintf("velGx_%d", carind))] = velGx_oth
                trajdata[frame, symbol(@sprintf("velGy_%d", carind))] = velGy_oth
            end
        end
    end

    # create smoothed trajdata for ego
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
        percent_outliers < params.threshold_percent_outliers_warn  || params.verbosity == 0 || warn("TOO MANY OUTLIERS ($percent_outliers \%) FOR VARIABLE $variable")

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

    # extract a RunLog for each continuous segment
    # --------------------------------------------

    segments = continuous_segments(ego_car_on_freeway)
    n_runlogs = length(segments)
    retval = RunLog[]
    for i in 1 : n_runlogs
        frame_lo, frame_hi = segments[i]
        duration = frame_hi - frame_lo + 1
        if duration ≥ params.min_frames_ego_on_freeway
            push!(retval, _extract_runlog(trajdata, trajdata_smoothed,
                                          sn, params, runlogheader,
                                          frame_lo, frame_hi,
                                          arr_time, arr_time_resampled,
                                          projections))
        end
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
        RunLogs.set!(runlog, 1:nframes, :environment, collect(repeated(params.default_control_status, nframes)))
    end

    # ego feature extraction
    # --------------------------------------------

    for (frame_new, frame_old) in enumerate(frame_lo : frame_hi)

        inertial = VecSE2(trajdata_smoothed[frame_old, :posGx],
                          trajdata_smoothed[frame_old, :posGy],
                          _angle_to_range(trajdata_smoothed[frame_old, :yawG ], -π))

        proj = projections[frame_old]
        @assert(proj.successful)
        @assert(proj.sqdist < params.threshold_proj_sqdist_ego)

        extind = proj.extind
        footpoint = proj.footpoint
        lanetag = proj.lane.id
        frenet = VecSE2(pt_to_frenet_xyy(footpoint, inertial.x, inertial.y, inertial.θ)...)

        ###########

        θ₁ = 0.0
        θ₂ = 0.0
        Δt = 0.25

        if frame_new > 1
                θ₁ = _angle_to_range(trajdata_smoothed[frame_old-1, :yawG], -π)
                θ₂ = inertial.θ
                Δt = RunLogs.get_elapsed_time(runlog, frame_new-1, frame_new)

            elseif frame_old < frame_hi
                θ₁ = inertial.θ
                θ₂ = _angle_to_range(trajdata_smoothed[frame_old+1, :yawG], -π)
                Δt = RunLogs.get_elapsed_time(runlog, frame_new, frame_new+1)
            end

        if θ₁ > 0.5π && θ₂ < -0.5π
            θ₁ -= 2π
        elseif θ₁ < -0.5π && θ₂ > 0.5π
            θ₁ += 2π
        end

        turnrate = (θ₂ - θ₁)/Δt

        ############

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
    trajdata_carids = Trajdata.get_carids(trajdata) # a Set{Int} of carids
    delete!(trajdata_carids, CARID_EGO_ORIG)

    for (i,trajdata_carid) in enumerate(trajdata_carids)

        if params.verbosity > 0
            tic()
            println(trajdata_carid, ": ", i, " / ", length(trajdata_carids))
        end

        carid_new = convert(UInt, CARID_EGO + i)

        # all frames where the car exists
        car_frameinds_raw = filter(frame->carid_exists(trajdata, trajdata_carid, frame), freeway_frameinds_raw)
        if isempty(car_frameinds_raw)
            params.verbosity ≤ 0 || toc()
            continue
        end

        data_arr_index = 0

        for (lo,hi) in near_continuous_segments(car_frameinds_raw, params.threshold_other_frame_gap)

            segment_frameinds = collect(car_frameinds_raw[lo] : car_frameinds_raw[hi]) # array of all frames for this segment
            n_frames_in_seg = length(segment_frameinds)

            # ------------------------------------------
            # enforce minimum segment length
            if n_frames_in_seg < params.threshold_other_segment_length
                if params.verbosity > 0
                    print_with_color(:red, "skipping due to insufficient length ($n_frames_in_seg < $(params.threshold_other_segment_length))\n")
                end
                continue
            end

            # ------------------------------------------
            # run smoothing + interpolation

            # map index to carind (-1 if no exist)
            carinds_raw = map(i->Trajdata.carid2ind_or_negative_one_otherwise(trajdata, trajdata_carid, segment_frameinds[i]), 1:n_frames_in_seg)

            # whether car exists in frame
            car_exists = falses(n_frames_in_seg)
            car_exists[car_frameinds_raw[lo:hi]-car_frameinds_raw[lo]+1] = true

            # remove frames where the car is basically stopped and ego is at highway speeds
            for (i,frameind) in enumerate(segment_frameinds)

                if car_exists[i]
                    carind = carinds_raw[i]
                    @assert(carind != -1)

                    velGx_oth = getc(trajdata, "velGx", carind, frameind)
                    velGy_oth = getc(trajdata, "velGy", carind, frameind)
                    velG_oth = hypot(velGx_oth, velGy_oth)

                    velEx_ego = trajdata[frameind, :velEx]
                    velEy_ego = trajdata[frameind, :velEy]
                    velG_ego = hypot(velEx_ego, velEy_ego)
                    if velG_oth < 5.0 &&  velG_ego > 20.0
                        car_exists[i] = false
                    end
                end
            end

            n_frames_exist = sum(car_exists)
            if n_frames_exist < 5
                if params.verbosity > 0
                    print_with_color(:red, "skipping due to insufficient existing frame count ($n_frames_exist < 5\n")
                end
                continue
            end

            # time_obs = arr_time[car_frameinds_raw[lo:hi]] # actual measured time
            time_obs = Array(Float64, n_frames_exist)
            let
                j = 0
                for (i, frameind) in enumerate(segment_frameinds)
                    if car_exists[i]
                        j += 1
                        time_obs[j] = arr_time[frameind] # actual measured time
                    end
                end
            end

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

                    data_obs[total, :posGx] = getc(trajdata, "posGx", carind, frameind)
                    data_obs[total, :posGy] = getc(trajdata, "posGy", carind, frameind)

                    velGx = getc(trajdata, "velGx", carind, frameind)
                    velGy = getc(trajdata, "velGy", carind, frameind)

                    if hypot(velGx, velGy) > 3.0
                        yawG = atan2(velGy, velGx)
                    else
                        yawG = trajdata[frameind, :yawG] # to fix problem with very low velocities
                    end

                    velBx = hypot(velGx, velGy)
                    data_obs[total, :velBx] = velBx
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
                    if params.verbosity > 0
                        print_with_color(:red, "skipping due to high outlier percentage in $variable ($percent_outliers > $(params.threshold_percent_outliers_toss))\n")
                    end
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
                if params.verbosity > 0
                    print_with_color(:red, "skipping due to insufficient length after smoothing ($n_frames_in_seg < $(params.threshold_other_segment_length))\n")
                end
                continue
            end

            # ------------------------------------------
            # map to frenet frame & extract values

            for (i,frame_old) in enumerate(smoothed_frameinds)

                frame_new = frame_old - frame_lo + 1

                inertial = VecSE2(data_smoothed[i, :posGx],
                                  data_smoothed[i, :posGy],
                                  _angle_to_range(data_smoothed[i, :yawG ], -π))

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
                if RunLogs.idinframe(runlog, carid_new, frame_new)

                    turnrate = 0.0
                    if frame_new > 1 && RunLogs.idinframe(runlog, carid_new, frame_new-1)
                        turnrate = _estimate_turnrate(runlog, carid_new, frame_new-1, frame_new)
                    elseif frame_old < frame_hi && RunLogs.idinframe(runlog, carid_new, frame_new+1)
                        turnrate = _estimate_turnrate(runlog, carid_new, frame_new, frame_new+1)
                    end

                    colset = id2colset(runlog, carid_new, frame_new)
                    ratesB = get(runlog, colset, frame_new, :ratesB)::VecSE2
                    ratesB = VecSE2(ratesB.x, ratesB.y, turnrate)
                    RunLogs.set!(runlog, colset, frame_new, :ratesB, ratesB)
                end
            end
        end

        if params.verbosity > 0
            toc()
        end
    end

    # Post Processing
    #  - front and rear for each vehicle
    #  - calc behavior for each vehicle
    # --------------------------------------------

    # front and rear for each vehicle
    # println("front and rear for each vehicle: "); tic()
    for frame in 1 : RunLogs.nframes(runlog)
        for colset in get_colset_range(runlog, frame)
            colset_front = calc_front_vehicle_colset(runlog, sn, colset, frame)::UInt
            RunLogs.set!(runlog, colset, frame, :colset_front, colset_front)

            colset_rear = calc_rear_vehicle_colset(runlog, sn, colset, frame)::UInt
            RunLogs.set!(runlog, colset, frame, :colset_rear, colset_rear)
        end
    end
    # toc()

    # behavior for each vehicle
    # println("behavior for each vehicle: "); tic()

    if !isa(params.csvfileset, Void)
        # set the behavior using the CSVFileSet

        csvfileset = params.csvfileset
        id = convert(UInt, csvfileset.carid)

        nframes_time = length(arr_time)
        freeflow = _resample_snap_to_closest(arr_time, _calc_subset_vector(csvfileset.freeflow, nframes_time), arr_time_resampled)
        carfollow = _resample_snap_to_closest(arr_time, _calc_subset_vector(csvfileset.carfollow, nframes_time), arr_time_resampled)
        lanechange = _resample_snap_to_closest(arr_time, _calc_subset_vector([csvfileset.lanechanges_normal;
                                                                              csvfileset.lanechanges_postpass;
                                                                              csvfileset.lanechanges_arbitrary], nframes_time),
                                               arr_time_resampled)

        for frame in 1 : RunLogs.nframes(runlog)
            colset = RunLogs.id2colset(runlog, id, frame)

            behavior = ContextClass.NULL
            if freeflow[frame]
                behavior |= ContextClass.FREEFLOW
            end
            if carfollow[frame]
                behavior |= ContextClass.FOLLOWING
            end
            if lanechange[frame]
                behavior |= ContextClass.LANECHANGE
            end

            RunLogs.set!(runlog, colset, frame, :behavior, behavior)
        end
    else
        # compute the behavior manually

        id = ID_EGO
        for frame in 1 : RunLogs.nframes(runlog)
            colset = RunLogs.id2colset(runlog, id, frame)

            inv_timegap_front = get(INV_TIMEGAP_FRONT, runlog, sn, colset, frame)
            Δv_front = get(DELTA_V_FRONT, runlog, sn, colset, frame)

            # freeflow and carfollow are mutually exclusive
            is_in_freeflow = isnan(inv_timegap_front) || (inv_timegap_front < 1.0/3.0 || Δv_front > 0.5)
            if is_in_freeflow
                set_behavior_flag!(runlog, colset, frame, ContextClass.FREEFLOW)
                @assert(!is_behavior_flag_set(runlog, colset, frame, ContextClass.FOLLOWING))
            else
                set_behavior_flag!(runlog, colset, frame, ContextClass.FOLLOWING)
                @assert(!is_behavior_flag_set(runlog, colset, frame, ContextClass.FREEFLOW))
            end
        end

        # lanechange can be set separately
        d_cl_prev = Inf
        for frame in 1 : RunLogs.nframes(runlog)
            colset = RunLogs.id2colset(runlog, id, frame)

            d_cl = get(DIST_FROM_CENTERLINE, runlog, sn, colset, frame)

            if frame > 1 && abs(d_cl - d_cl_prev) > 2.5
                # identifier lanechange
                # move forward and back until |velFy| < 0.1 for a max of 3 sec each direction

                # set_behavior_flag!(runlog, colset, frame, ContextClass.LANECHANGE)
                RunLogs.set!(runlog, colset, frame, :behavior, ContextClass.LANECHANGE)

                for frame_fut  in frame+1 : RunLogs.nframes(runlog)
                    colset_fut = RunLogs.id2colset(runlog, id, frame_fut)
                    velFt_fut = get(VELFT, runlog, sn, colset, frame_fut)
                    Δt = RunLogs.get_elapsed_time(runlog, frame, frame_fut)
                    if abs(velFt_fut) ≥ 0.1 && Δt < 3.0
                        # set_behavior_flag!(runlog, colset_fut, frame_fut, ContextClass.LANECHANGE)
                        RunLogs.set!(runlog, colset_fut, frame_fut, :behavior, ContextClass.LANECHANGE)
                    else
                        break
                    end
                end

                for frame_past  in frame-1 : -1 : 1
                    colset_past = RunLogs.id2colset(runlog, id, frame_past)
                    velFt_past = get(VELFT, runlog, sn, colset_past, frame_past)
                    Δt = RunLogs.get_elapsed_time(runlog, frame_past, frame)
                    if abs(velFt_past) ≥ 0.1 && Δt < 3.0
                        # set_behavior_flag!(runlog, colset_past, frame_past, ContextClass.LANECHANGE)
                        RunLogs.set!(runlog, colset_past, frame_past, :behavior, ContextClass.LANECHANGE)
                    else
                        break
                    end
                end
            end
            d_cl_prev = d_cl
        end
    end
    # toc()

    runlog
end

function _angle_to_range(θ::Float64, low::Float64)
    # ensures θ is within (low ≤ θ ≤ low + 2π)

    if !isnan(θ) && !isinf(θ)
        while θ < low
            θ += 2π
        end
        while θ > low + 2π
            θ -= 2π
        end
    end

    θ
end
function _angle_to_range!( arr::Vector{Float64}, low::Float64)
    for i = 1 : length(arr)
        arr[i] = _angle_to_range(arr[i], low)
    end
    arr
end

function _estimate_turnrate(runlog::RunLog, id::UInt, frame1::Integer, frame2::Integer)
    colset₁ = id2colset(runlog, id, frame1)
    colset₂ = id2colset(runlog, id, frame2)
    θ₁ = get(runlog, colset₁, frame1, :inertial).θ
    θ₂ = get(runlog, colset₂, frame2, :inertial).θ
    @assert(!isnan(θ₁))
    @assert(!isnan(θ₂))
    @assert(-π ≤ θ₁ ≤ π)
    @assert(-π ≤ θ₂ ≤ π)

    # correct for wrap-around

    if θ₁ > 0.5π && θ₂ < -0.5π
        θ₁ -= 2π
    elseif θ₁ < -0.5π && θ₂ > 0.5π
        θ₁ += 2π
    end

    ψ = (θ₂ - θ₁)/RunLogs.get_elapsed_time(runlog, frame1, frame2)

    if abs(ψ) > 10.0
        println("frame1: ", frame1, " out of ", nframes(runlog))
        println("frame2: ", frame2)
        println("θ₁ orig: ", rad2deg(get(runlog, colset₁, frame1, :inertial).θ))
        println("θ₂ orig: ", rad2deg(get(runlog, colset₂, frame2, :inertial).θ))
        println("θ₁: ", rad2deg(θ₁))
        println("θ₂: ", rad2deg(θ₂))
        println("Ψ: ", rad2deg(ψ))
        println("Δt: ", RunLogs.get_elapsed_time(runlog, frame1, frame2))
        ψ = Inf
    end

    ψ
end

function _calc_subset_vector(validfind_regions::AbstractVector{Int}, nframes::Int)

    n = length(validfind_regions)
    @assert(mod(n,2) == 0) # ensure even number of regions

    retval = falses(nframes)
    for i = 1 : 2 : n
        validfind_lo = validfind_regions[i]
        validfind_hi = validfind_regions[i+1]

        @assert(validfind_lo ≤ validfind_hi)
        @assert(validfind_lo ≥ 1)
        @assert(validfind_hi ≤ nframes)

        for j = validfind_lo : validfind_hi
            retval[j] = true
        end
    end

    retval
end

end # module