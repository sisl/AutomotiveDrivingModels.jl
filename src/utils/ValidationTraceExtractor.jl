module ValidationTraceExtractor

include(Pkg.dir("AutomotiveDrivingModels", "src", "io", "filesystem_utils.jl"))

using DataFrames
using Discretizers
using JLD

using AutomotiveDrivingModels.CommonTypes
using AutomotiveDrivingModels.Trajdata
using AutomotiveDrivingModels.RunLogs
using AutomotiveDrivingModels.StreetNetworks
using AutomotiveDrivingModels.Features
using AutomotiveDrivingModels.FeaturesNew
using AutomotiveDrivingModels.FeaturesetExtractor
# using AutomotiveDrivingModels.Curves # TODO: remove this

import Base.Collections: PriorityQueue, dequeue!
import AutomotiveDrivingModels.FeaturesetExtractor: create_dataframe_with_feature_columns

export
        OrigHistobinExtractParameters,
        # StartCondition,
        ModelTrainingData,
        FoldAssignment,

        RunLogSegmentExtractParameters,
        RunLogSegment,
        ModelTrainingData2,
        DatasetExtractParams,

        FOLD_TRAIN,
        FOLD_TEST,

        total_framecount,
        calc_traces_to_keep_with_frameskip,

        pull_model_training_data,

        is_strictly_monotonically_increasing,
        merge_region_segments,
        calc_row_count_from_region_segments,

        drop_fold!,
        get_cross_validation_fold_assignment,
        get_train_test_fold_assignment,
        get_fold_assignment_across_drives,
        get_fold_assignment_across_traces,
        calc_fold_size,
        calc_fold_inds!,
        is_in_fold,


        load_pdsets,
        load_runlogs,
        load_streetnets


########################################
# ORIGINAL HISTOBIN EXTRACT PARAMETERS #
########################################

type OrigHistobinExtractParameters
    behavior        :: AbstractString
    subsets         :: Vector{AbstractFeature} # Features that must be true (eg: SUBSET_FREE_FLOW)
    tol_d_cl        :: Float64 # [m]
    tol_yaw         :: Float64 # [rad]
    tol_turnrate    :: Float64 # [rad/s]
    tol_speed       :: Float64 # [m/s]
    tol_accel       :: Float64 # [m/s²]
    tol_d_x_front   :: Float64 # [m]
    tol_d_y_front   :: Float64 # [m]
    tol_d_v_front   :: Float64 # [m/s]

    target_speed    :: Float64 # [m/s]

    pdset_frames_per_sim_frame :: Int # number of pdset frames that occur in the space of one sim frame
    sim_horizon     :: Int # number of sim frames past the initial conditions to run (≥0)
    sim_history     :: Int # number of sim frames up to and including frame 1 (≥1)
    frameskip_between_extracted_scenes :: Int

    function OrigHistobinExtractParameters(
        behavior::AbstractString="",
        subsets::Vector{AbstractFeature}=AbstractFeature[],
        tol_d_cl::Float64=NaN,
        tol_yaw::Float64=NaN,
        tol_turnrate::Float64=NaN,
        tol_speed::Float64=NaN,
        tol_accel::Float64=NaN,
        tol_d_x_front::Float64=NaN,
        tol_d_y_front::Float64=NaN,
        tol_d_v_front::Float64=NaN,
        target_speed::Float64=NaN,
        pdset_frames_per_sim_frame::Int=-1,
        sim_horizon::Int=-1,
        sim_history::Int=-1,
        frameskip_between_extracted_scenes::Int=-1,
        )

        retval = new()
        retval.behavior = behavior
        retval.subsets = subsets
        retval.tol_d_cl = tol_d_cl
        retval.tol_yaw = tol_yaw
        retval.tol_turnrate = tol_turnrate
        retval.tol_speed = tol_speed
        retval.tol_accel = tol_accel
        retval.tol_d_x_front = tol_d_x_front
        retval.tol_d_y_front = tol_d_y_front
        retval.tol_d_v_front = tol_d_v_front
        retval.target_speed = target_speed
        retval.pdset_frames_per_sim_frame = pdset_frames_per_sim_frame
        retval.sim_horizon = sim_horizon
        retval.sim_history = sim_history
        retval.frameskip_between_extracted_scenes = frameskip_between_extracted_scenes
        retval
    end
end
total_framecount(stats::OrigHistobinExtractParameters) = stats.sim_history + stats.sim_horizon
function Base.(:(==))(a::OrigHistobinExtractParameters, b::OrigHistobinExtractParameters)
    a.behavior == b.behavior &&
    length(a.subsets) == length(b.subsets) &&
    all([findfirst(item->isa(item, typeof(feature_a)), b.subsets)!=0 for feature_a in a.subsets]) &&
    isapprox(a.tol_d_cl,      b.tol_d_cl)      &&
    isapprox(a.tol_yaw,       b.tol_yaw)       &&
    isapprox(a.tol_turnrate,  b.tol_turnrate)  &&
    isapprox(a.tol_speed,     b.tol_speed)     &&
    isapprox(a.tol_accel,     b.tol_accel)     &&
    isapprox(a.tol_d_x_front, b.tol_d_x_front) &&
    isapprox(a.tol_d_y_front, b.tol_d_y_front) &&
    isapprox(a.tol_d_v_front, b.tol_d_v_front) &&
    isapprox(a.target_speed,  b.target_speed)  &&
    a.pdset_frames_per_sim_frame         == b.pdset_frames_per_sim_frame &&
    a.sim_horizon                        == b.sim_horizon &&
    a.sim_history                        == b.sim_history &&
    a.frameskip_between_extracted_scenes == b.frameskip_between_extracted_scenes
end
Base.(:(!=))(a::OrigHistobinExtractParameters, b::OrigHistobinExtractParameters) = !(a==b)
Base.isequal(a::OrigHistobinExtractParameters, b::OrigHistobinExtractParameters) = a == b


########################################
#          START CONDITION             #
########################################

# immutable StartCondition
#     validfind :: Int # the starting validfind (includes history)
#     carids    :: Vector{Int} # vehicles to pull traces for
# end

########################################
# ORIGINAL HISTOBIN EXTRACT PARAMETERS #
########################################

type RunLogSegmentExtractParameters
    sim_horizon :: Int # number of sim frames past the initial conditions to run (≥0)
    sim_history :: Int # number of sim frames up to and including frame 1 (≥1)
    frameskip_between_extracted_scenes :: Int
    runlog_frames_per_sim_frame :: Int # number of pdset frames that occur in the space of one sim frame

    function RunLogSegmentExtractParameters(
        sim_horizon::Int,
        sim_history::Int,
        frameskip_between_extracted_scenes::Int = 0,
        runlog_frames_per_sim_frame::Int = PDSET_FRAMES_PER_SIM_FRAME,
        )

        @assert(sim_horizon ≥ 0)
        @assert(sim_history ≥ 1)

        retval = new()
        retval.sim_horizon = sim_horizon
        retval.sim_history = sim_history
        retval.frameskip_between_extracted_scenes = frameskip_between_extracted_scenes
        retval.runlog_frames_per_sim_frame = runlog_frames_per_sim_frame
        retval
    end
end
get_nframes(p::RunLogSegmentExtractParameters) = p.sim_history + p.sim_horizon
function Base.(:(==))(a::RunLogSegmentExtractParameters, b::RunLogSegmentExtractParameters)
    a.behavior_to_match == b.behavior_to_match &&
    a.behavior_to_avoid == b.behavior_to_avoid &&
    a.sim_horizon == b.sim_horizon &&
    a.sim_history == b.sim_history &&
    a.frameskip_between_extracted_scenes == b.frameskip_between_extracted_scenes &&
    a.pdset_frames_per_sim_frame == b.pdset_frames_per_sim_frame
end
Base.(:(!=))(a::RunLogSegmentExtractParameters, b::RunLogSegmentExtractParameters) = !(a==b)
Base.isequal(a::RunLogSegmentExtractParameters, b::RunLogSegmentExtractParameters) = a == b

type DatasetExtractParams

    id_target         :: UInt
    behavior_to_match :: UInt16 # all frames must have (behavior & behavior_to_match) > 1
    behavior_to_avoid :: UInt16 # all frames must have (behavior & behavior_to_avoid) == 0
    features          :: Vector{FeaturesNew.AbstractFeature}
    filters           :: Vector{FeaturesNew.AbstractFeature}
    seg               :: RunLogSegmentExtractParameters

    function DatasetExtractParams(
        behavior_to_match::UInt16,
        behavior_to_avoid::UInt16,
        features::Vector{FeaturesNew.AbstractFeature},
        seg::RunLogSegmentExtractParameters;
        filters::Vector{FeaturesNew.AbstractFeature} = Array(FeaturesNew.AbstractFeature, 0),
        id_target::UInt = ID_EGO,
        )

        retval = new()
        retval.id_target = id_target
        retval.behavior_to_match = behavior_to_match
        retval.behavior_to_avoid = behavior_to_avoid
        retval.features = features
        retval.filters = filters
        retval.seg = seg
        retval
    end
end

########################################
#            RunLog Segment            #
########################################

immutable RunLogSegment

    # NOTE: mapname can be identified using RunLog.hedaer.map_name

    runlog_id   :: Int # index within ModelTrainingData.runlog_filepaths of the relevant runlog
    carid       :: UInt # the active vehicle
    frame_start :: Int # the starting frame in runlog (does not count any sort of history)
    frame_end   :: Int # the ending frame in runlog
end
function Base.(:(==))(a::RunLogSegment, b::RunLogSegment)
    a.runlog_id   == b.runlog_id &&
    a.carid       == b.carid &&
    a.frame_start == b.frame_start &&
    a.frame_end   == b.frame_end
end
function Base.show(io::IO, seg::RunLogSegment)
    println(io, "RunLogSegment")
    @printf(io, "\trunlog_id:   %d\n", seg.pdset_id)
    @printf(io, "\tcarid:       %d\n", seg.carid)
    @printf(io, "\tframe_start: %d\n", seg.frame_start)
    @printf(io, "\tframe_end:   %d\n", seg.frame_end)
end

get_nframes(seg::RunLogSegment) = seg.frame_end - seg.frame_start + 1
function get_nsimframes(seg::RunLogSegment, nframes_per_simframe::Int=N_FRAMES_PER_SIM_FRAME)
    # NOTE(tim): this includes the first frame
    h = get_nframes(seg) - 1
    ceil(Int, h / nframes_per_simframe)
end

########################################
#         MODEL TRAINING DATA 2        #
########################################

type ModelTrainingData2
    runlog_filepaths::Vector{AbstractString}     # list of runlog full file paths
    runlog_segments::Vector{RunLogSegment}       # set of RunLogSegments, all should be of the same length
    dataframe::DataFrame                         # dataframe of features used in training. A bunch of vcat'ed runlog features
    dataframe_nona::DataFrame                    # dataframe of features in which na has been replaced using replacena()
    seg_start_to_index_in_dataframe::Vector{Int} # maps index of runlogseg.frame_start to the corresponding index in dataframe
end
function Base.print(io::IO, dset::ModelTrainingData2)
    println(io, "ModelTrainingData2:")
    @printf(io, "\t%-30s  %10s\n", "n frames:", @sprintf("%d", nrow(dset.dataframe)))
    @printf(io, "\t%-30s  %10s\n", "n traces:", @sprintf("%d", length(dset.runlog_segments)))
end

########################################
#         MODEL TRAINING DATA          #
########################################

type ModelTrainingData
    pdset_filepaths::Vector{AbstractString}     # list of pdset full filepaths
    streetnet_filepaths::Vector{AbstractString} # list of streetnet full filepaths
    pdset_segments::Vector{PdsetSegment} # set of PdsetSegments, all should be of the same length
    dataframe::DataFrame                 # dataframe of features used in training. A bunch of vcat'ed pdset data
    dataframe_nona::DataFrame            # dataframe of features in which na has been replaced (otherwise exactly equal)
    startframes::Vector{Int}             # dataframe[startframes[i], :] is data row for pdset_segments[i].validfind_start
                                         # seg_to_framestart[i] -> j
                                         # where i is the pdsetsegment index
                                         # and j is the row in dataframe for the start (pdsetsegment.log[pdsetsegment.history,:])
end
function Base.append!(A::ModelTrainingData, B::ModelTrainingData)

    # assume that the dataframe is non-overlapping (no way to tell)
    nrow_df_A = nrow(A.dataframe)
    append!(A.dataframe, B.dataframe)
    append!(A.dataframe_nona, B.dataframe_nona)

    B_pdset_filepath_id_map = Array(Int, length(B.pdset_filepaths))
    for (i,pdset_filepath) in enumerate(B.pdset_filepaths)
        j = findfirst(A.pdset_filepaths, pdset_filepath)
        if j == 0
            push!(A.pdset_filepaths, pdset_filepath)
            B_pdset_filepath_id_map[i] = length(A.pdset_filepaths)
        else
            B_pdset_filepath_id_map[i] = j
        end
    end

    B_sn_filepath_id_map = Array(Int, length(B.streetnet_filepaths))
    for (i,sn_filepath) in enumerate(B.streetnet_filepaths)
        j = findfirst(A.streetnet_filepaths, sn_filepath)
        if j == 0
            push!(A.streetnet_filepaths, sn_filepath)
            B_sn_filepath_id_map[i] = length(A.streetnet_filepaths)
        else
            B_sn_filepath_id_map[i] = j
        end
    end

    for (i,seg) in enumerate(B.pdset_segments)
        seg_new = PdsetSegment(B_pdset_filepath_id_map[seg.pdset_id],
                               B_sn_filepath_id_map[seg.streetnet_id],
                               seg.carid, seg.validfind_start,
                               seg.validfind_end
                               )
        if !in(seg_new, A.pdset_segments)
            push!(A.pdset_segments, seg_new)
            push!(A.startframes, B.startframes[i]+nrow_df_A)
        end
    end

    A
end
function Base.print(io::IO, dset::ModelTrainingData)
    println(io, "ModelTrainingData:")
    @printf(io, "\t%-30s  %10s\n", "n frames:", @sprintf("%d", nrow(dset.dataframe)))
    @printf(io, "\t%-30s  %10s\n", "n traces:", @sprintf("%d", length(dset.startframes)))
end


########################################
#              FUNCTIONS               #
########################################

function _get_pdsetfile(csvfile::AbstractString, pdset_dir::AbstractString=PRIMARYDATA_DIR)
    csvfilebase = basename(csvfile)
    joinpath(pdset_dir, toext("primarydata_" * csvfilebase, "jld"))
end
function _load_pdset(csvfile::AbstractString, pdset_dir::AbstractString=PRIMARYDATA_DIR)
    pdsetfile = _get_pdsetfile(csvfile, pdset_dir)
    pdset = load(pdsetfile, "pdset")
end

# function calc_traces_to_keep_with_frameskip(
#     start_conditions :: Vector{StartCondition},
#     pdset :: PrimaryDataset,
#     frameskip :: Int
#     )

#     keep = falses(length(start_conditions))
#     prev_frameind = -frameskip
#     for (i,start_condition) in enumerate(start_conditions)
#         frameind = validfind2frameind(pdset, start_condition.validfind)
#         keep[i] = (frameind - prev_frameind) ≥ frameskip
#         if keep[i]
#             prev_frameind = frameind
#         end
#     end

#     keep
# end

function _calc_frame_passes_subsets{F<:AbstractFeature}(
    pdset     :: PrimaryDataset,
    sn        :: StreetNetwork,
    carind    :: Int,
    validfind :: Int,
    subsets   :: Vector{F}
    )

    for subset in subsets
        val = get(subset, pdset, sn, carind, validfind)::Float64
        @assert(isapprox(val, 0.0) || isapprox(val, 1.0))
        @assert(isbool(subset))
        if !bool(val)
            return false
        end
    end
    true
end
function _calc_frame_passes_subsets{F<:AbstractFeature}(
    pdset     :: PrimaryDataset,
    sn        :: StreetNetwork,
    carind    :: Int,
    validfind :: Int,
    subsets   :: Vector{F},
    subsets_based_on_csvfileset :: DataFrame
    )

    for subset in subsets


        if isa(subset, typeof(SUBSET_FREE_FLOW))
            passes = subsets_based_on_csvfileset[validfind, symbol(SUBSET_FREE_FLOW)]
        elseif isa(subset, typeof(SUBSET_CAR_FOLLOWING))
            passes = subsets_based_on_csvfileset[validfind, symbol(SUBSET_CAR_FOLLOWING)]
        elseif isa(subset, typeof(SUBSET_LANE_CROSSING))
            passes = subsets_based_on_csvfileset[validfind, symbol(SUBSET_LANE_CROSSING)]
        else
            val = get(subset, pdset, sn, carind, validfind)::Float64
            @assert(isapprox(val, 0.0) || isapprox(val, 1.0))
            @assert(isbool(subset))
            passes = bool(val)
        end

        if !passes
            return false
        end
    end
    true
end
function _calc_frames_pass_subsets{F<:AbstractFeature}(
    pdset           :: PrimaryDataset,
    sn              :: StreetNetwork,
    carind          :: Int,
    validfind_start :: Int,
    validfind_end   :: Int,
    subsets         :: Vector{F}
    )

    for validfind in validfind_start : validfind_end
        if !_calc_frame_passes_subsets(pdset, sn, carind, validfind, subsets)
            return false
        end
    end
    true
end
function _calc_frames_pass_subsets{F<:AbstractFeature}(
    pdset           :: PrimaryDataset,
    sn              :: StreetNetwork,
    carind          :: Int,
    validfind_start :: Int,
    validfind_end   :: Int,
    subsets         :: Vector{F},
    subsets_based_on_csvfileset :: DataFrame
    )

    for validfind in validfind_start : validfind_end
        if !_calc_frame_passes_subsets(pdset, sn, carind, validfind, subsets, subsets_based_on_csvfileset)
            return false
        end
    end
    true
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
function _calc_subsets_based_on_csvfileset(csvfileset::CSVFileSet, nframes::Int)

    df = DataFrame()
    df[symbol(SUBSET_FREE_FLOW)]     = _calc_subset_vector(csvfileset.freeflow, nframes)
    df[symbol(SUBSET_CAR_FOLLOWING)] = _calc_subset_vector(csvfileset.carfollow, nframes)
    df[symbol(SUBSET_LANE_CROSSING)] = _calc_subset_vector([csvfileset.lanechanges_normal;
                                                        csvfileset.lanechanges_postpass;
                                                        csvfileset.lanechanges_arbitrary], nframes)

    df
end

function _control_input_exists_at_each_point(
    basics::FeatureExtractBasicsPdSet,
    carid::Integer,
    validfind_start::Integer,
    validfind_end::Integer,
    frames_per_sim::Integer,
    )

    for validfind = validfind_start : frames_per_sim : validfind_end

        carind = carid2ind(basics.pdset, carid, validfind)

        afut = get(FUTUREACCELERATION_250MS, basics, carind, validfind)
        if isnan(afut) || isinf(afut)
            return false
        end
        desang = get(FUTUREDESIREDANGLE_250MS, basics, carind, validfind)
        if isnan(desang) || isinf(desang)
            return false
        end
    end
    true
end

function pull_pdset_segments(
    extract_params::OrigHistobinExtractParameters,
    pdset::PrimaryDataset,
    sn::StreetNetwork,
    carid::Int,
    pdset_id::Integer,
    streetnet_id::Integer,
    validfinds::Vector{Int};
    max_dist::Float64 = 5000.0, # max_dist used in frenet_distance_between_points() [m]
    basics::FeatureExtractBasicsPdSet = FeatureExtractBasicsPdSet(pdset, sn),
    )

    #=
    Returns pdset_segments::PdsetSegments[],
        where each segment has length 0 -> SIM_HORIZON,
        they do not overlap over 0 : SIM_HORIZON,
        but the history SIM_HISTORY does exist.
    The control input must exist at every frame.

    Various conditions, as given in extract_params, must be true as well.
    =#

    const PDSET_FRAMES_PER_SIM_FRAME = extract_params.pdset_frames_per_sim_frame
    const SIM_HORIZON         = extract_params.sim_horizon
    const SIM_HISTORY         = extract_params.sim_history

    const TARGET_SPEED        = extract_params.target_speed
    const N_SIM_FRAMES        = total_framecount(extract_params)

    @assert(SIM_HISTORY ≥ 1)
    @assert(SIM_HORIZON ≥ 0)

    pdset_segments = PdsetSegment[]
    num_validfind_indeces = length(validfinds)

    validfind_index = 0
    while validfind_index < num_validfind_indeces

        validfind_index += 1

        validfind = validfinds[validfind_index]
        validfind_start = convert(Int, jumpframe(pdset, validfind, -(SIM_HISTORY-1) * PDSET_FRAMES_PER_SIM_FRAME))
        validfind_end   = convert(Int, jumpframe(pdset, validfind,   SIM_HORIZON    * PDSET_FRAMES_PER_SIM_FRAME))

        # println(validfind_start, "  ", validfind, "  ", validfind_end)
        # println(validfind_start != 0, "  ", validfind_end != 0)
        # println(are_validfinds_continuous(pdset, validfind_start, validfind_end))
        # println(_calc_frames_pass_subsets(pdset, sn, CARIND_EGO, validfind, validfind_end, extract_params.subsets, df_subsets))

        if validfind_start == 0 || validfind_end == 0
            # println(validfind_index, " failed start or end")
            continue
        end

        # ensure all points are within validfinds
        all_points_are_in_validfinds = true
        vi = 1
        for v in validfind_start : PDSET_FRAMES_PER_SIM_FRAME : validfind_end
            while vi < num_validfind_indeces && validfinds[vi] < v
                vi += 1
            end
            if validfinds[vi] != v
                all_points_are_in_validfinds = false
                break
            end
        end

        if all_points_are_in_validfinds &&
           are_validfinds_continuous(pdset, validfind_start, validfind_end)

            carind = carid2ind(pdset, carid, validfind)
            posFy     =      get(pdset, :posFy,   carind, validfind)::Float64
            ϕ         =      get(pdset, :posFyaw, carind, validfind)::Float64
            d_cl      =      get(pdset, :d_cl,    carind, validfind)::Float64
            v_orig    =      get(SPEED,     basics, carind, validfind)::Float64
            ω         =      get(TURNRATE,  basics, carind, validfind)::Float64
            a         =      get(ACC,       basics, carind, validfind)::Float64
            has_front =  convert(Bool, get(HAS_FRONT, basics, carind, validfind)::Float64)
            d_x_front =      get(D_X_FRONT, basics, carind, validfind)::Float64
            d_y_front =      get(D_Y_FRONT, basics, carind, validfind)::Float64
            v_x_front =      get(V_X_FRONT, basics, carind, validfind)::Float64
            nll       =  convert(Int, get(N_LANE_L,  basics, carind, validfind)::Float64)
            nlr       =  convert(Int, get(N_LANE_R,  basics, carind, validfind)::Float64)

            if  abs(d_cl)                ≤ extract_params.tol_d_cl &&
                abs(ϕ)                   ≤ extract_params.tol_yaw &&
                abs(ω)                   ≤ extract_params.tol_turnrate &&
                abs(a)                   ≤ extract_params.tol_accel &&
                abs(v_orig-TARGET_SPEED) ≤ extract_params.tol_speed &&
                d_x_front                ≤ extract_params.tol_d_x_front &&
                abs(d_y_front)           ≤ extract_params.tol_d_y_front &&
                v_x_front                ≤ extract_params.tol_d_v_front #&&
                #(nll + nlr) > 0

                carind_start = carid2ind(pdset, carid, validfind_start)
                carind_end = carid2ind(pdset, carid, validfind_end)
                posGxA = get(pdset, :posGx, carind_start, validfind_start)
                posGyA = get(pdset, :posGy, carind_start, validfind_start)
                posGxB = get(pdset, :posGx, carind_end, validfind_end)
                posGyB = get(pdset, :posGy, carind_end, validfind_end)

                # NOTE(tim): may be NAN if we cross from a lane that terminates into a new lane that doesn't
                #            In this case we cannot compute the frenet (Δx, Δy) unless we do something special
                #            like extrapolate the missing lane (which is iffy)
                (Δx, Δy) = frenet_distance_between_points(sn, posGxA, posGyA, posGxB, posGyB, max_dist)

                # if isnan(Δx)
                    # @printf("posA: %15.6f, %15.6f\n", posGxA, posGyA)
                    # @printf("posB: %15.6f, %15.6f\n", posGxB, posGyB)
                    # println("proj: ", project_point_to_streetmap(posGxA, posGyA, sn))
                    # println("proj: ", project_point_to_streetmap(posGxB, posGyB, sn))
                # end

                if !isnan(Δx)
                    push!(pdset_segments, PdsetSegment(pdset_id, streetnet_id, carid, validfind, validfind_end))
                    validfind_next = validfind_end + max(extract_params.frameskip_between_extracted_scenes * PDSET_FRAMES_PER_SIM_FRAME - 1, 0)
                    while validfind_index < num_validfind_indeces && validfinds[validfind_index] < validfind_next
                        validfind_index += 1
                    end
                end
            end
        end
    end

    # println(pdset_segments)

    pdset_segments
end

function is_strictly_monotonically_increasing(vec::AbstractVector{Int})
    if isempty(vec)
        return true
    end

    val = vec[1]
    for i = 2 : length(vec)
        val2 = vec[i]
        if val2 ≤ val
            return false
        end
        val = val2
    end
    true
end
function merge_region_segments(vec::AbstractVector{Int})

    #=
    sorts a set of region segments to be in monotonically increasing order
    will also merge them as necessary if they overlap
    =#

    if isempty(vec)
        return Int[]
    end

    n = length(vec)
    @assert(mod(n,2) == 0)

    validfind_min, validfind_max = extrema(vec)

    validfinds = falses(validfind_max-validfind_min+1)
    for i = 1 : div(n,2)
        validfind_lo = vec[2i-1]-validfind_min+1
        validfind_hi = vec[2i]-validfind_min+1
        for j = validfind_lo : validfind_hi
            validfinds[j] = true
        end
    end

    retval = Int[]
    validfind_lo = findfirst(validfinds)
    while validfind_lo != 0
        validfind_hi = findnext(val->!val, validfinds, validfind_lo+1)
        if validfind_hi == 0
            validfind_hi = length(validfinds)
        end
        push!(retval, validfind_lo+validfind_min-1)
        push!(retval, validfind_hi+validfind_min-1)

        validfind_lo = findnext(validfinds, validfind_hi+1)
    end

    retval
end
function calc_row_count_from_region_segments(validfind_regions::AbstractVector{Int})

    #=
    Computes the number of frames encoded in validfind_regions

    validfind_regions is an array in which subsequent value pairs correspond to continuous frames
    containing a particular behavior.
    validfind_regions should be strictly monotonically increasing

    ex: [1,4,8,10] means 1→4 and 8→10 are continuous frames
    =#

    n = length(validfind_regions)
    if n == 0
        return 0
    end

    if !is_strictly_monotonically_increasing(validfind_regions)
        println("not strincy increasing: ", validfind_regions)
    end

    @assert(mod(n, 2) == 0) # must be an even number
    @assert(is_strictly_monotonically_increasing(validfind_regions))

    estimated_row_count = 0
    index_lo = 1
    while index_lo < length(validfind_regions)
        estimated_row_count += validfind_regions[index_lo+1] - validfind_regions[index_lo] + 1
        index_lo += 2
    end
    estimated_row_count
end

function pull_pdset_segments_and_dataframe(
    extract_params  :: OrigHistobinExtractParameters,
    csvfileset      :: CSVFileSet,
    pdset           :: PrimaryDataset,
    sn              :: StreetNetwork,
    pdset_id        :: Integer,
    streetnet_id    :: Integer,
    features::Vector{AbstractFeature},
    filters::Vector{AbstractFeature};
    pdset_frames_per_sim_frame::Int=N_FRAMES_PER_SIM_FRAME
    )

    basics = FeatureExtractBasicsPdSet(pdset, sn)

    ########################################################################################################
    # pull all of the validfinds that pass the filters

    subsets_based_on_csvfileset = _calc_subsets_based_on_csvfileset(csvfileset, nvalidfinds(pdset))
    validfinds = filter(1 : nvalidfinds(pdset)) do validfind
                    carind = carid2ind(pdset, csvfileset.carid, validfind)
                    _calc_frame_passes_subsets(pdset, sn, carind, validfind, extract_params.subsets, subsets_based_on_csvfileset) &&
                        !does_violate_filter(filters, basics, carind, validfind)
                 end


    ########################################################################################################
    # pull all of the pdset_segments that lie within validfinds
    pdset_segments = pull_pdset_segments(extract_params, pdset, sn, csvfileset.carid,
                                         pdset_id, streetnet_id, validfinds,
                                         basics=basics)

    ########################################################################################################
    # now only extract feature frames for each pdset_segment, at intervals of pdset_frames_per_sim_frame
    # and valid frames spaced by pdset_frames_per_sim_frame

    # 0 -> do not use
    # 1 -> is potentially valid
    # 2 -> extracted me
    sample_from_frame = zeros(Int, nvalidfinds(pdset))

    # mark all validfind frames as potentially valid
    for validfind in validfinds
        sample_from_frame[validfind] = 1
    end

    # mark all frames from segments for extraction
    for seg in pdset_segments

        for Δ in seg.validfind_end+1 : seg.validfind_end+pdset_frames_per_sim_frame-1
            sample_from_frame[Δ] = 0 # forward buffer not extracted
        end
        for Δ in seg.validfind_start-1 : -1 : seg.validfind_start-pdset_frames_per_sim_frame+1
            sample_from_frame[Δ] = 0 # backward buffer not extracted
        end

        for validfind in seg.validfind_start : pdset_frames_per_sim_frame : seg.validfind_end
            @assert(sample_from_frame[validfind]==1)
            sample_from_frame[validfind] = 2
            for Δ in 1 : pdset_frames_per_sim_frame-1
                sample_from_frame[validfind+Δ] = 0 # in-between ones should not be used
            end
        end
    end

    # mark all remaining potential frames as extractable with buffer in-between
    for validfind in 1 : length(sample_from_frame)
        if sample_from_frame[validfind] == 1
            sample_from_frame[validfind] = 2 # extract it
            for Δ in 1 : pdset_frames_per_sim_frame-1
                sample_from_frame[validfind+Δ] = 0 # in-between ones should not be used
            end
        end
    end

    validfinds_to_extract = find(sample_from_frame)

    ########################################################################################################
    # extract dataframe on those frames

    dataframe = gen_featureset_from_validfinds(csvfileset.carid, basics, validfinds_to_extract, features, filters)
    dataframe[:pdset_id] = fill(pdset_id, nrow(dataframe)) # add a column for pdset_id

    ########################################################################################################
    # build dataframe_nona

    dataframe_nona = deepcopy(dataframe)
    for sym in names(dataframe_nona)
        if is_symbol_a_feature(sym)
            F = symbol2feature(sym)
            for (i,v) in enumerate(dataframe_nona[sym])
                @assert(!isnan(v))
                if isinf(v)
                    validfind = validfinds[i]
                    carind = carid2ind(pdset, csvfileset.carid, validfind)
                    dataframe_nona[i,sym] = replace_na(F, basics, carind, validfind)
                end
            end
        end
    end

    ########################################################################################################
    # reconstruct startframes


    startframes = Array(Int, length(pdset_segments))
    startframe = 1
    for (i,seg) in enumerate(pdset_segments)
        while validfinds_to_extract[startframe] != seg.validfind_start
            startframe += 1
        end
        startframes[i] = startframe
    end

    # println("nvalidfinds: ", nvalidfinds(pdset))
    # println("pdset_segments: ", pdset_segments)
    # println("startframes: ", startframes)
    # println("dataframe:", dataframe)

    (pdset_segments, dataframe, dataframe_nona, startframes)
end
function _pull_model_training_data(
    extract_params::OrigHistobinExtractParameters,
    csvfilesets::Vector{CSVFileSet},
    features::Vector{AbstractFeature},
    filters::Vector{AbstractFeature},
    pdset_dir::AbstractString,
    streetmap_base::AbstractString,
    )

    streetnet_cache = Tuple{AbstractString, StreetNetwork}[]

    pdset_filepaths = AbstractString[]
    pdset_segments = PdsetSegment[]
    dataframe = create_dataframe_with_feature_columns(features, 0)
    dataframe[:pdset_id] = Int[]
    dataframe_nona = deepcopy(dataframe)
    startframes = Int[]

    for csvfileset in csvfilesets

        # tic()

        csvfile = csvfileset.csvfile
        streetmapbasename = csvfileset.streetmapbasename

        if streetmapbasename == "skip"
            continue
        end

        pdset = _load_pdset(csvfile, pdset_dir)
        push!(pdset_filepaths, _get_pdsetfile(csvfile, pdset_dir))
        pdset_id = length(pdset_filepaths)


        streetnet_id = findfirst(tup->tup[1]==streetmapbasename, streetnet_cache)
        if streetnet_id == 0
            streetnet = load(joinpath(streetmap_base, "streetmap_"*streetmapbasename*".jld"))["streetmap"]
            push!(streetnet_cache, (streetmapbasename, streetnet))
            streetnet_id = length(streetnet_cache)
        end
        sn = streetnet_cache[streetnet_id][2]


        # println(csvfile)

        more_pdset_segments, more_dataframe, more_dataframe_nona, more_startframes = pull_pdset_segments_and_dataframe(
                                                                                        extract_params, csvfileset, pdset, sn,
                                                                                        pdset_id, streetnet_id,
                                                                                        features, filters)

        more_startframes .+= nrow(dataframe)

        append!(pdset_segments, more_pdset_segments)
        append!(dataframe, more_dataframe)
        append!(dataframe_nona, more_dataframe_nona)
        append!(startframes, more_startframes)

        # toc()
    end

    streetnet_filepaths = Array(AbstractString, length(streetnet_cache))
    for streetnet_id = 1 : length(streetnet_cache)
        streetnet_filepaths[streetnet_id] = streetmap_base*"streetmap_"*streetnet_cache[streetnet_id][1]*".jld"
    end

    ModelTrainingData(pdset_filepaths, streetnet_filepaths, pdset_segments, dataframe, dataframe_nona, startframes)
end
function _pull_model_training_data_parallel(
    extract_params::OrigHistobinExtractParameters,
    csvfilesets::Vector{CSVFileSet},
    features::Vector{AbstractFeature},
    filters::Vector{AbstractFeature},
    pdset_dir::AbstractString,
    streetmap_base::AbstractString,
    )

    num_csvfilesets = length(csvfilesets)
    num_workers = nworkers()
    csvfileset_assignment = Array(Vector{CSVFileSet}, num_workers)
    for i = 1 : num_workers
        csvfileset_assignment[i] = CSVFileSet[]
    end

    worker = 0
    for csvfileset in csvfilesets

        worker += 1
        if worker > num_workers
            worker = 1
        end

        push!(csvfileset_assignment[worker], csvfileset)
    end

    # TODO(tim): fix this
    more_stuff = pmap(assigned_csvfilesets->_pull_pdsets_streetnets_segments_and_dataframe(
                                                extract_params, assigned_csvfilesets,
                                                features, filters, pdset_dir, streetmap_base
                                            ), csvfileset_assignment)

    pdset_filepaths = Array(AbstractString, num_csvfilesets)
    streetnet_filepaths = AbstractString[]
    pdset_segments = PdsetSegment[]
    dataframe = create_dataframe_with_feature_columns(features, 0)
    dataframe[:pdset_id] = Int[]
    startframes = Int[]

    pdset_index = 0
    for (more_pdsets, more_streetnets, more_pdset_segments, more_dataframe, more_startframes) in more_stuff

        more_startframes .+= nrow(dataframe)

        for (i,seg) in enumerate(more_pdset_segments)
            target_streetnet = more_streetnets[seg.streetnet_id]

            streetnet_id = findfirst(streetnet_filepaths, target_streetnet)
            if streetnet_id == 0
                push!(streetnet_filepaths, target_streetnet)
                streetnet_id = length(streetnet_filepaths)
            end

            push!(pdset_segments, PdsetSegment(seg.pdset_id, streetnet_id, seg.carid,
                                  seg.validfind_start, seg.validfind_end))
        end

        for pdset in more_pdsets
            pdset_index += 1
            pdset_filepaths[pdset_index] = pdset
        end

        append!(dataframe, more_dataframe)
        append!(startframes, more_startframes)
    end

    return ModelTrainingData(pdset_filepaths, streetnet_filepaths, pdset_segments, dataframe, startframes)
end
function pull_model_training_data(
    extract_params::OrigHistobinExtractParameters,
    csvfilesets::Vector{CSVFileSet};
    features::Vector{AbstractFeature}=FEATURES,
    filters::Vector{AbstractFeature}=AbstractFeature[],
    pdset_dir::AbstractString=PRIMARYDATA_DIR,
    streetmap_base::AbstractString="/media/tim/DATAPART1/Data/Bosch/processed/streetmaps/"
    )

    if nworkers() > 1
        _pull_model_training_data_parallel(extract_params, csvfilesets,
                                           features, filters, pdset_dir, streetmap_base)
    else
        _pull_model_training_data(extract_params, csvfilesets,
                                           features, filters, pdset_dir, streetmap_base)
    end
end

########################################
#             RunLog Code              #
########################################

function pull_model_training_data{S<:AbstractString}(
    extract_params::DatasetExtractParams,
    runlog_filepaths::Vector{S},
    )

    streetnet_cache = Dict{AbstractString, StreetNetwork}()
    id_target = extract_params.id_target

    # -----------------------------------------------------
    # compute the number of frames to be extracted

    tot_n_frames = 0
    frames_that_pass = Array(BitVector, length(runlog_filepaths))
    for (runlog_index, runlog_filepath) in enumerate(runlog_filepaths)

        runlog = JLD.load(runlog_filepath, "runlog")::RunLog
        frames_that_pass[runlog_index] = falses(nframes(runlog))
        pass = frames_that_pass[runlog_index]

        streetmapbasename = runlog.header.map_name
        if !haskey(streetnet_cache, streetmapbasename)
            streetnet_cache[streetmapbasename] = load(joinpath("/media/tim/DATAPART1/Data/Bosch/processed/streetmaps/", "streetmap_" * streetmapbasename * ".jld"), "streetmap")
        end
        sn = streetnet_cache[streetmapbasename]

        # compute the number of frames that pass the filters
        # (assuming we are only extracting data for id_target)
        for frame in 1 : nframes(runlog)
            colset = id2colset(runlog, id_target, frame)
            if colset != COLSET_NULL

                behavior = get(runlog, colset, frame, :behavior)::UInt16

                if  (behavior & extract_params.behavior_to_match) > 0 &&
                    (behavior & extract_params.behavior_to_avoid) == 0 &&
                    findfirst(f->FeaturesNew.get(f, runlog, sn, colset, frame) != 1.0, extract_params.filters) == 0

                    tot_n_frames += 1
                    pass[frame] = true
                end
            end
        end
    end

    println("tot_n_frames: ", tot_n_frames)
    println("pass: ", sum(p->sum(p), frames_that_pass)) # should be equal to tot_n_frames

    # -----------------------------------------------------
    # extract dataframe and dataframe_nona

    # dataframe = create_dataframe_with_feature_columns(extract_params.features, tot_n_frames)
    dataframe = DataFrame()
    for f in extract_params.features
        dataframe[FeaturesNew.symbol(f)] = DataArray(Float64, tot_n_frames)
    end

    dataframe_nona = deepcopy(dataframe)

    df_index = 0
    for (runlog_index, runlog_filepath) in enumerate(runlog_filepaths)
        runlog = JLD.load(runlog_filepath, "runlog")::RunLog
        sn = streetnet_cache[runlog.header.map_name]
        pass = frames_that_pass[runlog_index]

        for frame in 1 : nframes(runlog)
            if pass[frame]

                df_index += 1

                colset = id2colset(runlog, id_target, frame)
                assert(colset != COLSET_NULL)

                # extract the frame
                for f in extract_params.features
                    v = get(f, runlog, sn, colset, frame)::Float64
                    dataframe[df_index, symbol(f)] = v
                    dataframe_nona[df_index, symbol(f)] = isinf(v) ?
                        FeaturesNew.replace_na(f)::Float64 : v
                end
            end
        end
    end

    # -----------------------------------------------------
    # extract the runlog segments

    runlog_segments = RunLogSegment[]
    seg_start_to_index_in_dataframe = Int[]

    runlog_frames_per_sim_frame = extract_params.seg.runlog_frames_per_sim_frame
    n_frames_for_history = (extract_params.seg.sim_history - 1) * runlog_frames_per_sim_frame
    n_frames_for_horizon = extract_params.seg.sim_horizon * runlog_frames_per_sim_frame
    n_frames_between_starts = n_frames_for_horizon + extract_params.seg.frameskip_between_extracted_scenes * runlog_frames_per_sim_frame

    runlog_to_index_in_dataframe = 0
    for (runlog_index, runlog_filepath) in enumerate(runlog_filepaths)

        pass = frames_that_pass[runlog_index]
        continuous_segments = CommonTypes.continuous_segments(pass)

        for (range_start, range_end) in continuous_segments

            range_start = max(range_start, n_frames_for_history) # ensure history is taken care of
            frame_start = range_start
            frame_end = frame_start + n_frames_for_horizon

            while range_start ≤ frame_start && frame_end ≤ range_end

                push!(runlog_segments, RunLogSegment(runlog_index, id_target, frame_start, frame_end))
                push!(seg_start_to_index_in_dataframe, runlog_to_index_in_dataframe + sum(pass[1:frame_start]))

                frame_start += n_frames_between_starts
                frame_end += n_frames_between_starts
            end
        end

        runlog_to_index_in_dataframe += sum(pass)
    end

    # -----------------------------------------------------
    # return

    ModelTrainingData2(runlog_filepaths, runlog_segments, dataframe, dataframe_nona, seg_start_to_index_in_dataframe)
end

########################################
#            Fold Assignment           #
########################################

type FoldAssignment
    # assigns each frame and pdsetseg to
    # an integer fold
    # first fold is fold 1
    frame_assignment::Vector{Int}
    seg_assignment::Vector{Int}
    nfolds::Int

    function FoldAssignment(frame_assignment::Vector{Int}, seg_assignment::Vector{Int}, nfolds::Int)

        # some precautions...
        # for a in frame_assignment
        #     @assert(a ≤ nfolds)
        # end
        # for a in seg_assignment
        #     @assert(a ≤ nfolds)
        # end

        new(frame_assignment, seg_assignment, nfolds)
    end
    function FoldAssignment(frame_assignment::Vector{Int}, seg_assignment::Vector{Int})

        # some precautions...
        nfolds = -1
        for a in frame_assignment
            @assert(a > 0)
            nfolds = max(a, nfolds)
        end
        for a in seg_assignment
            @assert(a > 0)
            nfolds = max(a, nfolds)
        end

        new(frame_assignment, seg_assignment, nfolds)
    end
end
const FOLD_TRAIN = 1
const FOLD_TEST = 2

Base.deepcopy(a::FoldAssignment) = FoldAssignment(deepcopy(a.frame_assignment), deepcopy(a.seg_assignment), a.nfolds)

function calc_fold_size{I<:Integer}(fold::Integer, fold_assignment::AbstractArray{I}, match_fold::Bool)
    fold_size = 0
    for a in fold_assignment
        if is_in_fold(fold, a, match_fold)
            fold_size += 1
        end
    end
    fold_size
end
function calc_fold_inds!{I<:Integer}(fold_inds::Vector{Int}, fold::Integer, fold_assignment::AbstractArray{I}, match_fold::Bool)
    k = 0
    for (i,a) in enumerate(fold_assignment)
        if is_in_fold(fold, a, match_fold)
            k += 1
            fold_inds[k] = i
        end
    end
    @assert(k == length(fold_inds))
    fold_inds
end
function is_in_fold(fold::Integer, fold_assignment::Integer, match_fold::Bool)
    (fold != 0) && (fold_assignment != 0) && # NOTE(tim): zero never matches
        ((match_fold && fold_assignment == fold) || (!match_fold && fold_assignment != fold))
end
function drop_fold!(a::FoldAssignment, fold::Integer)
    for (i,v) in enumerate(a.frame_assignment)
        if v == fold
            a.frame_assignment[i] = 0
        elseif v > fold
            a.frame_assignment[i] -= 1
        end
    end
    a.nfolds -= 1
    a
end

function get_cross_validation_fold_assignment(
    nfolds::Int,
    dset::ModelTrainingData,
    )

    #=
    Want: to split the dataframe into k disjoint training sets of nearly the same size
          and simultaneously split the pdset_segments into k disjoint sets of nearly the same size

    Assumptions
    - pdset_segments are all of the same history and horizon
    - pdset_segments do not overlap

    1) randomly divide pdset_segments into k roughly equally-sized groups
    2) for each group, identify the corresponding frames
    3) assign those frames to each training set
    4) divide the remaining training frames among the training sets
    =#

    pdset_segments = dset.pdset_segments
    dataframe = dset.dataframe
    seg_to_framestart = dset.startframes

    nframes = nrow(dataframe)
    npdsetsegments = length(pdset_segments)

    @assert(length(seg_to_framestart) ≥ npdsetsegments)

    npdsetsegments_per_fold = div(npdsetsegments, nfolds)
    nfolds_with_extra_pdsetsegment = mod(npdsetsegments, nfolds)

    # println("nframes: ", nframes)
    # println("npdsetsegments: ", npdsetsegments)
    # println("npdsetsegments_per_fold: ", npdsetsegments_per_fold)
    # println("nfolds_with_extra_pdsetsegment: ", nfolds_with_extra_pdsetsegment)

    frame_fold_assignment = zeros(Int, nframes)
    pdsetsegment_fold_assignment = zeros(Int, npdsetsegments)

    p = randperm(npdsetsegments)
    permind = 0
    for f = 1 : nfolds
        # println(pdsetsegment_fold_assignment)
        npdsetsegments_in_fold = npdsetsegments_per_fold + (f ≤ nfolds_with_extra_pdsetsegment ? 1 : 0)
        for i = 1 : npdsetsegments_in_fold
            permind += 1

            pdsetsegmentind = p[permind]
            pdsetsegment_fold_assignment[pdsetsegmentind] = f

            framestart = seg_to_framestart[pdsetsegmentind]
            frameend = framestart + get_horizon(pdset_segments[pdsetsegmentind])
            # println(framestart,  "  ", frameend)
            frameind = framestart
            for _ in framestart : N_FRAMES_PER_SIM_FRAME : frameend
                @assert(frame_fold_assignment[frameind] == 0)
                frame_fold_assignment[frameind] = f
                frameind += 1
            end
        end
    end

    frame_fold_sizes = zeros(Int, nfolds)
    nremaining_frames = 0
    for frameind in 1 : nframes
        if frame_fold_assignment[frameind] == 0
            nremaining_frames += 1
        else
            frame_fold_sizes[frame_fold_assignment[frameind]] += 1
        end
    end

    # println("nremaining_frames: ", nremaining_frames)
    # println("frame_fold_sizes (before): ", frame_fold_sizes)

    fold_priority = PriorityQueue{Int, Int}() # (fold, foldsize), min-ordered
    for (fold,foldsize) in enumerate(frame_fold_sizes)
        fold_priority[fold] = foldsize
    end

    remaining_frames = Array(Int, nremaining_frames)
    rem_frame_count = 0
    for frameind in 1 : nframes
        if frame_fold_assignment[frameind] == 0
            remaining_frames[rem_frame_count+=1] = frameind
        end
    end

    p = randperm(nremaining_frames)
    for rem_frame_ind in 1 : nremaining_frames
        fold = dequeue!(fold_priority)
        frame_fold_assignment[remaining_frames[p[rem_frame_ind]]] = fold
        frame_fold_sizes[fold] += 1
        fold_priority[fold] = frame_fold_sizes[fold]
    end

    # println("frame_fold_sizes (after): ", frame_fold_sizes)

    FoldAssignment(frame_fold_assignment, pdsetsegment_fold_assignment, nfolds)
end
function get_cross_validation_fold_assignment(
    nfolds::Int,
    dset::ModelTrainingData,
    train_test_split::FoldAssignment,
    )

    #=
    Want: to split the dataframe into k disjoint training sets of nearly the same size
          and simultaneously split the pdset_segments into k disjoint sets of nearly the same size

    Assumptions
    - pdset_segments are all of the same history and horizon
    - pdset_segments do not overlap

    1) randomly divide pdset_segments into k roughly equally-sized groups
    2) for each group, identify the corresponding frames
    3) assign those frames to each training set
    4) divide the remaining training frames among the training sets
    =#

    pdset_segments = dset.pdset_segments
    dataframe = dset.dataframe
    seg_to_framestart = dset.startframes

    frame_tv_assignment = train_test_split.frame_assignment
    pdsetseg_tv_assignment = train_test_split.seg_assignment

    nframes = nrow(dataframe)
    npdsetsegments = length(pdset_segments)

    nframes_assigned_to_training = 0
    for v in frame_tv_assignment
        if v == FOLD_TRAIN
            nframes_assigned_to_training += 1
        end
    end

    npdsetsegments_assigned_to_training = 0
    for v in pdsetseg_tv_assignment
        if v == FOLD_TRAIN
            npdsetsegments_assigned_to_training += 1
        end
    end

    indeces_of_train_frames = Array(Int, nframes_assigned_to_training)
    ind = 1
    for (i,v) in enumerate(frame_tv_assignment)
        if v == FOLD_TRAIN
            indeces_of_train_frames[ind] = i
            ind += 1
        end
    end

    indeces_of_train_pdsetsegs = Array(Int, npdsetsegments_assigned_to_training)
    ind = 1
    for (i,v) in enumerate(pdsetseg_tv_assignment)
        if v == FOLD_TRAIN
            indeces_of_train_pdsetsegs[ind] = i
            ind += 1
        end
    end

    @assert(length(seg_to_framestart) ≥ npdsetsegments)

    npdsetsegments_per_fold = div(npdsetsegments_assigned_to_training, nfolds)
    nfolds_with_extra_pdsetsegment = mod(npdsetsegments_assigned_to_training, nfolds)

    # println("nframes: ", nframes)
    # println("npdsetsegments: ", npdsetsegments)
    # println("npdsetsegments_per_fold: ", npdsetsegments_per_fold)
    # println("nfolds_with_extra_pdsetsegment: ", nfolds_with_extra_pdsetsegment)

    # NOTE(tim): will only set frames and segs that are marked TRAIN
    frame_fold_assignment = zeros(Int, nframes)
    pdsetsegment_fold_assignment = zeros(Int, npdsetsegments)

    p = randperm(npdsetsegments_assigned_to_training)
    permind = 0
    for f = 1 : nfolds
        # println(pdsetsegment_fold_assignment)
        npdsetsegments_in_fold = npdsetsegments_per_fold + (f ≤ nfolds_with_extra_pdsetsegment ? 1 : 0)
        for i = 1 : npdsetsegments_in_fold
            permind += 1

            pdsetsegmentind = indeces_of_train_pdsetsegs[p[permind]]
            pdsetsegment_fold_assignment[pdsetsegmentind] = f

            framestart = seg_to_framestart[pdsetsegmentind]
            frameend = framestart + get_horizon(pdset_segments[pdsetsegmentind])
            frameind = framestart

            for _ in framestart : N_FRAMES_PER_SIM_FRAME : frameend
                @assert(frame_fold_assignment[frameind] == 0)
                frame_fold_assignment[frameind] = f
                frameind += 1
            end
        end
    end

    frame_fold_sizes = zeros(Int, nfolds)
    nremaining_frames = 0
    for frameind in 1 : nframes_assigned_to_training
        if frame_fold_assignment[indeces_of_train_frames[frameind]] == 0
            nremaining_frames += 1
        else
            frame_fold_sizes[frame_fold_assignment[indeces_of_train_frames[frameind]]] += 1
        end
    end

    # println("nremaining_frames: ", nremaining_frames)
    # println("frame_fold_sizes (before): ", frame_fold_sizes)

    fold_priority = PriorityQueue{Int, Int}() # (fold, foldsize), min-ordered
    for (fold,foldsize) in enumerate(frame_fold_sizes)
        fold_priority[fold] = foldsize
    end

    remaining_frames = Array(Int, nremaining_frames)
    rem_frame_count = 0
    for frameind in 1 : nframes_assigned_to_training
        if frame_fold_assignment[indeces_of_train_frames[frameind]] == 0
            remaining_frames[rem_frame_count+=1] = indeces_of_train_frames[frameind]
        end
    end

    if nremaining_frames > 0
        p = randperm(nremaining_frames)
        for rem_frame_ind in 1 : nremaining_frames
            fold = dequeue!(fold_priority)
            frame_fold_assignment[remaining_frames[p[rem_frame_ind]]] = fold
            frame_fold_sizes[fold] += 1
            fold_priority[fold] = frame_fold_sizes[fold]
        end
    end

    # println("frame_fold_sizes (after): ", frame_fold_sizes)

    FoldAssignment(frame_fold_assignment, pdsetsegment_fold_assignment, nfolds)
end
function get_cross_validation_fold_assignment(
    nfolds::Int,
    dset::ModelTrainingData2,
    train_test_split::FoldAssignment,
    )

    #=
    Want: to split the dataframe into k disjoint training sets of nearly the same size
          and simultaneously split the runlog_segments into k disjoint sets of nearly the same size

    Assumptions
    - runlog_segments are all of the same history and horizon
    - runlog_segments do not overlap

    1) randomly divide runlog_segments into k roughly equally-sized groups
    2) for each group, identify the corresponding frames
    3) assign those frames to each training set
    4) divide the remaining training frames among the training sets
    =#

    runlog_segments = dset.runlog_segments
    dataframe = dset.dataframe
    seg_start_to_index_in_dataframe = dset.seg_start_to_index_in_dataframe

    frame_tv_assignment = train_test_split.frame_assignment
    runlogseg_tv_assignment = train_test_split.seg_assignment

    nframes = nrow(dataframe)
    nrunlogsegments = length(runlog_segments)

    nframes_assigned_to_training = 0
    for v in frame_tv_assignment
        if v == FOLD_TRAIN
            nframes_assigned_to_training += 1
        end
    end

    nrunlogsegments_assigned_to_training = 0
    for v in runlogseg_tv_assignment
        if v == FOLD_TRAIN
            nrunlogsegments_assigned_to_training += 1
        end
    end

    indeces_of_train_frames = Array(Int, nframes_assigned_to_training)
    ind = 1
    for (i,v) in enumerate(frame_tv_assignment)
        if v == FOLD_TRAIN
            indeces_of_train_frames[ind] = i
            ind += 1
        end
    end

    indeces_of_train_runlogsegs = Array(Int, nrunlogsegments_assigned_to_training)
    ind = 1
    for (i,v) in enumerate(runlogseg_tv_assignment)
        if v == FOLD_TRAIN
            indeces_of_train_runlogsegs[ind] = i
            ind += 1
        end
    end

    @assert(length(seg_start_to_index_in_dataframe) ≥ nrunlogsegments)

    nrunlogsegments_per_fold = div(nrunlogsegments_assigned_to_training, nfolds)
    nfolds_with_extra_runlogsegment = mod(nrunlogsegments_assigned_to_training, nfolds)

    # println("nframes: ", nframes)
    # println("nrunlogsegments: ", nrunlogsegments)
    # println("nrunlogsegments_per_fold: ", nrunlogsegments_per_fold)
    # println("nfolds_with_extra_runlogsegment: ", nfolds_with_extra_runlogsegment)

    # NOTE(tim): will only set frames and segs that are marked TRAIN
    frame_fold_assignment = zeros(Int, nframes)
    runlogsegment_fold_assignment = zeros(Int, nrunlogsegments)

    p = randperm(nrunlogsegments_assigned_to_training)
    permind = 0
    for f = 1 : nfolds
        # println(runlogsegment_fold_assignment)
        nrunlogsegments_in_fold = nrunlogsegments_per_fold + (f ≤ nfolds_with_extra_runlogsegment ? 1 : 0)
        for i = 1 : nrunlogsegments_in_fold
            permind += 1

            runlogsegmentind = indeces_of_train_runlogsegs[p[permind]]
            runlogsegment_fold_assignment[runlogsegmentind] = f

            framestart = seg_start_to_index_in_dataframe[runlogsegmentind]
            frameend = framestart + get_horizon(runlog_segments[runlogsegmentind])
            frameind = framestart

            for _ in framestart : N_FRAMES_PER_SIM_FRAME : frameend
                @assert(frame_fold_assignment[frameind] == 0)
                frame_fold_assignment[frameind] = f
                frameind += 1
            end
        end
    end

    frame_fold_sizes = zeros(Int, nfolds)
    nremaining_frames = 0
    for frameind in 1 : nframes_assigned_to_training
        if frame_fold_assignment[indeces_of_train_frames[frameind]] == 0
            nremaining_frames += 1
        else
            frame_fold_sizes[frame_fold_assignment[indeces_of_train_frames[frameind]]] += 1
        end
    end

    # println("nremaining_frames: ", nremaining_frames)
    # println("frame_fold_sizes (before): ", frame_fold_sizes)

    fold_priority = PriorityQueue{Int, Int}() # (fold, foldsize), min-ordered
    for (fold,foldsize) in enumerate(frame_fold_sizes)
        fold_priority[fold] = foldsize
    end

    remaining_frames = Array(Int, nremaining_frames)
    rem_frame_count = 0
    for frameind in 1 : nframes_assigned_to_training
        if frame_fold_assignment[indeces_of_train_frames[frameind]] == 0
            remaining_frames[rem_frame_count+=1] = indeces_of_train_frames[frameind]
        end
    end

    if nremaining_frames > 0
        p = randperm(nremaining_frames)
        for rem_frame_ind in 1 : nremaining_frames
            fold = dequeue!(fold_priority)
            frame_fold_assignment[remaining_frames[p[rem_frame_ind]]] = fold
            frame_fold_sizes[fold] += 1
            fold_priority[fold] = frame_fold_sizes[fold]
        end
    end

    # println("frame_fold_sizes (after): ", frame_fold_sizes)

    FoldAssignment(frame_fold_assignment, runlogsegment_fold_assignment, nfolds)
end

function get_train_test_fold_assignment(
    fraction_test::Float64,
    dset::ModelTrainingData,
    )

    #=
    returns a FoldAssignment with two folds: 1 for train, 2 for test

    Works by splitting over pdset_id's in order to get the closest match to
    fraction_validation on the test vs train frames
    =#

    @assert(0.0 ≤ fraction_test ≤ 1.0)

    pdset_segments = dset.pdset_segments
    dataframe = dset.dataframe

    nframes = nrow(dataframe)
    npdsetsegments = length(pdset_segments)

    # count the number of frames and traces in each pdset

    n_frames_per_pdset = Dict{Int,Int}()
    for id in dataframe[:pdset_id]

        if !haskey(n_frames_per_pdset, id)
            n_frames_per_pdset[id] = 0
        end
        n_frames_per_pdset[id] = n_frames_per_pdset[id] + 1
    end

    # solve the subset-sum problem of picking your pdset_ids such that we don't
    # go over fraction_test

    n_pdset_ids = length(n_frames_per_pdset)
    subsetsum_capacity = floor(Int, sum(Base.values(n_frames_per_pdset)) * fraction_test)
    selected_for_test = falses(n_pdset_ids)
    ids = collect(keys(n_frames_per_pdset))
    values = convert(Vector{Int}, map(i->n_frames_per_pdset[i], ids))

    _subset_sum!(selected_for_test, values, subsetsum_capacity)

    # now use that split

    a_frame = zeros(Int, nframes)
    a_seg = zeros(Int, npdsetsegments)

    for (i,id) in enumerate(ids)
        label = selected_for_test[i] ? FOLD_TEST : FOLD_TRAIN

        for (j,pdset_id) in enumerate(dataframe[:pdset_id])
            if id == pdset_id
                a_frame[j] = label
            end
        end
        for (j,pdset) in enumerate(pdset_segments)
            if pdset.pdset_id == id
                a_seg[j] = label
            end
        end
    end

    FoldAssignment(a_frame, a_seg, 2)
end

function get_fold_assignment_across_drives(dset::ModelTrainingData)

    #=
    returns a FoldAssignment with one fold for each drive
    Works by splitting over pdset_id's
    =#

    pdset_segments = dset.pdset_segments
    dataframe = dset.dataframe

    nframes = nrow(dataframe)
    npdsetsegments = length(pdset_segments)

    a_frame = zeros(Int, nframes)
    a_seg = zeros(Int, npdsetsegments)

    pdset_id_to_fold = Dict{Int,Int}()
    for (i, id) in enumerate(dataframe[:pdset_id])
        if !haskey(pdset_id_to_fold, id)
            pdset_id_to_fold[id] = length(pdset_id_to_fold) + 1
        end

        a_frame[i] = pdset_id_to_fold[id]
    end
    for (i, pdset) in enumerate(pdset_segments)
        a_seg[i] = pdset_id_to_fold[pdset.pdset_id]
    end

    FoldAssignment(a_frame, a_seg, length(pdset_id_to_fold))
end
function get_fold_assignment_across_drives(dset::ModelTrainingData2)

    #=
    returns a FoldAssignment with one fold for each drive
    Works by splitting over runlog_id's
    =#

    dataframe = dset.dataframe
    runlog_segments = dset.runlog_segments

    a_frame = zeros(Int, nrow(dataframe))
    a_seg = zeros(Int, length(runlog_segments))

    runlog_id_to_fold = Dict{Int,Int}()
    for (i, runlogseg) in enumerate(runlog_segments)

        id = runlogseg.runlog_id

        if !haskey(runlog_id_to_fold, id)
            runlog_id_to_fold[id] = length(runlog_id_to_fold) + 1
        end

        df_frame_start = dset.seg_start_to_index_in_dataframe[i]
        df_frame_end = df_frame_start + runlogseg.frame_end - runlogseg.frame_start
        for df_frame in df_frame_start : df_frame_end
            a_frame[df_frame] = runlog_id_to_fold[id]
        end

        a_seg[i] = runlog_id_to_fold[runlogseg.runlog_id]
    end

    FoldAssignment(a_frame, a_seg, length(runlog_id_to_fold))
end
function get_fold_assignment_across_drives(dset::ModelTrainingData2, nfolds::Int)

    #=
    returns a FoldAssignment with `nfolds` folds, where each drive is always entirely
    assigned to a particular fold.
    Attempts to split as evenly as possible (by number of traces).
    Works by splitting over runlog_id's
    =#

    # runlog_id
    # frame_start
    # frame_end
    # seg_start_to_index_in_dataframe

    dataframe = dset.dataframe
    runlog_segments = dset.runlog_segments

    runlog_id_to_index = Dict{Int,Int}()
    runlog_id_ntraces = Int[]

    for (i, runlogseg) in enumerate(runlog_segments)

        id = runlogseg.runlog_id

        if !haskey(runlog_id_to_index, id)
            runlog_id_to_index[id] = length(runlog_id_to_index) + 1
            push!(runlog_id_ntraces, 1)
        else
            runlog_id_ntraces[runlog_id_to_index[id]] += 1
        end
    end

    id_to_fold = multiprocessor_scheduling_longest_processing_time(runlog_id_ntraces, nfolds)

    a_frame = zeros(Int, nrow(dataframe))
    a_seg = zeros(Int, length(runlog_segments))

    for (i, runlogseg) in enumerate(runlog_segments)

        fold = id_to_fold[runlog_id_to_index[runlogseg.runlog_id]]

        a_seg[i] = fold

        df_frame_start = dset.seg_start_to_index_in_dataframe[i]
        df_frame_end = df_frame_start + runlogseg.frame_end - runlogseg.frame_start
        for df_frame in df_frame_start : df_frame_end
            a_frame[df_frame] = fold
        end
    end

    FoldAssignment(a_frame, a_seg, nfolds)
end

function get_fold_assignment_across_traces(dset::ModelTrainingData)

    #=
    returns a FoldAssignment with one fold for each pdsetsetseg
    Works by splitting over pdset_id's
    =#

    pdset_segments = dset.pdset_segments
    dataframe = dset.dataframe
    seg_to_framestart = dset.startframes

    nframes = nrow(dataframe)
    nfolds = npdsetsegments = length(pdset_segments)

    a_frame = zeros(Int, nframes)
    a_seg = zeros(Int, npdsetsegments)

    for (i, pdset) in enumerate(pdset_segments)
        a_seg[i] = i

        framestart = seg_to_framestart[i]
        frameend = framestart + get_horizon(pdset)
        frameind = framestart

        for _ in framestart : N_FRAMES_PER_SIM_FRAME : frameend
            @assert(a_frame[frameind] == 0) # not yet assigned
            a_frame[frameind] = i
            frameind += 1
        end
    end

    # determine how many frames must still be assigned
    # and how many each fold already has
    nremaining_frames = 0
    frame_fold_sizes = zeros(Int, nfolds)
    for i in 1 : nframes
        if a_frame[i] == 0
            nremaining_frames += 1
        else
            frame_fold_sizes[a_frame[i]] += 1
        end
    end

    # create a priority queue based on frame count
    fold_priority = PriorityQueue{Int, Int}() # (fold, foldsize), min-ordered
    for (fold,foldsize) in enumerate(frame_fold_sizes)
        fold_priority[fold] = foldsize
    end

    # determine what frames are remaining
    remaining_frames = Array(Int, nremaining_frames)
    rem_frame_count = 0
    for i in 1 : nframes
        if a_frame[i] == 0
            remaining_frames[rem_frame_count+=1] = i
        end
    end

    # assign the remaining frames to even out counts as best we can
    if nremaining_frames > 0
        for i in remaining_frames[randperm(nremaining_frames)]
            fold = dequeue!(fold_priority)
            a_frame[i] = fold
            frame_fold_sizes[fold] += 1
            fold_priority[fold] = frame_fold_sizes[fold]
        end
    end

    FoldAssignment(a_frame, a_seg, nfolds)
end

function load_pdsets(dset::ModelTrainingData)
    pdsets = Array(PrimaryDataset, length(dset.pdset_filepaths))
    for (i,pdset_filepath) in enumerate(dset.pdset_filepaths)
        pdsets[i] = load(pdset_filepath, "pdset")
    end
    pdsets
end
function load_runlogs(dset::ModelTrainingData2)
    runlogs = Array(RunLog, length(dset.runlog_filepaths))
    for (i,runlog_filepath) in enumerate(dset.runlog_filepaths)
        runlogs[i] = load(runlog_filepath, "runlog")
    end
    runlogs
end
function load_streetnets(dset::ModelTrainingData)
    streetnets = Array(StreetNetwork, length(dset.streetnet_filepaths))
    for (i,streetnet_filepath) in enumerate(dset.streetnet_filepaths)
        streetnets[i] = load(streetnet_filepath, "streetmap")
    end
    streetnets
end
function load_streetnets(runlogs::AbstractVector{RunLog})

    streetnets = Dict{AbstractString, StreetNetwork}()
    for runlog in runlogs
        streetmapbasename = runlog.header.map_name
        if !haskey(streetnets, streetmapbasename)
            streetnets[streetmapbasename] = JLD.load(joinpath("/media/tim/DATAPART1/Data/Bosch/processed/streetmaps/", "streetmap_" * streetmapbasename * ".jld"), "streetmap")
        end
    end
    streetnets
end

function _subset_sum!(
    selected::BitVector,    # whether the item is selected
    w::AbstractVector{Int}, # vector of item weights (bigger is worse)
    W::Int,                 # knapsack capacity (W ≤ ∑w)
    )

    #=
    Solves the 0-1 Subset Sum Problem
    https://en.wikipedia.org/wiki/Subset_sum_problem
    Returns the assigment vector such that
      the max weight ≤ W is obtained
    =#

    fill!(selected, false)

    if W ≤ 0
        return selected
    end

    n = length(w)
    @assert(all(w .> 0))

    ###########################################
    # allocate DP memory

    m = Array(Int, n+1, W+1)
    for j in 0:W
        m[1, j+1] = 0.0
    end

    ###########################################
    # solve knapsack with DP

    for i in 1:n
        for j in 0:W
            if w[i] ≤ j
                m[i+1, j+1] = max(m[i, j+1], m[i, j-w[i]+1] + w[i])
            else
                m[i+1, j+1] = m[i, j+1]
            end
        end
    end

    ###########################################
    # recover the value

    line = W
    for i in n : -1 : 1
        if line - w[i] + 1 > 0 && m[i+1,line+1] - m[i, line - w[i] + 1] == w[i]
            selected[i] = true
            line -= w[i]
        end
    end

    selected
end

"""
Solves the Multiprocessor Scheduling problem using the Longest Processing Time algorithm

    PROBLEM: (NP-hard)
        Given:
            - set of jobs, each with a length
            - a number of processors
        Find:
            - divide the jobs among the processors such that none overlap
              which minimizes the total processing time

    ALGORITHM:
        - sort the jobs by processing time
        - assign them to the machine with the earliest end time so far
        Achieves an upper bound of 4/3 - 1/(3m) of optimal

    RETURNS:
        assignments, ith index → machine for the ith job
"""
function multiprocessor_scheduling_longest_processing_time{R<:Real}(
    jobs::AbstractVector{R},
    m::Integer, # number of processors
    )

    # TODO: use a priority queue

    @assert(m > 0)

    durations = zeros(R, m)
    assignments = Array(Int, length(jobs))

    for i in sortperm(jobs, rev=true) # p[1] is the index of the longest job in `jobs`
        best_index = indmin(durations)
        durations[best_index] += jobs[i]
        assignments[i] = best_index
    end

    assignments
end


end # end module
