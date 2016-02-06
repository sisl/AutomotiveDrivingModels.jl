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
using AutomotiveDrivingModels.FeaturesetExtractor
# using AutomotiveDrivingModels.Curves # TODO: remove this

import Base.Collections: PriorityQueue, dequeue!
import AutomotiveDrivingModels.FeaturesetExtractor: create_dataframe_with_feature_columns

export
        OrigHistobinExtractParameters,
        # StartCondition,

        RunLogSegmentExtractParameters,
        RunLogSegment,
        ModelTrainingData2,
        DatasetExtractParams,

        ###########
        # Fold Assignment Stuff

        FoldAssignment,
        FoldSet,

        FOLD_TRAIN,
        FOLD_TEST,

        is_in_fold,
        drop_fold!,
        assign_all_non_test_to_train!,
        calc_fold_inds!,
        calc_fold_inds,

        #########

        total_framecount,
        calc_traces_to_keep_with_frameskip,

        pull_model_training_data,

        is_strictly_monotonically_increasing,
        merge_region_segments,
        calc_row_count_from_region_segments,

        get_cross_validation_fold_assignment,
        get_train_test_fold_assignment,
        get_fold_assignment_across_drives,
        get_fold_assignment_across_traces,


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
    features          :: Vector{AbstractFeature}
    filters           :: Vector{AbstractFeature}
    seg               :: RunLogSegmentExtractParameters

    function DatasetExtractParams(
        behavior_to_match::UInt16,
        behavior_to_avoid::UInt16,
        features::Vector{AbstractFeature},
        seg::RunLogSegmentExtractParameters;
        filters::Vector{AbstractFeature} = Array(AbstractFeature, 0),
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

    runlog_id   :: Int # index within ModelTrainingData2.runlog_filepaths of the relevant runlog
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
    @printf(io, "\trunlog_id:   %d\n", seg.runlog_id)
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
                    findfirst(f->get(f, runlog, sn, colset, frame) != 1.0, extract_params.filters) == 0

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
        dataframe[symbol(f)] = DataArray(Float64, tot_n_frames)
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
                        replace_na(f)::Float64 : v
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

const FOLD_TRAIN = 1
const FOLD_TEST = 2

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

Base.deepcopy(a::FoldAssignment) = FoldAssignment(deepcopy(a.frame_assignment), deepcopy(a.seg_assignment), a.nfolds)
function Base.show(io::IO, a::FoldAssignment)
    println(io, "FoldAssignment")
    @printf(io, "\tstored nfolds:   %d\n", a.nfolds)
    @printf(io, "\tnframes:         %d\n", length(a.frame_assignment))
    @printf(io, "\tnsegs:           %d", length(a.seg_assignment))
    # @printf(io, "fold       nframes        nsegs\n")
    # for fold in sort!(unique(vcat(a.frame_assignment, a.seg_assignment)))
    #     if fold == 0
    #         continue
    #     end
    #     @printf(io, "%5d    %10d  %10d\n", fold, calc_fold_size(fold, a.frame_assignment, true),
    #                                              calc_fold_size(fold, a.seg_assignment, true))
    # end
    # @printf(io, "%5d    %10d  %10d\n", 0, sum(v->v ≤ 0, a.frame_assignment), sum(v->v ≤ 0, a.seg_assignment))
end

calc_num_folds_frames(a::FoldAssignment) = sum(v->v > 0, unique(a.frame_assignment))
calc_num_folds_segs(a::FoldAssignment) = sum(v->v > 0, unique(a.seg_assignment))
calc_num_folds(a::FoldAssignment) = sum(v->v > 0, unique(vcat(a.frame_assignment, a.seg_assignment)))

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
function assign_all_non_test_to_train!(a::FoldAssignment)

    for (j,v) in enumerate(a.frame_assignment)
        if 0 < v && v != FOLD_TEST
            a.frame_assignment[j] = FOLD_TRAIN
        end
    end
    for (j,v) in enumerate(a.seg_assignment)
        if 0 < v && v != FOLD_TEST
            a.seg_assignment[j] = FOLD_TRAIN
        end
    end
    a.nfolds = 2

    a
end

#########################################

immutable FoldSet
    assignment::Vector{Int}
    fold::Int
    match_fold::Bool

    function FoldSet(assignment::FoldAssignment, fold::Int, match_fold::Bool, target::Symbol)

        if target != :frame && target != :seg
            error("unknown target $target")
        end

        if target == :frame
            new(assignment.frame_assignment, fold, match_fold)
        else
            new(assignment.seg_assignment, fold, match_fold)
        end
    end
end

is_in_fold(value::Integer, foldset::FoldSet) = is_in_fold(foldset.fold, value, foldset.match_fold)

# iterator
function _find_next_valid_fold_match(foldset::FoldSet, state::Int)
    while state < length(foldset.assignment)
        state += 1
        if is_in_fold(foldset.assignment[state], foldset)
            return state
        end
    end
    state + 1 # returns length(foldset.assignment) + 1 on fail
end
Base.start(foldset::FoldSet) = _find_next_valid_fold_match(foldset, 0)
Base.done(foldset::FoldSet, state::Int) = state > length(foldset.assignment)
function Base.next(foldset::FoldSet, state::Int)
    @assert(is_in_fold(foldset.assignment[state], foldset))
    state, _find_next_valid_fold_match(foldset, state)
end
function Base.length(foldset::FoldSet)
    len = 0
    state = start(foldset)
    while !done(foldset, state)
        item, state = next(foldset, state)
        len += 1
    end
    len
end

#########################################

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
            frameend = framestart + get_nframes(runlog_segments[runlogsegmentind])
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

    fold_priority = PriorityQueue{Int, Int, Base.Order.ForwardOrdering}() # (fold, foldsize), min-ordered
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

function load_runlogs(dset::ModelTrainingData2)
    runlogs = Array(RunLog, length(dset.runlog_filepaths))
    for (i,runlog_filepath) in enumerate(dset.runlog_filepaths)
        runlogs[i] = load(runlog_filepath, "runlog")
    end
    runlogs
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
