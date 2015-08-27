module ValidationTraceExtractor

using DataFrames
using Discretizers
using JLD

using AutomotiveDrivingModels.CommonTypes
using AutomotiveDrivingModels.Trajdata
using AutomotiveDrivingModels.StreetNetworks
using AutomotiveDrivingModels.Features
using AutomotiveDrivingModels.FeaturesetExtractor

import Base: ==, isequal
import Base.Collections: PriorityQueue, dequeue!
import AutomotiveDrivingModels.FeaturesetExtractor: create_dataframe_with_feature_columns

export 
        OrigHistobinExtractParameters,
        StartCondition,

        total_framecount,
        pull_start_conditions,
        calc_traces_to_keep_with_frameskip,

        pull_pdsets_streetnets_segments_and_dataframe,
        save_pdsets_streetnets_segements_and_dataframe,
        load_pdsets_streetnets_segements_and_dataframe,

        is_strictly_monotonically_increasing,
        merge_region_segments,
        calc_row_count_from_region_segments,

        cross_validation_sets
        # get_training_and_validation


########################################
# ORIGINAL HISTOBIN EXTRACT PARAMETERS #
########################################

type OrigHistobinExtractParameters
    behavior        :: String
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
        behavior::String="",
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
function ==(a::OrigHistobinExtractParameters, b::OrigHistobinExtractParameters) 
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
!=(a::OrigHistobinExtractParameters, b::OrigHistobinExtractParameters) = !(a==b)
isequal(a::OrigHistobinExtractParameters, b::OrigHistobinExtractParameters) = a == b


########################################
#          START CONDITION             #
########################################

immutable StartCondition
    validfind :: Int # the starting validfind (includes history)
    carids    :: Vector{Int} # vehicles to pull traces for
end

########################################
#              FUNCTIONS               #
########################################

function _get_pdsetfile(csvfile::String, pdset_dir::String=PRIMARYDATA_DIR)
    csvfilebase = basename(csvfile)
    joinpath(pdset_dir, toext("primarydata_" * csvfilebase, "jld"))
end
function _load_pdset(csvfile::String, pdset_dir::String=PRIMARYDATA_DIR)
    pdsetfile = _get_pdsetfile(csvfile, pdset_dir)
    pdset = load(pdsetfile, "pdset")
end

function pull_pdset_segments_and_dataframe(
    extract_params  :: OrigHistobinExtractParameters,
    csvfileset      :: CSVFileSet,
    pdset           :: PrimaryDataset,
    sn              :: StreetNetwork,
    pdset_id        :: Integer,
    streetnet_id    :: Integer;
    features::Vector{AbstractFeature}=FEATURES,
    filters::Vector{AbstractFeature}=AbstractFeature[],
    pdset_frames_per_sim_frame::Int=5
    )

    validfinds = 1 : pdset_frames_per_sim_frame : nvalidfinds(pdset)

    startframes = Int[]
    pdset_segments = PdsetSegment[]
    dataframe = create_dataframe_with_feature_columns(features, 0)
    for carid in get_carids(pdset)
        more_pdset_segments = pull_pdset_segments(extract_params, csvfileset, pdset, sn,
                                                    pdset_id, streetnet_id, carid)

        append!(pdset_segments, more_pdset_segments)
        append!(startframes, map(seg->nrow(dataframe)+div((seg.validfind_start-1), pdset_frames_per_sim_frame)+1, more_pdset_segments)) #fill(nrow(dataframe)+1, length(more_pdset_segments)))
        append!(dataframe, gen_featureset(carid, pdset, validfinds, sn, features, filters=filters))
    end

    # println("pdset_segments: ", pdset_segments)
    # println("startframes: ", startframes)
    # println("dataframe:", dataframe)
    
    (pdset_segments, dataframe, startframes)
end
function _pull_pdsets_streetnets_segments_and_dataframe(
    extract_params::OrigHistobinExtractParameters,
    csvfilesets::Vector{CSVFileSet},
    features::Vector{AbstractFeature},
    filters::Vector{AbstractFeature},
    pdset_dir::String
    )

    println("called! ", length(csvfilesets))

    streetnet_cache = (String, StreetNetwork)[]

    pdsets = String[]
    pdset_segments = PdsetSegment[]
    dataframe = create_dataframe_with_feature_columns(features, 0)
    startframes = Int[]

    for csvfileset in csvfilesets

        tic()

        csvfile = csvfileset.csvfile
        streetmapbasename = csvfileset.streetmapbasename

        if streetmapbasename == "skip"
            continue
        end

        pdset = _load_pdset(csvfile, pdset_dir)
        push!(pdsets, _get_pdsetfile(csvfile, pdset_dir))
        pdset_id = length(pdsets)


        streetnet_id = findfirst(tup->tup[1]==streetmapbasename, streetnet_cache)
        if streetnet_id == 0
            streetnet = load(STREETMAP_BASE*streetmapbasename*".jld")["streetmap"]
            push!(streetnet_cache, (streetmapbasename, streetnet))
            streetnet_id = length(streetnet_cache)
        end
        sn = streetnet_cache[streetnet_id][2]


        println(csvfile)

        more_pdset_segments, more_dataframe, more_startframes = pull_pdset_segments_and_dataframe(
                                                                    extract_params, csvfileset, pdset, sn,
                                                                    pdset_id, streetnet_id,
                                                                    features=features, filters=filters)

        more_startframes .+= nrow(dataframe)

        append!(pdset_segments, more_pdset_segments)
        append!(dataframe, more_dataframe)
        append!(startframes, more_startframes)

        toc()
    end

    streetnets = Array(String, length(streetnet_cache))
    for streetnet_id = 1 : length(streetnet_cache)
        streetnets[streetnet_id] = STREETMAP_BASE*streetnet_cache[streetnet_id][1]*".jld"
    end

    (pdsets, streetnets, pdset_segments, dataframe, startframes)
end
function _pull_pdsets_streetnets_segments_and_dataframe_parallel(
    extract_params::OrigHistobinExtractParameters,
    csvfilesets::Vector{CSVFileSet},
    features::Vector{AbstractFeature},
    filters::Vector{AbstractFeature},
    pdset_dir::String
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

    more_stuff = pmap(assigned_csvfilesets->_pull_pdsets_streetnets_segments_and_dataframe(
                                                extract_params, assigned_csvfilesets,
                                                features, filters, pdset_dir
                                            ), csvfileset_assignment)

    pdsets = Array(String, num_csvfilesets)
    streetnets = String[]
    pdset_segments = PdsetSegment[]
    dataframe = create_dataframe_with_feature_columns(features, 0)
    startframes = Int[]

    pdset_index = 0
    for (more_pdsets, more_streetnets, more_pdset_segments, more_dataframe, more_startframes) in more_stuff
    
        more_startframes .+= nrow(dataframe)

        for (i,seg) in enumerate(more_pdset_segments)
            target_streetnet = more_streetnets[seg.streetnet_id]

            streetnet_id = findfirst(streetnets, target_streetnet)
            if streetnet_id == 0
                push!(streetnets, target_streetnet)
                streetnet_id = length(streetnets)
            end
            
            push!(pdset_segments, PdsetSegment(seg.pdset_id, streetnet_id, seg.carid, 
                                  seg.validfind_start, seg.validfind_end))
        end

        for pdset in more_pdsets
            pdset_index += 1
            pdsets[pdset_index] = pdset
        end

        append!(dataframe, more_dataframe)
        append!(startframes, more_startframes)
    end

    return (pdsets, streetnets, pdset_segments, dataframe, startframes)
end
function pull_pdsets_streetnets_segments_and_dataframe(
    extract_params::OrigHistobinExtractParameters,
    csvfilesets::Vector{CSVFileSet};
    features::Vector{AbstractFeature}=FEATURES,
    filters::Vector{AbstractFeature}=AbstractFeature[],
    pdset_dir::String=PRIMARYDATA_DIR
    )

    if nworkers() > 1
        _pull_pdsets_streetnets_segments_and_dataframe_parallel(extract_params, csvfilesets, 
                                                                features, filters, pdset_dir)
    else
        _pull_pdsets_streetnets_segments_and_dataframe(extract_params, csvfilesets, 
                                                       features, filters, pdset_dir)
    end
end

function calc_traces_to_keep_with_frameskip(
    start_conditions :: Vector{StartCondition}, 
    pdset :: PrimaryDataset, 
    frameskip :: Int
    )

    keep = falses(length(start_conditions))
    prev_frameind = -frameskip
    for (i,start_condition) in enumerate(start_conditions)
        frameind = validfind2frameind(pdset, start_condition.validfind)
        keep[i] = (frameind - prev_frameind) ≥ frameskip
        if keep[i]
            prev_frameind = frameind
        end
    end

    keep
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
    df[symbol(SUBSET_LANE_CROSSING)] = _calc_subset_vector([csvfileset.lanechanges_normal, 
                                                        csvfileset.lanechanges_postpass, 
                                                        csvfileset.lanechanges_arbitrary], nframes)

    df
end
function pull_start_conditions(
    extract_params  :: OrigHistobinExtractParameters,
    pdset          :: PrimaryDataset,
    sn             :: StreetNetwork;
    MAX_DIST       :: Float64 = 5000.0, # max_dist used in frenet_distance_between_points() [m]
    )

    #=
    Identify the set of frameinds that are valid starting locations
    =#

    start_conditions = StartCondition[]

    const PDSET_FRAMES_PER_SIM_FRAME = extract_params.pdset_frames_per_sim_frame
    const SIM_HORIZON = extract_params.sim_horizon
    const SIM_HISTORY = extract_params.sim_history

    @assert(SIM_HISTORY ≥ 1)
    @assert(SIM_HORIZON ≥ 0)

    const TOLERANCE_D_CL      = extract_params.tol_d_cl
    const TOLERANCE_YAW       = extract_params.tol_yaw
    const TOLERANCE_TURNRATE  = extract_params.tol_turnrate
    const TOLERANCE_SPEED     = extract_params.tol_speed
    const TOLERANCE_ACCEL     = extract_params.tol_accel
    const TOLERANCE_D_X_FRONT = extract_params.tol_d_x_front
    const TOLERANCE_D_Y_FRONT = extract_params.tol_d_y_front
    const TOLERANCE_D_V_FRONT = extract_params.tol_d_v_front

    const TARGET_SPEED        = extract_params.target_speed

    const N_SIM_FRAMES = total_framecount(extract_params)

    for validfind = SIM_HISTORY : nvalidfinds(pdset)

        validfind_start = int(jumpframe(pdset, validfind, -(SIM_HISTORY-1) * PDSET_FRAMES_PER_SIM_FRAME))
        validfind_end   = int(jumpframe(pdset, validfind,   SIM_HORIZON    * PDSET_FRAMES_PER_SIM_FRAME))

        if are_validfinds_continuous(pdset, validfind_start, validfind_end) && 
            _calc_frames_pass_subsets(pdset, sn, CARIND_EGO, validfind, validfind_end, extract_params.subsets)

            frameind = validfind2frameind(pdset, validfind)

            posFy  = gete(pdset, :posFy, frameind)::Float64
            ϕ      = gete(pdset, :posFyaw, frameind)::Float64
            d_cl   = gete(pdset, :d_cl, frameind)::Float64
            v_orig = get(SPEED,    pdset, sn, CARIND_EGO, validfind)::Float64
            ω      = get(TURNRATE, pdset, sn, CARIND_EGO, validfind)::Float64
            a      = get(ACC,      pdset, sn, CARIND_EGO, validfind)::Float64
            has_front = bool(get(HAS_FRONT, pdset, sn, CARIND_EGO, validfind)::Float64)
            d_x_front = get(D_X_FRONT, pdset, sn, CARIND_EGO, validfind)::Float64
            d_y_front = get(D_Y_FRONT, pdset, sn, CARIND_EGO, validfind)::Float64
            v_x_front = get(V_X_FRONT, pdset, sn, CARIND_EGO, validfind)::Float64
            nll    = int(get(N_LANE_L, pdset, sn, CARIND_EGO, validfind)::Float64)
            nlr    = int(get(N_LANE_R, pdset, sn, CARIND_EGO, validfind)::Float64)
            
            if  abs(d_cl) ≤ TOLERANCE_D_CL      && 
                abs(ϕ)    ≤ TOLERANCE_YAW       &&
                abs(ω)    ≤ TOLERANCE_TURNRATE  &&
                abs(a)    ≤ TOLERANCE_ACCEL     &&
                abs(v_orig-TARGET_SPEED) ≤ TOLERANCE_SPEED && 
                d_x_front ≤ TOLERANCE_D_X_FRONT &&
                abs(d_y_front) ≤ TOLERANCE_D_Y_FRONT &&
                v_x_front ≤ TOLERANCE_D_V_FRONT #&&
                # (nll + nlr) > 0

                find_start = validfind2frameind(pdset, validfind_start)
                find_end = validfind2frameind(pdset, validfind_end)

                posGxA = gete(pdset, :posGx, find_start)
                posGyA = gete(pdset, :posGy, find_start)
                posGxB = gete(pdset, :posGx, find_end)
                posGyB = gete(pdset, :posGy, find_end)

                (Δx, Δy) = frenet_distance_between_points(sn, posGxA, posGyA, posGxB, posGyB, MAX_DIST)

                # if Δy < -6
                #     println("posFy  =  ", posFy)
                #     println("ϕ      =  ", ϕ)
                #     println("d_cl   =  ", d_cl)
                #     println("v_orig =  ", v_orig)
                #     println("ω      =  ", ω)
                #     println("a      =  ", a)
                #     println("has_front ", has_front)
                #     println("d_x_front ", d_x_front)
                #     println("d_y_front ", d_y_front)
                #     println("v_x_front ", v_x_front)
                #     println("nll    =  ", nll)
                #     println("nlr    =  ", nlr)
                # end

                if !isnan(Δx)
                    error("This code needs to be changed")
                    if extract_params.behavior == "freeflow"
                        startcon = StartCondition(validfind_start, Int[CARID_EGO])
                        push!(start_conditions, startcon)
                    elseif extract_params.behavior == "carfollow"
                        # need to pull the leading vehicle

                        @assert(has_front)
                        carind_front = int(get(INDFRONT, pdset, sn, CARIND_EGO, validfind))
                        carid_front = carind2id(pdset, carind_front, validfind)
                        if frames_contain_carid(pdset, carid_front, validfind_start, validfind_end, frameskip=PDSET_FRAMES_PER_SIM_FRAME)
                            startcon = StartCondition(validfind_start, Int[CARID_EGO, carid_front])
                            push!(start_conditions, startcon)
                        end
                    end
                end
            end
        end
    end

    start_conditions
end
function pull_start_conditions(
    extract_params :: OrigHistobinExtractParameters,
    csvfileset     :: CSVFileSet,
    pdset          :: PrimaryDataset,
    sn             :: StreetNetwork;
    MAX_DIST       :: Float64 = 5000.0, # max_dist used in frenet_distance_between_points() [m]
    )

    #=
    Identify the set of frameinds that are valid starting locations
    =#

    start_conditions = StartCondition[]

    const PDSET_FRAMES_PER_SIM_FRAME = extract_params.pdset_frames_per_sim_frame
    const SIM_HORIZON = extract_params.sim_horizon
    const SIM_HISTORY = extract_params.sim_history

    @assert(SIM_HISTORY ≥ 1)
    @assert(SIM_HORIZON ≥ 0)

    const TOLERANCE_D_CL      = extract_params.tol_d_cl
    const TOLERANCE_YAW       = extract_params.tol_yaw
    const TOLERANCE_TURNRATE  = extract_params.tol_turnrate
    const TOLERANCE_SPEED     = extract_params.tol_speed
    const TOLERANCE_ACCEL     = extract_params.tol_accel
    const TOLERANCE_D_X_FRONT = extract_params.tol_d_x_front
    const TOLERANCE_D_Y_FRONT = extract_params.tol_d_y_front
    const TOLERANCE_D_V_FRONT = extract_params.tol_d_v_front

    const TARGET_SPEED        = extract_params.target_speed

    const N_SIM_FRAMES = total_framecount(extract_params)

    num_validfinds = nvalidfinds(pdset)
    
    df_subsets = _calc_subsets_based_on_csvfileset(csvfileset, num_validfinds)

    for validfind = SIM_HISTORY : num_validfinds

        validfind_start = int(jumpframe(pdset, validfind, -(SIM_HISTORY-1) * PDSET_FRAMES_PER_SIM_FRAME))
        validfind_end   = int(jumpframe(pdset, validfind,   SIM_HORIZON    * PDSET_FRAMES_PER_SIM_FRAME))

        # println(validfind, "  ",
        #        int(are_validfinds_continuous(pdset, validfind_start, validfind_end)), "  ",
        #         int(_calc_frames_pass_subsets(pdset, sn, CARIND_EGO, validfind, validfind_end, extract_params.subsets, df_subsets)))

        if are_validfinds_continuous(pdset, validfind_start, validfind_end) && 
            _calc_frames_pass_subsets(pdset, sn, CARIND_EGO, validfind, validfind_end, extract_params.subsets, df_subsets)

            frameind = validfind2frameind(pdset, validfind)

            posFy  = gete(pdset, :posFy, frameind)::Float64
            ϕ      = gete(pdset, :posFyaw, frameind)::Float64
            d_cl   = gete(pdset, :d_cl, frameind)::Float64
            v_orig = get(SPEED,    pdset, sn, CARIND_EGO, validfind)::Float64
            ω      = get(TURNRATE, pdset, sn, CARIND_EGO, validfind)::Float64
            a      = get(ACC,      pdset, sn, CARIND_EGO, validfind)::Float64
            has_front = bool(get(HAS_FRONT, pdset, sn, CARIND_EGO, validfind)::Float64)
            d_x_front = get(D_X_FRONT, pdset, sn, CARIND_EGO, validfind)::Float64
            d_y_front = get(D_Y_FRONT, pdset, sn, CARIND_EGO, validfind)::Float64
            v_x_front = get(V_X_FRONT, pdset, sn, CARIND_EGO, validfind)::Float64
            nll    = int(get(N_LANE_L, pdset, sn, CARIND_EGO, validfind)::Float64)
            nlr    = int(get(N_LANE_R, pdset, sn, CARIND_EGO, validfind)::Float64)

            # println(frameind)
            
            if  abs(d_cl) ≤ TOLERANCE_D_CL      && 
                abs(ϕ)    ≤ TOLERANCE_YAW       &&
                abs(ω)    ≤ TOLERANCE_TURNRATE  &&
                abs(a)    ≤ TOLERANCE_ACCEL     &&
                abs(v_orig-TARGET_SPEED) ≤ TOLERANCE_SPEED && 
                d_x_front ≤ TOLERANCE_D_X_FRONT &&
                abs(d_y_front) ≤ TOLERANCE_D_Y_FRONT &&
                v_x_front ≤ TOLERANCE_D_V_FRONT #&&
                #(nll + nlr) > 0

                find_start = validfind2frameind(pdset, validfind_start)
                find_end = validfind2frameind(pdset, validfind_end)

                posGxA = gete(pdset, :posGx, find_start)
                posGyA = gete(pdset, :posGy, find_start)
                posGxB = gete(pdset, :posGx, find_end)
                posGyB = gete(pdset, :posGy, find_end)

                (Δx, Δy) = frenet_distance_between_points(sn, posGxA, posGyA, posGxB, posGyB, MAX_DIST)

                # if Δy < -6
                #     println("posFy  =  ", posFy)
                #     println("ϕ      =  ", ϕ)
                #     println("d_cl   =  ", d_cl)
                #     println("v_orig =  ", v_orig)
                #     println("ω      =  ", ω)
                #     println("a      =  ", a)
                #     println("has_front ", has_front)
                #     println("d_x_front ", d_x_front)
                #     println("d_y_front ", d_y_front)
                #     println("v_x_front ", v_x_front)
                #     println("nll    =  ", nll)
                #     println("nlr    =  ", nlr)
                # end

                # println(Δx, "  ", Δy)

                if !isnan(Δx)

                    # create a start condition with all of the vehicles
                    # println("selected")

                    startcon = StartCondition(validfind_start, Int[CARID_EGO])
                    for carid in get_carids(pdset)
                        if carid != CARID_EGO &&
                           frames_contain_carid(pdset, carid, validfind_start, validfind_end, frameskip=PDSET_FRAMES_PER_SIM_FRAME)

                            push!(startcon.carids, carid)
                        end
                    end
                    push!(start_conditions, startcon)


                    # if extract_params.behavior == "freeflow"
                    #     startcon = StartCondition(validfind_start, Int[CARID_EGO])
                    #     push!(start_conditions, startcon)
                    # elseif extract_params.behavior == "carfollow"
                    #     # need to pull the leading vehicle

                    #     @assert(has_front)
                    #     carind_front = int(get(INDFRONT, pdset, sn, CARIND_EGO, validfind))
                    #     carid_front = carind2id(pdset, carind_front, validfind)
                    #     if frames_contain_carid(pdset, carid_front, validfind_start, validfind_end, frameskip=PDSET_FRAMES_PER_SIM_FRAME)
                    #         startcon = StartCondition(validfind_start, Int[CARID_EGO, carid_front])
                    #         push!(start_conditions, startcon)
                    #     end
                    # end
                end
            end
        end
    end

    start_conditions
end

function pull_pdset_segments(
    extract_params::OrigHistobinExtractParameters,
    csvfileset::CSVFileSet,
    pdset::PrimaryDataset,
    sn::StreetNetwork,
    pdset_id::Integer,
    streetnet_id::Integer,
    carid::Integer;
    max_dist::Float64 = 5000.0, # max_dist used in frenet_distance_between_points() [m]
    )

    const PDSET_FRAMES_PER_SIM_FRAME = extract_params.pdset_frames_per_sim_frame
    const SIM_HORIZON         = extract_params.sim_horizon
    const SIM_HISTORY         = extract_params.sim_history

    const TARGET_SPEED        = extract_params.target_speed
    const N_SIM_FRAMES        = total_framecount(extract_params)

    @assert(SIM_HISTORY ≥ 1)
    @assert(SIM_HORIZON ≥ 0)

    pdset_segments = PdsetSegment[]
    num_validfinds = nvalidfinds(pdset)
    df_subsets = _calc_subsets_based_on_csvfileset(csvfileset, num_validfinds)

    basics = FeatureExtractBasicsPdSet(pdset, sn)

    validfind = 0
    while validfind < num_validfinds

        validfind += 1
        validfind_start = int(jumpframe(pdset, validfind, -(SIM_HISTORY-1) * PDSET_FRAMES_PER_SIM_FRAME))
        validfind_end   = int(jumpframe(pdset, validfind,   SIM_HORIZON    * PDSET_FRAMES_PER_SIM_FRAME))

        # println(validfind_start, "  ", validfind, "  ", validfind_end)
        # println(validfind_start != 0, "  ", validfind_end != 0)
        # println(are_validfinds_continuous(pdset, validfind_start, validfind_end))
        # println(_calc_frames_pass_subsets(pdset, sn, CARIND_EGO, validfind, validfind_end, extract_params.subsets, df_subsets))

        if validfind_start != 0 && validfind_end != 0 &&
           are_validfinds_continuous(pdset, validfind_start, validfind_end) && 
           _calc_frames_pass_subsets(pdset, sn, CARIND_EGO, validfind, validfind_end, extract_params.subsets, df_subsets)

            carind = carid2ind(pdset, carid, validfind)
            posFy     =      get(pdset, :posFy,   carind, validfind)::Float64
            ϕ         =      get(pdset, :posFyaw, carind, validfind)::Float64
            d_cl      =      get(pdset, :d_cl,    carind, validfind)::Float64
            v_orig    =      get(SPEED,     basics, carind, validfind)::Float64
            ω         =      get(TURNRATE,  basics, carind, validfind)::Float64
            a         =      get(ACC,       basics, carind, validfind)::Float64
            has_front = bool(get(HAS_FRONT, basics, carind, validfind)::Float64)
            d_x_front =      get(D_X_FRONT, basics, carind, validfind)::Float64
            d_y_front =      get(D_Y_FRONT, basics, carind, validfind)::Float64
            v_x_front =      get(V_X_FRONT, basics, carind, validfind)::Float64
            nll       =  int(get(N_LANE_L,  basics, carind, validfind)::Float64)
            nlr       =  int(get(N_LANE_R,  basics, carind, validfind)::Float64)
            
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

                # @printf("posA: %15.6f, %15.6f\n", posGxA, posGyA)
                # @printf("posB: %15.6f, %15.6f\n", posGxB, posGyB)
                # println("proj: ", project_point_to_streetmap(posGxA, posGyA, sn))
                # println("proj: ", project_point_to_streetmap(posGxB, posGyB, sn))

                (Δx, Δy) = frenet_distance_between_points(sn, posGxA, posGyA, posGxB, posGyB, max_dist)

                @assert(!isnan(Δx))
                push!(pdset_segments, PdsetSegment(pdset_id, streetnet_id, carid, validfind, validfind_end))
                validfind += max(extract_params.frameskip_between_extracted_scenes * PDSET_FRAMES_PER_SIM_FRAME - 1, 0)
            end
        end
    end

    # println(pdset_segments)

    pdset_segments
end

function save_pdsets_streetnets_segements_and_dataframe(
    filepath::String,
    pdsets::Vector{String},
    streetnets::Vector{String},
    pdset_segments::Vector{PdsetSegment},
    dataframe::DataFrame,
    startframes::Vector{Int},
    extract_params::OrigHistobinExtractParameters
    )

    save(filepath, "pdsets", pdsets, "streetnets", streetnets,
                    "pdset_segments", pdset_segments, "dataframe", dataframe,
                    "startframes", startframes, "extract_params", extract_params)
end
function load_pdsets_streetnets_segements_and_dataframe(filepath::String)

    input = load(filepath)
    pdsets         = input["pdsets"]::Vector{String}
    streetnets     = input["streetnets"]::Vector{String}
    pdset_segments = input["pdset_segments"]::Vector{PdsetSegment}
    dataframe      = input["dataframe"]::DataFrame
    startframes    = input["startframes"]::Vector{Int}
    extract_params = input["extract_params"]::OrigHistobinExtractParameters

    (pdsets, streetnets, pdset_segments, dataframe, startframes, extract_params)
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

function cross_validation_sets(
    nfolds::Int,
    pdset_segments::Vector{PdsetSegment},
    dataframe::DataFrame,
    seg_to_framestart::Vector{Int} # seg_to_framestart[i] -> j
                                   # where i is the pdsetsegment index
                                   # and j is the row in dataframe for the start (pdsetsegment.log[pdsetsegment.history,:])
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
            for _ in framestart : 5 : frameend
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

    (frame_fold_assignment, pdsetsegment_fold_assignment)
end

end # end module