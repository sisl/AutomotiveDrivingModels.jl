module FeaturesetExtractor

using DataFrames
using HDF5, JLD

using AutomotiveDrivingModels.Curves
using AutomotiveDrivingModels.Trajdata
using AutomotiveDrivingModels.StreetNetworks
using AutomotiveDrivingModels.Features
include("../io/filesystem_utils.jl")

export
        CSVFileSet,

        # PRIMARYDATA_DIR,
        # FEATUREMATRIX_DIR,
        # FEATURESETS_DIR,
        # STREETMAP_BASE,
        # CSVFILESETS,
        FEATURES,
        BEHAVIORS,

        get_validfind_regions,
        merge_region_segments,

        gen_featureset
        # extract,
        # extract_and_save,
        # extract_csvfile_set,
        # extract_csvfile_sets


type CSVFileSet
    carid::Int
    csvfile :: String
    streetmapbasename :: String

    lanechanges_normal    :: Vector{Int}
    lanechanges_postpass  :: Vector{Int}
    lanechanges_arbitrary :: Vector{Int}
    carfollow             :: Vector{Int}
    freeflow              :: Vector{Int}
end

# const PRIMARYDATA_DIR = "/media/tim/DATAPART1/Data/Bosch/processed/primarydata/"
# const FEATUREMATRIX_DIR = "/media/tim/DATAPART1/Data/Bosch/processed/featurematrices/"
# const FEATURESETS_DIR = "/media/tim/DATAPART1/Data/Bosch/processed/featuresets/"
# const STREETMAP_BASE = "/media/tim/DATAPART1/Data/Bosch/processed/streetmaps/streetmap_"

# const CSVFILESETS = [
#             CSVFileSet(CARIND_EGO, "philippdrive01_2015-03-24-15-21-12-raw", "280_pagemill_to_92", # 16
#                 [128,297,475,673,3520,3671,4363,4559,5724,5833,6816,6937,7086,7343,7650,7765,8102,8283,8826,9333,9922,10095],
#                 [2641,2833,3790,3981,5026,5255,10184,10321],
#                 [9816,9921],
#                 [1,127,298,475,1900,2293,5256,5723,6466,6815,6938,7085,7343,7649,7766,8101,8754,8825],
#                 [674,1899,2294,2641,2834,3519,3672,3789,3982,4363,4560,5025,5834,6465,8284,8753,9334,9815,10096,10183]
#                 ),
#             CSVFileSet(CARIND_EGO, "philippdrive03_2015-03-24-15-37-18-raw", "280_pagemill_to_92", # 16
#                 [1182,1386,1425,1562,1657,1826,3331,3568,4929,5238,5691,5840,5899,6112,7913,8040,8407,8532],
#                 [3191,3330,3785,3974,4027,4212,6275,6778,8111,8278],
#                 [7712,7888,8533,8722],
#                 [1387,1424,5239,5690,5841,5898,7889,7912,9055,9160],
#                 [1,1182,1563,1656,1827,3190,3569,3784,3975,4026,4213,4928,6113,6274,8041,8110,8279,8406,8723,9054]
#                 ),
#             CSVFileSet(CARIND_EGO, "philippdrive04_2015-03-24-15-47-10-raw", "280_pagemill_to_92", # 2
#                 [811,944,945,1148],
#                 Int[],
#                 Int[],
#                 Int[],
#                 [1,810]
#                 ),
#             CSVFileSet(CARIND_EGO, "philippdrive05_2015-03-24-15-48-42-raw", "280_pagemill_to_92", # 5
#                 [149,314,628,934,1571,1683,2819,3032,3107,3308],
#                 Int[],
#                 Int[],
#                 [1,148,935,1570,1684,2818,3033,3106,3309,4434],
#                 [315,626]
#                 ),
#             CSVFileSet(CARIND_EGO, "philippdrive07_2015-03-24-15-57-42-raw", "280_pagemill_to_92", # 12
#                 [31,138,965,1176,1481,1668,2753,2936,4135,4350,4351,4530,4672,4880,5271,5516,5856,6026,6195,6412,7775,7938,8519,8608],
#                 Int[],
#                 [6989,7156],
#                 [481,964,1177,1480,1669,2752,2937,4134,4531,4672,4999,5156,7157,7774,7939,8518],
#                 [1,30,139,480,4881,4998,5157,5270,5517,5856,6027,6194,6413,6988]
#                 ),
#             CSVFileSet(CARIND_EGO, "2014-08-03-12-01-09-edited", "its_inner_loop_v1_2", # 2
#                 Int[],
#                 Int[],
#                 [4372,4482,4831,4976],
#                 [417,4266],
#                 [1,416,4483,4830,4977,5900]
#                 ),
#             CSVFileSet(CARIND_EGO, "2014-08-03-11-52-58-raw", "its_inner_loop_v1_2", 
#                 Int[],
#                 Int[],
#                 [1,153,359,518],
#                 [1101,1700],
#                 [154,358,519,1100,2494,3940]
#                 ),
#             CSVFileSet(CARIND_EGO, "2014-08-03-12-12-53-raw", "its_inner_loop_v1_2",
#                 Int[],
#                 Int[],
#                 [1740,1892],
#                 [956,1348],
#                 [1624,1739,1893,3060]
#                 ),
#             CSVFileSet(CARIND_EGO, "2014-08-03-12-16-14-raw", "its_both_loops_v3_smoothed",
#                 Int[],
#                 Int[],
#                 Int[],
#                 [2477,3918],
#                 [1,1020,1540,1936,2120,2476]
#                 ),
#             CSVFileSet(CARIND_EGO, "2014-08-03-12-20-36-raw", "its_inner_loop_v1_2",
#                 Int[],
#                 Int[],
#                 [2733,2898],
#                 [68,2100],
#                 [2506,2732,2899,3292]
#                 ),
#             CSVFileSet(CARIND_EGO, "2014-08-03-12-54-24-raw", "its_inner_loop_v1_2",
#                 Int[],
#                 Int[],
#                 Int[],
#                 [3166,3534,3972,6272,6448,7678,8036,8228,9031,9262],
#                 [8235,8500,8730,9030]
#                 ),
#             CSVFileSet(CARIND_EGO, "2014-08-06-16-11-51_large_loop_3-raw", "detroit_v4",
#                 Int[],
#                 Int[],
#                 Int[],
#                 Int[],
#                 [1,1618,1642,1834,1844,1902,3810,4232,6618,8588]
#                 ),
#             CSVFileSet(CARIND_EGO, "280N_2013-09-16-18-52-15-p", "280N_big",
#                 Int[], Int[], Int[], Int[], [100,9190]),
#             CSVFileSet(CARIND_EGO, "280N_2013-09-17-11-56-01-p", "280N_big",
#                 Int[], Int[], Int[], Int[], [100,4996]),
#             CSVFileSet(CARIND_EGO, "280N_2014-02-07-15-06-38-p", "280N_big",
#                 Int[], Int[], Int[], Int[], [100,8091]),
#             CSVFileSet(CARIND_EGO, "280N_2014-02-14-13-56-16-p", "280N_big",
#                 Int[], Int[], Int[], Int[], [100,8660]),
#             CSVFileSet(CARIND_EGO, "280N_2014-03-20-10-08-35-p", "280N_big",
#                 Int[], Int[], Int[], Int[], [100,9117]),
#             CSVFileSet(CARIND_EGO, "280N_2014-03-20-10-39-03-p", "280N_big",
#                 Int[], Int[], Int[], Int[], [100,6741]),
#             CSVFileSet(CARIND_EGO, "280N_2014-03-21-09-06-04-p", "280N_big",
#                 Int[], Int[], Int[], Int[], [100,9145]),
#             CSVFileSet(CARIND_EGO, "280N_2014-03-24-10-44-20-p", "280N_big",
#                 Int[], Int[], Int[], Int[], [100,8921]),
#             CSVFileSet(CARIND_EGO, "280N_2014-03-25-09-44-45-p", "280N_big",
#                 Int[], Int[], Int[], Int[], [100,8100]),
#             CSVFileSet(CARIND_EGO, "280N_2014-04-03-11-46-34-p", "280N_big",
#                 Int[], Int[], Int[], Int[], [100,9067]),
#             CSVFileSet(CARIND_EGO, "280_north_2014-05-02-13-26-12-p", "280N_big",
#                 Int[], Int[], Int[], Int[], [100,9285]),
#             CSVFileSet(CARIND_EGO, "2013-09-19-14-10-20_280N-p", "280N_big",
#                 Int[], Int[], Int[], Int[], [100,9085]),
#             CSVFileSet(CARIND_EGO, "280S_2014-03-20-10-23-27-p", "280S_big",
#                 Int[], Int[], Int[], Int[], [100,5130]),
#             CSVFileSet(CARIND_EGO, "280S_2014-03-21-09-28-53-p", "280S_big",
#                 Int[], Int[], Int[], Int[], [100,12295]),
#             CSVFileSet(CARIND_EGO, "280S_2014-03-25-10-06-43-p", "280S_big",
#                 Int[], Int[], Int[], Int[], [100,12720]),
#             CSVFileSet(CARIND_EGO, "280S_2014-04-03-12-09-43-p", "280S_big",
#                 Int[], Int[], Int[], Int[], [100,12885]),
#             CSVFileSet(CARIND_EGO, "28280S_2014-03-24-11-07-37-p", "280S_big",
#                 Int[], Int[], Int[], Int[], [100,12775]),
#         ]
const FEATURES = allfeatures()
# [
#         YAW, POSFX, POSFY, SPEED, VELFX, VELFY, DELTA_SPEED_LIMIT,
#         D_CL, D_ML, D_MR, D_MERGE, D_SPLIT, 
#         TIMETOCROSSING_RIGHT, TIMETOCROSSING_LEFT, ESTIMATEDTIMETOLANECROSSING, TIMESINCELANECROSSING,
#         N_LANE_L, N_LANE_R, HAS_LANE_L, HAS_LANE_R,
#         TURNRATE, TURNRATE_GLOBAL, ACC, ACCFX, ACCFY, A_REQ_STAYINLANE, LANECURVATURE,

#         HAS_FRONT, D_X_FRONT, D_Y_FRONT, V_X_FRONT, V_Y_FRONT, YAW_FRONT, TURNRATE_FRONT,
#         HAS_REAR,  D_X_REAR,  D_Y_REAR,  V_X_REAR,  V_Y_REAR,  YAW_REAR,  TURNRATE_REAR,
#                    D_X_LEFT,  D_Y_LEFT,  V_X_LEFT,  V_Y_LEFT,  YAW_LEFT,  TURNRATE_LEFT,
#                    D_X_RIGHT, D_Y_RIGHT, V_X_RIGHT, V_Y_RIGHT, YAW_RIGHT, TURNRATE_RIGHT,
#         A_REQ_FRONT, TTC_X_FRONT, TIMEGAP_X_FRONT, GAINING_ON_FRONT,
#         A_REQ_REAR,  TTC_X_REAR,  TIMEGAP_X_REAR,
#         A_REQ_LEFT,  TTC_X_LEFT,  TIMEGAP_X_LEFT,
#         A_REQ_RIGHT, TTC_X_RIGHT, TIMEGAP_X_RIGHT,

#         SCENEVELFX,

#         OCCUPANCYSCHEDULEGRID_TIME_FRONT,      OCCUPANCYSCHEDULEGRID_ISOCCUPIED_FRONT, 
#         OCCUPANCYSCHEDULEGRID_TIME_FRONTRIGHT, OCCUPANCYSCHEDULEGRID_ISOCCUPIED_FRONTRIGHT, 
#         OCCUPANCYSCHEDULEGRID_TIME_RIGHT,      OCCUPANCYSCHEDULEGRID_ISOCCUPIED_RIGHT, 
#         OCCUPANCYSCHEDULEGRID_TIME_BACKRIGHT,  OCCUPANCYSCHEDULEGRID_ISOCCUPIED_BACKRIGHT, 
#         OCCUPANCYSCHEDULEGRID_TIME_BACK,       OCCUPANCYSCHEDULEGRID_ISOCCUPIED_BACK, 
#         OCCUPANCYSCHEDULEGRID_TIME_BACKLEFT,   OCCUPANCYSCHEDULEGRID_ISOCCUPIED_BACKLEFT, 
#         OCCUPANCYSCHEDULEGRID_TIME_LEFT,       OCCUPANCYSCHEDULEGRID_ISOCCUPIED_LEFT, 
#         OCCUPANCYSCHEDULEGRID_TIME_FRONTLEFT,  OCCUPANCYSCHEDULEGRID_ISOCCUPIED_FRONTLEFT, 

#         TIME_CONSECUTIVE_BRAKE, TIME_CONSECUTIVE_ACCEL, TIME_CONSECUTIVE_THROTTLE,
#              PASTACC250MS,      PASTACC500MS,      PASTACC750MS,      PASTACC1S,
#         PASTTURNRATE250MS, PASTTURNRATE500MS, PASTTURNRATE750MS, PASTTURNRATE1S,
#            PASTVELFY250MS,    PASTVELFY500MS,    PASTVELFY750MS,    PASTVELFY1S,
#             PASTD_CL250MS,     PASTD_CL500MS,     PASTD_CL750MS,     PASTD_CL1S,

#             MAXACCFX500MS,     MAXACCFX750MS,     MAXACCFX1S,     MAXACCFX1500MS,     MAXACCFX2S,     MAXACCFX2500MS,     MAXACCFX3S,     MAXACCFX4S,
#             MAXACCFY500MS,     MAXACCFY750MS,     MAXACCFY1S,     MAXACCFY1500MS,     MAXACCFY2S,     MAXACCFY2500MS,     MAXACCFY3S,     MAXACCFY4S,
#          MAXTURNRATE500MS,  MAXTURNRATE750MS,  MAXTURNRATE1S,  MAXTURNRATE1500MS,  MAXTURNRATE2S,  MAXTURNRATE2500MS,  MAXTURNRATE3S,  MAXTURNRATE4S,
#            MEANACCFX500MS,    MEANACCFX750MS,    MEANACCFX1S,    MEANACCFX1500MS,    MEANACCFX2S,    MEANACCFX2500MS,    MEANACCFX3S,    MEANACCFX4S,
#            MEANACCFY500MS,    MEANACCFY750MS,    MEANACCFY1S,    MEANACCFY1500MS,    MEANACCFY2S,    MEANACCFY2500MS,    MEANACCFY3S,    MEANACCFY4S,
#         MEANTURNRATE500MS, MEANTURNRATE750MS, MEANTURNRATE1S, MEANTURNRATE1500MS, MEANTURNRATE2S, MEANTURNRATE2500MS, MEANTURNRATE3S, MEANTURNRATE4S,
#             STDACCFX500MS,     STDACCFX750MS,     STDACCFX1S,     STDACCFX1500MS,     STDACCFX2S,     STDACCFX2500MS,     STDACCFX3S,     STDACCFX4S,
#             STDACCFY500MS,     STDACCFY750MS,     STDACCFY1S,     STDACCFY1500MS,     STDACCFY2S,     STDACCFY2500MS,     STDACCFY3S,     STDACCFY4S,
#          STDTURNRATE500MS,  STDTURNRATE750MS,  STDTURNRATE1S,  STDTURNRATE1500MS,  STDTURNRATE2S,  STDTURNRATE2500MS,  STDTURNRATE3S,  STDTURNRATE4S,

#         FUTUREACCELERATION_250MS, FUTURETURNRATE_250MS, FUTUREDESIREDANGLE_250MS, FUTUREDESIREDSPEED_250MS, FUTUREACCELCONTROL_250MS,
#         FUTUREACCELERATION_500MS, FUTURETURNRATE_500MS, FUTUREDESIREDANGLE_500MS, FUTUREDESIREDSPEED_500MS, FUTUREACCELCONTROL_500MS,
#         TIMETOLANECROSSING, TIMESINCELANECROSSING,

#         SUBSET_EMERGENCY, SUBSET_FREE_FLOW, SUBSET_CAR_FOLLOWING, SUBSET_LANE_CROSSING, SUBSET_SUSTAINED_CROSSING,
#         SUBSET_AT_SIXTYFIVE,
#     ]
const BEHAVIORS = ["lanechange", "carfollow", "freeflow"]

function extract_all_lanechanges(
    pdset::PrimaryDataset,
    sn::StreetNetwork,
    carid::Integer,
    validfind_contains_carid::AbstractVector{Bool},
    )

    #=
    Returns a list of frames in which the vehicle is, for the first time, in a new lane
    =#

    lanechange_validfinds = Int[]
    

    for validfind = 2 : length(validfind_contains_carid)
        if validfind_contains_carid[validfind] && validfind_contains_carid[validfind-1]

            carind = carid2ind(pdset, carid, validfind)

            cur_validfind = validfind-1
            cur_frameind = validfind2frameind(pdset, cur_validfind)
            cur_lanetag = get(pdset, :lanetag, carind, cur_frameind, cur_validfind)::LaneTag
            cur_lane = get_lane(sn, cur_lanetag)

            fut_frameind = validfind2frameind(pdset, validfind)
            fut_lanetag = get(pdset, :lanetag, carind, fut_frameind, validfind)::LaneTag

            if cur_lanetag != fut_lanetag
                # potential lane change - need to verify it wasn't just us crossing a tile section
                # or road segment

                lanechange = same_tile(cur_lanetag, fut_lanetag) || !has_next_lane(sn, cur_lane)
                if !lanechange
                    cur_lane = next_lane(sn, cur_lane)
                    cur_lanetag = LaneTag(sn, cur_lane)
                    lanechange = fut_lanetag != cur_lanetag
                end

                if lanechange
                    push!(lanechange_validfinds, validfind)
                end
            end
        end
    end

    lanechange_validfinds
end
function identify_lane_changes!(
    validfind_is_lanechange::AbstractVector{Bool},
    validfind_contains_carid::AbstractVector{Bool},
    lanechange_validfinds::AbstractVector{Int},
    pdset::PrimaryDataset,
    carid::Integer,
    lat_vel_threshold::Real=0.1, # [m/s]
    )

    #=
    scans back and fore for each lane change in lanechange_validfinds,
    marking all validfinds with an absolute lateral velocity greater than the threshold
    =#

    @assert(length(validfind_is_lanechange) == length(validfind_contains_carid))

    fill!(validfind_is_lanechange, false)

    for lanechange_validfind in lanechange_validfinds
        @assert(validfind_contains_carid[lanechange_validfind])
        validfind_is_lanechange[lanechange_validfind] = true

        # scan forward
        validfind = lanechange_validfind
        while validfind_inbounds(pdset, validfind+1) &&
              validfind_contains_carid[validfind+1] &&
              abs(get(pdset, :velFy, carid2ind(pdset, carid, validfind+1), validfind+1)) > lat_vel_threshold

            validfind += 1
            validfind_is_lanechange[validfind] = true
        end

        # scan backward
        validfind = lanechange_validfind
        while validfind_inbounds(pdset, validfind-1) &&
              validfind_contains_carid[validfind-1] &&
              abs(get(pdset, :velFy, carid2ind(pdset, carid, validfind-1), validfind-1)) > lat_vel_threshold

            validfind -= 1
            validfind_is_lanechange[validfind] = true
        end
    end

    validfind_is_lanechange
end
function identify_freeflow!(
    validfind_is_freeflow::AbstractVector{Bool},
    validfind_contains_carid::AbstractVector{Bool},
    basics::FeatureExtractBasicsPdSet,
    carid::Integer
    )

    #=
    Labels each frame as freeflow or not (ie, carfollow), based on:
     - front timegap > threshold
     - front vehicle faster by threshold
    =#

    @assert(length(validfind_is_freeflow) == length(validfind_contains_carid))

    for (validfind, contains_carid) in enumerate(validfind_contains_carid)
        if contains_carid
            carind = carid2ind(basics.pdset, carid, validfind)
            validfind_is_freeflow[validfind] = Features._get(SUBSET_FREE_FLOW, basics, carind, validfind)
        else
            validfind_is_freeflow[validfind] = false
        end
    end

    validfind_is_freeflow
end

function extract_csvfile_set(
    carid::Integer,
    csvfile::String,
    streetmapbasename::String,
    pdset::PrimaryDataset,
    sn::StreetNetwork,
    validfind_contains_carid::AbstractVector{Bool}=falses(nvalidfinds(pdset)),
    validfind_is_lanechange::AbstractVector{Bool}=falses(nvalidfinds(pdset)),
    validfind_is_freeflow::AbstractVector{Bool}=falses(nvalidfinds(pdset)),
    )

    # grab all frames with vehicle
    Trajdata._get_validfinds_containing_carid!(validfind_contains_carid, pdset, carid)

    # identify all continuous segments
    segments = extract_continuous_segments(validfind_contains_carid)

    # identify all lane changes
    lanechange_validfinds = extract_all_lanechanges(pdset, sn, carid, validfind_contains_carid)
    identify_lane_changes!(validfind_is_lanechange, validfind_contains_carid, 
                           lanechange_validfinds, pdset, carid)

    # identify all free flow frames
    identify_freeflow!(validfind_is_freeflow, validfind_contains_carid, FeatureExtractBasicsPdSet(pdset, sn), carid)

    lanechanges_normal = extract_continuous_segments(validfind_is_lanechange)
    lanechanges_postpass = Int[]
    lanechanges_arbitrary = Int[]
    carfollow = extract_continuous_segments(!validfind_is_freeflow & validfind_contains_carid)
    freeflow = extract_continuous_segments(validfind_is_freeflow)

    CSVFileSet(carid, csvfile, streetmapbasename, 
                             lanechanges_normal,
                             lanechanges_postpass,
                             lanechanges_arbitrary,
                             carfollow, freeflow)
end
function extract_csvfile_sets(
    csvfile::String,
    streetmapbasename::String,
    pdset::PrimaryDataset,
    sn::StreetNetwork,
    )


    #=
    Loads the given pdset and attempts to automatically classify segments into:
     - lane change:  absolute lateral velocity is > 0.1 m/s and lane centerline switch occurs
        - normal:    lane change using car follow
        - postpass:  used to be in the target lane behind a car and am now passing in front of said car
        - arbitrary: not normal or postpass
     - free flow:    front timegap > 3s or front vehicle faster by 0.5 m/s
     - car follow:   not in free flow

     NOTE(tim): for now it classifies everything as a 'normal' lanechange
    =#

    retval = CSVFileSet[]

    N = nvalidfinds(pdset)
    validfind_contains_carid = falses(N)
    validfind_is_lanechange = falses(N)
    validfind_is_freeflow = falses(N)

    for carid in get_carids(pdset)
        push!(retval, extract_csvfile_set(carid, csvfile, streetmapbasename, pdset, sn,
                validfind_contains_carid, validfind_is_lanechange, validfind_is_freeflow
            ))
    end
    retval
end
function extract_csvfile_sets(
    csvfile::String,
    streetmapbasename::String,
    )

    pdset = load(joinpath(PRIMARYDATA_DIR, toext("primarydata_" * csvfile, "jld")))["pdset"]
    sn = load(STREETMAP_BASE*streetmapbasename*".jld")["streetmap"]
    extract_csvfile_sets(csvfile, streetmapbasename, pdset, sn)
end

function get_validfind_regions(behavior::String, csvfileset::CSVFileSet)

    # in(behavior, BEHAVIORS) || error("unknown behavior $behavior")

    if behavior == "lanechange"
        merge_region_segments([csvfileset.lanechanges_normal, csvfileset.lanechanges_postpass, csvfileset.lanechanges_arbitrary])
    elseif behavior == "carfollow"
        merge_region_segments([csvfileset.carfollow])
    elseif behavior == "freeflow"
        merge_region_segments([csvfileset.freeflow])
    else behavior == "all"
        merge_region_segments([csvfileset.lanechanges_normal, csvfileset.lanechanges_postpass, 
                               csvfileset.lanechanges_arbitrary, csvfileset.freeflow,
                               csvfileset.carfollow])
    end
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
function is_strictly_monotonically_nondecreasing(vec::AbstractVector{Int})
    if isempty(vec)
        return true
    end

    val = vec[1]
    for i = 2 : length(vec)
        val2 = vec[i]
        if val2 < val
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

    if !is_strictly_monotonically_nondecreasing(validfind_regions)
        println("not strincy increasing: ", validfind_regions)
    end

    @assert(mod(n, 2) == 0) # must be an even number
    @assert(is_strictly_monotonically_nondecreasing(validfind_regions))

    estimated_row_count = 0
    index_lo = 1
    while index_lo < length(validfind_regions)
        estimated_row_count += validfind_regions[index_lo+1] - validfind_regions[index_lo] + 1
        index_lo += 2
    end
    estimated_row_count
end
function create_dataframe_with_feature_columns{F<:AbstractFeature}(features::AbstractVector{F}, nrows::Int)
    df = DataFrame()
    df[:validfind] = DataArray(Float64, nrows)
    for f in features
        df[symbol(f)] = DataArray(Float64, nrows)
    end
    df
end
function does_violate_filter{F<:AbstractFeature}(filters::AbstractVector{F}, pdset::PrimaryDataset, sn::StreetNetwork, carind::Int, validfind::Int)
    for f in filters
        if isapprox(get(f, pdset, sn, carind, validfind), 0.0)
            return true
        end
    end
    false
end

function gen_featureset{F<:AbstractFeature, G<:AbstractFeature}(
    carid::Integer,
    pdset::PrimaryDataset,
    validfind_regions::Vector{Int}, # array where subsequent pairs are validfinds of the particular behavior ([1,10,20,30]) means 1->10 and 20->30 are that behavior
    sn::StreetNetwork,
    features::Vector{F};
    filters::Vector{G}=AbstractFeature[] # list of boolean features; if true the car is kept
    )
    
    estimated_row_count = calc_row_count_from_region_segments(validfind_regions)
    df = create_dataframe_with_feature_columns(features, estimated_row_count)

    basics = FeatureExtractBasicsPdSet(pdset, sn)

    row_index = 0
    region_index_lo = 1
    while region_index_lo < length(validfind_regions)
        for validfind in validfind_regions[region_index_lo] : validfind_regions[region_index_lo+1]
            carind = carid2ind_or_negative_two_otherwise(pdset, carid, validfind)
            if carind != -2 && !does_violate_filter(filters, pdset, sn, carind, validfind)

                row_index += 1
                for f in features
                    value = get(f, basics, carind, validfind)::Float64
                    df[row_index, symbol(f)] = value
                end
                df[row_index, :validfind] = float64(validfind)
            end
        end

        region_index_lo += 2
    end

    # NOTE(tim): drop any extra rows
    df[1:row_index,:]
end
function gen_featureset{F <: AbstractFeature, G <: AbstractFeature}(
    carid::Integer,
    pdset::PrimaryDataset,
    validfinds::Range{Int64}, # array where subsequent pairs are validfinds of the particular behavior ([1,10,20,30]) means 1->10 and 20->30 are that behavior
    sn::StreetNetwork,
    features::Vector{F};
    filters::Vector{G}=AbstractFeature[] # list of boolean features; if true the car is kept
    )
    
    estimated_row_count = length(validfinds)
    df = create_dataframe_with_feature_columns(features, estimated_row_count)

    basics = FeatureExtractBasicsPdSet(pdset, sn)

    row_index = 0
    for validfind in validfinds
        carind = carid2ind_or_negative_two_otherwise(pdset, carid, validfind)
        if carind != -2 && !does_violate_filter(filters, pdset, sn, carind, validfind)

            row_index += 1
            for f in features
                value = get(f, basics, carind, validfind)::Float64
                df[row_index, symbol(f)] = value
            end
            df[row_index, :validfind] = float64(validfind)
        end
    end

    # NOTE(tim): drop any extra rows
    df[1:row_index,:]
end
# function replace_behavior_features_with_hand_labels!(df::DataFrame, behavior::String)
    
#     value_emergency      = 0.0
#     value_freeflow       = 0.0
#     value_carfollow      = 0.0
#     value_lanechange     = 0.0
#     value_sustainedcross = 0.0

#     in(behavior, BEHAVIORS) || error("unknown behavior $behavior")

#     if behavior == "lanechange"
#         value_lanechange = 1.0
#     elseif behavior == "carfollow"
#         value_carfollow = 1.0
#     else behavior == "freeflow"
#         value_freeflow = 1.0
#     end

#     fill!(df[symbol(SUBSET_EMERGENCY)],          value_emergency)
#     fill!(df[symbol(SUBSET_FREE_FLOW)],          value_freeflow)
#     fill!(df[symbol(SUBSET_CAR_FOLLOWING)],      value_carfollow)
#     fill!(df[symbol(SUBSET_LANE_CROSSING)],      value_lanechange)
#     fill!(df[symbol(SUBSET_SUSTAINED_CROSSING)], value_sustainedcross)
    
#     df
# end

# function _get_streetnetwork(streetmapbasename::String, streetnet_cache::Dict{String, StreetNetwork})
#     if !haskey(streetnet_cache, streetmapbasename)
#         streetnet_cache[streetmapbasename] = load(STREETMAP_BASE*streetmapbasename*".jld")["streetmap"]
#     end
#     streetnet_cache[streetmapbasename]
# end
# function _aggregate(dfarr::Vector{DataFrame})

#     nrows = 0
#     for df in dfarr
#         nrows += size(df,1)
#     end

#     aggregate_datafile = DataFrame()
#     for name in names(dfarr[1])
#         aggregate_datafile[name] = DataArray(Float64, nrows)
#     end

#     aggrow = 0
#     for df in dfarr
#         for row in 1 : size(df,1)
#             aggrow += 1
#             for col in 1 : size(df,2)
#                 aggregate_datafile[aggrow, col] = df[row, col]
#             end
#         end
#     end

#     aggregate_datafile
# end

# function extract(
#     behavior::String,
#     validfinds::Vector{Int},
#     pdset::PrimaryDataset,
#     sn::StreetNetwork,
#     features::Vector{AbstractFeature}=FEATURES,
#     filters::Vector{AbstractFeature}=AbstractFeature[]
#     )

#     featureset = gen_featureset(CARID_EGO, pdset, validfinds, sn, features, filters=filters)
#     if in(behavior, BEHAVIORS)
#         replace_behavior_features_with_hand_labels!(featureset, behavior)
#     end

#     featureset
# end
# function extract(
#     csvfileset::CSVFileSet,
#     behavior::String,
#     validfinds::Vector{Int},
#     features::Vector{AbstractFeature}=FEATURES,
#     streetnet_cache::Dict{String, StreetNetwork}= Dict{String, StreetNetwork}();
#     filters::Vector{AbstractFeature}=AbstractFeature[]
#     )

#     pdsetfile = joinpath(PRIMARYDATA_DIR, toext("primarydata_" * csvfileset.csvfile, "jld"))

#     sn = _get_streetnetwork(csvfileset.streetmapbasename, streetnet_cache)
#     pdset = load(pdsetfile)["pdset"]

#     extract(behavior, validfinds, pdset, sn, features, filters)
# end
# function extract(
#     behavior::String,
#     features::Vector{AbstractFeature}=FEATURES,
#     streetnet_cache::Dict{String, StreetNetwork}= Dict{String, StreetNetwork}();
#     filters::Vector{AbstractFeature}=AbstractFeature[],
#     verbosity::Int=0
#     )

#     featuresets = Array(DataFrame, length(CSVFILESETS))
#     featuresetind = 0

#     for csvfileset in CSVFILESETS

#         if csvfileset.streetmapbasename == "skip"
#             continue
#         end

#         validfinds = get_validfind_regions(behavior, csvfileset)::Vector{Int}

#         verbosity == 0 || tic()
#         featuresets[featuresetind+=1] = extract(csvfileset, behavior, validfinds, features, streetnet_cache, filters=filters)
#         verbosity == 0 || toc()
#     end
    
#     featuresets[1:featuresetind]
# end
# function extract(
#     features::Vector{AbstractFeature}=FEATURES,
#     behaviors::Vector{String}=BEHAVIORS,
#     streetnet_cache::Dict{String, StreetNetwork}= Dict{String, StreetNetwork}();
#     filters::Vector{AbstractFeature}=AbstractFeature[],
#     verbosity::Int=0
#     )

#     aggregate_datafiles = Array(DataFrame, length(featuresets))

#     i = 0
#     for behavior in behaviors

#         verbosity == 0 || println(uppercase(behavior))

#         output_name = "ego_" * behavior

#         verbosity == 0 || tic()
#         featuresets = extract(behavior, features, streetnet_cache, filters=filters, verbosity=verbosity)
#         verbosity == 0 || toc()

#         aggregate_datafiles[i+=1] = _aggregate(featuresets)
#     end

#     aggregate_datafiles
# end

# function extract_and_save(
#     output_name::String,
#     csvfileset::CSVFileSet,
#     featureset_name::String,
#     validfinds::Vector{Int},
#     features::Vector{AbstractFeature}=FEATURES,
#     streetnet_cache::Dict{String, StreetNetwork}= Dict{String, StreetNetwork}();
#     filters::Vector{AbstractFeature}=AbstractFeature[]
#     )

#     featureset_dir  = joinpath(FEATURESETS_DIR, output_name*"/")
#     if !isdir(featureset_dir)
#         mkdir(featureset_dir)
#     end

#     featureset = extract(csvfileset, featureset_name, validfinds, features, streetnet_cache, filters=filters)

#     outfile = joinpath(featureset_dir, "featureset_" * csvfileset.csvfile * ".jld")
#     JLD.save(outfile, "featureset", featureset)

#     featureset
# end
# function extract_and_save(
#     output_name::String,
#     featureset_name::String,
#     f_regions::Function,
#     features::Vector{AbstractFeature}=FEATURES,
#     streetnet_cache::Dict{String, StreetNetwork}= Dict{String, StreetNetwork}();
#     filters::Vector{AbstractFeature}=AbstractFeature[],
#     )

#     featuresets = extract(featureset_name, f_regions, features, streetnet_cache, filters=filters, verbosity=verbosity)
#     aggregate_datafile = _aggregate(featuresets)

#     outfile = joinpath(FEATUREMATRIX_DIR, output_name*".jld")
#     save(outfile, "data", aggregate_datafile)

#     aggregate_datafile
# end

end # module