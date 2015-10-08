using AutomotiveDrivingModels
using AutomotiveDrivingModels.StreetNetworks.RoadNetwork

const STREETNET_CACHE = Dict{String, StreetNetwork}()
const PRIMARYDATA_DIR = "/media/tim/DATAPART1/Data/Bosch/processed/primarydata/"
const STREETMAP_BASE = "/media/tim/DATAPART1/Data/Bosch/processed/streetmaps/"
const OUTPUT_DIR = "/media/tim/DATAPART1/PublicationData/2015_TrafficEvolutionModels/realworld/"
const PDSET_OUTPUT_DIR = joinpath(OUTPUT_DIR, "pdsets")

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
const CSVFILESETS = (
            CSVFileSet(CARID_EGO,
                "/media/tim/DATAPART1/Data/Bosch/philippruns_20150324/philippdrive01_2015-03-24-15-21-12-raw.csv",
                "280_pagemill_to_92", # 16
                [128,297,475,673,3520,3671,4363,4559,5724,5833,6816,6937,7086,7343,7650,7765,8102,8283,8826,9333,9922,10095],
                [2641,2833,3790,3981,5026,5255,10184,10321],
                [9816,9921],
                [1,127,298,475,1900,2293,5256,5723,6466,6815,6938,7085,7343,7649,7766,8101,8754,8825],
                [674,1899,2294,2641,2834,3519,3672,3789,3982,4363,4560,5025,5834,6465,8284,8753,9334,9815,10096,10183]
                ),
            CSVFileSet(CARID_EGO,
                "/media/tim/DATAPART1/Data/Bosch/philippruns_20150324/philippdrive03_2015-03-24-15-37-18-raw.csv",
                "280_pagemill_to_92", # 16
                [1182,1386,1425,1562,1657,1826,3331,3568,4929,5238,5691,5840,5899,6112,7913,8040,8407,8532],
                [3191,3330,3785,3974,4027,4212,6275,6778,8111,8278],
                [7712,7888,8533,8722],
                [1387,1424,5239,5690,5841,5898,7889,7912,9055,9160],
                [1,1182,1563,1656,1827,3190,3569,3784,3975,4026,4213,4928,6113,6274,8041,8110,8279,8406,8723,9054]
                ),
            CSVFileSet(CARID_EGO,
                "/media/tim/DATAPART1/Data/Bosch/philippruns_20150324/philippdrive04_2015-03-24-15-47-10-raw.csv",
                "280_pagemill_to_92", # 2
                [811,944,945,1148],
                Int[],
                Int[],
                Int[],
                [1,810]
                ),
            CSVFileSet(CARID_EGO,
                "/media/tim/DATAPART1/Data/Bosch/philippruns_20150324/philippdrive05_2015-03-24-15-48-42-raw.csv",
                "280_pagemill_to_92", # 5
                [149,314,628,934,1571,1683,2819,3032,3107,3308],
                Int[],
                Int[],
                [1,148,935,1570,1684,2818,3033,3106,3309,4434],
                [315,626]
                ),
            CSVFileSet(CARID_EGO,
                "/media/tim/DATAPART1/Data/Bosch/philippruns_20150324/philippdrive07_2015-03-24-15-57-42-raw.csv",
                "280_pagemill_to_92", # 12
                [31,138,965,1176,1481,1668,2753,2936,4135,4350,4351,4530,4672,4880,5271,5516,5856,6026,6195,6412,7775,7938,8519,8608],
                Int[],
                [6989,7156],
                [481,964,1177,1480,1669,2752,2937,4134,4531,4672,4999,5156,7157,7774,7939,8518],
                [1,30,139,480,4881,4998,5157,5270,5517,5856,6027,6194,6413,6988]
                ),
            CSVFileSet(CARID_EGO,
                "/media/tim/DATAPART1/Data/Bosch/source2/2014-08-03-12-01-09-edited.csv",
                "its_inner_loop_v1_2", # 2
                Int[],
                Int[],
                [4372,4482,4831,4976],
                [417,4266],
                [1,416,4483,4830,4977,5900]
                ),
            CSVFileSet(CARID_EGO,
                "/media/tim/DATAPART1/Data/Bosch/source2/2014-08-03-11-52-58-raw.csv",
                "its_inner_loop_v1_2",
                Int[],
                Int[],
                [1,153,359,518],
                [1101,1700],
                [154,358,519,1100,2494,3940]
                ),
            CSVFileSet(CARID_EGO,
                "/media/tim/DATAPART1/Data/Bosch/source2/2014-08-03-12-12-53-raw.csv",
                "its_inner_loop_v1_2",
                Int[],
                Int[],
                [1740,1892],
                [956,1348],
                [1624,1739,1893,3060]
                ),
            CSVFileSet(CARID_EGO,
                "/media/tim/DATAPART1/Data/Bosch/source2/2014-08-03-12-16-14-raw.csv",
                "its_both_loops_v3_smoothed",
                Int[],
                Int[],
                Int[],
                [2477,3918],
                [1,1020,1540,1936,2120,2476]
                ),
            CSVFileSet(CARID_EGO,
                "/media/tim/DATAPART1/Data/Bosch/source2/2014-08-03-12-20-36-raw.csv",
                "its_inner_loop_v1_2",
                Int[],
                Int[],
                [2733,2898],
                [68,2100],
                [2506,2732,2899,3292]
                ),
            CSVFileSet(CARID_EGO,
                "/media/tim/DATAPART1/Data/Bosch/source2/2014-08-03-12-54-24-raw.csv",
                "its_inner_loop_v1_2",
                Int[],
                Int[],
                Int[],
                [3166,3534,3972,6272,6448,7678,8036,8228,9031,9262],
                [8235,8500,8730,9030]
                ),
            CSVFileSet(CARID_EGO,
                "/media/tim/DATAPART1/Data/Bosch/source2/2014-08-06-16-11-51_large_loop_3-raw.csv",
                "detroit_v4",
                Int[],
                Int[],
                Int[],
                Int[],
                [1,1618,1642,1834,1844,1902,3810,4232,6618,8588]
                ),
        )
const FEATURES = [
        YAW, POSFX, POSFY, SPEED, VELFX, VELFY, DELTA_SPEED_LIMIT,
        D_CL, D_ML, D_MR, D_MERGE, D_SPLIT,
        TIMETOCROSSING_RIGHT, TIMETOCROSSING_LEFT, ESTIMATEDTIMETOLANECROSSING, TIMESINCELANECROSSING,
        N_LANE_L, N_LANE_R, HAS_LANE_L, HAS_LANE_R,
        TURNRATE, TURNRATE_GLOBAL, ACC, ACCFX, ACCFY, A_REQ_STAYINLANE, LANECURVATURE,

        HAS_FRONT, D_X_FRONT, D_Y_FRONT, V_X_FRONT, V_Y_FRONT, YAW_FRONT, TURNRATE_FRONT,
        HAS_REAR,  D_X_REAR,  D_Y_REAR,  V_X_REAR,  V_Y_REAR,  YAW_REAR,  TURNRATE_REAR,
                   D_X_LEFT,  D_Y_LEFT,  V_X_LEFT,  V_Y_LEFT,  YAW_LEFT,  TURNRATE_LEFT,
                   D_X_RIGHT, D_Y_RIGHT, V_X_RIGHT, V_Y_RIGHT, YAW_RIGHT, TURNRATE_RIGHT,
        A_REQ_FRONT, TTC_X_FRONT, TIMEGAP_X_FRONT, GAINING_ON_FRONT,
        A_REQ_REAR,  TTC_X_REAR,  TIMEGAP_X_REAR,
        A_REQ_LEFT,  TTC_X_LEFT,  TIMEGAP_X_LEFT,
        A_REQ_RIGHT, TTC_X_RIGHT, TIMEGAP_X_RIGHT,

        SCENEVELFX,

        OCCUPANCYSCHEDULEGRID_TIME_FRONT,      OCCUPANCYSCHEDULEGRID_ISOCCUPIED_FRONT,
        OCCUPANCYSCHEDULEGRID_TIME_FRONTRIGHT, OCCUPANCYSCHEDULEGRID_ISOCCUPIED_FRONTRIGHT,
        OCCUPANCYSCHEDULEGRID_TIME_RIGHT,      OCCUPANCYSCHEDULEGRID_ISOCCUPIED_RIGHT,
        OCCUPANCYSCHEDULEGRID_TIME_BACKRIGHT,  OCCUPANCYSCHEDULEGRID_ISOCCUPIED_BACKRIGHT,
        OCCUPANCYSCHEDULEGRID_TIME_BACK,       OCCUPANCYSCHEDULEGRID_ISOCCUPIED_BACK,
        OCCUPANCYSCHEDULEGRID_TIME_BACKLEFT,   OCCUPANCYSCHEDULEGRID_ISOCCUPIED_BACKLEFT,
        OCCUPANCYSCHEDULEGRID_TIME_LEFT,       OCCUPANCYSCHEDULEGRID_ISOCCUPIED_LEFT,
        OCCUPANCYSCHEDULEGRID_TIME_FRONTLEFT,  OCCUPANCYSCHEDULEGRID_ISOCCUPIED_FRONTLEFT,

        TIME_CONSECUTIVE_BRAKE, TIME_CONSECUTIVE_ACCEL, TIME_CONSECUTIVE_THROTTLE,
             PASTACC250MS,      PASTACC500MS,      PASTACC750MS,      PASTACC1S,
        PASTTURNRATE250MS, PASTTURNRATE500MS, PASTTURNRATE750MS, PASTTURNRATE1S,
           PASTVELFY250MS,    PASTVELFY500MS,    PASTVELFY750MS,    PASTVELFY1S,
            PASTD_CL250MS,     PASTD_CL500MS,     PASTD_CL750MS,     PASTD_CL1S,

            MAXACCFX500MS,     MAXACCFX750MS,     MAXACCFX1S,     MAXACCFX1500MS,     MAXACCFX2S,     MAXACCFX2500MS,     MAXACCFX3S,     MAXACCFX4S,
            MAXACCFY500MS,     MAXACCFY750MS,     MAXACCFY1S,     MAXACCFY1500MS,     MAXACCFY2S,     MAXACCFY2500MS,     MAXACCFY3S,     MAXACCFY4S,
         MAXTURNRATE500MS,  MAXTURNRATE750MS,  MAXTURNRATE1S,  MAXTURNRATE1500MS,  MAXTURNRATE2S,  MAXTURNRATE2500MS,  MAXTURNRATE3S,  MAXTURNRATE4S,
           MEANACCFX500MS,    MEANACCFX750MS,    MEANACCFX1S,    MEANACCFX1500MS,    MEANACCFX2S,    MEANACCFX2500MS,    MEANACCFX3S,    MEANACCFX4S,
           MEANACCFY500MS,    MEANACCFY750MS,    MEANACCFY1S,    MEANACCFY1500MS,    MEANACCFY2S,    MEANACCFY2500MS,    MEANACCFY3S,    MEANACCFY4S,
        MEANTURNRATE500MS, MEANTURNRATE750MS, MEANTURNRATE1S, MEANTURNRATE1500MS, MEANTURNRATE2S, MEANTURNRATE2500MS, MEANTURNRATE3S, MEANTURNRATE4S,
            STDACCFX500MS,     STDACCFX750MS,     STDACCFX1S,     STDACCFX1500MS,     STDACCFX2S,     STDACCFX2500MS,     STDACCFX3S,     STDACCFX4S,
            STDACCFY500MS,     STDACCFY750MS,     STDACCFY1S,     STDACCFY1500MS,     STDACCFY2S,     STDACCFY2500MS,     STDACCFY3S,     STDACCFY4S,
         STDTURNRATE500MS,  STDTURNRATE750MS,  STDTURNRATE1S,  STDTURNRATE1500MS,  STDTURNRATE2S,  STDTURNRATE2500MS,  STDTURNRATE3S,  STDTURNRATE4S,

        FUTUREACCELERATION_250MS, FUTURETURNRATE_250MS, FUTUREDESIREDANGLE_250MS, FUTUREDESIREDSPEED_250MS, FUTUREACCELCONTROL_250MS,
        FUTUREACCELERATION_500MS, FUTURETURNRATE_500MS, FUTUREDESIREDANGLE_500MS, FUTUREDESIREDSPEED_500MS, FUTUREACCELCONTROL_500MS,
        TIMETOLANECROSSING, TIMESINCELANECROSSING,

        SUBSET_EMERGENCY, SUBSET_FREE_FLOW, SUBSET_CAR_FOLLOWING, SUBSET_LANE_CROSSING, SUBSET_SUSTAINED_CROSSING,
        SUBSET_AT_SIXTYFIVE,
    ]
const FILTERS = AbstractFeature[]

#############################################

@assert(isdir(OUTPUT_DIR))
if !isdir(PDSET_OUTPUT_DIR)
    mkdir(PDSET_OUTPUT_DIR)
end

#############################################

pdset_extract_params = PrimaryDataExtractionParams()

tic()
csvfilesets = CSVFileSet[]
for csvfileset in CSVFILESETS
    csvfilename = csvfileset.csvfile
    streetmapbasename = csvfileset.streetmapbasename

    if !haskey(STREETNET_CACHE, streetmapbasename)
        STREETNET_CACHE[streetmapbasename] = load(joinpath(STREETMAP_BASE, "streetmap_" * streetmapbasename*".jld"))["streetmap"]
    end
    sn = STREETNET_CACHE[streetmapbasename]

    println(csvfilename)

    trajdata = load_trajdata(csvfilename)
    pdset = gen_primary_data(trajdata, sn, pdset_extract_params)

    if isa(pdset, PrimaryDataset)
        csvfilebase = basename(csvfilename)
        pdsetname = joinpath(PDSET_OUTPUT_DIR, toext("primarydata_" * csvfilebase, "jld"))
        JLD.save(pdsetname, "pdset", pdset::PrimaryDataset)

        push!(csvfilesets, csvfileset)
    end
end
toc()

save(joinpath(OUTPUT_DIR, "csvfilesets.jld"), "csvfilesets", CSVFILESETS)