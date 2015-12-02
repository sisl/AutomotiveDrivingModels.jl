# VARIABLES
const TRAIN_TEST_SPLIT_TEST_FRACTION = 0.10 # amount of data went to test
const NFOLDS = 5
const TRAJDATA_SEC_PER_FRAME = 0.05
const MAX_SIMLOGS_ALLOCATION_SIZE = 10^9 # [bytes]

const SIM_SEC_PER_FRAME      = 0.25 # NOTE(tim): currently this must be an even multiple of TRAJDATA_SEC_PER_FRAME
const SIM_HORIZON_IN_SECONDS = 4.0
const SIM_HISTORY_IN_FRAMES  = 8

const EVALUATION_DIR = let
    hostname = gethostname()
    if hostname == "Cupertino"
        "/media/tim/DATAPART1/PublicationData/2015_TrafficEvolutionModels/realworld/"
    elseif hostname == "tula"
        "/home/wheelert/PublicationData/2015_TrafficEvolutionModels/realworld/"
    else
        "unknown"
    end
end

const PDSET_DIR = joinpath(EVALUATION_DIR, "pdsets")
const CSVFILESETS = [load(joinpath(EVALUATION_DIR, "csvfilesets.jld"), "csvfilesets")...]::Vector{CSVFileSet}

const PDSET_FRAMES_PER_SIM_FRAME = round(Int, SIM_SEC_PER_FRAME/TRAJDATA_SEC_PER_FRAME, RoundNearestTiesUp)
const SIM_HORIZON_IN_FRAMES = round(Int, SIM_HORIZON_IN_SECONDS/SIM_SEC_PER_FRAME, RoundNearestTiesUp)
const NFRAMES_TOTAL = SIM_HORIZON_IN_FRAMES + SIM_HISTORY_IN_FRAMES
const FRAMESKIP_BETWEEN_EXTRACTED_SCENES = SIM_HORIZON_IN_FRAMES + 1

const TARGET_SET = ModelTargets(FUTUREDESIREDANGLE_250MS, FUTUREACCELERATION_250MS)
const SIMPARAMS = DEFAULT_SIM_PARAMS

const EXTRACT_PARAMS = OrigHistobinExtractParameters( "all",
                        AbstractFeature[],
                        Inf, Inf, Inf, Inf, Inf, Inf, Inf, Inf, Inf,
                        PDSET_FRAMES_PER_SIM_FRAME, SIM_HORIZON_IN_FRAMES, SIM_HISTORY_IN_FRAMES,
                        FRAMESKIP_BETWEEN_EXTRACTED_SCENES)

const HISTOBIN_PARAMS = ParamsHistobin(
                    LinearDiscretizer(collect(linspace(4.0 - 0.5, 4.0 + 0.5, 50)), force_outliers_to_closest=true),
                    LinearDiscretizer(collect(linspace(     -3.5,       3.5, 50)), force_outliers_to_closest=true))

const INDICATOR_SET = [
                    YAW, SPEED, VELFX, VELFY, #DELTA_SPEED_LIMIT,
                    D_CL, D_ML, D_MR, #D_MERGE, D_SPLIT,
                    TIMETOCROSSING_RIGHT, TIMETOCROSSING_LEFT, TIMESINCELANECROSSING, ESTIMATEDTIMETOLANECROSSING,
                    N_LANE_L, N_LANE_R, HAS_LANE_L, HAS_LANE_R,
                    TURNRATE,
                    ACC,
                    ACCFX, ACCFY, A_REQ_STAYINLANE,
                    TURNRATE_GLOBAL, LANECURVATURE,
                    SUMO, IDM,

                    HAS_FRONT, D_X_FRONT, D_Y_FRONT, V_X_FRONT, V_Y_FRONT, YAW_FRONT, TURNRATE_FRONT,
                    HAS_REAR,  D_X_REAR,  D_Y_REAR,  V_X_REAR,  V_Y_REAR,  YAW_REAR,  TURNRATE_REAR,
                               D_X_LEFT,  D_Y_LEFT,  V_X_LEFT,  V_Y_LEFT,  YAW_LEFT,  TURNRATE_LEFT,
                               D_X_RIGHT, D_Y_RIGHT, V_X_RIGHT, V_Y_RIGHT, YAW_RIGHT, TURNRATE_RIGHT,
                    A_REQ_FRONT, TTC_X_FRONT, TIMEGAP_X_FRONT,
                    A_REQ_REAR,  TTC_X_REAR,  TIMEGAP_X_REAR,
                    A_REQ_LEFT,  TTC_X_LEFT,  TIMEGAP_X_LEFT,
                    A_REQ_RIGHT, TTC_X_RIGHT, TIMEGAP_X_RIGHT,

                    SCENEVELFX,

                    TIME_CONSECUTIVE_THROTTLE, # TIME_CONSECUTIVE_BRAKE, TIME_CONSECUTIVE_ACCEL,
                         PASTACC250MS,      PASTACC500MS,      PASTACC750MS,      PASTACC1S,
                    PASTTURNRATE250MS, PASTTURNRATE500MS, PASTTURNRATE750MS, PASTTURNRATE1S,
                       PASTVELFY250MS,    PASTVELFY500MS,    PASTVELFY750MS,    PASTVELFY1S,
                        PASTD_CL250MS,     PASTD_CL500MS,     PASTD_CL750MS,     PASTD_CL1S,

                     #     MAXACCFX500MS,     MAXACCFX750MS,     MAXACCFX1S,     MAXACCFX1500MS,     MAXACCFX2S,     MAXACCFX2500MS,     MAXACCFX3S,     MAXACCFX4S,
                     #     MAXACCFY500MS,     MAXACCFY750MS,     MAXACCFY1S,     MAXACCFY1500MS,     MAXACCFY2S,     MAXACCFY2500MS,     MAXACCFY3S,     MAXACCFY4S,
                     #  MAXTURNRATE500MS,  MAXTURNRATE750MS,  MAXTURNRATE1S,  MAXTURNRATE1500MS,  MAXTURNRATE2S,  MAXTURNRATE2500MS,  MAXTURNRATE3S,  MAXTURNRATE4S,
                     #    MEANACCFX500MS,    MEANACCFX750MS,    MEANACCFX1S,    MEANACCFX1500MS,    MEANACCFX2S,    MEANACCFX2500MS,    MEANACCFX3S,    MEANACCFX4S,
                     #    MEANACCFY500MS,    MEANACCFY750MS,    MEANACCFY1S,    MEANACCFY1500MS,    MEANACCFY2S,    MEANACCFY2500MS,    MEANACCFY3S,    MEANACCFY4S,
                     # MEANTURNRATE500MS, MEANTURNRATE750MS, MEANTURNRATE1S, MEANTURNRATE1500MS, MEANTURNRATE2S, MEANTURNRATE2500MS, MEANTURNRATE3S, MEANTURNRATE4S,
                     #     STDACCFX500MS,     STDACCFX750MS,     STDACCFX1S,     STDACCFX1500MS,     STDACCFX2S,     STDACCFX2500MS,     STDACCFX3S,     STDACCFX4S,
                     #     STDACCFY500MS,     STDACCFY750MS,     STDACCFY1S,     STDACCFY1500MS,     STDACCFY2S,     STDACCFY2500MS,     STDACCFY3S,     STDACCFY4S,
                     #  STDTURNRATE500MS,  STDTURNRATE750MS,  STDTURNRATE1S,  STDTURNRATE1500MS,  STDTURNRATE2S,  STDTURNRATE2500MS,  STDTURNRATE3S,  STDTURNRATE4S,
                ]