# VARIABLES
const INCLUDE_FILE_BASE = "realworld"
const N_SIMULATIONS_PER_TRACE = 50
const DEFAULT_TRACE_HISTORY = 2*DEFAULT_FRAME_PER_SEC
const N_BAGGING_SAMPLES = 10
const N_FOLDS = 5
const CONFIDENCE_LEVEL = 0.95
const MAX_CV_OPT_TIME_PER_MODEL = 60.0 # [s]

const PDSET_FRAMES_PER_SIM_FRAME = 5
const SIM_SEC_PER_FRAME      = PDSET_FRAMES_PER_SIM_FRAME*DEFAULT_SEC_PER_FRAME
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

const RUNLOG_DIR = joinpath(EVALUATION_DIR, "runlogs")

const SIM_HORIZON_IN_FRAMES = round(Int, SIM_HORIZON_IN_SECONDS/SIM_SEC_PER_FRAME, RoundNearestTiesUp)
const NFRAMES_TOTAL = SIM_HORIZON_IN_FRAMES + SIM_HISTORY_IN_FRAMES
const FRAMESKIP_BETWEEN_EXTRACTED_SCENES = 1

const INDICATOR_SET2 = [
                    POSFYAW, POSFT, SPEED, VELBX, VELBY, VELFS, VELFT,
                    TURNRATE, ACC, ACCFS, ACCFT, ACCBX, ACCBY,
                    MARKERDIST_LEFT, MARKERDIST_RIGHT,
                    TIMETOCROSSING_LEFT, TIMETOCROSSING_RIGHT, ESTIMATEDTIMETOLANECROSSING, A_REQ_STAYINLANE,
                    N_LANE_LEFT, N_LANE_RIGHT, HAS_LANE_RIGHT, HAS_LANE_LEFT, LANECURVATURE,

                    DIST_MERGE, DIST_SPLIT, SCENE_SPEED_DIFFERENCE,
                    HAS_FRONT, DIST_FRONT, D_Y_FRONT, DELTA_V_FRONT, DELTA_V_Y_FRONT, YAW_FRONT, TURNRATE_FRONT, ACC_REQ_FRONT, INV_TTC_FRONT, INV_TIMEGAP_FRONT, GAINING_ON_FRONT,
                    HAS_REAR,  DIST_REAR,  D_Y_REAR,  DELTA_V_REAR,  DELTA_V_Y_REAR,  YAW_REAR,  TURNRATE_REAR,  ACC_REQ_REAR,  INV_TTC_REAR,  INV_TIMEGAP_REAR,  REAR_IS_GAINING,
                    HAS_LEFT,  DIST_LEFT,  D_Y_LEFT,  DELTA_V_LEFT,  DELTA_V_Y_LEFT,  YAW_LEFT,  TURNRATE_LEFT,  ACC_REQ_LEFT,  INV_TTC_LEFT,  INV_TIMEGAP_LEFT,  LEFT_IS_GAINING,
                    HAS_RIGHT, DIST_RIGHT, D_Y_RIGHT, DELTA_V_RIGHT, DELTA_V_Y_RIGHT, YAW_RIGHT, TURNRATE_RIGHT, ACC_REQ_RIGHT, INV_TTC_RIGHT, INV_TIMEGAP_RIGHT, RIGHT_IS_GAINING,
                ]
for feature in [VELFT, POSFT, DIST_FRONT, DELTA_V_FRONT, INV_TTC_FRONT, INV_TIMEGAP_FRONT, ACC, FUTUREDESIREDANGLE, TURNRATE, ESTIMATEDTIMETOLANECROSSING, A_REQ_STAYINLANE, LANECURVATURE]
    for history in 1 : 4
        push!(INDICATOR_SET2, Feature_Past{symbol(feature), history}())
    end
end
for feature in [ACC, FUTUREDESIREDANGLE, ]
    for history in 4 : 8
        push!(INDICATOR_SET2, Feature_Mean_Over_History{symbol(feature),history}())
        push!(INDICATOR_SET2, Feature_Std_Over_History{symbol(feature),history}())
        push!(INDICATOR_SET2, Feature_Max_Over_History{symbol(feature),history}())
        push!(INDICATOR_SET2, Feature_Min_Over_History{symbol(feature),history}())
    end
end


const INDICATOR_SET_SMALL = [
                    SPEED, ACC,
                    DIST_FRONT, DELTA_V_FRONT,
                    TURNRATE, POSFT,

                    Feature_Past{symbol(ACC), 1}(),
                    Feature_Past{symbol(TURNRATE), 1}(),
                    Feature_Past{symbol(POSFT), 1}(),
                ]
const INDICATOR_SET_SMALL2 = [SPEED, ACC, DIST_FRONT, DELTA_V_FRONT, TURNRATE, POSFT]
for feature in [VELFT, ACC, DIST_FRONT, DELTA_V_FRONT, TURNRATE, POSFT]
    for history in 1:4
        push!(INDICATOR_SET_SMALL2, Feature_Past{symbol(feature), history}())
    end
end

const INDICATOR_SET_CLEAN = filter!(f->!couldna(f), deepcopy(INDICATOR_SET2))