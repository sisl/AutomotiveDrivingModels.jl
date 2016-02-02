# VARIABLES
const INCLUDE_FILE_BASE = "realworld"
const N_SIMULATIONS_PER_TRACE = 10
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

                    Feature_Past{symbol(VELFT), 1}(),  Feature_Past{symbol(VELFT), 2}(),  Feature_Past{symbol(VELFT), 3}(),  Feature_Past{symbol(VELFT), 4}(),
                    Feature_Past{symbol(POSFT), 1}(),  Feature_Past{symbol(POSFT), 2}(),  Feature_Past{symbol(POSFT), 3}(),  Feature_Past{symbol(POSFT), 4}(),
                    Feature_Past{symbol(INV_TTC_FRONT), 1}(),  Feature_Past{symbol(INV_TTC_FRONT), 2}(),  Feature_Past{symbol(INV_TTC_FRONT), 3}(),  Feature_Past{symbol(INV_TTC_FRONT), 4}(),
                    Feature_Past{symbol(ACC), 1}(),    Feature_Past{symbol(ACC), 2}(),    Feature_Past{symbol(ACC), 3}(),    Feature_Past{symbol(ACC), 4}(),
                    Feature_Past{:f_des_angle, 1}(),   Feature_Past{:f_des_angle, 2}(),   Feature_Past{:f_des_angle, 3}(),   Feature_Past{:f_des_angle, 4}(),

                    Feature_Mean_Over_History{symbol(ACC),4}(), Feature_Mean_Over_History{symbol(ACC),8}(),
                    Feature_Mean_Over_History{:f_des_angle,4}(), Feature_Mean_Over_History{:f_des_angle,8}(),

                    Feature_Std_Over_History{symbol(ACC),4}(), Feature_Std_Over_History{symbol(ACC),8}(),
                    Feature_Std_Over_History{:f_des_angle,4}(), Feature_Std_Over_History{:f_des_angle,8}(),

                    Feature_Max_Over_History{symbol(ACC),4}(), Feature_Max_Over_History{symbol(ACC),8}(),
                    Feature_Max_Over_History{:f_des_angle,4}(), Feature_Max_Over_History{:f_des_angle,8}(),

                    Feature_Min_Over_History{symbol(ACC),4}(), Feature_Min_Over_History{symbol(ACC),8}(),
                    Feature_Min_Over_History{:f_des_angle,4}(), Feature_Min_Over_History{:f_des_angle,8}(),
                ]