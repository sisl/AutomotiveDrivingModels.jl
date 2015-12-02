using AutomotiveDrivingModels

include(Pkg.dir("AutomotiveDrivingModels", "src", "io", "filesystem_utils.jl"))

##############################
# PARAMETERS
##############################

const INCLUDE_FILE = Pkg.dir("AutomotiveDrivingModels", "scripts", "extract_params.jl")

include(INCLUDE_FILE)


DATASET_MODIFIER = ""
# DATASET_MODIFIER = "_small"; CSVFILESETS = CSVFILESETS[1:4]
# DATASET_MODIFIER = "_medium"; CSVFILESETS = CSVFILESETS[1:100]

##############################
# LOAD
##############################

const SAVE_DIR = joinpath(EVALUATION_DIR, "bincounts_opt")
if !isdir(SAVE_DIR)
    mkdir(SAVE_DIR)
end

features = [
                    FUTUREACCELERATION_250MS, FUTUREDESIREDANGLE_250MS,

                    YAW, SPEED, VELFX, VELFY, DELTA_SPEED_LIMIT,
                    D_CL, D_ML, D_MR, D_MERGE, D_SPLIT,
                    TIMETOCROSSING_RIGHT, TIMETOCROSSING_LEFT, TIMESINCELANECROSSING, ESTIMATEDTIMETOLANECROSSING,
                    N_LANE_L, N_LANE_R, HAS_LANE_L, HAS_LANE_R,
                    TURNRATE,
                    ACC,
                    ACCFX, ACCFY, A_REQ_STAYINLANE,
                    TURNRATE_GLOBAL, LANECURVATURE,
                    IDM, SUMO,

                    HAS_FRONT, D_X_FRONT, D_Y_FRONT, V_X_FRONT, V_Y_FRONT, YAW_FRONT, TURNRATE_FRONT,
                    HAS_REAR,  D_X_REAR,  D_Y_REAR,  V_X_REAR,  V_Y_REAR,  YAW_REAR,  TURNRATE_REAR,
                               D_X_LEFT,  D_Y_LEFT,  V_X_LEFT,  V_Y_LEFT,  YAW_LEFT,  TURNRATE_LEFT,
                               D_X_RIGHT, D_Y_RIGHT, V_X_RIGHT, V_Y_RIGHT, YAW_RIGHT, TURNRATE_RIGHT,
                    A_REQ_FRONT, TTC_X_FRONT, TIMEGAP_X_FRONT,
                    A_REQ_REAR,  TTC_X_REAR,  TIMEGAP_X_REAR,
                    A_REQ_LEFT,  TTC_X_LEFT,  TIMEGAP_X_LEFT,
                    A_REQ_RIGHT, TTC_X_RIGHT, TIMEGAP_X_RIGHT,

                    SCENEVELFX,

                    TIME_CONSECUTIVE_THROTTLE, TIME_CONSECUTIVE_BRAKE, TIME_CONSECUTIVE_ACCEL,
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
                ]
filters = AbstractFeature[Feature_IsClean{symbol(TARGET_SET.lat)}(),
                          Feature_IsClean{symbol(TARGET_SET.lon)}()]

for subset in (SUBSET_FREE_FLOW, SUBSET_CAR_FOLLOWING, SUBSET_LANE_CROSSING)

    subset_name = string(symbol(subset))
    println("Extracting ", subset_name)

    EXTRACT_PARAMS.subsets = AbstractFeature[subset]

    tic()
    model_training_data = pull_model_training_data(EXTRACT_PARAMS, CSVFILESETS, pdset_dir=PDSET_DIR, features=features, filters=filters)
    toc()

    for i in nrow(model_training_data.dataframe)
        v = model_training_data.dataframe[i, symbol(FUTUREACCELERATION_250MS)]
        @assert(!isnan(v) && !isinf(v))

        v = model_training_data.dataframe[i, symbol(FUTUREDESIREDANGLE_250MS)]
        @assert(!isnan(v) && !isinf(v))
    end

    println("dataset extraction for $subset_name complete")

    dataset_filepath = joinpath(EVALUATION_DIR, "dataset_" * subset_name * ".jld")
    JLD.save(dataset_filepath, "model_training_data", model_training_data,
                               "extract_params", EXTRACT_PARAMS)

    println("num pdset_segments: ", length(model_training_data.pdset_segments))
    println("size of dataframe:  ", size(model_training_data.dataframe))
end

println("[DONE]")

