using AutomotiveDrivingModels
using RandomForestBehaviors

################################
# LOAD PARAMETERS
################################

const INCLUDE_FILE = let
    hostname = gethostname()
    if hostname == "Cupertino"
        "/media/tim/DATAPART1/PublicationData/2015_TrafficEvolutionModels/realworld/extract_params.jl"
    elseif hostname == "tula"
        "/home/wheelert/PublicationData/2015_TrafficEvolutionModels/vires_highway_2lane_sixcar/extract_params.jl"
    else
        error("unknown hostname")
    end
end
const INCLUDE_NAME = splitdir(splitext(INCLUDE_FILE)[1])[2]

include(INCLUDE_FILE)

const SAVE_FILE_MODIFIER = "_subset_car_following"
const BEST_MODEL_CSV_FILE = joinpath(EVALUATION_DIR, "best_model_df" * SAVE_FILE_MODIFIER * ".csv")
const TRAIN_VALIDATION_JLD_FILE = joinpath(EVALUATION_DIR, "train_validation_split" * SAVE_FILE_MODIFIER * ".jld")
const DATASET_JLD_FILE = joinpath(EVALUATION_DIR, "dataset" * SAVE_FILE_MODIFIER * ".jld")


################################
# LOAD TRAIN AND VALIDATION SETS
################################

dset = JLD.load(DATASET_JLD_FILE, "model_training_data")
cross_validation_split = JLD.load(TRAIN_VALIDATION_JLD_FILE, "cross_validation_split")

##############################
# LOAD PARAMETERS
##############################

type Parameter
    sym::Symbol
    range::AbstractArray
end

params = [
    Parameter(:ntrees, 51:-5:1),
    Parameter(:max_depth, 20:-1:1),
    Parameter(:min_samples_split, 10:10:50),
    Parameter(:min_samples_leaves, [20,10,4,2]),
    Parameter(:min_split_improvement, [10.0,5.0, 1.0,0.5,0.1,0.0]),
    Parameter(:partial_sampling, [0.5,0.6,0.7,0.8,0.9,0.95,1.0]),
    Parameter(:n_split_tries, [50,100,200,25,10]),
]

nparams = length(params)

##############################
# SET UP PARAMS
##############################

indicator_set = [
                    YAW, SPEED,
                    # VELFX,
                    VELFY, #DELTA_SPEED_LIMIT,
                    D_CL,
                    # D_ML, D_MR, #D_MERGE, D_SPLIT,
                    # TIMETOCROSSING_RIGHT, TIMETOCROSSING_LEFT, TIMESINCELANECROSSING,
                    ESTIMATEDTIMETOLANECROSSING,
                    # N_LANE_L, N_LANE_R,
                    # HAS_LANE_L, HAS_LANE_R,
                    TURNRATE, ACC,
                    # ACCFX, ACCFY, A_REQ_STAYINLANE,
                    # TURNRATE_GLOBAL, LANECURVATURE,

                    # HAS_FRONT,
                    D_X_FRONT,
                    # D_Y_FRONT,
                    V_X_FRONT,
                    # V_Y_FRONT, YAW_FRONT, TURNRATE_FRONT,
                    # HAS_REAR,  D_X_REAR,  D_Y_REAR,  V_X_REAR,  V_Y_REAR,  YAW_REAR,  TURNRATE_REAR,
                               # D_X_LEFT,  D_Y_LEFT,  V_X_LEFT,  V_Y_LEFT,  YAW_LEFT,  TURNRATE_LEFT,
                               # D_X_RIGHT, D_Y_RIGHT, V_X_RIGHT, V_Y_RIGHT, YAW_RIGHT, TURNRATE_RIGHT,
                    # A_REQ_FRONT, TTC_X_FRONT, TIMEGAP_X_FRONT,
                    # A_REQ_REAR,  TTC_X_REAR,  TIMEGAP_X_REAR,
                    # A_REQ_LEFT,  TTC_X_LEFT,  TIMEGAP_X_LEFT,
                    # A_REQ_RIGHT, TTC_X_RIGHT, TIMEGAP_X_RIGHT,
                    # SCENEVELFX,

                    TIME_CONSECUTIVE_THROTTLE, # TIME_CONSECUTIVE_BRAKE, TIME_CONSECUTIVE_ACCEL,
                         PASTACC250MS,      PASTACC500MS,      PASTACC750MS,      PASTACC1S,
                    PASTTURNRATE250MS, PASTTURNRATE500MS, PASTTURNRATE750MS, PASTTURNRATE1S,
                    #    PASTVELFY250MS,    PASTVELFY500MS,    PASTVELFY750MS,    PASTVELFY1S,
                        # PASTD_CL250MS,     PASTD_CL500MS,     PASTD_CL750MS,     PASTD_CL1S,

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

behavior_type = DynamicForestBehavior
behavior_name = "Dynamic Forest"
behavior_train_params = Dict{Symbol,Any}()
behavior_train_params[:indicators] = INDICATOR_SET
for p in params
    behavior_train_params[p.sym] = p.range[1]
end
param_indeces = ones(Int, nparams)

##############################
# LOAD PREVIOUSLY BEST MODEL
##############################

best_param_score = -Inf
param_hash = Set{UInt}()

# prev_best = readtable(BEST_MODEL_CSV_FILE)
# if prev_best[1,:logl_mean] < logl_mean
#     prev_best[1, :logl_mean] = logl_mean
#     prev_best[1, :logl_stdev] = logl_stdev
#     prev_best[1, :ntrees] = ntrees
#     writetable(BEST_MODEL_CSV_FILE, prev_best)
# end

df_results = DataFrame()
df_results[:logl_mean] = Array(Float64, 0)
df_results[:logl_stdev] = Array(Float64, 0)
augment_array = Any[NaN,NaN]
for p in params
    df_results[p.sym] = Array(eltype(p.range), 0)
    push!(augment_array, zero(eltype(p.range)))
end

##############################
# COORDINATE ASCENT
##############################

metric_types_train_frames = DataType[]
metric_types_test_frames = [LoglikelihoodMetric]

behaviorset = BehaviorSet()
add_behavior!(behaviorset, behavior_type, behavior_name, behavior_train_params)

metric_arr = Array(Float64, cross_validation_split.nfolds)

iteration_count = 0
while true # do forever
    iteration_count += 1
    print("Iteration ", iteration_count)

    # sample new parameters until we get a fresh one
    print("\tsampling... ")
    tic()
    param_index = rand(1:nparams)
    prev_index = param_indeces[param_index]
    param_indeces[param_index] = rand(1:length(params[param_index].range))
    while in(hash(param_indeces), param_hash)
        param_indeces[param_index] = prev_index
        param_index = rand(1:nparams)
        param_indeces[param_index] = rand(1:length(params[param_index].range))
    end
    toc()

    push!(param_hash, hash(param_indeces))

    sym = params[param_index].sym
    val = params[param_index].range[param_indeces[param_index]]
    print("trying ", sym, " <- ", val)
    prev_param = behavior_train_params[sym]
    behavior_train_params[sym] = val

    tic()
    cv_res = cross_validate(behaviorset, dset, cross_validation_split,
                    metric_types_train_frames, metric_types_test_frames)
    Δt = toq()

    for (fold,cvfold_results) in enumerate(cv_res.models[1].results)
        m = cvfold_results.metrics_test_frames[1]::LoglikelihoodMetric
        metric_arr[fold] = m.logl
    end

    μ = mean(metric_arr)
    σ = stdm(metric_arr, μ)

    augment_array[1] = μ
    augment_array[2] = σ
    for (i,p) in enumerate(params)
        augment_array[i+2] = behavior_train_params[p.sym]
        param_indeces[param_index] = prev_index
    end

    if μ > best_param_score
        best_param_score = μ
    else # previous was better
        behavior_train_params[sym] = prev_param
        param_indeces[param_index] = prev_index
    end

    println("  ", μ, "  elapsed time: ", Δt, "  best score: ", best_param_score)

    push!(df_results, augment_array)
    writetable(BEST_MODEL_CSV_FILE, df_results)

    println("sleeping")
    sleep(1.0)
end