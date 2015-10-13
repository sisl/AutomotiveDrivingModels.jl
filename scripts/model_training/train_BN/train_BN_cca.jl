using AutomotiveDrivingModels
using DynamicBayesianNetworkBehaviors

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
const BEST_MODEL_CSV_FILE = joinpath(EVALUATION_DIR, "best_model_bn" * SAVE_FILE_MODIFIER * ".csv")
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
    # Parameter(:forced, [ParentFeatures(),
    #                     ParentFeatures(AbstractFeature[D_CL], AbstractFeature[])
    #                    ]),
    # Parameter(:nbins_lat, 5:10),
    # Parameter(:nbins_lon, 5:10),
    Parameter(:max_parents, 6:-1:0),
    Parameter(:ncandidate_bins, 50:-5:5),
    Parameter(:preoptimize_target_bins, [true]),
    Parameter(:optimize_structure, [true]),
    Parameter(:optimize_target_bins, [false]),
    Parameter(:optimize_parent_bins, [false]),
]

nparams = length(params)

##############################
# SET UP PARAMS
##############################

srand(1)

behavior_type = DynamicBayesianNetworkBehavior
behavior_name = "Bayesian Network"
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
param_hash = Set{Uint}()

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