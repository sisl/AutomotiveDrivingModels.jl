using AutomotiveDrivingModels
using RandomForestBehaviors

# mongod --fork --logpath /home/tim/Documents/code/Spearmint/db/log.txt --dbpath /home/tim/Documents/code/Spearmint/db
# cd ~/Documents/code/Spearmint/spearmint
# python main.py /home/tim/.julia/v0.3/AutomotiveDrivingModels/scripts/model_training/train_RF/

#=
1 - receive parameters from Spearmint
2 - load train and validation datasets
3 - train using cross validation, maximizing logl, on training set
4 - compute validation metrics on the validation set
5 - load the previously best model (stored as a .csv)
6 - if we have a better model, override it
7 - return our logl metric score
=#

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
const BEST_MODEL_CSV_FILE = joinpath(EVALUATION_DIR, "best_model_rf" * SAVE_FILE_MODIFIER * ".csv")
const TRAIN_VALIDATION_JLD_FILE = joinpath(EVALUATION_DIR, "train_validation_split" * SAVE_FILE_MODIFIER * ".jld")
const DATASET_JLD_FILE = joinpath(EVALUATION_DIR, "dataset" * SAVE_FILE_MODIFIER * ".jld")

################################
# LOAD TRAIN AND VALIDATION SETS
################################

dset = JLD.load(DATASET_JLD_FILE, "model_training_data")
cross_validation_split = JLD.load(TRAIN_VALIDATION_JLD_FILE, "cross_validation_split")

##############################
# TRAIN
##############################

behavior_type = GindeleRandomForestBehavior
behavior_name = "Random Forest"
behavior_train_params = Dict{Symbol,Any}()
behavior_train_params[:indicators] = INDICATOR_SET

arg_index = 1
while arg_index < length(ARGS)
    if ARGS[arg_index] == "ntrees"
        behavior_train_params[:ntrees] = int(ARGS[arg_index+=1])
    elseif ARGS[arg_index] == "max_depth"
        behavior_train_params[:max_depth] = int(ARGS[arg_index+=1])
    elseif ARGS[arg_index] == "min_samples_split"
        behavior_train_params[:min_samples_split] = int(ARGS[arg_index+=1])
    elseif ARGS[arg_index] == "min_samples_leaves"
        behavior_train_params[:min_samples_leaves] = int(ARGS[arg_index+=1])
    elseif ARGS[arg_index] == "min_split_improvement"
        behavior_train_params[:min_split_improvement] = float(ARGS[arg_index+=1])
    elseif ARGS[arg_index] == "partial_sampling"
        behavior_train_params[:partial_sampling] = float(ARGS[arg_index+=1])
    else
        error("UNRECOGNIZED PARAMETER $(ARGS[arg_index])")
    end
    arg_index += 1
end

# obtain mean and stdev of logl over the folds
behaviorset = BehaviorSet()
add_behavior!(behaviorset, behavior_type, behavior_name, behavior_train_params)

metric_types_train_frames = DataType[]
metric_types_test_frames = [LoglikelihoodMetric]

cv_res = cross_validate(behaviorset, dset, cross_validation_split,
                        metric_types_train_frames, metric_types_test_frames)

# ##############################
# # LOAD PREVIOUSLY BEST MODEL
# ##############################

# prev_best = readtable(BEST_MODEL_CSV_FILE)
# if prev_best[1,:logl_mean] < logl_mean
#     prev_best[1, :logl_mean] = logl_mean
#     prev_best[1, :logl_stdev] = logl_stdev
#     prev_best[1, :ntrees] = ntrees
#     writetable(BEST_MODEL_CSV_FILE, prev_best)
# end

##############################
# RETURN SCORE
##############################

score = get_score(cv_res.models[1].results[1].metrics_test_frames[1])
println(score)