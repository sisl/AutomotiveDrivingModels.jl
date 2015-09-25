using AutomotiveDrivingModels
using DynamicBayesianNetworkBehaviors

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
        "/media/tim/DATAPART1/PublicationData/2015_TrafficEvolutionModels/vires_highway_2lane_sixcar/extract_params.jl"
    elseif hostname == "tula"
        "/home/wheelert/PublicationData/2015_TrafficEvolutionModels/vires_highway_2lane_sixcar/extract_params.jl"
    else
        error("unknown hostname")
    end
end
const INCLUDE_NAME = splitdir(splitext(INCLUDE_FILE)[1])[2]

include(INCLUDE_FILE)

const SAVE_FILE_MODIFIER = ""
const BEST_MODEL_CSV_FILE = joinpath(EVALUATION_DIR, "best_model_bn" * SAVE_FILE_MODIFIER * ".csv")
const TRAIN_VALIDATION_JLD_FILE = joinpath(EVALUATION_DIR, "train_validation_split" * SAVE_FILE_MODIFIER * ".jld")
const DATASET_JLD_FILE = joinpath(EVALUATION_DIR, "dataset" * SAVE_FILE_MODIFIER * ".jld")

################################
# LOAD TRAIN AND VALIDATION SETS
################################

pdsets, streetnets, pdset_segments, dataframe, startframes, extract_params_loaded =
            load_pdsets_streetnets_segements_and_dataframe(DATASET_JLD_FILE)

frame_tv_assignment, pdsetseg_tv_assignment, frame_cv_assignment, pdsetseg_cv_assignment = let
    jld = JLD.load(TRAIN_VALIDATION_JLD_FILE)
    frame_tv_assignment = jld["frame_tv_assignment"]
    pdsetseg_tv_assignment = jld["pdsetseg_tv_assignment"]
    frame_cv_assignment = jld["frame_cv_assignment"]
    pdsetseg_cv_assignment = jld["pdsetseg_cv_assignment"]
    (frame_tv_assignment, pdsetseg_tv_assignment, frame_cv_assignment, pdsetseg_cv_assignment)
end

# replace foldername
for (i,str) in enumerate(pdsets)
    pdsets[i] = joinpath(EVALUATION_DIR, "pdsets", splitdir(str)[2])
end
for (i,str) in enumerate(streetnets)
    streetnets[i] = joinpath(EVALUATION_DIR, "streetmaps", splitdir(str)[2])
end

##############################
# TRAIN
##############################

behavior_type = DynamicBayesianNetworkBehavior
behavior_train_params = Dict{Symbol,Any}()
behavior_train_params[:indicators] = INDICATOR_SET
behavior_train_params[:preoptimize_target_bins] = true
behavior_train_params[:optimize_structure] = true
behavior_train_params[:optimize_target_bins] = false
behavior_train_params[:optimize_parent_bins] = false

arg_index = 1
while arg_index < length(ARGS)
    if ARGS[arg_index] == "ncandidate_bins"
        behavior_train_params[:ncandidate_bins] = int(ARGS[arg_index+=1])
    elseif ARGS[arg_index] == "preoptimize_indicator_bins"
        behavior_train_params[:preoptimize_indicator_bins] = convert(Bool, int(ARGS[arg_index+=1]))
    else
        error("UNRECOGNIZED PARAMETER $(ARGS[arg_index])")
    end
    arg_index += 1
end

# obtain mean and stdev of logl over the folds
logl_mean, logl_stdev = cross_validate_logl(behavior_type, behavior_train_params,
                                            dataframe, frame_cv_assignment)

##############################
# LOAD PREVIOUSLY BEST MODEL
##############################

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

score = logl_mean
println(score)