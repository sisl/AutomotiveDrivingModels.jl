using AutomotiveDrivingModels

using RandomForestBehaviors
using DynamicBayesianNetworkBehaviors

#=
1 - load dataset
2 - split dataset into train and validation
3 - save them so they can be used by the Spearmint package
=#

##############################
# PARAMETERS
##############################

const INCLUDE_FILE = let
    hostname = gethostname()
    if hostname == "Cupertino"
        "/media/tim/DATAPART1/PublicationData/2015_TrafficEvolutionModels/vires_highway_2lane_sixcar/extract_params.jl"
    elseif hostname == "tula"
        "/home/wheelert/PublicationData/2015_TrafficEvolutionModels/vires_highway_2lane_sixcar/extract_params.jl"
    else
        "unknown"
    end
end
const INCLUDE_NAME = splitdir(splitext(INCLUDE_FILE)[1])[2]

include(INCLUDE_FILE)

const SAVE_FILE_MODIFIER = ""
const TRAIN_VALIDATION_JLD_FILE = joinpath(EVALUATION_DIR, "train_validation_split" * SAVE_FILE_MODIFIER * ".jld")

println("results will be exported to:")
println("\t", TRAIN_VALIDATION_JLD_FILE)

##############################
# DATA
##############################

dataset_filepath = joinpath(EVALUATION_DIR, "dataset_small.jld")
# dataset_filepath = joinpath(EVALUATION_DIR, "dataset_medium.jld")
# dataset_filepath = joinpath(EVALUATION_DIR, "dataset.jld")
pdsets, streetnets, pdset_segments, dataframe, startframes, extract_params_loaded =
            load_pdsets_streetnets_segements_and_dataframe(dataset_filepath)

#################################
# SPLIT INTO TRAIN AND VALIDATION
#################################

const FRACTION_VALIDATION = 0.10

srand(1)

(frame_tv_assignment, pdsetseg_tv_assignment) = split_into_train_and_validation(FRACTION_VALIDATION, pdset_segments, dataframe, startframes)

println("n_other_frame: ", sum(v->v!=1 && v!=2, frame_tv_assignment))
println("n_train_frame: ", sum(v->v==1, frame_tv_assignment))
println("n_valid_frame: ", sum(v->v==2, frame_tv_assignment))
println("n_other_pdset: ", sum(v->v!=1 && v!=2, pdsetseg_tv_assignment))
println("n_train_pdset: ", sum(v->v==1, pdsetseg_tv_assignment))
println("n_valid_pdset: ", sum(v->v==2, pdsetseg_tv_assignment))

frame_cv_assignment, pdsetseg_cv_assignment = cross_validation_sets(NFOLDS, pdset_segments, dataframe, startframes,
                                                                    frame_tv_assignment, pdsetseg_tv_assignment)

counts = zeros(Int, 10)
counts2 = zeros(Int, 10)
for v in frame_cv_assignment
    if v != 0
        counts[v] += 1
    end
end
for v in pdsetseg_cv_assignment
    if v != 0
        counts2[v] += 1
    end
end

println("counts frame_cv_assignment: ", counts)
println("counts pdsetseg_cv_assignment: ", counts2)

JLD.save(TRAIN_VALIDATION_JLD_FILE, "frame_tv_assignment", frame_tv_assignment,
                                    "pdsetseg_tv_assignment", pdsetseg_tv_assignment,
                                    "frame_cv_assignment", frame_cv_assignment,
                                    "pdsetseg_cv_assignment", pdsetseg_cv_assignment)

println("DONE")

