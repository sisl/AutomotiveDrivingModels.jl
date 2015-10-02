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

# const INCLUDE_FILE_BASE = "vires_highway_2lane_sixcar"
const INCLUDE_FILE_BASE = "realworld"

const AM_ON_TULA = gethostname() == "tula"
const INCLUDE_FILE = AM_ON_TULA ? joinpath("/home/wheelert/PublicationData/2015_TrafficEvolutionModels", INCLUDE_FILE_BASE, "extract_params.jl") :
                                  joinpath("/media/tim/DATAPART1/PublicationData/2015_TrafficEvolutionModels", INCLUDE_FILE_BASE, "extract_params.jl")
const INCLUDE_NAME = splitdir(splitext(INCLUDE_FILE)[1])[2]

include(INCLUDE_FILE)

const SAVE_FILE_MODIFIER = ""
const TRAIN_VALIDATION_JLD_FILE = joinpath(EVALUATION_DIR, "train_validation_split" * SAVE_FILE_MODIFIER * ".jld")

println("results will be exported to:")
println("\t", TRAIN_VALIDATION_JLD_FILE)

##############################
# DATA
##############################

println(joinpath(EVALUATION_DIR, "dataset.jld"))

# dataset_filepath = joinpath(EVALUATION_DIR, "dataset_small.jld")
# dataset_filepath = joinpath(EVALUATION_DIR, "dataset_medium.jld")
dataset_filepath = joinpath(EVALUATION_DIR, "dataset.jld")

dset = JLD.load(dataset_filepath, "model_training_data")

#################################
# SPLIT INTO TRAIN AND VALIDATION
#################################

srand(1) # <- initialize random number generator to enforce consistency

println("size(dataframe): ", size(dset.dataframe))

train_test_split = get_train_test_fold_assignment(TRAIN_TEST_SPLIT_TEST_FRACTION, dset)

println("n_other_frame: ", sum(v->v!=1 && v!=2, train_test_split.frame_assignment))
println("n_train_frame: ", sum(v->v==1, train_test_split.frame_assignment))
println("n_valid_frame: ", sum(v->v==2, train_test_split.frame_assignment))
println("n_other_pdset: ", sum(v->v!=1 && v!=2, train_test_split.pdsetseg_assignment))
println("n_train_pdset: ", sum(v->v==1, train_test_split.pdsetseg_assignment))
println("n_valid_pdset: ", sum(v->v==2, train_test_split.pdsetseg_assignment))

cross_validation_split = get_cross_validation_fold_assignment(NFOLDS, dset, train_test_split)

counts = zeros(Int, 10)
counts2 = zeros(Int, 10)
for v in cross_validation_split.frame_assignment
    if v != 0
        counts[v] += 1
    end
end
for v in cross_validation_split.pdsetseg_assignment
    if v != 0
        counts2[v] += 1
    end
end

println("counts frame_cv_assignment: ", counts)
println("counts pdsetseg_cv_assignment: ", counts2)

JLD.save(TRAIN_VALIDATION_JLD_FILE, "train_test_split", train_test_split,
                                    "cross_validation_split", cross_validation_split)

println("DONE")

