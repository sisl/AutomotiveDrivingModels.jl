using AutomotiveDrivingModels

##############################
# PARAMETERS
##############################

const INCLUDE_FILE = "/media/tim/DATAPART1/PublicationData/2015_TrafficEvolutionModels/realworld/extract_params.jl"

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

dataset_filepath = joinpath(EVALUATION_DIR, "dataset" * DATASET_MODIFIER * ".jld")
features = unique([INDICATOR_SET, TARGET_SET.lat, TARGET_SET.lon])

tic()
pdsets, streetnets, pdset_segments, dataframe, startframes =
    pull_pdsets_streetnets_segments_and_dataframe(EXTRACT_PARAMS, CSVFILESETS, pdset_dir=PDSET_DIR, features=features)
toc()

println("dataset extraction complete")

save_pdsets_streetnets_segements_and_dataframe(dataset_filepath, pdsets, streetnets,
                                               pdset_segments, dataframe,
                                               startframes, EXTRACT_PARAMS)

println("num pdset_segments: ", length(pdset_segments))
println("size of dataframe:  ", size(dataframe))

println("output saved to ", dataset_filepath)
println("[DONE]")

