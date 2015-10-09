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

features = unique([INDICATOR_SET, TARGET_SET.lat, TARGET_SET.lon])
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

