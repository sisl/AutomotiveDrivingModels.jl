using AutomotiveDrivingModels

using RandomForestBehaviors
using DynamicBayesianNetworkBehaviors

#=
1 - load the full datasets for each scenario used
2 - train a model using its optimal hyperparameter set on each dataset amount
3 - compute the validation log likelihood
4 - save results to a .csv file in results/
=#

##############################
# PARAMETERS
##############################

const SIM_HISTORY_IN_FRAMES  = 8
const SIMPARAMS = DEFAULT_SIM_PARAMS
const HISTOBIN_PARAMS = ParamsHistobin(
                    LinearDiscretizer(linspace(4.0 - 0.5, 4.0 + 0.5, 50), force_outliers_to_closest=true),
                    LinearDiscretizer(linspace(     -3.5,       3.5, 50), force_outliers_to_closest=true))

const SCENARIO_DATASETS = [
    # "/media/tim/DATAPART1/PublicationData/2015_TrafficEvolutionModels/vires_highway_1lane_freeflow/dataset.jld",
    "/media/tim/DATAPART1/PublicationData/2015_TrafficEvolutionModels/vires_highway_2lane_sixcar/dataset_small.jld",
]
const DATASET_VALIDATION = "/media/tim/DATAPART1/PublicationData/2015_TrafficEvolutionModels/vires_highway_2lane_sixcar/dataset_small.jld"
const OPTIMAL_MODEL_PARAMS = "/home/tim/.julia/v0.3/AutomotiveDrivingModels/scripts/model_training/opt_model_params.jld"
const DATASET_PERCENTAGES = linspace(0.1, 1.0, 3)

type Dataset
    pdset_filepaths::Vector{String}
    streetnet_filepaths::Vector{String}
    pdset_segments::Vector{PdsetSegment}
    dataframe::DataFrame
    startframes::Vector{Int}
    extract_params::OrigHistobinExtractParameters

    function Dataset(filename::String)
        pdset_filepaths, streetnet_filepaths, pdset_segments, dataframe, startframes, extract_params_loaded =
            load_pdsets_streetnets_segements_and_dataframe(filename)

        new(pdset_filepaths, streetnet_filepaths, pdset_segments, dataframe, startframes, extract_params_loaded)
    end
end

dsets = map(filepath->Dataset(filepath), SCENARIO_DATASETS)
dset_validation = Dataset(DATASET_VALIDATION)

behaviorset = JLD.load(OPTIMAL_MODEL_PARAMS, "behaviorset")
behaviorset.behaviors = [
                            VehicleBehaviorGaussian,
                            GindeleRandomForestBehavior,
                            DynamicForestBehavior,
                            DynamicBayesianNetworkBehavior
                        ]

df_train = dsets[1].dataframe
for i = 2 : length(dsets)
    append!(df_train, dsets[i])
end

##############################
# TRAIN MODELS
##############################

evalparams = EvaluationParams(SIM_HISTORY_IN_FRAMES, SIMPARAMS, HISTOBIN_PARAMS)

df = DataFrame(dataset_amount=DATASET_PERCENTAGES, nframes=Array(Int, length(DATASET_PERCENTAGES)))
for name in behaviorset.names
    df[symbol(name)] = Array(Float64, length(DATASET_PERCENTAGES))
end

pdsetseg_assignment = ones(Int, length(dset_validation.pdset_segments))
nframes_train = nrow(df_train)

pdsets = Array(PrimaryDataset, length(dset_validation.pdset_filepaths))
for (i,pdset_filepath) in enumerate(dset_validation.pdset_filepaths)
    pdsets[i] = load(pdset_filepath, "pdset")
end

streetnets = Array(StreetNetwork, length(dset_validation.streetnet_filepaths))
for (i,streetnet_filepath) in enumerate(dset_validation.streetnet_filepaths)
    streetnets[i] = load(streetnet_filepath, "streetmap")
end

pdsets_for_simulation = deepcopy(pdsets)

for i in 1 : nrow(df)

    println("i")

    dataset_percent = df[i, :dataset_amount]::Float64

    # cut it down
    nframes = int(nframes_train * dataset_percent)
    @assert(nframes ≤ nframes_train)

    println("nframes: ", nframes)

    models = train(behaviorset, df_train[1:nframes, :])
    metrics_sets = create_metrics_sets_no_tracemetrics(
                    models, pdsets, pdsets_for_simulation,
                    streetnets, dset_validation.pdset_segments,
                    evalparams, 1, pdsetseg_assignment, true)

    df[i, :nframes] = nframes
    for (metric_set, name) in zip(metrics_sets, behaviorset.names)
        df[i, symbol(name)] = metric_set.aggmetrics[:logl_mean]
    end

    println(df)
end

writetable("results/data_vs_performance_metrics.csv", df)