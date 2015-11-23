using AutomotiveDrivingModels

using RandomForestBehaviors
using DynamicBayesianNetworkBehaviors

#=
1 - load the raw traces used for each scenario
2 - add gaussian measurement error to each position measurement
3 - extract the dataset from the shifted traces
    TODO(tim): need to run it through smoothing!
4 - train a model using its optimal hyperparameter set on each dataset amount
5 - compute the validation log likelihood
6 - save results to a .csv file in results/
=#



##############################
# PARAMETERS
##############################

const SIM_HISTORY_IN_FRAMES  = 8
const SIMPARAMS = DEFAULT_SIM_PARAMS
const HISTOBIN_PARAMS = ParamsHistobin(
                    LinearDiscretizer(linspace(4.0 - 0.5, 4.0 + 0.5, 50), force_outliers_to_closest=true),
                    LinearDiscretizer(linspace(     -3.5,       3.5, 50), force_outliers_to_closest=true))
const EVAL_PARAMS = EvaluationParams(SIM_HISTORY_IN_FRAMES, SIMPARAMS, HISTOBIN_PARAMS)
const STREETNET_CACHE = Dict{String, StreetNetwork}()
const STREETMAP_BASE = "/media/tim/DATAPART1/Data/Bosch/processed/streetmaps/"

const DATASET_VALIDATION = "/media/tim/DATAPART1/PublicationData/2015_TrafficEvolutionModels/vires_highway_2lane_sixcar/dataset_small.jld"
const OPTIMAL_MODEL_PARAMS = "/home/tim/.julia/v0.3/AutomotiveDrivingModels/scripts/model_training/opt_model_params.jld"
const ERROR_AMOUNTS = linspace(0.0, 1.0, 5) # gaussian error
const PDSET_EXTRACT_PARAMS = PrimaryDataExtractionParams()

const features = [
    YAW, SPEED, VELFX, VELFY, DELTA_SPEED_LIMIT, TURNRATE, TURNRATE_GLOBAL, ACC, ACCFX, ACCFY,
    D_CL, D_ML, D_MR, SCENEVELFX,
    TIMETOCROSSING_RIGHT, TIMETOCROSSING_LEFT, ESTIMATEDTIMETOLANECROSSING,
    D_MERGE, D_SPLIT,
    A_REQ_STAYINLANE, N_LANE_R, N_LANE_L, HAS_LANE_R, HAS_LANE_L, LANECURVATURE,
    HAS_FRONT, D_X_FRONT, D_Y_FRONT, V_X_FRONT, V_Y_FRONT, YAW_FRONT, TURNRATE_FRONT, A_REQ_FRONT, TTC_X_FRONT, TIMEGAP_X_FRONT,
    HAS_REAR,  D_X_REAR,  D_Y_REAR,  V_X_REAR,  V_Y_REAR,  YAW_REAR,  TURNRATE_REAR,  A_REQ_REAR,  TTC_X_REAR,  TIMEGAP_X_REAR,
               D_X_LEFT,  D_Y_LEFT,  V_X_LEFT,  V_Y_LEFT,  YAW_LEFT,  TURNRATE_LEFT,  A_REQ_LEFT,  TTC_X_LEFT,  TIMEGAP_X_LEFT,
               D_X_RIGHT, D_Y_RIGHT, V_X_RIGHT, V_Y_RIGHT, YAW_RIGHT, TURNRATE_RIGHT, A_REQ_RIGHT, TTC_X_RIGHT, TIMEGAP_X_RIGHT,
    GAINING_ON_FRONT, TIMESINCELANECROSSING, TIME_CONSECUTIVE_BRAKE, TIME_CONSECUTIVE_ACCEL, TIME_CONSECUTIVE_THROTTLE,
    FUTUREACCELERATION_250MS, FUTUREDESIREDANGLE_250MS
]

type Dataset
    pdset_filepaths::Vector{String}
    streetnet_filepaths::Vector{String}
    pdset_segments::Vector{PdsetSegment}
    dataframe::DataFrame
    startframes::Vector{Int}
    extract_params::OrigHistobinExtractParameters

    function Dataset(filename::AbstractString)
        pdset_filepaths, streetnet_filepaths, pdset_segments, dataframe, startframes, extract_params_loaded =
            load_pdsets_streetnets_segements_and_dataframe(filename)

        new(pdset_filepaths, streetnet_filepaths, pdset_segments, dataframe, startframes, extract_params_loaded)
    end
end
type RunAndMap
    csvfilename::AbstractString
    streetmapbasename::AbstractString
end

##############################
# VALIDATION DATASET
##############################

dset_validation = Dataset(DATASET_VALIDATION)

pdsetseg_assignment = ones(Int, length(dset_validation.pdset_segments))

pdsets = Array(PrimaryDataset, length(dset_validation.pdset_filepaths))
for (i,pdset_filepath) in enumerate(dset_validation.pdset_filepaths)
    pdsets[i] = load(pdset_filepath, "pdset")
end

streetnets = Array(StreetNetwork, length(dset_validation.streetnet_filepaths))
for (i,streetnet_filepath) in enumerate(dset_validation.streetnet_filepaths)
    streetnets[i] = load(streetnet_filepath, "streetmap")
end

pdsets_for_simulation = deepcopy(pdsets)

##############################
# BEHAVIORSET
##############################

behaviorset = JLD.load(OPTIMAL_MODEL_PARAMS, "behaviorset")
behaviorset.behaviors = [
                            VehicleBehaviorGaussian,
                            GindeleRandomForestBehavior,
                            DynamicForestBehavior,
                            DynamicBayesianNetworkBehavior
                        ]

##############################
# LOAD ALL RAW CSV DATA FILES
##############################

runandmaps = RunAndMap[]
for (input_dir, streetmap_basename) in [
    # ("/media/tim/DATAPART1/PublicationData/2015_TrafficEvolutionModels/vires_highway_1lane_freeflow/csv_bosch/", "highway_1lane"),
      ("/media/tim/DATAPART1/PublicationData/2015_TrafficEvolutionModels/vires_highway_2lane_sixcar/csv_bosch/", "highway_2lane"),
    ]

    for content in readdir(input_dir)
        csvfilename = joinpath(input_dir, content)
        if isfile(csvfilename) && endswith(csvfilename, ".csv")
            push!(runandmaps, RunAndMap(csvfilename, streetmap_basename))
        end
    end
end

trajdatas = Array(DataFrame, length(runandmaps))
for (i,runandmap) in enumerate(runandmaps)
    csvfilename = runandmap.csvfilename
    streetmapbasename = runandmap.streetmapbasename

    if !haskey(STREETNET_CACHE, streetmapbasename)
        STREETNET_CACHE[streetmapbasename] = load(joinpath(STREETMAP_BASE, "streetmap_" * streetmapbasename*".jld"))["streetmap"]
    end
    sn = STREETNET_CACHE[streetmapbasename]

    trajdatas[i] = load_trajdata(csvfilename)
end

trajdatas_for_sim = deepcopy(trajdatas)

##############################
# TRAIN MODELS
##############################

function add_gaussian_error_to_sample!(dest::DataFrame, source::DataFrame, σ::Float64, carind::Integer, frameind::Integer)

    #=
    Adds gaussian error to a single coordinate in trajdata.
    The error is sampled independently for each coordinate.
    Error magnitude is controlled by σ across all coordiantes
    but is scaled appropriately.
    =#

    ΔposGx = σ*rand()
    ΔposGy = σ*rand()
    ΔyawG  = σ*rand()*0.01
    ΔvelEx = σ*rand()

    if carind == CARIND_EGO
        dest[frameind, :posGx] = source[frameind, :posGx] + ΔposGx
        dest[frameind, :posGy] = source[frameind, :posGy] + ΔposGy
        dest[frameind, :yawG]  = source[frameind, :yawG]  + ΔyawG
        dest[frameind, :velEx] = source[frameind, :velEx] + ΔvelEx
    else
        setc!(dest, "posEx", carind, frameind, getc(source, "posEx", carind, frameind) + ΔposGx)
        setc!(dest, "posEy", carind, frameind, getc(source, "posEy", carind, frameind) + ΔposGy)

        velEx = getc(source, "velEx", carind, frameind) # NOTE(tim): velocity in the ego frame but pre-compensated for ego velocity
        velEy = getc(source, "velEy", carind, frameind)

        yawG_ego  = source[frameind, :yawG]
        velGx, velGy = Trajdata.ego2global(0.0, 0.0, yawG_ego, velEx, velEy)
        speed = hypot(velGx, velGy)
        if hypot(velGx, velGy) > 3.0
            yawG = atan2(velGy, velGx)
        else
            yawG = yawG_ego # to fix problem with very low velocities
        end

        yawG += ΔyawG
        speed += ΔvelEx

        velGx = speed * cos(yawG)
        velGy = speed * sin(yawG)

        velEx, velEy = Trajdata.global2ego(0.0, 0.0, yawG_ego, velGx, velGy)

        setc!(dest, "velEx",  carind, frameind, velEx)
        setc!(dest, "velEy", carind, frameind, velEy)
    end

    dest
end

df = DataFrame(gaussian_error_in_meters=ERROR_AMOUNTS)
for name in behaviorset.names
    df[symbol(name)] = Array(Float64, nrow(df))
end

tic()
for i in 1 : nrow(df)

    gaussian_error = df[i, :gaussian_error_in_meters]::Float64

    println("$i) Gaussian Error: ", gaussian_error, " [m]")

    df_train = create_dataframe_with_feature_columns(features, 0)
    for j in 1 : length(trajdatas_for_sim)

        trajdata = trajdatas[j]
        trajdata_for_sim = trajdatas_for_sim[j]

        # add error to the traces of every vehicle
        if mod(j, 2) == 1
            for frameind in nframeinds(trajdata)
                for carind in CARIND_EGO : get_num_other_cars_in_frame(trajdata, frameind)-1
                    add_gaussian_error_to_sample!(trajdata_for_sim, trajdata, gaussian_error,
                                                  carind, frameind)
                end
            end
        end

        # extract the dataset
        sn = STREETNET_CACHE[runandmaps[j].streetmapbasename]
        pdset = gen_primary_data_no_smoothing(trajdata_for_sim, sn, PDSET_EXTRACT_PARAMS)
        for carid in get_carids(pdset)
            append!(df_train, gen_featureset(carid, pdset, [1,nvalidfinds(pdset)], sn, features))
        end
    end

    # train the models on the training set
    models = train(behaviorset, df_train)

    # validate the models on the validation set
    metrics_sets = create_metrics_sets_no_tracemetrics(
                    models, pdsets, pdsets_for_simulation,
                    streetnets, dset_validation.pdset_segments,
                    EVAL_PARAMS, 1, pdsetseg_assignment, true)

    for (metric_set, name) in zip(metrics_sets, behaviorset.names)
        df[i, symbol(name)] = metric_set.aggmetrics[:logl_mean]
    end

    println(df)
end
toc()

writetable("results/half_error_vs_performance_metrics.csv", df)