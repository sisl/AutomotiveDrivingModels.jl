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

const STREETNET_CACHE = Dict{String, StreetNetwork}()
const STREETMAP_BASE = "/media/tim/DATAPART1/Data/Bosch/processed/streetmaps/"
const ERROR_AMOUNTS = linspace(0.0, 0.5, 3) # gaussian error
const PDSET_EXTRACT_PARAMS = PrimaryDataExtractionParams()
const DATA_INPUT_DIR = "/media/tim/DATAPART1/PublicationData/2015_TrafficEvolutionModels/realworld/"
const SCENARIO_DATASETS = [
    "/media/tim/DATAPART1/PublicationData/2015_TrafficEvolutionModels/realworld/dataset_subset_car_following.jld",
    "/media/tim/DATAPART1/PublicationData/2015_TrafficEvolutionModels/realworld/dataset_subset_free_flow.jld",
    "/media/tim/DATAPART1/PublicationData/2015_TrafficEvolutionModels/realworld/dataset_subset_lane_crossing.jld",
]

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

const INCLUDE_FILE_BASE = "realworld"
const AM_ON_TULA = gethostname() == "tula"
const INCLUDE_FILE = AM_ON_TULA ? joinpath("/home/wheelert/PublicationData/2015_TrafficEvolutionModels", INCLUDE_FILE_BASE, "extract_params.jl") :
                                  joinpath("/media/tim/DATAPART1/PublicationData/2015_TrafficEvolutionModels", INCLUDE_FILE_BASE, "extract_params.jl")
const INCLUDE_NAME = splitdir(splitext(INCLUDE_FILE)[1])[2]

include(INCLUDE_FILE)

##############################
# Test Dataset
##############################

# construct a mega dset
dset = JLD.load(SCENARIO_DATASETS[1], "model_training_data")
for i = 2 : length(SCENARIO_DATASETS)
    append!(dset, JLD.load(SCENARIO_DATASETS[i], "model_training_data"))
end

# construct a train/test split
srand(1)
TRAIN_TEST_SPLIT_TEST_FRACTION = 0.1
train_test_split = get_train_test_fold_assignment(TRAIN_TEST_SPLIT_TEST_FRACTION, dset)

##############################
# LOAD ALL RAW CSV DATA FILES
##############################

csvfilesets = JLD.load(joinpath(DATA_INPUT_DIR, "csvfilesets.jld"), "csvfilesets")

trajdatas = Array(DataFrame, length(csvfilesets))
for (i,csvfileset) in enumerate(csvfilesets)
    csvfile = csvfileset.csvfile
    streetmapbasename = csvfileset.streetmapbasename

    if !haskey(STREETNET_CACHE, streetmapbasename)
        STREETNET_CACHE[streetmapbasename] = load(joinpath(STREETMAP_BASE, "streetmap_" * streetmapbasename*".jld"))["streetmap"]
    end
    sn = STREETNET_CACHE[streetmapbasename]

    trajdatas[i] = load_trajdata(csvfile)
end

trajdatas_for_sim = deepcopy(trajdatas)

##############################
# BEHAVIORSET
##############################

behaviorset = BehaviorSet()
add_behavior!(behaviorset, VehicleBehaviorGaussian, "Gaussian Filter")
add_behavior!(behaviorset, VehicleBehaviorLinearGaussian, "Single Variable",
    [:indicators=>INDICATOR_SET,
     :ridge_regression_constant=>0.75,
    ])
add_behavior!(behaviorset, GindeleRandomForestBehavior, "Random Forest",
    [:indicators=>INDICATOR_SET,
     :ntrees=>6,
     :max_depth=>18,
     :min_samples_split=>30,
     :min_samples_leaves=>2,
     :min_split_improvement=>0.0,
     :partial_sampling=>1.0,
    ])
add_behavior!(behaviorset, DynamicForestBehavior, "Dynamic Forest",
    [:indicators=>INDICATOR_SET,
     :ntrees=>36,
     :max_depth=>10,
     :min_samples_split=>20,
     :min_samples_leaves=>2,
     :min_split_improvement=>5.0,
     :partial_sampling=>1.0,
    ])
add_behavior!(behaviorset, DynamicBayesianNetworkBehavior, "Bayesian Network",
    [:indicators=>INDICATOR_SET,
     :preoptimize_target_bins=>true,
     :preoptimize_parent_bins=>true,
     :optimize_structure=>true,
     :optimize_target_bins=>false,
     :optimize_parent_bins=>false,
     :ncandidate_bins=>20,
     ])

##############################
# METRICS
##############################

metric_types_test_frames = [LoglikelihoodMetric]

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

df_results = DataFrame(gaussian_error_in_meters=ERROR_AMOUNTS)
for name in behaviorset.names
    df_results[symbol(name)] = Array(Float64, nrow(df_results))
end

for i in 1 : nrow(df_results)

    gaussian_error = df_results[i, :gaussian_error_in_meters]::Float64
    println("$i) Gaussian Error: ", gaussian_error, " [m]")

    df_train = create_dataframe_with_feature_columns(features, 0)
    for j in 1 : length(trajdatas_for_sim)

        trajdata = trajdatas[j]
        trajdata_for_sim = trajdatas_for_sim[j]

        # add error to the traces of every vehicle
        for frameind in nframeinds(trajdata)
            for carind in CARIND_EGO : get_num_other_cars_in_frame(trajdata, frameind)-1
                add_gaussian_error_to_sample!(trajdata_for_sim, trajdata, gaussian_error,
                                              carind, frameind)
            end
        end

        # extract the dataset
        sn = STREETNET_CACHE[csvfilesets[j].streetmapbasename]
        pdset = gen_primary_data_no_smoothing(trajdata_for_sim, sn, PDSET_EXTRACT_PARAMS)
        basics = FeatureExtractBasicsPdSet(pdset, sn)
        for carid in get_carids(pdset)
            append!(df_train, gen_featureset_from_validfinds(carid, basics, get_validfinds_containing_carid(Vector{Int}, pdset, carid), features))
        end
    end

    println("done extraction")

    # train the models on the training set
    models = train(behaviorset, df_train)

    println("done training")

    # validate the models on the validation set
    metrics_sets_test_frames = extract_metrics(metric_types_test_frames, models,
                                               dset, train_test_split, FOLD_TEST, true)

    println("done metric extraction")

    for (metric_set, name) in zip(metrics_sets_test_frames, behaviorset.names)
        df_results[i, symbol(name)] = (metric_set[1]::LoglikelihoodMetric).logl
    end

    println(df_results)
end

writetable("results/error_vs_performance_metrics.csv", df)