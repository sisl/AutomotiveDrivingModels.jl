using AutomotiveDrivingModels

using RandomForestBehaviors
using DynamicBayesianNetworkBehaviors

##############################
# PARAMETERS
##############################

const STREETNET_CACHE = Dict{String, StreetNetwork}()
const MAX_CV_OPT_TIME_PER_MODEL = 10.0 # [s]
const NFOLDS = 5
const ERROR_AMOUNTS = linspace(0.0, 0.5, 3) # gaussian error
const METRIC_TYPES_TEST_FRAMES = [LoglikelihoodMetric]
const PDSET_EXTRACT_PARAMS = PrimaryDataExtractionParams()
const STREETMAP_BASE = "/media/tim/DATAPART1/Data/Bosch/processed/streetmaps/"
const SCENARIO_DATASETS = [
    "_subset_car_following",
    "_subset_free_flow",
    "_subset_lane_crossing",
]
const EVALUATION_DIR = "/media/tim/DATAPART1/PublicationData/2015_TrafficEvolutionModels/realworld/"
const INDICATOR_SET = [
                    YAW, SPEED, VELFX, VELFY, #DELTA_SPEED_LIMIT,
                    D_CL, D_ML, D_MR, #D_MERGE, D_SPLIT,
                    TIMETOCROSSING_RIGHT, TIMETOCROSSING_LEFT, TIMESINCELANECROSSING, ESTIMATEDTIMETOLANECROSSING,
                    N_LANE_L, N_LANE_R, HAS_LANE_L, HAS_LANE_R,
                    TURNRATE, ACC, ACCFX, ACCFY, A_REQ_STAYINLANE,
                    TURNRATE_GLOBAL, LANECURVATURE,

                    HAS_FRONT, D_X_FRONT, D_Y_FRONT, V_X_FRONT, V_Y_FRONT, YAW_FRONT, TURNRATE_FRONT,
                    # HAS_REAR,  D_X_REAR,  D_Y_REAR,  V_X_REAR,  V_Y_REAR,  YAW_REAR,  TURNRATE_REAR,
                               D_X_LEFT,  D_Y_LEFT,  V_X_LEFT,  V_Y_LEFT,  YAW_LEFT,  TURNRATE_LEFT,
                               D_X_RIGHT, D_Y_RIGHT, V_X_RIGHT, V_Y_RIGHT, YAW_RIGHT, TURNRATE_RIGHT,
                    A_REQ_FRONT, TTC_X_FRONT, TIMEGAP_X_FRONT,
                    # A_REQ_REAR,  TTC_X_REAR,  TIMEGAP_X_REAR,
                    A_REQ_LEFT,  TTC_X_LEFT,  TIMEGAP_X_LEFT,
                    A_REQ_RIGHT, TTC_X_RIGHT, TIMEGAP_X_RIGHT,

                    SCENEVELFX,

                    TIME_CONSECUTIVE_THROTTLE, # TIME_CONSECUTIVE_BRAKE, TIME_CONSECUTIVE_ACCEL,
                    #      PASTACC250MS,      PASTACC500MS,      PASTACC750MS,      PASTACC1S,
                    # PASTTURNRATE250MS, PASTTURNRATE500MS, PASTTURNRATE750MS, PASTTURNRATE1S,
                    #    PASTVELFY250MS,    PASTVELFY500MS,    PASTVELFY750MS,    PASTVELFY1S,
                        # PASTD_CL250MS,     PASTD_CL500MS,     PASTD_CL750MS,     PASTD_CL1S,

                     #     MAXACCFX500MS,     MAXACCFX750MS,     MAXACCFX1S,     MAXACCFX1500MS,     MAXACCFX2S,     MAXACCFX2500MS,     MAXACCFX3S,     MAXACCFX4S,
                     #     MAXACCFY500MS,     MAXACCFY750MS,     MAXACCFY1S,     MAXACCFY1500MS,     MAXACCFY2S,     MAXACCFY2500MS,     MAXACCFY3S,     MAXACCFY4S,
                     #  MAXTURNRATE500MS,  MAXTURNRATE750MS,  MAXTURNRATE1S,  MAXTURNRATE1500MS,  MAXTURNRATE2S,  MAXTURNRATE2500MS,  MAXTURNRATE3S,  MAXTURNRATE4S,
                     #    MEANACCFX500MS,    MEANACCFX750MS,    MEANACCFX1S,    MEANACCFX1500MS,    MEANACCFX2S,    MEANACCFX2500MS,    MEANACCFX3S,    MEANACCFX4S,
                     #    MEANACCFY500MS,    MEANACCFY750MS,    MEANACCFY1S,    MEANACCFY1500MS,    MEANACCFY2S,    MEANACCFY2500MS,    MEANACCFY3S,    MEANACCFY4S,
                     # MEANTURNRATE500MS, MEANTURNRATE750MS, MEANTURNRATE1S, MEANTURNRATE1500MS, MEANTURNRATE2S, MEANTURNRATE2500MS, MEANTURNRATE3S, MEANTURNRATE4S,
                     #     STDACCFX500MS,     STDACCFX750MS,     STDACCFX1S,     STDACCFX1500MS,     STDACCFX2S,     STDACCFX2500MS,     STDACCFX3S,     STDACCFX4S,
                     #     STDACCFY500MS,     STDACCFY750MS,     STDACCFY1S,     STDACCFY1500MS,     STDACCFY2S,     STDACCFY2500MS,     STDACCFY3S,     STDACCFY4S,
                     #  STDTURNRATE500MS,  STDTURNRATE750MS,  STDTURNRATE1S,  STDTURNRATE1500MS,  STDTURNRATE2S,  STDTURNRATE2500MS,  STDTURNRATE3S,  STDTURNRATE4S,
                     ]
const FEATURES = [INDICATOR_SET, FUTUREACCELERATION_250MS, FUTUREDESIREDANGLE_250MS]

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

df_results = DataFrame()
df_results[:gaussian_error] = Float64[]
df_results[:context_class] = String[]
df_results[:logl_train] = Float64[]
df_results[:logl_test] = Float64[]
df_results[:model_name] = String[]

behaviorset = BehaviorSet()
model_param_sets = Dict{String, BehaviorParameterSet}()
add_behavior!(behaviorset, VehicleBehaviorGaussian, "Static Gaussian")
model_param_sets["Static Gaussian"] = BehaviorParameterSet()
# add_behavior!(behaviorset, VehicleBehaviorLinearGaussian, "Linear Gaussian")
# model_param_sets["Linear Gaussian"] = BehaviorParameterSet(
#     convert(Vector{(Symbol,Any)}, [(:indicators,INDICATOR_SET)]),
#     [BehaviorParameter(:ridge_regression_constant, linspace(0.0,1.0,20), 5)]
#     )
# add_behavior!(behaviorset, GindeleRandomForestBehavior, "Random Forest")
# model_param_sets["Random Forest"] = BehaviorParameterSet(
#     convert(Vector{(Symbol,Any)}, [(:indicators,INDICATOR_SET)]),
#     [BehaviorParameter(:ntrees, 1:5:51, 3),
#      BehaviorParameter(:max_depth, 1:20, 5),
#      BehaviorParameter(:min_samples_split, 10:10:50, 3),
#      BehaviorParameter(:min_samples_leaves, [2,4,10,20,50], 3),
#      BehaviorParameter(:min_split_improvement, [10.0, 5.0, 1.0,0.5,0.1,0.0], 3),
#      BehaviorParameter(:partial_sampling, [0.5,0.6,0.7,0.8,0.9,0.95,1.0], 5),
#      BehaviorParameter(:n_split_tries, [10,25,50,100,200,500,1000], 5),]
#     )
# add_behavior!(behaviorset, DynamicForestBehavior, "Dynamic Forest")
# model_param_sets["Dynamic Forest"] = BehaviorParameterSet(
#     convert(Vector{(Symbol,Any)}, [(:indicators,INDICATOR_SET)]),
#     [BehaviorParameter(:ntrees, 1:5:51, 3),
#      BehaviorParameter(:max_depth, 1:20, 5),
#      BehaviorParameter(:min_samples_split, 10:10:50, 3),
#      BehaviorParameter(:min_samples_leaves, [2,4,10,20,50], 3),
#      BehaviorParameter(:min_split_improvement, [10.0, 5.0, 1.0,0.5,0.1,0.0], 3),
#      BehaviorParameter(:partial_sampling, [0.5,0.6,0.7,0.8,0.9,0.95,1.0], 5),
#      BehaviorParameter(:n_split_tries, [10,25,50,100,200,500,1000], 5),]
#     )
# add_behavior!(behaviorset, GMRBehavior, "Gaussian Mixture Regression")
# model_param_sets["Gaussian Mixture Regression"] = BehaviorParameterSet(
#     convert(Vector{(Symbol,Any)}, [(:indicators,[YAW, SPEED, VELFX, VELFY, TURNRATE, ACC, ACCFX, ACCFY, A_REQ_STAYINLANE, TIME_CONSECUTIVE_THROTTLE])]),
#     [BehaviorParameter(:n_components, 2:10, 3),
#      BehaviorParameter(:max_n_indicators, 2:8, 1),
#      #BehaviorParameter(:Σ_type, [:full, :diag], 1),
#      ]
#     )
# add_behavior!(behaviorset, DynamicBayesianNetworkBehavior, "Bayesian Network")
# model_param_sets["Bayesian Network"] = BehaviorParameterSet(
#     convert(Vector{(Symbol,Any)}, [(:indicators,INDICATOR_SET),
#                                    (:preoptimize_target_bins,true),
#                                    (:preoptimize_parent_bins,true),
#                                    (:optimize_structure,true),
#                                    (:optimize_target_bins,false),
#                                    (:optimize_parent_bins,false),
#         ]),
#     [BehaviorParameter(:ncandidate_bins, 1:5:51, 7),
#      BehaviorParameter(:max_parents, 1:20, 5)],
#     )

for dset_filepath_modifier in SCENARIO_DATASETS
    context_class = dset_filepath_modifier
    println("context_class: ", context_class)

    METRICS_OUTPUT_FILE = joinpath(EVALUATION_DIR, "validation_results" * dset_filepath_modifier * ".jld")
    MODEL_OUTPUT_JLD_FILE = joinpath(EVALUATION_DIR, "validation_models" * dset_filepath_modifier * ".jld")
    TRAIN_VALIDATION_JLD_FILE = joinpath(EVALUATION_DIR, "train_validation_split" * dset_filepath_modifier * ".jld")
    DATASET_JLD_FILE = joinpath(EVALUATION_DIR, "dataset" * dset_filepath_modifier * ".jld")
    CSVFILESET_DIR = joinpath(EVALUATION_DIR, "csvfilesets.jld")

    dset = JLD.load(DATASET_JLD_FILE, "model_training_data")
    dset_train = deepcopy(dset)
    train_test_split = JLD.load(TRAIN_VALIDATION_JLD_FILE, "train_test_split")
    cross_validation_split = JLD.load(TRAIN_VALIDATION_JLD_FILE, "cross_validation_split")

    ##############################
    # LOAD ALL RAW CSV DATA FILES
    ##############################

    csvfilesets = JLD.load(CSVFILESET_DIR, "csvfilesets")[1:2] # TODO(tim): remove this

    println("extracting trajdatas "); tic()
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
    toc()

    ##############################
    # TRAIN MODELS
    ##############################

    carids = nothing

    for ϵ in ERROR_AMOUNTS

        println("\tϵ = ", ϵ)

        println("\tadding error"); tic()

        df_train = create_dataframe_with_feature_columns(FEATURES, 0)
        df_train_nona = deepcopy(df_train)
        for j in 1 : length(trajdatas_for_sim)

            trajdata = trajdatas[j]
            trajdata_for_sim = trajdatas_for_sim[j]

            # add error to the traces of every vehicle
            for frameind in nframeinds(trajdata)
                for carind in CARIND_EGO : get_num_other_cars_in_frame(trajdata, frameind)-1
                    add_gaussian_error_to_sample!(trajdata_for_sim, trajdata, ϵ,
                                                  carind, frameind)
                end
            end

            # extract the dataset
            sn = STREETNET_CACHE[csvfilesets[j].streetmapbasename]

            print("\t\tgen primary data no smoothing  "); tic()
            pdset = gen_primary_data_no_smoothing(trajdata_for_sim, sn, PDSET_EXTRACT_PARAMS, carids=carids)
            toc()

            print("\t\tgen featureset from validfinds "); tic()
            basics = FeatureExtractBasicsPdSet(pdset, sn)
            carids = get_carids(pdset)
            for (k, carid) in enumerate(carids)
                print(k, "  /  ", length(carids)); tic()
                validfinds = get_validfinds_containing_carid(Vector{Int}, pdset, carid)
                dfset = gen_featureset_from_validfinds(carid, basics, validfinds, FEATURES)
                append!(df_train, dfset)

                df_nona = deepcopy(dfset)
                for sym in names(df_nona)
                    if is_symbol_a_feature(sym)
                        F = symbol2feature(sym)
                        for (i,v) in enumerate(df_nona[sym])
                            @assert(!isnan(v))
                            if isinf(v)
                                validfind = validfinds[i]
                                carind = carid2ind(pdset, carid, validfind)
                                df_nona[i,sym] = replace_na(F, basics, carind, validfind)
                            end
                        end
                    end
                end
                append!(df_train_nona, dfset)
                toc()
            end
            toc()
        end

        toc()

        # train the models on the training set
        dset_train.dataframe = df_train
        dset_train.dataframe_nona = df_train_nona

        println("\ttraining "); tic()
        models = train(behaviorset, dset_train, train_test_split, cross_validation_split, model_param_sets, max_cv_opt_time_per_model=MAX_CV_OPT_TIME_PER_MODEL)
        toc()

        # validate the models on the validation set
        println("\textracting metrics "); tic()
        metrics_sets_test_frames = extract_metrics(METRIC_TYPES_TEST_FRAMES, models,
                                                   dset, train_test_split, FOLD_TEST, true)
        metrics_sets_train_frames = extract_metrics(METRIC_TYPES_TEST_FRAMES, models,
                                                   dset, train_test_split, FOLD_TEST, false)
        toc()

        for (i, model_name) in enumerate(behaviorset.names)
            logl_train = (metrics_sets_train_frames[i][1]::LoglikelihoodMetric).logl
            logl_test = (metrics_sets_test_frames[i][1]::LoglikelihoodMetric).logl
            push!(df_results, [ϵ, context_class, logl_train, logl_test, model_name])
        end

        println(df_results)
    end
end

writetable("results/error_vs_performance_metrics.csv", df)
println("DONE")