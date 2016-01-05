using AutomotiveDrivingModels

using RandomForestBehaviors
using DynamicBayesianNetworkBehaviors

#=
We are testing how much model performance degrades as the amount of data decreases
The way we are testing this is to:
 - load the full datasets for each context class
 - split them into train/test
 - train a model for each reduced-quantity context class (based on cross-validated optimal hyperparams)
 - evaluate the model on the withheld training set via the logl
 - save results to csv file
=#

##############################
# PARAMETERS
##############################

const DATASET_PERCENTAGES = logspace(-2.0, 0.0, 20)
const MAX_CV_OPT_TIME_PER_MODEL = 60.0 # [s]
const NFOLDS = 5
const METRIC_TYPES_TEST_FRAMES = [LoglikelihoodMetric]
const EVALUATION_DIR = "/media/tim/DATAPART1/PublicationData/2015_TrafficEvolutionModels/realworld/"

df_results = DataFrame()
df_results[:dataset_percentage] = Float64[]
df_results[:nframes] = Int[]
df_results[:context_class] = String[]
df_results[:logl_train] = Float64[]
df_results[:logl_test] = Float64[]
df_results[:model_name] = String[]

include(Pkg.dir("AutomotiveDrivingModels", "scripts", "model_params.jl"))
nmodels = length(behaviorset)

for dset_filepath_modifier in (
    "_freeflow",
    # "_following",
    # "_lanechange",
    )

    context_class = dset_filepath_modifier
    println("context_class: ", context_class)

    METRICS_OUTPUT_FILE = joinpath(EVALUATION_DIR, "validation_results" * dset_filepath_modifier * ".jld")
    MODEL_OUTPUT_JLD_FILE = joinpath(EVALUATION_DIR, "validation_models" * dset_filepath_modifier * ".jld")
    TRAIN_VALIDATION_JLD_FILE = joinpath(EVALUATION_DIR, "train_validation_split" * dset_filepath_modifier * ".jld")
    DATASET_JLD_FILE = joinpath(EVALUATION_DIR, "dataset" * dset_filepath_modifier * ".jld")

    dset = JLD.load(DATASET_JLD_FILE, "model_training_data")::ModelTrainingData2
    nframes = nrow(dset.dataframe)
    train_test_split = get_fold_assignment_across_drives(dset)
    fold_test = 1

    nframes_train = calc_fold_size(fold_test, train_test_split.frame_assignment, false)
    println("nframes train: ", nframes_train)
    println("nframes test:  ", calc_fold_size(fold_test, train_test_split.frame_assignment, true))

    preallocated_data_dict = Dict{AbstractString, AbstractVehicleBehaviorPreallocatedData}()
    for (behavior_name, train_def) in behaviorset
        preallocated_data_dict[behavior_name] = preallocate_learning_data(dset, train_def.trainparams)
    end

    ##############################
    # TRAIN MODELS
    ##############################

    # train_frames = randperm(nframes_train)
    # train_index_to_assignment_index = Array(Int, nframes_train)
    # let
    #     j = 0
    #     for (i,a) in enumerate(train_test_split.frame_assignment)
    #         if a != fold_test
    #             j += 1
    #             train_index_to_assignment_index[j] = train_frames[j]
    #         end
    #     end
    # end

    for dset_percentage in DATASET_PERCENTAGES

        nframes_train_reduced = int(nframes_train * dset_percentage)
        @assert(nframes_train_reduced ≤ nframes_train)

        # build a subset to train on
        train_test_split_copy = deepcopy(train_test_split)
        for nframes_train_partial in nframes_train : -1 : nframes_train_reduced + 1
            remove_this_train_index = rand(1:nframes_train_partial)

            i = 0
            for (j,v) in enumerate(train_test_split_copy.frame_assignment)
                if v == fold_train
                    i += 1
                    if i == remove_this_train_index
                        train_test_split_copy.frame_assignment[j] = 0
                        break
                    end
                end
            end
        end
        @assert(calc_fold_size(fold_train, train_test_split_copy.frame_assignment, true) == nframes_train_reduced)

        cv_split_inner = get_cross_validation_fold_assignment(NFOLDS, dset, train_test_split_copy) # TODO THIS

        println("\tdset_percentage: ", dset_percentage, "   nframes train reduced: ", nframes_train_reduced)

        print("\toptimizing hyperparameters\n"); tic()
        for (behavior_name, train_def) in behaviorset
            println(behavior_name)
            preallocated_data = preallocated_data_dict[behavior_name]
            AutomotiveDrivingModels.optimize_hyperparams_cyclic_coordinate_ascent!(
                    train_def, dset, preallocated_data, cv_split_inner)
        end
        toc()

        print("\ttraining models  "); tic()
        models = train(behaviorset, dset, preallocated_data_dict, fold_test, train_test_split)
        toc()

        print("\tcomputing likelihoods  "); tic()
        for (i,behavior_name) in enumerate(model_names)
            behavior = models[behavior_name]

            logl_train = 0.0
            logl_test = 0.0

            for frameind in 1 : nframes
                if trains_with_nona(behavior) # TODO(tim): fix this
                    logl = calc_action_loglikelihood(behavior, dset.dataframe_nona, frameind)
                else
                    logl = calc_action_loglikelihood(behavior, dset.dataframe, frameind)
                end

                if train_test_split_copy.frame_assignment[frameind] != fold_test
                    logl_train += logl
                else
                    logl_test += logl
                end
            end

            push!(df_results, [dset_percentage, nframes_train_reduced, context_class, logl_train, logl_test, behavior_name])
        end
        toc()

        println(df_results)
    end
end

writetable("results/data_vs_performance_metrics.csv", df_results)