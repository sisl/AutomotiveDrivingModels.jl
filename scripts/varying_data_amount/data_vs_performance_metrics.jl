using AutomotiveDrivingModels
using RandomForestBehaviors
using DynamicBayesianNetworkBehaviors

##############################
# PARAMETERS
##############################

include(Pkg.dir("AutomotiveDrivingModels", "scripts", "extract_params.jl"))
include(Pkg.dir("AutomotiveDrivingModels", "scripts", "model_params.jl"))

const DATASET_PERCENTAGES = logspace(-2.0, 0.0, 21)

##############################
# PARAMETERS
##############################

nmodels = length(behaviorset)
model_names = collect(keys(behaviorset))

df_results = Dict{AbstractString, DataFrame}()
for model_name in model_names
    df = DataFrame()
    df[:dataset_percentage] = Float64[]
    df[:nframes] = Int[]
    df[:context_class] = AbstractString[]
    df[:logl_train] = Float64[]
    df[:logl_test] = Float64[]
    df[:model_name] = AbstractString[]
    df_results[model_name] = df
end

for dset_filepath_modifier in (
    # "_freeflow",
    "_following",
    # "_lanechange",
    )

    println(dset_filepath_modifier)

    METRICS_OUTPUT_FILE = joinpath(EVALUATION_DIR, "validation_results" * dset_filepath_modifier * ".jld")
    DATASET_JLD_FILE = joinpath(EVALUATION_DIR, "dataset2" * dset_filepath_modifier * ".jld")

    dset = JLD.load(DATASET_JLD_FILE, "model_training_data")::ModelTrainingData2

    nframes = nrow(dset.dataframe)
    train_test_split = get_fold_assignment_across_drives(dset, N_FOLDS)

    assign_all_non_test_to_train!(train_test_split)

    nframes_train = calc_fold_size(FOLD_TRAIN, train_test_split.frame_assignment, true)
    nframes_test = calc_fold_size(FOLD_TEST, train_test_split.frame_assignment, true)
    println("nframes train: ", nframes_train)
    println("nframes test:  ", nframes_test)

    preallocated_data_dict = Dict{AbstractString, AbstractVehicleBehaviorPreallocatedData}()
    for (model_name, train_def) in behaviorset
        preallocated_data_dict[model_name] = preallocate_learning_data(dset, train_def.trainparams)
    end

    ##############################
    # TRAIN MODELS
    ##############################

    for dset_percentage in DATASET_PERCENTAGES

        nframes_train_reduced = round(Int, nframes_train * dset_percentage)
        @assert(nframes_train_reduced ≤ nframes_train)

        # build a subset to train on
        train_test_split_copy = deepcopy(train_test_split)
        for nframes_train_partial in nframes_train : -1 : nframes_train_reduced + 1
            remove_this_train_index = rand(1:nframes_train_partial)

            i = 0
            for (j,v) in enumerate(train_test_split_copy.frame_assignment)
                if v == FOLD_TRAIN
                    i += 1
                    if i == remove_this_train_index
                        train_test_split_copy.frame_assignment[j] = 0
                        break
                    end
                end
            end
        end
        @assert(calc_fold_size(FOLD_TRAIN, train_test_split_copy.frame_assignment, true) == nframes_train_reduced)

        println("nframes_train_reduced: ", nframes_train_reduced)
        println("fold size: ", calc_fold_size(FOLD_TRAIN, train_test_split_copy.frame_assignment, true))

        cv_split_inner = get_cross_validation_fold_assignment(N_FOLDS, dset, train_test_split_copy)

        println("\tdset_percentage: ", dset_percentage, "   nframes train reduced: ", nframes_train_reduced)

        print("\toptimizing hyperparameters\n"); tic()
        for (model_name, train_def) in behaviorset
            println(model_name)
            preallocated_data = preallocated_data_dict[model_name]
            AutomotiveDrivingModels.optimize_hyperparams_cyclic_coordinate_ascent!(
                    train_def, dset, preallocated_data, cv_split_inner)
        end
        toc()

        print("\ttraining models  "); tic()
        models = train(behaviorset, dset, preallocated_data_dict, FOLD_TEST, train_test_split_copy)
        toc()

        print("\tcomputing likelihoods  "); tic()
        logl_train_arr = Array(Float64, nframes_train_reduced)
        logl_test_arr = Array(Float64, nframes_test)
        for (i,model_name) in enumerate(model_names)

            behavior = models[model_name]
            ind_test = 0
            ind_train = 0

            for frameind in 1 : nframes
                logl = calc_action_loglikelihood(behavior, dset.dataframe, frameind)

                if train_test_split_copy.frame_assignment[frameind] == FOLD_TRAIN
                    logl_train_arr[ind_train += 1] = logl
                elseif train_test_split_copy.frame_assignment[frameind] == FOLD_TEST
                    logl_test_arr[ind_test += 1] = logl
                end
            end

            @assert(ind_train == length(logl_train_arr))
            @assert(ind_test == length(logl_test_arr))

            median_logl_train = median(logl_train_arr)
            median_logl_test = median(logl_test_arr)

            push!(df_results[model_name], [dset_percentage, nframes_train_reduced, dset_filepath_modifier, median_logl_train, median_logl_test, model_name])
        end
        toc()
    end
end

# print results
for model_name in model_names
    println(model_name)
    println(df_results[model_name])
end

# export results
for model_name in model_names
    model_output_name = replace(lowercase(model_name), " ", "_")
    outpath = joinpath(".", "results", "data_vs_performance_metrics_" * model_output_name * ".csv")
    writetable(outpath, df_results[model_name])
end

println("DONE")