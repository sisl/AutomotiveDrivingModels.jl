using AutomotiveDrivingModels
using RandomForestBehaviors
using DynamicBayesianNetworkBehaviors

##############################
# PARAMETERS
##############################

include(Pkg.dir("AutomotiveDrivingModels", "scripts", "extract_params.jl"))
include(Pkg.dir("AutomotiveDrivingModels", "scripts", "model_params.jl"))

##############################
# PARAMETERS
##############################

context_classes = ["freeflow", "following", "lanechange"]
ncontext_classes = length(context_classes)

model_names = AbstractString[]
# push!(model_names, "Static Gaussian")
# push!(model_names, "Linear Gaussian")
# push!(model_names, "Random Forest")
# push!(model_names, "Dynamic Forest")
# push!(model_names, "Mixture Regression")
# push!(model_names, "Bayesian Network")
push!(model_names, "Linear Bayesian")

for model_name in model_names

    model_output_name = replace(lowercase(model_name), " ", "_")    # ex: bayesian_network
    model_short_name = convert_model_name_to_short_name(model_name) # ex: BN

    println("Loading Datasets: "); tic()
    dsets = Dict{AbstractString, ModelTrainingData2}() # context_class target -> dset
    streetnets = Dict{AbstractString, StreetNetwork}()
    cv_splits = Dict{AbstractString, FoldAssignment}() # context class target -> cv_splut
    trace_likelihoods = Dict{AbstractString, Matrix{Float64}}() # context class target -> [context_class_used_to_pred × ntraces]
    trace_counts = Dict{AbstractString, Matrix{Int}}() # context class target -> [context_class_used_to_pred × ntraces]
    for context_class in context_classes
        dataset_jld_file = joinpath(EVALUATION_DIR, "dataset2_" * context_class * ".jld")
        dset = JLD.load(dataset_jld_file, "model_training_data")::ModelTrainingData2

        cv_split = get_fold_assignment_across_drives(dset, N_FOLDS)

        dsets[context_class] = dset
        cv_splits[context_class] = cv_split
        trace_likelihoods[context_class] = fill(NaN, ncontext_classes, length(cv_split.seg_assignment)) # context_class_train × ntraces
        trace_counts[context_class] = zeros(Int, ncontext_classes, length(cv_split.seg_assignment))
    end
    toc()

    println("Loading models for each context class and computing trace likelihoods"); tic()
    for (cind, context_class_train) in enumerate(context_classes)

        println("#################################################\n\n\n\n")
        println("#################################################")
        println("context_class: ", context_class_train)

        dset = dsets[context_class_train]
        cv_split = cv_splits[context_class_train]
        dset_filepath_modifier = "_" * context_class_train

        # load models and eval logl for each trace
        println("loading models and evaluating: "); tic()
        for fold in 1 : N_FOLDS
            println("\tfold: ", fold, " / ", N_FOLDS)

            println("\tloading model...")
            model_for_fold_path_jld = joinpath(EVALUATION_DIR, model_short_name * dset_filepath_modifier * "_fold" * @sprintf("%02d", fold) * ".jld")
            model = JLD.load(model_for_fold_path_jld, "model")

            println("\ttrace likelihoods... "); #tic()
            for context_class_test in context_classes

                # println("\t\t", context_class_test)

                dset_test = dsets[context_class_test]

                # println("\t\tloading runlogs"); tic()
                runlogs_test = load_runlogs(dset_test)
                # toc()

                if isempty(streetnets)
                    # println("\t\tloading streetnets"); #tic()
                    streetnets = load_streetnets(runlogs_test)
                    # toc()
                end

                cv_split_test = cv_splits[context_class_test]
                likelihoods_test = trace_likelihoods[context_class_test]
                counts_test = trace_counts[context_class_test]

                # evaluate likelihood of trames
                #      - do so for all traces in other contexts
                #      - do so only on those not trained on for this context
                for segind in 1:length(cv_split_test.seg_assignment)
                    if context_class_train != context_class_test || cv_split_test.seg_assignment[segind] == fold

                        # println("\t\t\t", segind, " / ", length(cv_split_test.seg_assignment))

                        seg = dset_test.runlog_segments[segind]
                        runlog = runlogs_test[seg.runlog_id]
                        sn = streetnets[runlog.header.map_name]

                        if isnan(likelihoods_test[cind, segind])
                            likelihoods_test[cind, segind] = 0.0
                        end
                        likelihoods_test[cind, segind] += calc_trace_likelihood(runlog, sn, seg, model)
                        counts_test[cind, segind] += 1
                    end
                end
            end
            # toc()
        end
        toc()

        for context_class_test in context_classes

            likelihoods_test = trace_likelihoods[context_class_test]
            counts_test = trace_counts[context_class_test]

            # compute average likelihood across all segment indeces
            likelihood_test = 0.0
            n_segs = size(likelihoods_test, 2)
            for segind in 1 : n_segs
                likelihood_test += trace_likelihoods[context_class_test][cind, segind]/counts_test[cind, segind]
            end
            likelihood_test /= n_segs

            # @printf("%20s  %20s  logl: %20.6f  %8d\n", context_class_train, context_class_test, likelihood_test, n_segs)
        end
    end
    toc()


    println("Building confusion matrix"); tic()
    confusion = zeros(Int, ncontext_classes, ncontext_classes)

    for (i, context_class_train) in enumerate(context_classes) # true labeling
        likelihoods = trace_likelihoods[context_class_train] # recall: [context_class_used_to_pred × ntraces]
        counts_test = trace_counts[context_class_train]

        for traceind in 1 : size(likelihoods, 2)

            # take class index as the one that maximizes likelihood
            best_likelihood = -Inf
            best_classind = 0
            for index_of_contextclass_used_to_predict in 1 : size(likelihoods, 1)
                likelihood = likelihoods[index_of_contextclass_used_to_predict, traceind] / counts_test[index_of_contextclass_used_to_predict, traceind]
                if likelihood > best_likelihood
                    best_likelihood, best_classind = likelihood, index_of_contextclass_used_to_predict
                end
            end

            confusion[i, best_classind] += 1
        end
    end
    toc()

    df_results = Dict{AbstractString, DataFrame}()

    n_samples = sum(confusion)
    n_correct = trace(confusion) # number of correct classifications
    n_incorrect = n_samples - n_correct # number of incorrect classifications
    accuracy = n_correct / n_samples

    println(model_name)
    @printf("[%10d  %10d  %10d]\n", confusion[1,1], confusion[1,2], confusion[1,3])
    @printf("[%10d  %10d  %10d]\n", confusion[2,1], confusion[2,2], confusion[2,3])
    @printf("[%10d  %10d  %10d]\n", confusion[3,1], confusion[3,2], confusion[3,3])
    println("")
    println("N Samples:   ", n_samples)
    println("N Correct:   ", n_correct)
    println("N Incorrect: ", n_incorrect)
    println("Accuracy:    ", accuracy)

    for (i,context_class) in enumerate(context_classes)
        subcorrect = confusion[i,i]
        subsamples = sum(confusion[i,:])
        subaccuracy = subcorrect / subsamples
        println("Accuracy ", context_class, "   ", subaccuracy)
    end

    outpath = "context_recognition_" * model_output_name * ".txt"

    open(outpath, "w") do fout
        println(fout, model_name * "\n")
        println(fout, "Confusion Matrix")
        @printf(fout, "[%10d  %10d  %10d]\n", confusion[1,1], confusion[1,2], confusion[1,3])
        @printf(fout, "[%10d  %10d  %10d]\n", confusion[2,1], confusion[2,2], confusion[2,3])
        @printf(fout, "[%10d  %10d  %10d]\n", confusion[3,1], confusion[3,2], confusion[3,3])
        println(fout, "")
        @printf(fout, "N Samples:   %10.6f\n", n_samples)
        @printf(fout, "N Correct:   %10.6f\n", n_correct)
        @printf(fout, "N Incorrect: %10.6f\n", n_incorrect)
        @printf(fout, "Accuracy:    %10.6f\n", accuracy)

        for (i,context_class) in enumerate(context_classes)
            subcorrect = confusion[i,i]
            subsamples = sum(confusion[i,:])
            subaccuracy = subcorrect / subsamples
            println(fout, "Accuracy ", context_class, "   ", subaccuracy)
        end
    end
end

println("DONE")