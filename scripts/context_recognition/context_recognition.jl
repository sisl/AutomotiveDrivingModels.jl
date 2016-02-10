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

behaviorset_full = behaviorset

for (model_name_outer, traindef) in behaviorset_full

    behaviorset = Dict{AbstractString, BehaviorTrainDefinition}()
    behaviorset[model_name_outer] = traindef

    # try
        nmodels = length(behaviorset)
        model_names = collect(keys(behaviorset))

        context_classes = ["freeflow", "following", "lanechange"]
        ncontext_classes = length(context_classes)

        println("Loading Datasets: "); tic()
        dsets = Dict{AbstractString, ModelTrainingData2}()
        streetnets = Dict{AbstractString, StreetNetwork}()
        cv_splits = Dict{AbstractString, FoldAssignment}()
        trace_likelihoods = Dict{AbstractString, Array{Float64, 3}}() # each matrix is [context_class × nmodels × ntraces]
        trace_counts = Dict{AbstractString, Array{Int, 3}}()
        for context_class in context_classes
            dataset_jld_file = joinpath(EVALUATION_DIR, "dataset2_" * context_class * ".jld")
            dset = JLD.load(dataset_jld_file, "model_training_data")::ModelTrainingData2

            cv_split = get_fold_assignment_across_drives(dset, N_FOLDS)

            dsets[context_class] = dset
            cv_splits[context_class] = cv_split
            trace_likelihoods[context_class] = fill(NaN, ncontext_classes, nmodels, length(cv_split.seg_assignment)) # context_class_train × nmodels × ntraces
            trace_counts[context_class] = zeros(Int, ncontext_classes, nmodels, length(cv_split.seg_assignment))
        end
        toc()

        println("Training models for each context class and computing trace likelihoods"); tic()
        for (cind, context_class_train) in enumerate(context_classes)

            println("#################################################\n#\n#\n#\n")
            println("#################################################")
            println("context_class: ", context_class_train)

            dset = dsets[context_class_train]
            cv_split = cv_splits[context_class_train]

            # preallocate data
            println("preallocating data: ", context_class_train); tic()
            preallocated_data_dict = Dict{AbstractString, AbstractVehicleBehaviorPreallocatedData}()
            for (model_name, train_def) in behaviorset
                preallocated_data_dict[model_name] = preallocate_learning_data(dset, train_def.trainparams)
            end
            toc()

            # train models and eval logl for each trace
            println("training models and evaluating: "); tic()
            for fold in 1:N_FOLDS
                println("\tfold: ", fold, " / ", N_FOLDS)
                println("\ttraining... "); tic()
                models = train(behaviorset, dset, preallocated_data_dict, fold, cv_split)
                toc()

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

                    # train on all samples in other contexts and only on those matching the fold in this context
                    for segind in 1:length(cv_split_test.seg_assignment)
                        if context_class_train != context_class_test || cv_split_test.seg_assignment[segind] == fold

                            # println("\t\t\t", segind, " / ", length(cv_split_test.seg_assignment))

                            seg = dset_test.runlog_segments[segind]
                            runlog = runlogs_test[seg.runlog_id]
                            sn = streetnets[runlog.header.map_name]

                            for (i,model_name2) in enumerate(model_names)
                                if isnan(likelihoods_test[cind, i, segind])
                                    likelihoods_test[cind, i, segind] = 0.0
                                end
                                likelihoods_test[cind, i, segind] += calc_trace_likelihood(runlog, sn, seg, models[model_name2])
                                counts_test[cind, i, segind] += 1
                            end
                        end
                    end
                end
                # toc()
            end
            toc()

            for context_class_test in context_classes
                i = 1
                segind = 1
                likelihoods_test = trace_likelihoods[context_class_test]
                counts_test = trace_counts[context_class_test]
                @printf("%20s  %20s  logl: %20.6f  %5d  %6d  %6d\n", context_class_train, context_class_test, likelihoods_test[cind, i, segind]/counts_test[cind, i, segind], i, segind, counts_test[cind, i, segind])
            end
        end
        toc()


        println("Building confusion matrices"); tic()
        confusion_matrices = Dict{AbstractString, Matrix{Int}}()
        for model_name in model_names
            confusion_matrices[model_name] = zeros(Int, ncontext_classes, ncontext_classes) # actual vs. predicted
        end
        for (i, context_class_train) in enumerate(context_classes) # true labeling
            likelihoods = trace_likelihoods[context_class_train]
            counts_test = trace_counts[context_class_train]

            for (modelind, model_name) in enumerate(model_names)

                confusion = confusion_matrices[model_name]
                # segind = 1
                # for cind in 1 : size(likelihoods, 1)
                #     @printf("%20s  %20s  logl: %20.6f  %5d  %6d  %6d\n", context_class_train, context_classes[cind], likelihoods[cind, modelind, segind]/counts_test[cind, modelind, segind], i, segind, counts_test[cind, modelind, segind])
                # end

                for traceind in 1 : size(likelihoods, 3)

                    # take class index as the one that maximizes likelihood
                    best_score = -Inf
                    best_classind = 0
                    for classind in 1 : size(likelihoods, 1)
                        score = likelihoods[classind, modelind, traceind] / counts_test[classind, modelind, traceind]
                        if score > best_score
                            best_score, best_classind = score, classind
                        end
                    end

                    confusion[i, best_classind] += 1
                end
            end
        end
        toc()

        for model_name in model_names
            C = confusion_matrices[model_name]
        end

        df_results = Dict{AbstractString, DataFrame}()
        for model_name in model_names

            C = confusion_matrices[model_name]
            n_samples = sum(C)
            n_correct = trace(C) # number of correct classifications
            n_incorrect = n_samples - n_correct # number of incorrect classifications
            accuracy = n_correct / n_samples

            println(model_name)
            @printf("[%10d  %10d  %10d]\n", C[1,1], C[1,2], C[1,3])
            @printf("[%10d  %10d  %10d]\n", C[2,1], C[2,2], C[2,3])
            @printf("[%10d  %10d  %10d]\n", C[3,1], C[3,2], C[3,3])
            println("")
            println("N Samples:   ", n_samples)
            println("N Correct:   ", n_correct)
            println("N Incorrect: ", n_incorrect)
            println("Accuracy:    ", accuracy)

            for (i,context_class) in enumerate(context_classes)
                subcorrect = C[i,i]
                subsamples = sum(C[i,:])
                subaccuracy = subcorrect / subsamples
                println("Accuracy ", context_class, "   ", subaccuracy)
            end

            model_output_name = replace(lowercase(model_name), " ", "_")
            outpath = "context_recognition_" * model_output_name * ".txt"
            open(outpath, "w") do fout
                println(fout, model_name * "\n")
                println(fout, "Confusion Matrix")
                @printf(fout, "[%10d  %10d  %10d]\n", C[1,1], C[1,2], C[1,3])
                @printf(fout, "[%10d  %10d  %10d]\n", C[2,1], C[2,2], C[2,3])
                @printf(fout, "[%10d  %10d  %10d]\n", C[3,1], C[3,2], C[3,3])
                println(fout, "")
                @printf(fout, "N Samples:   %10.6f\n", n_samples)
                @printf(fout, "N Correct:   %10.6f\n", n_correct)
                @printf(fout, "N Incorrect: %10.6f\n", n_incorrect)
                @printf(fout, "Accuracy:    %10.6f\n", accuracy)

                for (i,context_class) in enumerate(context_classes)
                    subcorrect = C[i,i]
                    subsamples = sum(C[i,:])
                    subaccuracy = subcorrect / subsamples
                    println(fout, "Accuracy ", context_class, "   ", subaccuracy)
                end
            end
        end
    # catch
    #     println("CAUGHT SOME ERROR, model: ", model_name)
    # end
end

println("DONE")