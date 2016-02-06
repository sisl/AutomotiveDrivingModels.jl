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

nmodels = length(behaviorset)
model_names = collect(keys(behaviorset))

context_classes = ["freeflow", "following", "lanechange"]
ncontext_classes = length(context_classes)

println("Loading Datasets: "); tic()
dsets = Dict{AbstractString, ModelTrainingData2}()
cv_splits = Dict{AbstractString, FoldAssignment}()
trace_likelihoods = Dict{AbstractString, Array{Float64, 3}}() # each matrix is [context_class × nmodels × ntraces]
for context_class in context_classes
    dataset_jld_file = joinpath(EVALUATION_DIR, "dataset2_" * context_class * ".jld")
    dset = JLD.load(dataset_jld_file, "model_training_data")::ModelTrainingData2

    cv_split = get_fold_assignment_across_drives(dset, N_FOLDS)

    dsets[context_class] = dset
    cv_splits[context_class] = cv_split
    trace_likelihoods[context_class] = zeros(Float64, ncontext_classes, nmodels, length(cv_split.seg_assignment)) # context_class_of_model × nmodels × ntraces
end
toc()

println("Training models for each context class and computing trace likelihoods"); tic()
for (cind, context_class) in enumerate(context_classes)

    println("#################################################\n#\n#\n#\n")
    println("#################################################")
    println("context_class: ", context_class)

    dset = dsets[context_class]
    cv_split = cv_splits[context_class]

    # preallocate data
    println("preallocating data: ", context_class); tic()
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

        println("\ttrace likelihoods... "); tic()
        for context_class2 in context_classes

            println("\t\t", context_class2)


            dset2 = dsets[context_class2]

            println("\t\tloading runlogs"); tic()
            runlogs2 = load_runlogs(dset2)
            toc()

            println("\t\tloading streetnets"); tic()
            streetnets2 = load_streetnets(runlogs2)
            toc()

            cv_split2 = cv_splits[context_class2]
            likelihoods2 = trace_likelihoods[context_class2]

            for segind in 1:length(cv_split2.seg_assignment)
                if context_class != context_class2 || cv_split2.seg_assignment[segind] == fold

                    println("\t\t\t", segind, " / ", length(cv_split2.seg_assignment))

                    seg = dset2.runlog_segments[segind]
                    frac = context_class != context_class2 ? ncontext_classes : 1.0

                    for (i,model_name) in enumerate(model_names)
                        likelihoods2[cind, i, segind] += calc_trace_likelihood(runlogs2, streetnets2, seg, models[model_name])/frac
                    end
                end
            end
        end
        toc()
    end
    toc()
end
toc()


println("Building confusion matrices"); tic()
confusion_matrices = Dict{AbstractString, Matrix{Int}}()
for model_name in model_names
    confusion_matrices[model_name] = zeros(Int, ncontext_classes, ncontext_classes) # actual vs. predicted
end
for (i, context_class) in enumerate(context_classes) # true labeling
    likelihoods = trace_likelihoods[context_class]

    for (modelind, model_name) in enumerate(model_names)

        confusion = confusion_matrices[model_name]

        for traceind in 1 : size(likelihoods, 3)

            best_score = -Inf
            best_classind = 0
            for classind in 1 : size(likelihoods, 1)
                score = likelihoods[classind, modelind, traceind]
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
    println("Accuracy FreeFlow:    ", accuracy)
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
        println(fout, "N Samples:   ", n_samples)
        println(fout, "N Correct:   ", n_correct)
        println(fout, "N Incorrect: ", n_incorrect)
        println(fout, "Accuracy:    ", accuracy)

        for (i,context_class) in enumerate(context_classes)
            subcorrect = C[i,i]
            subsamples = sum(C[i,:])
            subaccuracy = subcorrect / subsamples
            println(fout, "Accuracy ", context_class, "   ", subaccuracy)
        end
    end
end

println("DONE")