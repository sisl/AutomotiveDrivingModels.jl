export
    BehaviorParameter,
    BehaviorTrainDefinition,

    # CVFoldResults,
    # ModelCrossValidationResults,
    # CrossValidationResults,

    optimize_hyperparams_cyclic_coordinate_ascent!,
    pull_design_and_target_matrices!,
    pull_target_matrix!,
    copy_matrix_fold!,
    copy_matrix_fold
    # cross_validate

    # add_behavior!(behaviorset, GindeleRandomForestBehavior, "Random Forest")
    # model_trainparams["Random Forest"] = GRF_TrainParams(indicators=INDICATOR_SET)
    # model_hyperparams["Random Forest"] = [
    #         BehaviorParameter(:ntrees, 1:10:51, 3),
    #         BehaviorParameter(:max_tree_depth, 1:20, 5),
    #         # BehaviorParameter(:min_samples_split, 10:10:50, 3),
    #         # BehaviorParameter(:min_samples_leaves, [2,4,10,20,50], 3),
    #         # BehaviorParameter(:min_split_improvement, [10.0, 5.0, 1.0,0.5,0.1,0.0], 3),
    #         # BehaviorParameter(:partial_sampling, [0.5,0.6,0.7,0.8,0.9,0.95,1.0], 5),
    #         # BehaviorParameter(:n_split_tries, [10,25,50,100,200,500,1000], 5),
    #     ]

###############################################################

type BehaviorParameter
    sym::Symbol # symbol of the associated field
    range::AbstractVector # set of values that the param can take on
    index_of_default::Int # index of the default value

    function BehaviorParameter(sym::Symbol, range::AbstractVector, index_of_default::Int=1)
        @assert(!isempty(range))
        @assert(index_of_default > 0)
        @assert(index_of_default ≤ length(range))
        new(sym, range, index_of_default)
    end
end

Base.length(p::BehaviorParameter) = length(p.range)

###############################################################

type BehaviorTrainDefinition
    trainparams::AbstractVehicleBehaviorTrainParams # object of train parameters for the model
    hyperparams::Vector{BehaviorParameter} # define the search space over parameters

    function BehaviorTrainDefinition(trainparams::AbstractVehicleBehaviorTrainParams, hyperparams::Vector{BehaviorParameter}=BehaviorParameter[])
        field_names = fieldnames(trainparams)
        for λ in hyperparams
            @assert(in(λ.sym, field_names))
        end
        new(trainparams, hyperparams)
    end
end

function Base.print(io::IO, traindef::BehaviorTrainDefinition)
    println(io, "BehaviorTrainDefinition")
    println(io, traindef.trainparams)
    println(io, "hyperparams:")
    for λ in traindef.hyperparams
        @printf(io, "\t%-20s [", string(λ.sym)*":")
        for (i,v) in enumerate(λ.range)
            if i == λ.index_of_default
                print(io, ">", v, "<")
            else
                print(io, v)
            end
            if i < length(λ.range)
                print(io, ", ")
            end
        end
        println(io, "]")
    end
end

###############################################################

function train(
    behaviorset::Dict{AbstractString, BehaviorTrainDefinition},
    dset::ModelTrainingData,
    preallocated_data_dict::Dict{AbstractString, AbstractVehicleBehaviorPreallocatedData},
    fold::Int,
    fold_assignment::FoldAssignment,
    )

    retval = Dict{AbstractString,AbstractVehicleBehavior}()
    for (behavior_name, train_def) in behaviorset
        preallocated_data = preallocated_data_dict[behavior_name]
        retval[behavior_name] = train(dset, preallocated_data, train_def.trainparams, fold, fold_assignment, false)
    end
    retval
end
function train(
    behaviorset::Dict{AbstractString, BehaviorTrainDefinition},
    dset::ModelTrainingData2,
    preallocated_data_dict::Dict{AbstractString, AbstractVehicleBehaviorPreallocatedData},
    fold::Int,
    fold_assignment::FoldAssignment,
    )

    retval = Dict{AbstractString,AbstractVehicleBehavior}()
    for (behavior_name, train_def) in behaviorset
        preallocated_data = preallocated_data_dict[behavior_name]
        retval[behavior_name] = train(dset, preallocated_data, train_def.trainparams, fold, fold_assignment, false)
    end
    retval
end

function optimize_hyperparams_cyclic_coordinate_ascent!(
    traindef::BehaviorTrainDefinition,
    dset::ModelTrainingData2,
    preallocated_data::AbstractVehicleBehaviorPreallocatedData,
    cross_validation_split::FoldAssignment,
    )

    hyperparams = traindef.hyperparams
    trainparams = traindef.trainparams

    if isempty(hyperparams) # nothing varies
        return traindef
    end

    n_varying_params = length(hyperparams)
    param_indeces = Array(Int, n_varying_params)
    for (param_index, λ) in enumerate(hyperparams)
        param_indeces[param_index] = λ.index_of_default
        sym = λ.sym
        val = λ.range[λ.index_of_default]
        setfield!(trainparams, sym, val)
    end

    best_logl = -Inf # mean cross-validated log likelihood
    param_hash_set = Set{UInt}() # set of param hashes that have already been done

    @printf("%40s %3s %6s %6s\n", "λ", "i", "logl", "logl*")

    t_start = time()
    t_prev_printout = 0
    iteration_count = 0
    finished = false
    while !finished

        iteration_count += 1
        has_improved = false

        if (time() - t_prev_printout) > 5.0
            println("optimize_hyperparams_cyclic_coordinate_ascent iteration ", iteration_count, "  et: ", time() - t_start)
            t_prev_printout = time()
        end

        for (param_index, λ) in enumerate(hyperparams)

            sym = λ.sym
            best_param_val = getfield(trainparams, sym) # current one

            for (i, v) in enumerate(λ.range)
                param_indeces[param_index] = i

                param_hash = hash(param_indeces)
                if !in(param_hash, param_hash_set)
                    push!(param_hash_set, param_hash)

                    setfield!(trainparams, sym, v)

                    logl = 0.0
                    for fold in 1 : cross_validation_split.nfolds
                        model = train(dset, preallocated_data, trainparams, fold, cross_validation_split, false)
                        logl += extract(MedianLoglikelihoodMetric, dset, model, cross_validation_split, fold, true).logl
                    end
                    logl /= cross_validation_split.nfolds

                    @printf("%40s %3d %6.4f %6.4f\n", string(λ.sym), i, logl, best_logl)

                    if logl > best_logl
                        best_logl = logl
                        best_param_val = v
                        has_improved = true
                    end
                end
            end

            # reset to best param val
            setfield!(trainparams, sym, best_param_val)
        end

        finished = !has_improved # terminate if we never improved
    end

    traindef
end

####################################################

function pull_design_and_target_matrices!{F}(
    X::Matrix{Float64}, # column-wise concatenation of predictor (features) [p×n]
    Y::Matrix{Float64}, # column-wise concatenation of output (actions) [o×n]
    trainingframes::DataFrame,
    targets::ModelTargets{F},
    indicators::Vector{F},
    )

    #=
    Pulls the data for X and Y, taking all of the data

    RETURN: nothing
    =#

    sym_lat = symbol(targets.lat)
    sym_lon = symbol(targets.lon)

    n = nrow(trainingframes)

    @assert(size(X,1) == length(indicators))
    @assert(size(X,2) == n)
    @assert(size(Y,1) == 2)
    @assert(size(Y,2) == n)

    for row in 1 : n

        action_lat = trainingframes[row, sym_lat]
        action_lon = trainingframes[row, sym_lon]

        @assert(!isinf(action_lat))
        @assert(!isinf(action_lon))

        Y[1, row] = action_lat
        Y[2, row] = action_lon

        for (j,feature) in enumerate(indicators)
            v = trainingframes[row, symbol(feature)]
            X[j, row] = v
        end
    end

    nothing
end
function pull_design_and_target_matrices!{F}(
    X::Matrix{Float64}, # column-wise concatenation of predictor (features) [p×n]
    Y::Matrix{Float64}, # column-wise concatenation of output (actions) [o×n]
    trainingframes::DataFrame,
    targets::ModelTargets{F},
    indicators::Vector{F},
    fold::Int,
    fold_assignment::FoldAssignment,
    match_fold::Bool;
    )

    #=
    Pulls the data for X and Y, filling the first m frames that match the fold
    The remaining allocated frames are zeroed
    RETURN: the number of frames that match the fold [::Int]
    =#

    sym_lat = symbol(targets.lat)
    sym_lon = symbol(targets.lon)

    n = size(trainingframes, 1)

    @assert(length(fold_assignment.frame_assignment) == n)
    @assert(size(X,1) == length(indicators))
    @assert(size(X,2) == n)
    @assert(size(Y,1) == 2)
    @assert(size(Y,2) == n)

    m = 0
    for row in 1 : n
        if is_in_fold(fold, fold_assignment.frame_assignment[row], match_fold)

            m += 1
            action_lat = trainingframes[row, sym_lat]
            action_lon = trainingframes[row, sym_lon]

            Y[1, m] = action_lat
            Y[2, m] = action_lon

            @assert(!isinf(action_lat))
            @assert(!isinf(action_lon))

            for (j,feature) in enumerate(indicators)
                v = trainingframes[row, symbol(feature)]
                X[j, m] = v
            end
        end
    end

    # zero out the rest
    for row in m+1 : n
        Y[1,row] = 0.0
        Y[2,row] = 0.0
        for j in 1 : size(X, 1)
            X[j,row] = 0.0
        end
    end

    return m
end
function pull_target_matrix!{F}(
    Y::Matrix{Float64}, # column-wise concatenation of output (actions) [o×n]
    trainingframes::DataFrame,
    targets::ModelTargets{F},
    indicators::Vector{F},
    )

    #=
    Pulls the data for X and Y, taking all of the data

    RETURN: modified Y
    =#

    sym_lat = symbol(targets.lat)
    sym_lon = symbol(targets.lon)

    n = nrow(trainingframes)

    @assert(size(Y,1) == 2)
    @assert(size(Y,2) == n)

    for row in 1 : n

        action_lat = trainingframes[row, sym_lat]
        action_lon = trainingframes[row, sym_lon]

        @assert(!isinf(action_lat))
        @assert(!isinf(action_lon))

        Y[1, row] = action_lat
        Y[2, row] = action_lon
    end

    Y
end

function copy_matrix_fold!{A}(
    dest::Matrix{A}, # [nframes × p]
    src::Matrix{A}, # [p × nframes]
    fold::Int,
    fold_assignment::FoldAssignment,
    match_fold::Bool,
    )

    i = 0
    for (frame, fold_a) in enumerate(fold_assignment.frame_assignment)
        if is_in_fold(fold, fold_a, match_fold)
            i += 1

            for j in 1 : size(src, 1)
                dest[i,j] = src[j,frame]
            end
        end
    end

    @assert(i == size(dest, 1)) # must fill the entire matrix

    dest
end
function copy_matrix_fold{A}(
    X::Matrix{A}, # [p × nframes]
    fold::Int,
    fold_assignment::FoldAssignment,
    match_fold::Bool,
    )

    nframes = calc_fold_size(fold, fold_assignment.frame_assignment, match_fold)
    X2 = Array(A, nframes, size(X, 1)) # NOTE: transposed from YX

    copy_matrix_fold!(X2, X, fold, fold_assignment, match_fold)

    X2 # [nframes × p]
end
