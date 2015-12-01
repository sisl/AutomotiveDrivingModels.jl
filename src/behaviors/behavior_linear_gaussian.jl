export
    VehicleBehaviorLinearGaussian,
    LG_TrainParams

using PDMats

# This vehicle derives a linear gaussian representation based on a single feature
# - for features which can be Inf we include a separate Gaussian

type VehicleBehaviorLinearGaussian <: AbstractVehicleBehavior

    F::AbstractFeature # the predictor
    A::Vector{Float64} # the autoregression matrix (here use vector of len 2), μ = A⋅ϕ
    μ∞::Vector{Float64} # used for Inf values (also length 2)
    N::MvNormal # with action vector (accel, turnrate) (Σ is const)

    action::Vector{Float64} # the action, preallocated

    function VehicleBehaviorLinearGaussian(
        F::Union{AbstractFeature,Int},
        A::Vector{Float64},
        μ∞::Vector{Float64},
        N::MvNormal,
        )

        @assert(length(A) == 2)
        @assert(length(μ∞) == 2)

        new(F, A, μ∞, N, [0.0,0.0])
    end
end

type LG_TrainParams <: AbstractVehicleBehaviorTrainParams

    targets::ModelTargets
    indicators::Vector{AbstractFeature}

    ridge_regression_constant::Float64

    function LG_TrainParams(;
        targets::ModelTargets = ModelTargets(FUTUREDESIREDANGLE_250MS, FUTUREACCELERATION_250MS),
        indicators::Vector{AbstractFeature} = [
                    POSFY, YAW, SPEED, DELTA_SPEED_LIMIT, VELFX, VELFY, SCENEVELFX, TURNRATE,
                    D_CL, D_ML, D_MR, TIMETOCROSSING_LEFT, TIMETOCROSSING_RIGHT,
                    N_LANE_L, N_LANE_R, HAS_LANE_L, HAS_LANE_R, ACC, ACCFX, ACCFY,
                    A_REQ_STAYINLANE,
                    HAS_FRONT, D_X_FRONT, D_Y_FRONT, V_X_FRONT, V_Y_FRONT, TTC_X_FRONT,
                    A_REQ_FRONT, TIMEGAP_X_FRONT,
                 ],
        ridge_regression_constant::Real=0.5,
        )

        @assert(0.0 ≤ ridge_regression_constant ≤ 1.0)

        new(targets, indicators, ridge_regression_constant)
    end
end

type LG_PreallocatedData <: AbstractVehicleBehaviorPreallocatedData

    X::Matrix{Float64} # column-wise concatenation of predictor (features) [p+1×n]
                        # the first is the bias feature, always 1.0. Setting bias to 0 means sample does not exist
    Y::Matrix{Float64} # column-wise concatenation of output (actions) [o×n]
    Φ::Matrix{Float64} # single-column of features and the singleton feature [2×n]
    μ∞::Vector{Float64} # mean values for when features are infinite (missing)
    best_μ∞::Vector{Float64}
    best_A::Matrix{Float64}
    best_Σ::Matrix{Float64}

    function LG_PreallocatedData(dset::ModelTrainingData, params::LG_TrainParams)

        nframes = size(dset.dataframe, 1)
        p = length(params.indicators)
        X = Array(Float64, p+1, nframes)
        Y = Array(Float64, 2, nframes)
        Φ = Array(Float64, 2, nframes)
        μ∞ = Array(Float64, 2)
        best_μ∞ = Array(Float64, 2)
        best_A = Array(Float64, 2, 2)
        best_Σ = Array(Float64, 2, 2)

        new(X, Y, Φ, μ∞, best_μ∞, best_A, best_Σ)
    end
end
function preallocate_learning_data(
    dset::ModelTrainingData,
    params::LG_TrainParams)

    LG_PreallocatedData(dset, params)
end

function _regress_on_feature!(behavior::VehicleBehaviorLinearGaussian, ϕ::Float64)
    if isinf(ϕ)
        behavior.N.μ[1] = behavior.μ∞[1]
        behavior.N.μ[2] = behavior.μ∞[2]
    else
        A = behavior.A
        behavior.N.μ[1] = A[1,1] + A[1,2] * ϕ
        behavior.N.μ[2] = A[2,1] + A[2,2] * ϕ
    end

    behavior
end
function select_action(
    basics::FeatureExtractBasicsPdSet,
    behavior::VehicleBehaviorLinearGaussian,
    carind::Int,
    validfind::Int
    )

    ϕ = get(behavior.F, basics, carind, validfind)::Float64
    _regress_on_feature!(behavior, ϕ)

    Distributions._rand!(behavior.N, behavior.action)

    action_lat = behavior.action[1]
    action_lon = behavior.action[2]

    (action_lat, action_lon)
end

function calc_action_loglikelihood(
    ::FeatureExtractBasicsPdSet,
    behavior::VehicleBehaviorLinearGaussian,
    carind::Int,
    validfind::Int,
    action_lat::Float64,
    action_lon::Float64
    )

    #=
    Compute the log-likelihood of the action taken during a single frame
    given the VehicleBehaviorLinearGaussian.
    =#

    ϕ = get(behavior.F, basics, carind, validfind)::Float64
    _regress_on_feature!(behavior, ϕ)

    behavior.action[1] = action_lat
    behavior.action[2] = action_lon

    logpdf(behavior.N, behavior.action)
end
function calc_action_loglikelihood(
    behavior::VehicleBehaviorLinearGaussian,
    features::DataFrame,
    frameind::Integer,
    )


    ϕ = features[frameind, symbol(behavior.F)]::Float64
    _regress_on_feature!(behavior, ϕ)

    behavior.action[1] = features[frameind, symbol(FUTUREDESIREDANGLE_250MS)]::Float64
    behavior.action[2] = features[frameind, symbol(FUTUREACCELERATION_250MS)]::Float64

    logpdf(behavior.N, behavior.action)
end

trains_with_nona(::Type{VehicleBehaviorLinearGaussian}) = false
trains_with_nona(::VehicleBehaviorLinearGaussian) = false
function train(
    training_data::ModelTrainingData,
    preallocated_data::LG_PreallocatedData,
    params::LG_TrainParams,
    fold::Int,
    fold_assignment::FoldAssignment,
    match_fold::Bool,
    )

    X = preallocated_data.X
    Y = preallocated_data.Y
    Φ = preallocated_data.Φ
    μ∞ = preallocated_data.μ∞
    best_μ∞ = preallocated_data.best_μ∞
    best_A = preallocated_data.best_A
    best_Σ = preallocated_data.best_Σ
    γ = params.ridge_regression_constant

    ntrainframes = pull_design_and_target_matrices!(
        X, Y, training_data.dataframe, params.targets, params.indicators,
        fold, fold_assignment, match_fold,
        first_feature_is_bias=true)

    # A = (ΦΦᵀ + γI)⁻¹ ΦUᵀ
    # rₜ = yₜ - A⋅ϕ

    # U = Y, column-wise concatenation of output (actions) [o×n]
    # Φ = X, column-wise concatenation of predictor [p×n]
    # γ, ridge-regression constant

    # Try each predictor and use the one that minimizes the loss
    # Loss: mean normed deviation between prediction and true value

    best_predictor_index = 1
    best_predictor_loss = Inf # want to minimize

    fill!(best_μ∞, 0.0)
    fill!(best_A, 0.0)
    fill!(best_Σ, 0.0)

    Γ = [γ 0.0; 0.0 γ]
    Σ = Array(Float64, 2, 2)

    for i in 1 : size(X, 1) # for each feature

        fill!(μ∞, 0.0)
        fill!(Σ, 0.0)
        n_inf = 0
        n_noninf = 0

        for j = 1 : ntrainframes
            if !isinf(X[i, j])
                n_noninf += 1

                Φ[1,n_noninf] = 1.0
                Φ[2,n_noninf] = X[i, j]
            else
                n_inf += 1
                μ∞[1] += Y[1, j]
                μ∞[2] += Y[2, j]
            end
        end

        for j = n_noninf : size(Φ, 2)
            Φ[1,j] = 0.0
            Φ[2,j] = 0.0
        end

        # solve the regression problem
        A = (Φ*Y)/(Φ*Φ' + Γ)

        if n_inf > 0
            μ∞ ./= n_inf
        end

        # compute the loss from the residual vectors and Σ
        # rₜ = yₜ - A⋅ϕ
        # ε = ∑(|rₜ|₂)²

        loss = 0.0
        for j in 1 : ntrainframes
            if !isinf(X[i, j])
                R_lat = Y[1,j] - (A[1,1] + A[1,2]*Φ[j])
                R_lon = Y[2,j] - (A[2,1] + A[2,2]*Φ[j])
            else
                R_lat = Y[1,j] - μ∞[1]
                R_lon = Y[2,j] - μ∞[2]
            end
            loss += R_lat*R_lat + R_lon*R_lon

            Σ[1,1] += R_lat*R_lat
            Σ[1,2] += R_lat*R_lon
            Σ[2,2] += R_lon*R_lon
        end
        Σ[1,1] /= (ntrainframes-1)
        Σ[2,1] /= (ntrainframes-1)
        Σ[2,2] /= (ntrainframes-1)
        Σ[1,2] = Σ[2,1] # copy over symmetric part

        if loss < best_predictor_loss
            # store the model results
            best_predictor_loss = loss
            best_predictor_index = predictor_index
            copy!(best_μ∞, μ∞)
            copy!(best_A, A)
            copy!(best_Σ, Σ)
        end
    end

    VehicleBehaviorLinearGaussian(indicators[best_predictor_index], best_A, best_μ∞, MvNormal([0.0,0.0], PDMat(best_Σ)))
end
