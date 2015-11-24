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

    indicators::Vector{AbstractFeature}

    ridge_regression_constant::Float64

    function LG_TrainParams(;
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

        new(indicators, ridge_regression_constant)
    end
end

type LG_PreallocatedData <: AbstractVehicleBehaviorPreallocatedData
    
    # TODO(tim): use this

    function LG_PreallocatedData(dset::ModelTrainingData, params::LG_TrainParams)
        new()
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
        behavior.N.μ[1] = behavior.A[1] * ϕ
        behavior.N.μ[2] = behavior.A[2] * ϕ
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

    # if isa(behavior.F, AbstractFeature)
        ϕ = features[frameind, symbol(behavior.F)]::Float64
    # else
    #     ϕ = features[frameind, behavior.F]::Float64
    # end
    _regress_on_feature!(behavior, ϕ)

    behavior.action[1] = features[frameind, symbol(FUTUREDESIREDANGLE_250MS)]::Float64
    behavior.action[2] = features[frameind, symbol(FUTUREACCELERATION_250MS)]::Float64

    logpdf(behavior.N, behavior.action)
end

trains_with_nona(::Type{VehicleBehaviorLinearGaussian}) = false
trains_with_nona(::VehicleBehaviorLinearGaussian) = false
function train(::Type{VehicleBehaviorLinearGaussian}, trainingframes::DataFrame;
    indicators::Vector{AbstractFeature} = [
                    POSFY, YAW, SPEED, DELTA_SPEED_LIMIT, VELFX, VELFY, SCENEVELFX, TURNRATE,
                    D_CL, D_ML, D_MR, TIMETOCROSSING_LEFT, TIMETOCROSSING_RIGHT,
                    N_LANE_L, N_LANE_R, HAS_LANE_L, HAS_LANE_R, ACC, ACCFX, ACCFY,
                    A_REQ_STAYINLANE,
                    HAS_FRONT, D_X_FRONT, D_Y_FRONT, V_X_FRONT, V_Y_FRONT, TTC_X_FRONT,
                    A_REQ_FRONT, TIMEGAP_X_FRONT,
                 ],
    ridge_regression_constant::Float64=0.5,
    args::Dict=Dict{Symbol,Any}()
    )

    for (k,v) in args
        if k == :indicators
            indicators = v
        elseif k == :ridge_regression_constant
            ridge_regression_constant = v
        else
            warn("Train VehicleBehaviorLinearGaussian: ignoring $k")
        end
    end

    γ = ridge_regression_constant
    nframes = size(trainingframes, 1)

    # A = (ΦΦᵀ + γI)⁻¹ ΦUᵀ
    # rₜ = yₜ - A⋅ϕ

    # U, column-wise contatenation of output (actions) [o×n]
    # Φ, column-wise contatenation of predictor [p×n]
    # γ, ridge-regression constant

    # Try each predictor and use the one that minimizes the loss
    #  Loss: mean normed deviation between prediction and true value

    framecount = 0
    is_frame_valid = falses(nframes)
    U = Array(Float64, 2, nframes)
    for i = 1 : nframes

        action_lat = trainingframes[i, :f_des_angle_250ms]
        action_lon = trainingframes[i, :f_accel_250ms]

        # TODO(tim): this if shouldn't be necessary
        if !isnan(action_lat) && !isnan(action_lon) &&
           !isinf(action_lat) && !isinf(action_lon)

            framecount += 1
            is_frame_valid[i] = true
            U[1, framecount] = action_lat
            U[2, framecount] = action_lon
        end
    end
    n_validframes = framecount
    U = U[:, 1:n_validframes]

    Φ = Array(Float64, framecount, 1)
    R = Array(Float64, 2, framecount) # residual vectors [o×n]
    μ∞ = Array(Float64, 2)

    best_predictor_index = 1
    best_predictor_loss = Inf # want to minimize
    best_R = Array(Float64, 2, framecount)
    best_A = Array(Float64, 2)
    best_μ∞ = Array(Float64, 2)
    for (predictor_index, F) in enumerate(indicators)
        sym = symbol(F)

        μ∞[1], μ∞[2] = 0.0, 0.0
        n∞ = 0
        Φdot = 0.0

        framecount = 0
        for i = 1 : nframes
            if is_frame_valid[i]
                framecount += 1
                ϕ = trainingframes[i, sym]
                if !isinf(ϕ)
                    Φ[framecount] = ϕ
                    Φdot += Φ[framecount]*Φ[framecount]
                else
                    n∞ += 1
                    μ∞[1] += U[1, framecount]
                    μ∞[2] += U[2, framecount]
                    Φ[framecount] = 0.0
                end
            end
        end

        # println(size(U))
        # println(size(Φ))
        A = (U*Φ)./(Φdot + γ)
        if n∞ > 0
            μ∞[1] /= n∞
            μ∞[2] /= n∞
        end

        # compute the loss from the residual vectors
        # rₜ = yₜ - A⋅ϕ
        # ε = ∑(|rₜ|₂)²

        loss = 0.0
        framecount = 0
        for i in 1 : nframes
            if is_frame_valid[i]
                framecount += 1
                if !isinf(trainingframes[i, sym])
                    R[1,framecount] = U[1,framecount] - A[1]*Φ[framecount]
                    R[2,framecount] = U[2,framecount] - A[2]*Φ[framecount]
                else
                    R[1,framecount] = U[1,framecount] - μ∞[1]
                    R[2,framecount] = U[2,framecount] - μ∞[2]
                end
                loss += R[1,framecount]*R[1,framecount] + R[2,framecount]*R[2,framecount]
            end
        end

        if loss < best_predictor_loss
            # store the model results
            best_predictor_loss = loss
            copy!(best_R, R)
            best_predictor_index = predictor_index
            copy!(best_μ∞, μ∞)
            copy!(best_A, A)
        end
    end

    # compute the associated Σ with the residual error
    Σ = zeros(Float64, 2, 2)
    for i = 1 : n_validframes
        for j = 1 : 2
            Σ[1,1] += best_R[1,i]*best_R[1,i]
            Σ[2,1] += best_R[1,i]*best_R[2,i]
            Σ[2,2] += best_R[2,i]*best_R[2,i]
        end
    end
    Σ[1,1] /= (nframes-1)
    Σ[2,1] /= (nframes-1)
    Σ[2,2] /= (nframes-1)
    Σ[1,2] = Σ[2,1]

    N = MvNormal([0.0,0.0], PDMat(Σ))

    VehicleBehaviorLinearGaussian(indicators[best_predictor_index], best_A, best_μ∞, N)
end
function train(
    training_data::ModelTrainingData,
    preallocated_data::LG_PreallocatedData,
    params::LG_TrainParams,
    fold::Int,
    fold_assignment::FoldAssignment,
    match_fold::Bool,
    )

    indicators = params.indicators
    γ = params.ridge_regression_constant
    trainingframes = training_data.dataframe

    nframes = size(trainingframes, 1)

    # A = (ΦΦᵀ + γI)⁻¹ ΦUᵀ
    # rₜ = yₜ - A⋅ϕ

    # U, column-wise contatenation of output (actions) [o×n]
    # Φ, column-wise contatenation of predictor [p×n]
    # γ, ridge-regression constant

    # Try each predictor and use the one that minimizes the loss
    #  Loss: mean normed deviation between prediction and true value

    framecount = 0
    is_frame_valid = falses(nframes)
    U = Array(Float64, 2, nframes)
    for i = 1 : nframes

        # TODO(tim): shouldn't use hard-coded symbols
        action_lat = trainingframes[i, :f_des_angle_250ms]
        action_lon = trainingframes[i, :f_accel_250ms]

        # TODO(tim): this if shouldn't be necessary
        if !isnan(action_lat) && !isnan(action_lon) &&
           !isinf(action_lat) && !isinf(action_lon)

            framecount += 1
            is_frame_valid[i] = true
            U[1, framecount] = action_lat
            U[2, framecount] = action_lon
        end
    end
    n_validframes = framecount
    U = U[:, 1:n_validframes]

    Φ = Array(Float64, framecount, 1)
    R = Array(Float64, 2, framecount) # residual vectors [o×n]
    μ∞ = Array(Float64, 2)

    best_predictor_index = 1
    best_predictor_loss = Inf # want to minimize
    best_R = Array(Float64, 2, framecount)
    best_A = Array(Float64, 2)
    best_μ∞ = Array(Float64, 2)
    for (predictor_index, F) in enumerate(indicators)
        sym = symbol(F)

        μ∞[1], μ∞[2] = 0.0, 0.0
        n∞ = 0
        Φdot = 0.0

        framecount = 0
        for i = 1 : nframes
            if is_frame_valid[i]
                framecount += 1
                ϕ = trainingframes[i, sym]
                if !isinf(ϕ)
                    Φ[framecount] = ϕ
                    Φdot += Φ[framecount]*Φ[framecount]
                else
                    n∞ += 1
                    μ∞[1] += U[1, framecount]
                    μ∞[2] += U[2, framecount]
                    Φ[framecount] = 0.0
                end
            end
        end

        # println(size(U))
        # println(size(Φ))
        A = (U*Φ)./(Φdot + γ)
        if n∞ > 0
            μ∞[1] /= n∞
            μ∞[2] /= n∞
        end

        # compute the loss from the residual vectors
        # rₜ = yₜ - A⋅ϕ
        # ε = ∑(|rₜ|₂)²

        loss = 0.0
        framecount = 0
        for i in 1 : nframes
            if is_frame_valid[i]
                framecount += 1
                if !isinf(trainingframes[i, sym])
                    R[1,framecount] = U[1,framecount] - A[1]*Φ[framecount]
                    R[2,framecount] = U[2,framecount] - A[2]*Φ[framecount]
                else
                    R[1,framecount] = U[1,framecount] - μ∞[1]
                    R[2,framecount] = U[2,framecount] - μ∞[2]
                end
                loss += R[1,framecount]*R[1,framecount] + R[2,framecount]*R[2,framecount]
            end
        end

        if loss < best_predictor_loss
            # store the model results
            best_predictor_loss = loss
            copy!(best_R, R)
            best_predictor_index = predictor_index
            copy!(best_μ∞, μ∞)
            copy!(best_A, A)
        end
    end

    # compute the associated Σ with the residual error
    Σ = zeros(Float64, 2, 2)
    for i = 1 : n_validframes
        for j = 1 : 2
            Σ[1,1] += best_R[1,i]*best_R[1,i]
            Σ[2,1] += best_R[1,i]*best_R[2,i]
            Σ[2,2] += best_R[2,i]*best_R[2,i]
        end
    end
    Σ[1,1] /= (nframes-1)
    Σ[2,1] /= (nframes-1)
    Σ[2,2] /= (nframes-1)
    Σ[1,2] = Σ[2,1]

    N = MvNormal([0.0,0.0], PDMat(Σ))

    VehicleBehaviorLinearGaussian(indicators[best_predictor_index], best_A, best_μ∞, N)
end
