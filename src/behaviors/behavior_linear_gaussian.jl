export
    VehicleBehaviorLinearGaussian,
    LG_TrainParams

using PDMats

# This vehicle derives a linear gaussian representation based on a single feature
# - for features which can be Inf we include a separate Gaussian

type VehicleBehaviorLinearGaussian <: AbstractVehicleBehavior

    F::AbstractFeature # the predictor
    A::Matrix{Float64} # the autoregression matrix (2×2), μ = A⋅ϕ
    μ∞::Vector{Float64} # used for Inf values (length 2)
    N::MvNormal # with action vector (accel, turnrate) (Σ is const)

    targets::ModelTargets
    action::Vector{Float64} # the action, preallocated

    function VehicleBehaviorLinearGaussian(
        F::AbstractFeature,
        A::Matrix{Float64},
        μ∞::Vector{Float64},
        N::MvNormal,
        targets::ModelTargets
        )

        @assert(size(A, 1) == 2)
        @assert(size(A, 2) == 2)
        @assert(length(μ∞) == 2)

        new(F, A, μ∞, N, targets, [0.0,0.0])
    end
end
function Base.print(io::IO, LG::VehicleBehaviorLinearGaussian)
    println(io, "LG")
    println(io, "\tindicator: ", symbol(LG.F))
    println(io, "\tA: ")
    @printf(io, "\t\t[%12.6f, %12.6f]\n", LG.A[1,1], LG.A[1,2])
    @printf(io, "\t\t[%12.6f, %12.6f]\n", LG.A[2,1], LG.A[2,2])
    println(io, "\tμ∞:")
    @printf(io, "\t\t[%12.6f, %12.6f]\n", LG.μ∞[1], LG.μ∞[2])
    println(io, "\tΣ:")
    @printf(io, "\t\t[%12.6f, %12.6f]\n", LG.N.Σ.mat[1,1], LG.N.Σ.mat[1,2])
    @printf(io, "\t\t[%12.6f, %12.6f]\n", LG.N.Σ.mat[2,1], LG.N.Σ.mat[2,2])
end

get_targets(behavior::VehicleBehaviorLinearGaussian) = behavior.targets
get_indicators(behavior::VehicleBehaviorLinearGaussian) = AbstractFeature[behavior.F]

type LG_TrainParams <: AbstractVehicleBehaviorTrainParams

    targets::ModelTargets
    indicators::Vector{AbstractFeature}

    ridge_regression_constant::Float64

    function LG_TrainParams(;
        targets::ModelTargets = ModelTargets(Features.FUTUREDESIREDANGLE, Features.FUTUREACCELERATION),
        indicators = [
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
function Base.print(io::IO, trainparams::LG_TrainParams)
    println(io, "targets:    ", trainparams.targets)
    println(io, "indicators: ", map(f->symbol(f), trainparams.indicators))
    println(io, "λ:          ", trainparams.ridge_regression_constant)
end

type LG_PreallocatedData <: AbstractVehicleBehaviorPreallocatedData

    X::Matrix{Float64} # column-wise concatenation of predictor (features) [p×n]
    Y::Matrix{Float64} # column-wise concatenation of output (actions) [o×n]
    Φ::Matrix{Float64} # single-column of features and the singleton feature [2×n]
    μ∞::Vector{Float64} # mean values for when features are infinite (missing)
    best_μ∞::Vector{Float64}
    best_A::Matrix{Float64}
    best_Σ::Matrix{Float64}

    function LG_PreallocatedData(dset::ModelTrainingData2, params::LG_TrainParams)

        nframes = size(dset.dataframe, 1)
        p = length(params.indicators)
        X = Array(Float64, p, nframes)
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
    dset::ModelTrainingData2,
    params::LG_TrainParams)

    LG_PreallocatedData(dset, params)
end

function _diagonal_shrinkage!{T<:Real}(
    Σ::Matrix{T}, # 2×2 matrix with upper diagonal filled in
                  # this method ONLY affect the upper diagonal
    ε::T = convert(T, 1e-6),
    )

    # now we want to use Shrinkage Estimation - https://en.wikipedia.org/wiki/Estimation_of_covariance_matrices#Shrinkage_estimation
    # Since with a small number of samples we might get a Σ that isn't positive semidefinite
    # A matrix is pos. semidefinite it all of its eigenvalues are > 0
    # We choose γ such that A = γΣ + (1-γI) with all of eigenvalues are > ε
    # This method will work in general, but what is coded below assumes Σ is 2×2
    #      Σ = [a b; b d]
    @assert(size(Σ,1)==2 && size(Σ,2)==2)

    a, b, d = Σ[1,1], Σ[1,2], Σ[2,2]

    γ = 2.0*(1.0-ε) / (2.0 - a - d + sqrt(a*a -2*a*d + 4*b*b + d*d))

    if 0.0 < γ < 1.0
        # @assert(γ ≥ 0.0)

        Σ[1,1] = γ*a + (1.0-γ)
        Σ[1,2] = γ*b
        Σ[2,2] = γ*d + (1.0-γ)
    end

    Σ
end
function _regress_on_feature!(behavior::VehicleBehaviorLinearGaussian, ϕ::Float64)
    if isinf(ϕ)
        behavior.N.μ[1] = behavior.μ∞[1]
        behavior.N.μ[2] = behavior.μ∞[2]
    else
        A = behavior.A
        behavior.N.μ[1] = A[1,1] + A[1,2]*ϕ
        behavior.N.μ[2] = A[2,1] + A[2,2]*ϕ
    end

    behavior
end
function select_action(
    behavior::VehicleBehaviorLinearGaussian,
    runlog::RunLog,
    sn::StreetNetwork,
    colset::UInt,
    frame::Int
    )

    ϕ = get(behavior.F, runlog, sn, colset, frame)::Float64
    _regress_on_feature!(behavior, ϕ)

    Distributions._rand!(behavior.N, behavior.action)

    action_lat = behavior.action[1]
    action_lon = behavior.action[2]

    (action_lat, action_lon)
end

function calc_action_loglikelihood(
    behavior::VehicleBehaviorLinearGaussian,
    features::DataFrame,
    frameind::Integer,
    )


    ϕ = features[frameind, symbol(behavior.F)]::Float64
    _regress_on_feature!(behavior, ϕ)

    behavior.action[1] = features[frameind, symbol(behavior.targets.lat)]::Float64
    behavior.action[2] = features[frameind, symbol(behavior.targets.lon)]::Float64

    logpdf(behavior.N, behavior.action)
end
function calc_action_loglikelihood(
    behavior::VehicleBehaviorLinearGaussian,
    runlog::RunLog,
    sn::StreetNetwork,
    colset::UInt,
    frame::Int,
    action_lat::Float64,
    action_lon::Float64,
    )

    ϕ = get(behavior.F, runlog, sn, colset, frame)::Float64
    _regress_on_feature!(behavior, ϕ)

    behavior.action[1] = action_lat
    behavior.action[2] = action_lon

    logpdf(behavior.N, behavior.action)
end

function train(
    training_data::ModelTrainingData2,
    preallocated_data::LG_PreallocatedData,
    params::LG_TrainParams,
    frame::FoldSet,
    )

    X = preallocated_data.X
    Y = preallocated_data.Y
    Φ = preallocated_data.Φ
    μ∞ = preallocated_data.μ∞
    best_μ∞ = preallocated_data.best_μ∞
    best_A = preallocated_data.best_A
    best_Σ = preallocated_data.best_Σ
    γ = params.ridge_regression_constant

    ntrainframes = pull_design_and_target_matrices!( X, Y, training_data.dataframe, params.targets, params.indicators, frame)

    # U = Y, column-wise concatenation of output (actions) [o×n]
    # Φ = X, column-wise concatenation of predictor [p×n]
    # γ, ridge-regression constant

    # A = UΦᵀ*(ΦΦᵀ + γI)⁻¹, [o×p]
    # rₜ = yₜ - A⋅ϕ

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
        n_inf = 0
        n_noninf = 0

        for j in 1 : ntrainframes
            if !isnan(X[i, j]) && !isinf(X[i,j])
                n_noninf += 1

                Φ[1,n_noninf] = 1.0
                Φ[2,n_noninf] = X[i, j]
            else
                n_inf += 1
                μ∞[1] += Y[1, j]
                μ∞[2] += Y[2, j]
            end
        end

        for j in n_noninf + 1 : size(Φ, 2)
            Φ[1,j] = 0.0
            Φ[2,j] = 0.0
        end

        # solve the regression problem
        den = Φ*Φ' + Γ
        _diagonal_shrinkage!(den)
        den[2,1] = den[1,2] # copy over symmetric component

        A = (Y*Φ')/den # [o×p]

        if n_inf > 0
            μ∞ ./= n_inf
        end

        # compute the loss from the residual vectors and Σ
        # rₜ = yₜ - A⋅ϕ
        # ε = ∑(|rₜ|₂)²

        loss = 0.0
        fill!(Σ, 0.0)
        for j in 1 : ntrainframes
            R_lat, R_lon = NaN, NaN
            if !isnan(X[i, j]) && !isinf(X[i,j])
                val = X[i, j]
                R_lat = Y[1,j] - (A[1,1] + A[1,2]*val)
                R_lon = Y[2,j] - (A[2,1] + A[2,2]*val)
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
        Σ[1,2] /= (ntrainframes-1)
        Σ[2,2] /= (ntrainframes-1)
        Σ[2,1] = Σ[1,2] # copy over symmetric part

        if loss < best_predictor_loss
            # store the model results
            best_predictor_loss = loss
            best_predictor_index = i
            copy!(best_μ∞, μ∞)
            copy!(best_A, A)
            copy!(best_Σ, Σ)
        end

        # if verbosity > 0
            # println("\tA: ")
            # @printf("\t\t[%12.6f, %12.6f]\n", A[1,1], A[1,2])
            # @printf("\t\t[%12.6f, %12.6f]\n", A[2,1], A[2,2])
            # println("\tμ∞:")
            # @printf("\t\t[%12.6f, %12.6f]\n", μ∞[1], μ∞[2])
            # println("\tΣ:")
            # @printf("\t\t[%12.6f, %12.6f]\n", Σ[1,1], Σ[1,2])
            # @printf("\t\t[%12.6f, %12.6f]\n", Σ[2,1], Σ[2,2])

        #     @printf("%-20s  %10.6f  %10.6f\n", string(symbol(params.indicators[i])), loss, best_predictor_loss)
        # end
    end

    VehicleBehaviorLinearGaussian(params.indicators[best_predictor_index], best_A, best_μ∞, MvNormal([0.0,0.0], PDMat(best_Σ)), params.targets)
end