export VehicleBehaviorLinearGaussian

# This vehicle derives a linear gaussian representation based on a single feature
# - for features which can be Inf we include a separate Gaussian

type VehicleBehaviorLinearGaussian <: AbstractVehicleBehavior

    F::AbstractFeature # the predictor
    A::Vector{Float64} # the autoregression matrix (here use vector of len 2), μ = A⋅ϕ
    μ_Inf::Vector{Float64} # used for Inf values (also length 2)
    N::MvNormal # with action vector (accel, turnrate) (Σ is const)

    action::Vector{Float64} # the action, preallocated


    function VehicleBehaviorLinearGaussian(
        F::AbstractFeature,
        A::Vector{Float64},
        μ_inf::Vector{Float64},
        N::MvNormal,
        )

        @assert(length(A) == 2)
        @assert(length(μ_Inf) == 2)

        new(F, A, μ_inf, N, [0.0,0.0])
    end
end

function _regress_on_feature!(behavior::VehicleBehaviorLinearGaussian, ϕ::Float64)
    if isinf(ϕ)
        behavior.N.μ[1] = behavior.μ_Inf[1]
        behavior.N.μ[2] = behavior.μ_Inf[2]
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

    ϕ = features[frameind, symbol(behavior.F)]::Float64
    _regress_on_feature!(behavior, ϕ)

    behavior.action[1] = features[frameind, symbol(FUTUREDESIREDANGLE_250MS)]::Float64
    behavior.action[2] = features[frameind, symbol(FUTUREACCELERATION_250MS)]::Float64

    logpdf(behavior.N, behavior.action)
end

function train(::Type{VehicleBehaviorLinearGaussian}, trainingframes::DataFrame;
    indicators::Vector{AbstractFeature} = [
                    POSFY, YAW, SPEED, DELTA_SPEED_LIMIT, VELFX, VELFY, SCENEVELFX, TURNRATE,
                    D_CL, D_ML, D_MR, TIMETOCROSSING_LEFT, TIMETOCROSSING_RIGHT,
                    N_LANE_L, N_LANE_R, HAS_LANE_L, HAS_LANE_R, ACC, ACCFX, ACCFY,
                    A_REQ_STAYINLANE,
                    HAS_FRONT, D_X_FRONT, D_Y_FRONT, V_X_FRONT, V_Y_FRONT, TTC_X_FRONT,
                    A_REQ_FRONT, TIMEGAP_X_FRONT,
                 ],
    args::Dict=Dict{Symbol,Any}()
    )

    for (k,v) in args
        if k == :indicators
            indicators = v
        else
            warn("Train VehicleBehaviorLinearGaussian: ignoring $k")
        end
    end

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
    U = U[:, 1:framecount]

    Φ = Array(Float64, framecount, 1)

    best_predictor_index = 1
    best_predictor_loss = Inf # want to minimize
    for F in indicators
        sym = symbol(F)

        Φdot = 0.0
        framecount = 0
        for i = 1 : nframes
            if is_frame_valid[i]
                framecount += 1
                Φ[framecount] = trainingframes[i, sym]
                Φdot += Φ[framecount]*Φ[framecount]
            end
        end

        A = (Φ*U')./(Φdot + γ)

        # compute the loss from the residual vectors
        # compute the associated Σ from the residual error
        # store the model results
    end

    VehicleBehaviorLinearGaussian(fit_mle(MvNormal, trainingmatrix))
end
