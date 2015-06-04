export VehicleBehaviorGaussian

# The vehicle drives following a multivariabkle Gaussian noise over accel & turnrate
type VehicleBehaviorGaussian <: AbstractVehicleBehavior 
    Σ::MvNormal # with action vector (accel, turnrate)

    function VehicleBehaviorGaussian(σ_accel::Float64, σ_turnrate::Float64, σ_correlation::Float64=0.0)
        new(MvNormal([σ_accel σ_correlation; σ_correlation σ_turnrate]))
    end
    function VehicleBehaviorGaussian(Σ::MvNormal)
        @assert(length(Σ) == 2)
        new(Σ)
    end
end

function select_action(
    basics::FeatureExtractBasics,
    behavior::VehicleBehaviorGaussian,
    carind::Int,
    frameind::Int
    )

    action = rand(behavior.Σ)
    logl = logpdf(behavior.Σ, action)

    a = action[1]
    ω = action[2]

    logindexbase = calc_logindexbase(carind)

    _record_frame_values!(basics.simlog, frameind, logindexbase, 
                          logPa=logl/2, logPω=logl/2) # NOTE(tim): splitting logl in half

    (a, ω)
end
function calc_action_loglikelihood(
    basics::FeatureExtractBasics,
    behavior::VehicleBehaviorGaussian,
    carind::Int,
    frameind::Int
    )

    #=
    Compute the log-likelihood of the action taken during a single frame
    given the VehicleBehaviorGaussian.
    =#

    logindexbase = calc_logindexbase(carind)

    a = basics.simlog[frameind, logindexbase + LOG_COL_A]
    ω = basics.simlog[frameind, logindexbase + LOG_COL_T]

    logpdf(behavior.Σ, [a, ω])
end

function train(::Type{VehicleBehaviorGaussian}, trainingframes::DataFrame)

    nframes = size(trainingframes, 1)

    count = 0
    trainingmatrix = Array(Float64, 2, nframes)
    for i = 1 : nframes

        accel = trainingframes[i, :acc]
        turnrate = trainingframes[i, :turnrate]

        if !isnan(accel) && !isnan(turnrate) &&
           !isinf(accel) && !isinf(turnrate)

            count += 1
            trainingmatrix[1, count] = accel
            trainingmatrix[2, count] = turnrate
        end
    end
    trainingmatrix = trainingmatrix[:, 1:count]

    VehicleBehaviorGaussian(fit_mle(MvNormal, trainingmatrix))
end
