export
    VehicleBehaviorGaussian,
    SG_TrainParams

type SG_TrainParams <: AbstractVehicleBehaviorTrainParams

    targets::ModelTargets

    SG_TrainParams(; targets::ModelTargets = ModelTargets(Features.FUTUREDESIREDANGLE, Features.FUTUREACCELERATION)) = new(targets)
end

type SG_PreallocatedData <: AbstractVehicleBehaviorPreallocatedData

    # TODO(tim): use this

    SG_PreallocatedData(dset::ModelTrainingData2, params::SG_TrainParams) = new()
end
function preallocate_learning_data(
    dset::ModelTrainingData2,
    params::SG_TrainParams)

    SG_PreallocatedData(dset, params)
end

#############
#
#
#
#############

# The vehicle drives following a multivariable Gaussian noise over accel & turnrate
type VehicleBehaviorGaussian <: AbstractVehicleBehavior

    targets::ModelTargets
    Σ::MvNormal # with action vector (accel, turnrate)

    # preallocated memory
    action::Vector{Float64}

    function VehicleBehaviorGaussian(σ_lat::Float64, σ_lon::Float64, σ_correlation::Float64=0.0;
        targets::ModelTargets = ModelTargets(Features.FUTUREDESIREDANGLE, Features.FUTUREACCELERATION),
        )

        new(targets, MvNormal([σ_lat σ_correlation; σ_correlation σ_lon]), [0.0,0.0])
    end
    function VehicleBehaviorGaussian(Σ::MvNormal, params::SG_TrainParams;
        )

        @assert(length(Σ) == 2)
        new(params.targets, Σ, [0.0,0.0])
    end
end
function Base.print(io::IO, SG::VehicleBehaviorGaussian)
    println(io, "SG")
    println(io, "\tμ:")
    @printf(io, "\t\t[%12.6f, %12.6f]\n", SG.Σ.μ[1], SG.Σ.μ[2])
    println(io, "\tΣ:")
    @printf(io, "\t\t[%12.6f, %12.6f]\n", SG.Σ.Σ.mat[1,1], SG.Σ.Σ.mat[1,2])
    @printf(io, "\t\t[%12.6f, %12.6f]\n", SG.Σ.Σ.mat[2,1], SG.Σ.Σ.mat[2,2])
end

get_targets(behavior::VehicleBehaviorGaussian) = behavior.targets
get_indicators(behavior::VehicleBehaviorGaussian) = AbstractFeature[]

#############
#
#
#
#############

function select_action(
    behavior::VehicleBehaviorGaussian,
    runlog::RunLog,
    sn::StreetNetwork,
    colset::UInt,
    frame::Int
    )

    Distributions._rand!(behavior.Σ, behavior.action)

    action_lat = behavior.action[1]
    action_lon = behavior.action[2]

    (action_lat, action_lon)
end

function calc_action_loglikelihood(
    behavior::VehicleBehaviorGaussian,
    features::DataFrame,
    frameind::Integer,
    )


    behavior.action[1] = features[frameind, symbol(behavior.targets.lat)]::Float64
    behavior.action[2] = features[frameind, symbol(behavior.targets.lon)]::Float64

    logpdf(behavior.Σ, behavior.action)
end
function calc_action_loglikelihood(
    behavior::VehicleBehaviorGaussian,
    runlog::RunLog,
    sn::StreetNetwork,
    colset::UInt,
    frame::Int,
    action_lat::Float64,
    action_lon::Float64,
    )

    behavior.action[1] = action_lat
    behavior.action[2] = action_lon

    logpdf(behavior.Σ, behavior.action)
end

function train(
    training_data::ModelTrainingData2,
    preallocated_data::SG_PreallocatedData,
    params::SG_TrainParams,
    foldset::FoldSet,
    )

    trainingframes = training_data.dataframe_nona
    nframes = size(trainingframes, 1)

    sym_lat = symbol(params.targets.lat)
    sym_lon = symbol(params.targets.lon)

    total = 0
    trainingmatrix = Array(Float64, 2, nframes)
    for frame in foldset

        action_lat = trainingframes[frame, sym_lat]
        action_lon = trainingframes[frame, sym_lon]

        if !isnan(action_lat) && !isnan(action_lon) &&
           !isinf(action_lat) && !isinf(action_lon)

            total += 1
            trainingmatrix[1, total] = action_lat
            trainingmatrix[2, total] = action_lon
        end
    end
    trainingmatrix = trainingmatrix[:, 1:total]

    VehicleBehaviorGaussian(fit_mle(MvNormal, trainingmatrix), params)
end