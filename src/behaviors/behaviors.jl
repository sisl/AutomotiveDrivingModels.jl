#= #################################################
                 VEHICLE BEHAVIOR
=# #################################################

# define behavior for a given vehicle in the scene

export  AbstractVehicleBehavior,
        AbstractVehicleBehaviorTrainParams,
        AbstractVehicleBehaviorPreallocatedData,
        ModelTargets,

        select_action,
        calc_action_loglikelihood,

        train,
        trains_with_nona,

        create_train_params,
        preallocate_learning_data,

        VehicleBehaviorNone,
        VEHICLE_BEHAVIOR_NONE


abstract AbstractVehicleBehavior

###############################################################
#=
    VehicleBehaviorTrainParams

    Structure containing the training params for the model
=#

abstract AbstractVehicleBehaviorTrainParams # parameters defined for training the model

###############################################################
#=
    AbstractVehicleBehaviorPreallocatedData

    Preallocated learning data for the model
    Each behavior model should implement preallocate_learning_data()
=#

abstract AbstractVehicleBehaviorPreallocatedData
preallocate_learning_data(dset::ModelTrainingData, params::AbstractVehicleBehaviorTrainParams) = error("preallocate_learning_data not implemented for ")

###############################################################

type ModelTargets{F<:Union{AbstractFeature, FeaturesNew.AbstractFeature}}
    lat::F
    lon::F
end
Base.eltype{F}(modeltarget::ModelTargets{F}) = F
Base.print(io::IO, targets::ModelTargets) = print(io, (symbol(targets.lat), symbol(targets.lon)))

function calc_action_loglikelihood(
    basics::FeatureExtractBasicsPdSet,
    behavior::AbstractVehicleBehavior,
    carind::Int,
    validfind::Int
    )

    action_lat = Features._get(FUTUREDESIREDANGLE_250MS, basics, carind, validfind)::Float64
    action_lon = Features._get(FUTUREACCELERATION_250MS, basics, carind, validfind)::Float64

    calc_action_loglikelihood(basics, behavior, carind, validfind, action_lat, action_lon)
end

trains_with_nona(::AbstractVehicleBehavior) = true
trains_with_nona{B<:AbstractVehicleBehavior}(::Type{B}) = true
train{B<:AbstractVehicleBehavior}(::Type{B}, ::DataFrame) = error("train not implemented for $B")
train(
    ::ModelTrainingData,
    a::AbstractVehicleBehaviorPreallocatedData,
    b::AbstractVehicleBehaviorTrainParams,
    fold::Int,
    fold_assignment::FoldAssignment,
    match_fold::Bool,
    ) =  error("train not implemented for $(typeof(a)) and $(typeof(b))")

###############################################################

# The vehicle's actions are pre-determined
# No update is performed on the simlog for this vehicle
type VehicleBehaviorNone <: AbstractVehicleBehavior end
VEHICLE_BEHAVIOR_NONE = VehicleBehaviorNone()
