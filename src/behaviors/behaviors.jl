#= #################################################
                 VEHICLE BEHAVIOR
=# #################################################

# define behavior for a given vehicle in the scene

export  AbstractVehicleBehavior,
        AbstractVehicleBehaviorTrainParams,
        AbstractVehicleBehaviorPreallocatedData,
        ModelTargets,

        get_targets,
        get_indicators,
        select_action,
        calc_action_loglikelihood,
        train,

        create_train_params,
        preallocate_learning_data,

        VehicleBehaviorNone,
        VEHICLE_BEHAVIOR_NONE


abstract AbstractVehicleBehavior

get_targets(behavior::AbstractVehicleBehavior) = error("get_indicators not implemented for behavior")
get_indicators(behavior::AbstractVehicleBehavior) = error("get_indicators not implemented for behavior")

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
preallocate_learning_data(dset::ModelTrainingData2, params::AbstractVehicleBehaviorTrainParams) = error("preallocate_learning_data not implemented for ")

###############################################################

type ModelTargets
    lat::AbstractFeature
    lon::AbstractFeature
end
Base.print(io::IO, targets::ModelTargets) = print(io, (symbol(targets.lat), symbol(targets.lon)))

train{B<:AbstractVehicleBehavior}(::Type{B}, ::DataFrame) = error("train not implemented for $B")
train(
    ::ModelTrainingData2,
    a::AbstractVehicleBehaviorPreallocatedData,
    b::AbstractVehicleBehaviorTrainParams,
    foldset::FoldSet,
    ) =  error("train not implemented for $(typeof(a)) and $(typeof(b))")

###############################################################

# The vehicle's actions are pre-determined
# No update is performed on the simlog for this vehicle
type VehicleBehaviorNone <: AbstractVehicleBehavior end
VEHICLE_BEHAVIOR_NONE = VehicleBehaviorNone()
