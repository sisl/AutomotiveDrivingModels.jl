#= #################################################
                 VEHICLE BEHAVIOR
=# #################################################

# define behavior for a given vehicle in the scene

export  AbstractVehicleBehavior,

        select_action,
        calc_action_loglikelihood,

        train,
        train_special,

        VehicleBehaviorNone,
        VEHICLE_BEHAVIOR_NONE


abstract AbstractVehicleBehavior

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

train{B<:AbstractVehicleBehavior}(::Type{B}, ::DataFrame) = error("train not implemented for $B")
train_special{B<:AbstractVehicleBehavior}(::Type{B}, ::DataFrame) = error("train_special not implemented for $B")

###############################################################

# The vehicle's actions are pre-determined
# No update is performed on the simlog for this vehicle
type VehicleBehaviorNone <: AbstractVehicleBehavior end
VEHICLE_BEHAVIOR_NONE = VehicleBehaviorNone()