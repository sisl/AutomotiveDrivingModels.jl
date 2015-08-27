export
    VehicleBehaviorDriveStraight,
    VEHICLE_BEHAVIOR_DRIVE_STRAIGHT

# The vehicle drives perfectly straight
type VehicleBehaviorDriveStraight <: AbstractVehicleBehavior end
const VEHICLE_BEHAVIOR_DRIVE_STRAIGHT = VehicleBehaviorDriveStraight()

function select_action(
    basics::FeatureExtractBasicsPdSet,
    behavior::VehicleBehaviorDriveStraight,
    carind::Int,
    frameind::Int
    )

    action_lat = 0.0
    action_lon = 0.0

    (action_lat, action_lon)
end

function calc_action_loglikelihood(
    basics::FeatureExtractBasicsPdSet,
    behavior::VehicleBehaviorDriveStraight,
    carind::Int,
    validfind::Int,
    action_lat::Float64,
    action_lon::Float64
    )

    if action_lat == 0.0 && action_lon == 0.0
        Inf
    else
        -Inf
    end
end

function train(::Type{VehicleBehaviorDriveStraight}, trainingframes::DataFrame; args::Dict=Dict{Symbol,Any}())

    for (k,v) in args
        warn("Train VehicleBehaviorDriveStraight: ignoring $k")
    end

    VEHICLE_BEHAVIOR_DRIVE_STRAIGHT
end