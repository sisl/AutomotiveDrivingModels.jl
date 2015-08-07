export
    VehicleBehaviorDriveStraight,
    VEHICLE_BEHAVIOR_DRIVE_STRAIGHT

# The vehicle drives perfectly straight
type VehicleBehaviorDriveStraight <: AbstractVehicleBehavior end
const VEHICLE_BEHAVIOR_DRIVE_STRAIGHT = VehicleBehaviorDriveStraight()

function select_action(
    basics    :: FeatureExtractBasics,
    behavior  :: VehicleBehaviorDriveStraight,
    carind    :: Int,
    frameind  :: Int
    )


    action_lat = 0.0
    action_lon = 0.0

    logindexbase = calc_logindexbase(carind)
    record_frame_loglikelihoods!(basics.simlog, frameind, logindexbase)

    (action_lat, action_lon)
end