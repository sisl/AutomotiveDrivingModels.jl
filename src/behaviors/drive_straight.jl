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


    a = 0.0
    ω = 0.0

    logindexbase = calc_logindexbase(carind)
    _record_frame_values!(basics.simlog, frameind, logindexbase)

    (a, ω)
end