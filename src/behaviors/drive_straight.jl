export VehicleBehaviorDriveStraight

# The vehicle drives perfectly straight
type VehicleBehaviorDriveStraight <: AbstractVehicleBehavior end

function select_action(
    basics    :: FeatureExtractBasics,
    carind    :: Int,
    behavior  :: VehicleBehaviorDriveStraight,
    frameind  :: Int
    )


    a = 0.0
    ω = 0.0

    logindexbase = calc_logindexbase(carind)
    _record_frame_values!(basics.simlog, frameind, logindexbase)

    (a, ω)
end