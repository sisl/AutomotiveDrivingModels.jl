export VehicleBehaviorPerfect

# Vehicle does exactly what is in the pdset

type VehicleBehaviorPerfect <: AbstractVehicleBehavior
end

function select_action(
    basics::FeatureExtractBasicsPdSet,
    behavior::VehicleBehaviorPerfect,
    carind::Int,
    validfind::Int
    )

    action_lat = get(FUTUREDESIREDANGLE_250MS, basics, carind, validfind)
    action_lon = get(FUTUREACCELERATION_250MS, basics, carind, validfind)

    (action_lat, action_lon)
end

function calc_action_loglikelihood(
    basics::FeatureExtractBasicsPdSet,
    behavior::VehicleBehaviorPerfect,
    carind::Int,
    validfind::Int,
    action_lat::Float64,
    action_lon::Float64
    )

    #=
    Compute the log-likelihood of the action taken during a single frame
    given the VehicleBehaviorPerfect.
    =#

    action_lat_true = get(FUTUREDESIREDANGLE_250MS, basics, carind, validfind)
    action_lon_true = get(FUTUREACCELERATION_250MS, basics, carind, validfind)

    if action_lat == action_lat_true &&
       action_lon == action_lon_true

       return log(1.0)
    end

    -Inf
end
function calc_action_loglikelihood(
    behavior::VehicleBehaviorPerfect,
    features::DataFrame,
    frameind::Integer,
    )

    action_lat_true = features[frameind, symbol(FUTUREDESIREDANGLE_250MS)]::Float64
    action_lon_true = features[frameind, symbol(FUTUREACCELERATION_250MS)]::Float64

    if action_lat == action_lat_true &&
       action_lon == action_lon_true

       return log(1.0)
    end

    -Inf
end

function train(::Type{VehicleBehaviorPerfect}, trainingframes::DataFrame; args::Dict=Dict{Symbol,Any}())

    for (k,v) in args
        warn("Train VehicleBehaviorPerfect: ignoring $k")
    end

    VehicleBehaviorPerfect()
end