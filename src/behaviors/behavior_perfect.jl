export VehicleBehaviorPerfect

# Vehicle does exactly what is in the pdset

type VehicleBehaviorPerfect <: AbstractVehicleBehavior
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