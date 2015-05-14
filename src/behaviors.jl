# for each vehicle, I need their behavior definition
#  em - if assumed to always follow one behavior
#  ss - if has a scenario selector
#  none - if behavior was predetermined
#         in this case we *must* have the log pre-initialized with the vehicle's states

export AbstractVehicleBehavior, VehicleBehaviorNone, VehicleBehaviorEM, VehicleBehaviorSS

abstract AbstractVehicleBehavior

# The vehicle's actions are pre-determined
# The simulation log *must* be appropriately initialized
type VehicleBehaviorNone <: AbstractVehicleBehavior end

# The vehicle always selects actions using the given encounter model
type VehicleBehaviorEM <: AbstractVehicleBehavior
    em :: EM
    indicators :: Vector{AbstractFeature} # get_indicators(em)
end

# The vehicle always selects actions using a scenario selector
type VehicleBehaviorSS <: AbstractVehicleBehavior
    ss :: ScenarioSelector
end