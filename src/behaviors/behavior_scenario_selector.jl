export VehicleBehaviorSS

# The vehicle always selects actions using a scenario selector
type VehicleBehaviorSS <: AbstractVehicleBehavior
    ss :: ScenarioSelector
end