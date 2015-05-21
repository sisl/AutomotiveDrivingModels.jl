#= #################################################
                 VEHICLE BEHAVIOR
=# #################################################

# define behavior for each vehicle in the scene
#  em - if assumed to always follow one behavior
#  ss - if has a scenario selector
#  none - if behavior was predetermined
#         in this case we *must* have the log pre-initialized with the vehicle's states

export  AbstractVehicleBehavior,
        VehicleBehaviorNone,
        VehicleBehaviorEM, 
        VehicleBehaviorSS,

        ModelSimParams,

        VEHICLE_BEHAVIOR_NONE

# abstract type
abstract AbstractVehicleBehavior

# The vehicle's actions are pre-determined
# The simulation log *must* be appropriately initialized
type VehicleBehaviorNone <: AbstractVehicleBehavior end
VEHICLE_BEHAVIOR_NONE = VehicleBehaviorNone()

# The vehicle always selects actions using the given encounter model
type ModelSimParams
    sampling_scheme  :: AbstractSampleMethod
    smoothing        :: Symbol # :none, :SMA, :WMA
    smoothing_counts :: Int    # number of previous counts to use

    function ModelSimParams(
        sampling_scheme :: AbstractSampleMethod = SAMPLE_UNIFORM,
        smoothing       :: Symbol = :none,
        smoothing_counts :: Int = 1
        )

        @assert(smoothing_counts > 0)
        new(sampling_scheme, smoothing, smoothing_counts)
    end
end
type VehicleBehaviorEM <: AbstractVehicleBehavior
    em :: EM
    indicators :: Vector{AbstractFeature}
    symbol_lat :: Symbol
    symbol_lon :: Symbol

    simparams_lat :: ModelSimParams
    simparams_lon :: ModelSimParams

    function VehicleBehaviorEM(
        em            :: EM,
        simparams_lat :: ModelSimParams,
        simparams_lon :: ModelSimParams
        )

        retval = new()
        retval.em = em
        retval.indicators = get_indicators(em)

        targets = get_targets(em)
        retval.symbol_lat = symbol(get_target_lat(em, targets))
        retval.symbol_lon = symbol(get_target_lon(em, targets))

        retval.simparams_lat = simparams_lat
        retval.simparams_lon = simparams_lon

        retval
    end
end

# The vehicle always selects actions using a scenario selector
type VehicleBehaviorSS <: AbstractVehicleBehavior
    ss :: ScenarioSelector
end