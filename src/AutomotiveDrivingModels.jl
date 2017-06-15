__precompile__()

module AutomotiveDrivingModels

using Reexport
using Parameters

@reexport using Vec
@reexport using Records
@reexport using Distributions

export
    DriverModel,

    StaticDriver,

    get_name,
    action_type,
    observe!,
    get_actions!,
    tick!,
    simulate!,
    reset_hidden_states!,
    prime_with_history!,
    run_callback,

    AgentClass,

    PosSpeed1D,
    Frenet,
    FrenetSpeed,

    BoundingBoxDef,

    AbstractFeature,
    FeatureValue,
    FeatureState,

    is_feature_valid,
    is_symbol_a_feature,
    allfeatures,
    symbol2feature,

    LaneBoundary,
    SpeedLimit,
    LaneTag

include("drivermodels.jl")
include("simulation.jl")
include("callbacks.jl")
include("features.jl")

include("roadways/main.jl")
include("states/main.jl")
include("defs/main.jl")
include("actions/main.jl")
include("behaviors/main.jl")
include("typing/main.jl")

end # module
