# Car Encounter Model

module CarEM

    push!(LOAD_PATH, "/home/tim/Documents/wheelerworkspace/Bosch/model/")

    using BinMaps # TODO(tim): remove this once we stop using BinMaps in training EMs
    using Discretizers
    using Distributions
    using DataFrames
    using BayesNets
    using Features
    using HDF5, JLD

    import Base: get

    include("common.jl")
    include("collision.jl")
    include("encounter_model.jl")
    include("scenario_selector.jl")
    include("feature_extract.jl")
    include("behaviors.jl")
    include("io.jl")
    include("sim.jl")
    include("sim_param_calibration.jl")
    include("sim_metrics.jl")
    include("risk_estimation.jl")

    include("behaviors/behavior_em.jl")
    include("behaviors/drive_straight.jl")
    include("behaviors/behavior_gaussian.jl")
end # module