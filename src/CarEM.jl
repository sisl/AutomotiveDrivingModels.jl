# Car Encounter Model

module CarEM

    # TODO(tim): remove this dependency
    push!(LOAD_PATH, "/home/tim/Documents/wheelerworkspace/Bosch/model/")

    using Discretizers
    using Distributions
    using StreamStats
    using DataFrames
    using BayesNets
    using Features
    using JLD
    using Trajdata
    using StreetMap
    using Curves

    import Base: get

    include("polynomials.jl")
    include("common.jl")
    include("collision.jl")
    # include("encounter_model.jl")
    # include("scenario_selector.jl")
    include("feature_extract.jl")
    include("behaviors.jl")
    include("io.jl")
    include("candidate_trajectories.jl")
    include("sim.jl")
    include("sim_param_calibration.jl")
    include("sim_metrics.jl")
    include("risk_estimation.jl")
    include("model_evaluation.jl")

    # include("behaviors/behavior_em.jl")
    include("behaviors/drive_straight.jl")
    include("behaviors/behavior_gaussian.jl")
end # module