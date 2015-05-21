# Car Encounter Model

module CarEM

    push!(LOAD_PATH, "/home/tim/Documents/wheelerworkspace/Bosch/model/")

    using BinMaps # TODO(tim): remove this once we stop using BinMaps in training EMs
    using Discretizers
    using BayesNets
    using Features
    using HDF5, JLD

    import Base: get

    include("common.jl")
    include("encounter_model.jl")
    include("scenario_selector.jl")
    include("behaviors.jl")
    include("io.jl")
    include("feature_extract.jl")
    include("sim.jl")
    include("sim_param_calibration.jl")

end # module