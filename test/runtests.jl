using Base.Test
# using Lint

# lintpkg("AutomotiveDrivingModels")

using AutomotiveDrivingModels

include(Pkg.dir("AutomotiveDrivingModels", "test", "core", "test_core.jl"))

include(Pkg.dir("AutomotiveDrivingModels", "test", "test_actions.jl"))
include(Pkg.dir("AutomotiveDrivingModels", "test", "test_driver_models.jl"))

include(Pkg.dir("AutomotiveDrivingModels", "test", "test_roadway_generation.jl"))

include(Pkg.dir("AutomotiveDrivingModels", "test", "test_features.jl"))