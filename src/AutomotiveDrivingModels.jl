VERSION >= v"0.4.0-dev+6521" && __precompile__(true)

module AutomotiveDrivingModels

using Reexport

@reexport using DataFrames
@reexport using Distributions
@reexport using Vec

include("core/AutoCore.jl")
@reexport using .AutoCore

include(Pkg.dir("AutomotiveDrivingModels", "src", "utils", "minkowski.jl"))
include(Pkg.dir("AutomotiveDrivingModels", "src", "features", "features.jl"))

include(Pkg.dir("AutomotiveDrivingModels", "src", "simulation", "actions.jl"))
include(Pkg.dir("AutomotiveDrivingModels", "src", "behaviors", "driver_models.jl"))
include(Pkg.dir("AutomotiveDrivingModels", "src", "simulation", "simulation.jl"))

include(Pkg.dir("AutomotiveDrivingModels", "src", "behaviors", "behaviors.jl"))

include(Pkg.dir("AutomotiveDrivingModels", "src", "utils", "roadway_generation.jl"))
include(Pkg.dir("AutomotiveDrivingModels", "src", "utils", "trajdata_cleaning.jl"))

include(Pkg.dir("AutomotiveDrivingModels", "src", "evaluation", "metrics.jl"))
include(Pkg.dir("AutomotiveDrivingModels", "src", "evaluation", "foldsets.jl"))
include(Pkg.dir("AutomotiveDrivingModels", "src", "evaluation", "trajdata_segments.jl"))
include(Pkg.dir("AutomotiveDrivingModels", "src", "evaluation", "evaluation_data.jl"))

end # module
