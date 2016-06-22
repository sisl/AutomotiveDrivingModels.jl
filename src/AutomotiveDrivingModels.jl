VERSION >= v"0.4.0-dev+6521" && __precompile__(true)

module AutomotiveDrivingModels

using Reexport
# using StreamStats

@reexport using DataFrames
@reexport using Distributions
@reexport using Vec
# @reexport using Discretizers
# @reexport using JLD
# @reexport using LaTeXStrings

include("core/AutoCore.jl")
@reexport using .AutoCore

# include("utils/Curves.jl")
# @reexport using .Curves

# include("utils/StreetNetworks.jl")
# @reexport using .StreetNetworks

# include("utils/Trajdata.jl")
# @reexport using .Trajdata

# include("utils/RunLogs.jl")
# @reexport using .RunLogs

# include("features/Features.jl")
# @reexport using .Features

# include("utils/FeaturesetExtractor.jl")
# @reexport using .FeaturesetExtractor

# include("utils/PrimaryDataExtractor.jl")
# @reexport using .PrimaryDataExtractor

# include("utils/ValidationTraceExtractor.jl")
# @reexport using .ValidationTraceExtractor

# import Base: get, ==

# include("utils/polynomials.jl")
# include("utils/collision.jl")

# include("behaviors/behaviors.jl")

# include("io/io.jl")

# include("simulation/simulation.jl")

# include("evaluation/sim_param_calibration.jl")
# include("evaluation/sim_metrics.jl")
# include("evaluation/training.jl")

# include("behaviors/behavior_perfect.jl")
# include("behaviors/behavior_gaussian.jl")
# include("behaviors/behavior_linear_gaussian.jl")

end # module
