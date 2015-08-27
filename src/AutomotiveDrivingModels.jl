module AutomotiveDrivingModels

using Reexport
# using StreamStats

@reexport using Vec
@reexport using DataFrames
@reexport using Discretizers
@reexport using Distributions
@reexport using JLD
@reexport using LaTeXStrings

include("utils/CommonTypes.jl")
@reexport using .CommonTypes

include("utils/Curves.jl")
@reexport using .Curves

include("utils/Trajdata.jl")
@reexport using .Trajdata

include("utils/StreetNetworks.jl")
@reexport using .StreetNetworks

include("features/Features.jl")
@reexport using .Features

include("utils/FeaturesetExtractor.jl")
@reexport using .FeaturesetExtractor

include("utils/PrimaryDataExtractor.jl")
@reexport using .PrimaryDataExtractor

include("utils/ValidationTraceExtractor.jl")
@reexport using .ValidationTraceExtractor

import Base: get, ==

include("utils/polynomials.jl")
include("utils/common.jl")
include("utils/collision.jl")
include("behaviors/behaviors.jl")
include("io/io.jl")
include("simulation/candidate_trajectories.jl")
include("simulation/simulation.jl")
include("evaluation/sim_param_calibration.jl")
include("evaluation/sim_metrics.jl")
include("evaluation/risk_estimation.jl")
include("evaluation/model_evaluation.jl")

include("behaviors/drive_straight.jl")
include("behaviors/behavior_gaussian.jl")

end # module