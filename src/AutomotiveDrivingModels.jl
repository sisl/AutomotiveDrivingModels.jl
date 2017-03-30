__precompile__()

module AutomotiveDrivingModels

# using Compat
using Reexport
# using Discretizers
using Parameters

@reexport using Records
@reexport using Distributions
@reexport using Vec
# @reexport using DataFrames


include("simulation/main.jl") # core interface
include("mobius/main.jl") # 1D interface
include("roadway/main.jl") # Roadways (based on the RNDF format)
include("vehicles/main.jl") # Vehicles (2D interface based on Roadways)
include("lane_following_drivers/main.jl") # Vehicles (2D interface based on Roadways)



# include(Pkg.dir("AutomotiveDrivingModels", "src", "utils", "minkowski.jl"))
# include(Pkg.dir("AutomotiveDrivingModels", "src", "features", "features.jl"))
# include(Pkg.dir("AutomotiveDrivingModels", "src", "features", "feature_extractors.jl"))
# include(Pkg.dir("AutomotiveDrivingModels", "src", "features", "aggregate.jl"))

# include(Pkg.dir("AutomotiveDrivingModels", "src", "simulation", "actions.jl"))
# include(Pkg.dir("AutomotiveDrivingModels", "src", "behaviors", "behaviors.jl"))

# include(Pkg.dir("AutomotiveDrivingModels", "src", "utils", "trajdata_cleaning.jl"))

# include(Pkg.dir("AutomotiveDrivingModels", "src", "evaluation", "metrics.jl"))
# include(Pkg.dir("AutomotiveDrivingModels", "src", "evaluation", "foldsets.jl"))
# include(Pkg.dir("AutomotiveDrivingModels", "src", "evaluation", "fold_assigners.jl"))
# include(Pkg.dir("AutomotiveDrivingModels", "src", "evaluation", "trajdata_segments.jl"))
# include(Pkg.dir("AutomotiveDrivingModels", "src", "evaluation", "evaluation_data.jl"))

end # module
