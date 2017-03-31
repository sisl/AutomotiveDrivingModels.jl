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

export nframes


include("interface/main.jl") # core interface
include("1d/main.jl") # 1D interface
include("2d/main.jl") # 2D interface

end # module
