using Base.Test

using AutomotiveDrivingModels

include("interface/runtests.jl")
include("2d/runtests.jl")

# using NBInclude
# nbinclude(joinpath(dirname(@__FILE__), "..", "docs", "1DMobius.ipynb"))
# nbinclude(joinpath(dirname(@__FILE__), "..", "docs", "2DStadium.ipynb"))