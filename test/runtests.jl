using Base.Test

using AutomotiveDrivingModels

include("roadways/runtests.jl")
include("typing/PosSpeed1D_BoundingBoxDef_Int/runtests.jl")

using NBInclude
nbinclude(joinpath(dirname(@__FILE__), "..", "docs", "1DMobius.ipynb"))
nbinclude(joinpath(dirname(@__FILE__), "..", "docs", "2DStadium.ipynb"))
nbinclude(joinpath(dirname(@__FILE__), "..", "docs", "Crosswalk.ipynb"))