module AutomotiveDrivingModels 

using Reexport
@reexport using Vec 
@reexport using Records
using Printf
using LinearAlgebra

# Roadways

export StraightRoadway,
       mod_position_to_roadway,
       get_headway
       
include("roadways/straight_1d_roadways.jl")

export CurvePt,
       Curve,
       CurveIndex,
       CurveProjection,
       CURVEINDEX_START,
       get_lerp_time,
       index_closest_to_point,
       get_curve_index,
       curveindex_end

include("roadways/curves.jl")

end # AutomotiveDrivingModels