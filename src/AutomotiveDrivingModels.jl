module AutomotiveDrivingModels 

using Vec 
using Records

export StraightRoadway,
       mod_position_to_roadway,
       get_headway
       
include("roadways/straight_1d_roadways.jl")

end # AutomotiveDrivingModels