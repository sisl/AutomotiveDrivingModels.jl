var documenterSearchIndex = {"docs": [

{
    "location": "api/#",
    "page": "List of functions",
    "title": "List of functions",
    "category": "page",
    "text": ""
},

{
    "location": "api/#AutomotiveDrivingModels.DIR_RIGHT",
    "page": "List of functions",
    "title": "AutomotiveDrivingModels.DIR_RIGHT",
    "category": "constant",
    "text": "LaneChangeChoice\n\nA choice of whether to change lanes, and what direction to do it in\n\n\n\n\n\n"
},

{
    "location": "api/#AutomotiveDrivingModels.AbstractFeature",
    "page": "List of functions",
    "title": "AutomotiveDrivingModels.AbstractFeature",
    "category": "type",
    "text": "Features can be extracted from QueueRecords.\n\nThey always return a FeatureValue, which allows the encoding of discrete / continuous / missing values, which can also be forced to a Float64.\n\n\n\n\n\n"
},

{
    "location": "api/#AutomotiveDrivingModels.CollisionCallback",
    "page": "List of functions",
    "title": "AutomotiveDrivingModels.CollisionCallback",
    "category": "type",
    "text": "CollisionCallback\n\nTerminates the simulation once a collision occurs\n\n\n\n\n\n"
},

{
    "location": "api/#AutomotiveDrivingModels.CurveProjection",
    "page": "List of functions",
    "title": "AutomotiveDrivingModels.CurveProjection",
    "category": "type",
    "text": "CurveProjection\n\nThe result of a point projected to a Curve\n\n\n\n\n\n"
},

{
    "location": "api/#AutomotiveDrivingModels.Frenet",
    "page": "List of functions",
    "title": "AutomotiveDrivingModels.Frenet",
    "category": "type",
    "text": "Frenet ______ roadind: road index s: distance along lane t: lane offset, positive is to left. zero point is the centerline of the lane. ϕ: lane relative heading\n\n\n\n\n\n"
},

{
    "location": "api/#AutomotiveDrivingModels.FrenetRelativePosition",
    "page": "List of functions",
    "title": "AutomotiveDrivingModels.FrenetRelativePosition",
    "category": "type",
    "text": "Project the given point to the same lane as the given RoadIndex.\n\nThis will return the projection of the point, along with the Δs along the lane from the RoadIndex.\n\n\n\n\n\n"
},

{
    "location": "api/#AutomotiveDrivingModels.IntelligentDriverModel",
    "page": "List of functions",
    "title": "AutomotiveDrivingModels.IntelligentDriverModel",
    "category": "type",
    "text": "Commonly referred to as IDM\n\n\n\n\n\n"
},

{
    "location": "api/#AutomotiveDrivingModels.LaneFollowingAccel",
    "page": "List of functions",
    "title": "AutomotiveDrivingModels.LaneFollowingAccel",
    "category": "type",
    "text": "LaneFollowingAccel\n\nLongitudinal acceleration\n\n\n\n\n\n"
},

{
    "location": "api/#AutomotiveDrivingModels.LatLonAccel",
    "page": "List of functions",
    "title": "AutomotiveDrivingModels.LatLonAccel",
    "category": "type",
    "text": "LatLonAccel\n\nAcceleration in the frenet frame\n\n\n\n\n\n"
},

{
    "location": "api/#AutomotiveDrivingModels.MOBIL",
    "page": "List of functions",
    "title": "AutomotiveDrivingModels.MOBIL",
    "category": "type",
    "text": "MOBIL\n\nSee Treiber & Kesting, \'Modeling Lane-Changing Decisions with MOBIL\'\n\n\n\n\n\n"
},

{
    "location": "api/#AutomotiveDrivingModels.NeighborLongitudinalResult",
    "page": "List of functions",
    "title": "AutomotiveDrivingModels.NeighborLongitudinalResult",
    "category": "type",
    "text": "get_neighbor_index_fore(scene::Scene, vehicle_index::Int, roadway::Roadway)\n\nReturn the index of the vehicle that is in the same lane as scene[vehicle_index] and in front of it with the smallest distance along the lane\n\nThe method will search on the current lane first, and if no vehicle is found it\nwill continue to travel along the lane following next_lane(lane, roadway).\nIf no vehicle is found within `max_distance_fore,` a value of `nothing` is returned instead.\n\n\n\n\n\n"
},

{
    "location": "api/#AutomotiveDrivingModels.PedestrianLatLonAccel",
    "page": "List of functions",
    "title": "AutomotiveDrivingModels.PedestrianLatLonAccel",
    "category": "type",
    "text": "PedestrianLatLonAccel\n\nPedestrian walking action. Acceleration in the Frenet frame, along with desired lane after crossing the street.\n\n\n\n\n\n"
},

{
    "location": "api/#AutomotiveDrivingModels.SidewalkPedestrianModel",
    "page": "List of functions",
    "title": "AutomotiveDrivingModels.SidewalkPedestrianModel",
    "category": "type",
    "text": "SidewalkPedestrianModel\n\nWalks along the sidewalk until approaching the crosswalk. Waits for the cars to pass, then crosses.\n\n\n\n\n\n"
},

{
    "location": "api/#AutomotiveDrivingModels.VehicleDef",
    "page": "List of functions",
    "title": "AutomotiveDrivingModels.VehicleDef",
    "category": "type",
    "text": "Vehicle definition which contains a class and a bounding box.\n\n\n\n\n\n"
},

{
    "location": "api/#AutomotiveDrivingModels.gen_stadium_roadway-Tuple{Int64}",
    "page": "List of functions",
    "title": "AutomotiveDrivingModels.gen_stadium_roadway",
    "category": "method",
    "text": "gen_stadium_roadway(nlanes::Int, length::Float64)\n\nGenerate a roadway that is a rectangular racetrack with rounded corners.     length = length of the x-dim straight section for the innermost (leftmost) lane [m]     width  = length of the y-dim straight section for the innermost (leftmost) lane [m]     radius = turn radius [m]\n\n  ______________________\n /                      \\ \n|                        |\n|                        |\n\\______________________/\n\n\n\n\n\n"
},

{
    "location": "api/#AutomotiveDrivingModels.gen_straight_roadway",
    "page": "List of functions",
    "title": "AutomotiveDrivingModels.gen_straight_roadway",
    "category": "function",
    "text": "gen_straight_roadway(nlanes::Int, length::Float64)\n\nGenerate a roadway with a single straight segment whose rightmost lane center starts at starts at (0,0), and proceeds in the positive x direction.\n\n\n\n\n\n"
},

{
    "location": "api/#AutomotiveDrivingModels.get_curve_index-Tuple{Array{CurvePt,1},Float64}",
    "page": "List of functions",
    "title": "AutomotiveDrivingModels.get_curve_index",
    "category": "method",
    "text": "get_curve_index(curve::Curve, s::Float64)\n\nReturn the CurveIndex for the closest s-location on the curve\n\n\n\n\n\n"
},

{
    "location": "api/#AutomotiveDrivingModels.get_curve_index-Tuple{CurveIndex,Array{CurvePt,1},Float64}",
    "page": "List of functions",
    "title": "AutomotiveDrivingModels.get_curve_index",
    "category": "method",
    "text": "get_curve_index(ind::CurveIndex, curve::Curve, Δs::Float64)\n\nReturn the CurveIndex at ind\'s s position + Δs\n\n\n\n\n\n"
},

{
    "location": "api/#AutomotiveDrivingModels.get_first_collision-Union{Tuple{I}, Tuple{D}, Tuple{S}, Tuple{Frame{Entity{S,D,I}},AbstractArray{Int64,1}}, Tuple{Frame{Entity{S,D,I}},AbstractArray{Int64,1},CPAMemory}} where I where D<:Union{BicycleModel, VehicleDef} where S<:VehicleState",
    "page": "List of functions",
    "title": "AutomotiveDrivingModels.get_first_collision",
    "category": "method",
    "text": "Loops through the scene and finds the first collision between any two vehicles\n\n\n\n\n\n"
},

{
    "location": "api/#AutomotiveDrivingModels.get_first_collision-Union{Tuple{I}, Tuple{D}, Tuple{S}, Tuple{Frame{Entity{S,D,I}},Int64}, Tuple{Frame{Entity{S,D,I}},Int64,CPAMemory}} where I where D<:Union{BicycleModel, VehicleDef} where S<:VehicleState",
    "page": "List of functions",
    "title": "AutomotiveDrivingModels.get_first_collision",
    "category": "method",
    "text": "Loops through the scene and finds the first collision between a vehicle and scene[target_index]\n\n\n\n\n\n"
},

{
    "location": "api/#AutomotiveDrivingModels.get_lerp_time-Tuple{VecE2,VecE2,VecE2}",
    "page": "List of functions",
    "title": "AutomotiveDrivingModels.get_lerp_time",
    "category": "method",
    "text": "get_lerp_time(A::VecE2, B::VecE2, Q::VecE2)\n\nGet lerp time t∈[0,1] such that lerp(A, B) is as close as possible to Q\n\n\n\n\n\n"
},

{
    "location": "api/#AutomotiveDrivingModels.is_potentially_colliding-Tuple{Entity{VehicleState,VehicleDef,Int64},Entity{VehicleState,VehicleDef,Int64}}",
    "page": "List of functions",
    "title": "AutomotiveDrivingModels.is_potentially_colliding",
    "category": "method",
    "text": "A fast collision check to remove things clearly not colliding\n\n\n\n\n\n"
},

{
    "location": "api/#AutomotiveDrivingModels.move_along",
    "page": "List of functions",
    "title": "AutomotiveDrivingModels.move_along",
    "category": "function",
    "text": "move_along(roadind::RoadIndex, road::Roadway, Δs::Float64)\n\nReturn the RoadIndex at ind\'s s position + Δs\n\n\n\n\n\n"
},

{
    "location": "api/#AutomotiveDrivingModels.propagate-Union{Tuple{R}, Tuple{A}, Tuple{I}, Tuple{D}, Tuple{S}, Tuple{Entity{S,D,I},A,R,Float64}} where R where A where I where D where S",
    "page": "List of functions",
    "title": "AutomotiveDrivingModels.propagate",
    "category": "method",
    "text": "propagate(veh, action, roadway, Δt)\n\nTake an entity of type {S,D,I} and move it over Δt seconds to produce a new entity based on the action on the given roadway.\n\n\n\n\n\n"
},

{
    "location": "api/#AutomotiveDrivingModels.read_dxf-Tuple{IO,Type{Roadway}}",
    "page": "List of functions",
    "title": "AutomotiveDrivingModels.read_dxf",
    "category": "method",
    "text": "read_dxf(io::IO, ::Type{Roadway})\n\nReturn a Roadway generated from a DXF file\n\nLayers with names such as seg001 will contain LWPOLYLINEs.\nEach LWPOLYLINE corresponds to a lane centerline, which together\nare all neighbored.\n\n\n\n\n\n"
},

{
    "location": "api/#AutomotiveDrivingModels.simulate!-Union{Tuple{I}, Tuple{D}, Tuple{S}, Tuple{QueueRecord{Entity{S,D,I}},DriverModel,I,ListRecord{Entity{S,D,I},D1,I1} where I1 where D1,Int64,Int64}} where I where D where S",
    "page": "List of functions",
    "title": "AutomotiveDrivingModels.simulate!",
    "category": "method",
    "text": "Run a simulation and store the resulting scenes in the provided QueueRecord.\n\nOnly the ego vehicle is simulated; the other vehicles are as they were in the provided trajdata Other vehicle states will be interpolated\n\n\n\n\n\n"
},

{
    "location": "api/#AutomotiveDrivingModels.simulate!-Union{Tuple{M}, Tuple{R}, Tuple{A}, Tuple{I}, Tuple{D}, Tuple{S}, Tuple{Type{A},QueueRecord{Entity{S,D,I}},Frame{Entity{S,D,I}},R,Dict{I,M},Int64}} where M<:DriverModel where R where A where I where D where S",
    "page": "List of functions",
    "title": "AutomotiveDrivingModels.simulate!",
    "category": "method",
    "text": "Run nticks of simulation and place all nticks+1 scenes into the QueueRecord\n\n\n\n\n\n"
},

{
    "location": "api/#Vec.lerp-Tuple{VecE2,VecE2,VecE2,Float64}",
    "page": "List of functions",
    "title": "Vec.lerp",
    "category": "method",
    "text": "quadratic bezier lerp\n\n\n\n\n\n"
},

{
    "location": "api/#Vec.lerp-Tuple{VecE2,VecE2,VecE2,VecE2,Float64}",
    "page": "List of functions",
    "title": "Vec.lerp",
    "category": "method",
    "text": "cubic bezier lerp\n\n\n\n\n\n"
},

{
    "location": "api/#Vec.proj-Tuple{VecSE2{Float64},Array{CurvePt,1}}",
    "page": "List of functions",
    "title": "Vec.proj",
    "category": "method",
    "text": "Vec.proj(posG::VecSE2{Float64}, curve::Curve)\n\nReturn a CurveProjection obtained by projecting posG onto the curve\n\n\n\n\n\n"
},

{
    "location": "api/#Vec.proj-Tuple{VecSE2{Float64},Lane,Roadway}",
    "page": "List of functions",
    "title": "Vec.proj",
    "category": "method",
    "text": "proj(posG::VecSE2{Float64}, lane::Lane, roadway::Roadway)\n\nReturn the RoadProjection for projecting posG onto the lane. This will automatically project to the next or prev curve as appropriate.\n\n\n\n\n\n"
},

{
    "location": "api/#Vec.proj-Tuple{VecSE2{Float64},RoadSegment,Roadway}",
    "page": "List of functions",
    "title": "Vec.proj",
    "category": "method",
    "text": "proj(posG::VecSE2{Float64}, seg::RoadSegment, roadway::Roadway)\n\nReturn the RoadProjection for projecting posG onto the segment. Tries all of the lanes and gets the closest one\n\n\n\n\n\n"
},

{
    "location": "api/#Vec.proj-Tuple{VecSE2{Float64},Roadway}",
    "page": "List of functions",
    "title": "Vec.proj",
    "category": "method",
    "text": "proj(posG::VecSE2{Float64}, seg::RoadSegment, roadway::Roadway)\n\nReturn the RoadProjection for projecting posG onto the roadway. Tries all of the lanes and gets the closest one\n\n\n\n\n\n"
},

{
    "location": "api/#AutomotiveDrivingModels.CrossingPhase",
    "page": "List of functions",
    "title": "AutomotiveDrivingModels.CrossingPhase",
    "category": "type",
    "text": "CrossingPhase\n\nCrossing phases for SidewalkPedestrianModel.\n\nFor a crossing pedestrian, phases go: -2, -1, 0, 1 For a non-crossing pedestrian that pauses at the crosswalk, phases go: -2, -1, 1 For a non-crossing pedestrian that does not pause at the crosswalk, phases go: -2, 1\n\nModel based on Feliciani et al (2017) - A simulation model for non-signalized pedestrian crosswalks based on evidence from on field observation.\n\n\n\n\n\n"
},

{
    "location": "api/#AutomotiveDrivingModels.get_lerp_time_unclamped-Tuple{VecE2,VecE2,VecE2}",
    "page": "List of functions",
    "title": "AutomotiveDrivingModels.get_lerp_time_unclamped",
    "category": "method",
    "text": "get_lerp_time_unclamped(A::VecE2, B::VecE2, Q::VecE2)\n\nGet the interpolation scalar t for the point on the line AB closest to Q This point is P = A + (B-A)*t\n\n\n\n\n\n"
},

{
    "location": "api/#Base.getindex-Tuple{Lane,CurveIndex,Roadway}",
    "page": "List of functions",
    "title": "Base.getindex",
    "category": "method",
    "text": "lane[ind::CurveIndex, roadway::Roadway]\n\nAccessor for lanes based on a CurveIndex. Note that we extend the definition of a CurveIndex, previously ind.i ∈ [1, length(curve)-1], to:\n\nind.i ∈ [0, length(curve)]\n\nwhere 1 ≤ ind.i ≤ length(curve)-1 is as before, but if the index is on the section between two lanes, we use:\n\nind.i = length(curve), ind.t ∈ [0,1] for the region between curve[end] → next\nind.i = 0,             ind.t ∈ [0,1] for the region between prev → curve[1]\n\n\n\n\n\n"
},

{
    "location": "api/#List-of-functions-1",
    "page": "List of functions",
    "title": "List of functions",
    "category": "section",
    "text": "Modules = [AutomotiveDrivingModels]"
},

{
    "location": "#",
    "page": "About",
    "title": "About",
    "category": "page",
    "text": ""
},

{
    "location": "#About-1",
    "page": "About",
    "title": "About",
    "category": "section",
    "text": "This is the documentation for AutomotiveDrivingModels.jl. Placeholder for now"
},

]}
