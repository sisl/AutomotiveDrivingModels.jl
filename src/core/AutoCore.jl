"""
    AutoCore
Defines the core Automotive Types
"""
module AutoCore

using Vec

export

    # curves
    CurvePt,
    Curve,
    CurveIndex,
    CurveProjection,

    get_lerp_time,
    index_closest_to_point,
    get_curve_index,

    # roadway
    LaneTag,
    LaneBoundary,
    Lane,
    RoadSegment,
    Roadway,
    RoadProjection,
    RoadIndex,

    NULL_BOUNDARY,
    NULL_LANETAG,

    has_next,
    has_prev,
    connect!,
    prev,
    project_to_closest_lane



include(Pkg.dir("AutomotiveDrivingModels", "src", "core", "utils.jl"))
include(Pkg.dir("AutomotiveDrivingModels", "src", "core", "curves.jl"))
include(Pkg.dir("AutomotiveDrivingModels", "src", "core", "roadway.jl"))

end # module