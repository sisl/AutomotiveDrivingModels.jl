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
    NULL_ROADINDEX,

    has_next,
    has_prev,
    connect!,
    prev,
    project_to_closest_lane,
    move_along,
    n_lanes_left,
    n_lanes_right,
    get_neighbor_lanetag_left,
    get_neighbor_lanetag_right,

    # vehicles
    Frenet,
    VehicleState,
    Vehicle,
    AgentClass,

    NULL_FRENET,

    get_vel_s,
    get_vel_t,
    get_footpoint,
    get_center



include(Pkg.dir("AutomotiveDrivingModels", "src", "core", "utils.jl"))
include(Pkg.dir("AutomotiveDrivingModels", "src", "core", "curves.jl"))
include(Pkg.dir("AutomotiveDrivingModels", "src", "core", "roadway.jl"))
include(Pkg.dir("AutomotiveDrivingModels", "src", "core", "vehicles.jl"))

end # module