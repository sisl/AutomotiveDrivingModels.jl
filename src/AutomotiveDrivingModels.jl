module AutomotiveDrivingModels 

using Printf
using LinearAlgebra
using Parameters
using StaticArrays
using Distributions
using Reexport
@reexport using Vec 
@reexport using Records

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

export     
    LaneTag,
    LaneBoundary,
    Lane,
    LaneConnection,
    SpeedLimit,
    RoadSegment,
    Roadway,
    RoadProjection,
    RoadIndex,

    NULL_BOUNDARY,
    NULL_LANETAG,
    NULL_ROADINDEX,

    DEFAULT_SPEED_LIMIT,
    DEFAULT_LANE_WIDTH,

    is_in_exits,
    is_in_entrances,
    is_at_curve_end,
    is_between_segments_lo,
    is_between_segments_hi,
    is_between_segments,
    has_segment,
    has_lanetag,
    has_next,
    has_prev,
    next_lane,
    prev_lane,
    next_lane_point,
    prev_lane_point,
    connect!,
    prev,
    project_to_closest_lane,
    move_along,
    n_lanes_left,
    n_lanes_right,
    get_neighbor_lanetag_left,
    get_neighbor_lanetag_right,
    read_dxf

include("roadways/roadways.jl")

export 
    Frenet,
    get_posG,
    NULL_FRENET

include("roadways/frenet.jl")

export
        gen_straight_curve,
        gen_straight_segment,
        gen_straight_roadway,
        gen_stadium_roadway,
        gen_bezier_curve

include("roadways/roadway_generation.jl")

## agent definitions

export
    AbstractAgentDefinition,
    AgentClass,
    VehicleDef

include("agent-definitions/agent_definitions.jl")


## states

export
    State1D,
    Vehicle1D,
    Scene1D

include("states/1d_states.jl")

export
    VehicleState,
    Vehicle,
    get_vel_s,
    get_vel_t

include("states/vehicle_state.jl")

export
    TrajdataFrame,
    TrajdataState,
    Trajdata,
    TrajdataVehicleIterator

include("states/trajdatas.jl")

export
    Scene,
    SceneRecord

include("states/scenes.jl")


## Collision Checkers

export
    ConvexPolygon,
    CPAMemory,
    CollisionCheckResult,
    LineSegment,
    to_oriented_bounding_box!,
    get_oriented_bounding_box,
    is_colliding,
    is_potentially_colliding,
    get_collision_time,
    get_first_collision,
    get_time_and_dist_of_closest_approach,
    is_collision_free,
    get_distance,
    get_edge,
    get_side,
    collision_checker

include("collision-checkers/minkowski.jl")
include("collision-checkers/parallel_axis.jl")


## Actions 

export
    propagate,
    LaneFollowingAccel,
    AccelTurnrate,
    AccelDesang,
    LatLonAccel,
    AccelSteeringAngle,
    PedestrianLatLonAccel

include("actions/interface.jl")
include("actions/lane_following_accel.jl")
include("actions/accel_turnrate.jl")
include("actions/accel_desang.jl")
include("actions/lat_lon_accel.jl")
include("actions/accel_steering.jl")
include("actions/pedestrian_lat_lon_accel.jl")

export
    DriverModel,
    StaticDriver,
    get_name,
    action_type,
    set_desired_speed!,
    observe!,
    reset_hidden_state!,
    prime_with_history!

include("behaviors/interface.jl")

export
    LaneFollowingDriver,
    StaticLaneFollowingDriver,
    PrincetonDriver,
    IntelligentDriverModel,
    ProportionalSpeedTracker,
    track_longitudinal!,
    LateralDriverModel,
    ProportionalLaneTracker,
    LatLonSeparableDriver,
    Tim2DDriver,
    track_lane!,
    SidewalkPedestrianModel,
    LaneChangeChoice,
    LaneChangeModel,
    get_lane_offset,
    DIR_RIGHT,
    DIR_MIDDLE,
    DIR_LEFT,
    MOBIL,
    TimLaneChanger

include("behaviors/lane_following_drivers.jl")
include("behaviors/princeton_driver.jl")
include("behaviors/speed_trackers.jl")
include("behaviors/intelligent_driver_model.jl")
include("behaviors/lateral_driver_models.jl")
include("behaviors/lane_change_models.jl")
include("behaviors/MOBIL.jl")
include("behaviors/tim_lane_changer.jl")
include("behaviors/lat_lon_separable_driver.jl")
include("behaviors/tim_2d_driver.jl")
include("behaviors/sidewalk_pedestrian_model.jl")


end # AutomotiveDrivingModels