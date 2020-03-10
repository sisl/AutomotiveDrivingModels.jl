module AutomotiveDrivingModels 

using Printf
using LinearAlgebra
using Parameters
using StaticArrays
using Distributions
using Reexport
using Random

@reexport using Records

include("vec/Vec.jl")
using .Vec
export
    AbstractVec,
    VecE,  # an abstract euclidean-group vector
    VecSE, # an abstract special euclidean-group element

    VecE2, # two-element Float64 vector, {x, y}
    VecE3, # three-element Float64 vector, {x, y, z}
    VecSE2, # point in special euclidean group of order 2, {x, y, θ}

    polar, # construct a polar vector with (r,θ)

    proj,  # vector projection
           # proj(a::vec, b::vec, ::Type{Float64}) will do scalar projection of a onto b
           # proj(a::vec, b::vec, ::Type{Vec}) will do vector projection of a onto b

    lerp,  # linear interpolation between two vec's
    invlerp,
    lerp_angle,

    normsquared,         # see docstrings below
    scale_euclidean,
    clamp_euclidean,
    normalize_euclidian,

    dist,  # scalar distance between two vec's
    dist2, # squared scalar distance between two vec's

    rot,   # rotate the vector, always using the Right Hand Rule
    rot_normalized, # like rot, but assumes axis is normalized

    deltaangle, # signed delta angle
    angledist,  # distance between two angles

    inertial2body,
    body2inertial,

    orientation,
    are_collinear,
    get_intersection,

    Circ,
    AABB,
    OBB

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
    move_along,
    n_lanes_left,
    n_lanes_right,
    lanes,
    lanetags,
    read

include("roadways/roadways.jl")

export 
    Frenet,
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
    VehicleDef,
    BicycleModel,
    length,
    width,
    class

include("agent-definitions/agent_definitions.jl")

export
    posf,
    posg,
    vel,
    velf,
    velg,
    VehicleState,
    Vehicle,
    get_vel_s,
    get_vel_t,
    get_center,
    get_footpoint,
    get_front,
    get_rear,
    get_lane

include("states/interface.jl")
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
    to_oriented_bounding_box!,
    get_oriented_bounding_box,
    is_colliding,
    is_potentially_colliding,
    get_collision_time,
    get_first_collision,
    is_collision_free,
    get_distance,
    get_edge,
    collision_checker,
    polygon

include("collision-checkers/minkowski.jl")
include("collision-checkers/parallel_axis.jl")

## Feature Extraction

export
    VehicleTargetPoint,
    VehicleTargetPointFront,
    VehicleTargetPointCenter,
    VehicleTargetPointRear,
    get_targetpoint_delta,
    NeighborLongitudinalResult,
    get_neighbor_fore,
    get_neighbor_rear,
    get_headway,
    get_neighbor_fore_along_lane,
    get_neighbor_fore_along_left_lane,
    get_neighbor_fore_along_right_lane,
    get_neighbor_rear_along_lane,
    get_neighbor_rear_along_left_lane,
    get_neighbor_rear_along_right_lane,
    FrenetRelativePosition,
    get_frenet_relative_position,
    get_lane_width,
    get_markerdist_left,
    get_markerdist_right

include("feature-extraction/neighbors_features.jl")
include("feature-extraction/lane_features.jl")

export 
    AbstractFeature,
    FeatureValue,
    FeatureState,
    is_feature_valid,
    is_symbol_a_feature,
    allfeatures,
    symbol2feature,
    AbstractFeatureExtractor,
    FeatureExtractor,
    SubsetExtractor,
    StandardizingExtractor,
    pull_features!,
    rec_length

include("feature-extraction/interface.jl")
include("feature-extraction/features.jl")
include("feature-extraction/features_extractors.jl")

export
    LidarSensor,
    nbeams,
    observe!,
    RoadlineLidarSensor,
    nlanes,
    LanePortion,
    RoadwayLidarCulling,
    ensure_leaf_in_rlc!,
    get_lane_portions

include("feature-extraction/lidar_sensor.jl")

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

export 
    get_actions!,
    tick!,
    reset_hidden_states!,
    simulate,
    simulate!,
    EntityAction,
    run_callback,
    CollisionCallback

include("simulation/simulation.jl")
include("simulation/callbacks.jl")


export
    State1D,
    Vehicle1D,
    Scene1D

include("deprecated.jl")

end # AutomotiveDrivingModels
