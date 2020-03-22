module AutomotiveDrivingModels 

@warn("This package is deprecated: use AutomotiveSimulator.jl instead.")

using Printf
using LinearAlgebra
using Parameters
using StaticArrays
using Distributions
using Reexport
using Random
using DataFrames
using Tricks: static_hasmethod

include(joinpath(@__DIR__, "Vec", "Vec.jl"))
@reexport using .Vec

# Roadways

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
    leftlane,
    n_lanes_right,
    rightlane,
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
    Entity,
    Scene,
    EntityScene,
    capacity,
    id2index,
    get_by_id,
    get_first_available_id,    
    posf,
    posg,
    vel,
    velf,
    velg,
    VehicleState,
    get_center,
    get_footpoint,
    get_front,
    get_rear,
    get_lane

include("states/entities.jl")
include("states/scenes.jl")
include("states/interface.jl")
include("states/vehicle_state.jl")

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

export AbstractFeature,
       EntityFeature,
       SceneFeature,
       TemporalFeature,
       extract_features,
       extract_feature,
       featuretype,

       # provided feature functions 
       posgx,
       posgy,
       posgθ,
       posfs,
       posft,
       posfϕ,
       vel,
       velfs,
       velft,
       velgx,
       velgy,
       time_to_crossing_right,
       time_to_crossing_left,
       estimated_time_to_lane_crossing,
       iswaiting,
       iscolliding,
       distance_to,
       acc,
       accfs,
       accft,
       jerk,
       jerkft,
       turn_rate_g,
       turn_rate_f,
       isbraking,
       isaccelerating

include("feature-extraction/features.jl")

export lane_width,
       markerdist_left,
       markerdist_right,
       road_edge_dist_left,
       road_edge_dist_right,
       lane_offset_left,
       lane_offset_right,
       has_lane_left,
       has_lane_right,
       lane_curvature

include("feature-extraction/lane_features.jl")

export
    VehicleTargetPoint,
    VehicleTargetPointFront,
    VehicleTargetPointCenter,
    VehicleTargetPointRear,
    targetpoint_delta,
    find_neighbor,
    NeighborLongitudinalResult,
    FrenetRelativePosition,
    get_frenet_relative_position,
    dist_to_front_neighbor,
    front_neighbor_speed,
    time_to_collision

include("feature-extraction/neighbors_features.jl")

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
    EntityAction,
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
    action_type,
    set_desired_speed!,
    observe!,
    reset_hidden_state!,
    reset_hidden_states!

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
    simulate,
    simulate!,
    run_callback,
    CollisionCallback,
    observe_from_history!,
    simulate_from_history!,
    simulate_from_history,
    maximum_entities

include("simulation/simulation.jl")
include("simulation/callbacks.jl")
include("simulation/simulation_from_history.jl")

end # AutomotiveDrivingModels
