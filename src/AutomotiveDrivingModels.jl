__precompile__()

module AutomotiveDrivingModels

using Reexport
using Parameters

@reexport using Vec
@reexport using Records
@reexport using Distributions

export
    DriverModel,

    Vehicle,
    Scene,
    Trajdata,
    SceneRecord,

    LaneChangeChoice,
    LaneChangeModel,

    LaneFollowingDriver,
    StaticLaneFollowingDriver,
    StochasticLaneFollowingDriver,
    PrincetonDriver,
    IntelligentDriverModel,
    ProportionalSpeedTracker,
    WrappedLaneFollowingDriver,

    StaticDriver,
    LatLonSeparableDriver,
    ProportionalLaneTracker,
    Tim2DDriver,
    MOBIL,

    track_longitudinal!,
    set_desired_speed!,

    get_name,
    action_type,
    observe!,
    get_actions!,
    tick!,
    simulate!,
    reset_hidden_states!,
    prime_with_history!,
    run_callback,

    AgentClass,

    PosSpeed1D,
    Frenet,
    RoadwayState,

    BoundingBoxDef,

    Accel,
    AccelDesang,
    AccelSteering,
    AccelTurnrate,
    LatLonAccel,
    StoppingAccel,

    AbstractFeature,
    FeatureValue,
    FeatureState,
    NeighborLongitudinalResult,
    LeadFollowRelationships,

    is_feature_valid,
    is_symbol_a_feature,
    allfeatures,
    symbol2feature,

    is_colliding,
    has_collision,
    get_first_collision,

    LaneBoundary,
    SpeedLimit,
    LaneTag,

    NULL_BOUNDARY,
    NULL_SPEED_LIMIT,
    NULL_LANETAG,

    LaneGeometry,
    GeomLine,
    GeomArc,
    LaneGeometries,
    LaneGeometryRecord,

    Straight1DRoadway,
    Wraparound,
    Roadway,

    DEFAULT_LANE_WIDTH,

    get_s_max,
    mod_position_to_roadway,
    get_lane_offset,

    get_center,
    get_footpoint,
    get_front,
    get_rear,
    get_headway,
    get_neighbor_fore,
    get_neighbor_rear,

    gen_straight_curve,
    gen_straight_segment,
    gen_straight_roadway,
    gen_stadium_roadway,
    gen_bezier_curve


include("drivermodels.jl")
include("simulation.jl")
include("callbacks.jl")
include("features.jl")

include("roadways/main.jl")
include("states/main.jl")
include("defs/main.jl")
include("actions/main.jl")
include("behaviors/main.jl")
include("typing/main.jl")

end # module
