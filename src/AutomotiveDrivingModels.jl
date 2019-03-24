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

## VehicleDefinition

export
    AbstractAgentDefinition,
    AgentClass,
    VehicleDef

include("agent-definitions/agent_definitions.jl")


## STATES

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


end # AutomotiveDrivingModels