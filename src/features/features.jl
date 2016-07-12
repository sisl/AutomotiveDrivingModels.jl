export
    AbstractFeature,
    FeatureValue,
    FeatureState,

    is_feature_valid,
    is_symbol_a_feature,
    allfeatures,
    symbol2feature


# export POSFYAW, POSFT, SPEED, VELBX, VELBY, VELFS, VELFT, SCENE_SPEED_DIFFERENCE
# export TURNRATE, ACC, ACCFS, ACCFT, ACCBX, ACCBY, JERK
# export MARKERDIST_LEFT, MARKERDIST_RIGHT, DIST_FROM_CENTERLINE
# export DIST_MERGE, DIST_SPLIT
# export SUMO, IDM
# export HAS_FRONT, DIST_FRONT, D_Y_FRONT, DELTA_V_FRONT, DELTA_V_Y_FRONT, YAW_FRONT, TURNRATE_FRONT, ACC_REQ_FRONT, INV_TTC_FRONT, INV_TIMEGAP_FRONT, GAINING_ON_FRONT
# export HAS_REAR,  DIST_REAR,  D_Y_REAR,  DELTA_V_REAR,  DELTA_V_Y_REAR,  YAW_REAR,  TURNRATE_REAR,  ACC_REQ_REAR,  INV_TTC_REAR,  INV_TIMEGAP_REAR,  REAR_IS_GAINING
# export HAS_LEFT,  DIST_LEFT,  D_Y_LEFT,  DELTA_V_LEFT,  DELTA_V_Y_LEFT,  YAW_LEFT,  TURNRATE_LEFT,  ACC_REQ_LEFT,  INV_TTC_LEFT,  INV_TIMEGAP_LEFT,  LEFT_IS_GAINING
# export HAS_RIGHT, DIST_RIGHT, D_Y_RIGHT, DELTA_V_RIGHT, DELTA_V_Y_RIGHT, YAW_RIGHT, TURNRATE_RIGHT, ACC_REQ_RIGHT, INV_TTC_RIGHT, INV_TIMEGAP_RIGHT, RIGHT_IS_GAINING
# export TIMETOCROSSING_LEFT, TIMETOCROSSING_RIGHT, ESTIMATEDTIMETOLANECROSSING, A_REQ_STAYINLANE
# export N_LANE_LEFT, N_LANE_RIGHT, HAS_LANE_RIGHT, HAS_LANE_LEFT, LANECURVATURE
# export TIME_CONSECUTIVE_BRAKE, TIME_CONSECUTIVE_ACCEL, TIME_CONSECUTIVE_THROTTLE
# export FUTUREACCELERATION, FUTUREDESIREDANGLE, FUTURETURNRATE, FUTUREVELFT, FUTURE_DELTA_ACCEL
# export Feature_IsClean, Feature_Past
# export Feature_Mean_Over_History, Feature_Std_Over_History, Feature_Max_Over_History, Feature_Min_Over_History

# export
#     get_feature,
#     symbol2feature,
#     is_symbol_a_feature,
#     is_feature_na,
#     allfeatures,
#     units,
#     isint,
#     isbool,
#     upperbound,
#     lowerbound,
#     couldna,
#     replace_na,
#     lsymbol,
#     get_feature_derivative_backwards

"""
    Features can be extracted from SceneRecords.
They always return a FeatureValue, which allows the encoding of discrete / continuous / missing values,
which can also be forced to a Float64.
"""
abstract AbstractFeature

baremodule FeatureState
    # good
    const GOOD        = 0 # value is perfectly A-okay

    # caution
    const INSUF_HIST  = 1 # value best-guess was made due to insufficient history (ex, acceleration set to zero due to one timestamp)

    # bad (in these cases fval.v is typically set to NaN as well)
    const MISSING     = 2 # value is missing (no car in front, etc.)
    const CENSORED_HI = 3 # value is past an operating threshold
    const CENSORED_LO = 4 # value is below an operating threshold
end

immutable FeatureValue
    v::Float64 # feature value
    i::Int # used to encode

    FeatureValue(v::Float64, i::Int=FeatureState.GOOD) = new(v, i)
end

is_feature_valid(fval::FeatureValue) = fval.i == FeatureState.GOOD || fval.i == FeatureState.INSUF_HIST
Base.convert(::Type{Float64}, fval::FeatureValue) = fval.v

const SYMBOL_TO_FEATURE = Dict{Symbol, AbstractFeature}()

is_symbol_a_feature(sym::Symbol) = haskey(SYMBOL_TO_FEATURE, sym)
allfeatures() = values(SYMBOL_TO_FEATURE)
function symbol2feature(sym::Symbol)
    # if !haskey(SYMBOL_TO_FEATURE, sym)
    #     str = string(sym)
    #     if startswith(str, "isclean_")
    #         target = symbol(str[9:end])
    #         println("IsClean target: ", target)
    #         SYMBOL_TO_FEATURE[sym] = Feature_IsClean{target}()
    #     elseif ismatch(r"^past_(\d)+_", str)
    #         history = parse(Int, match(r"(\d)+", str).match)
    #         len_of_header = length(match(r"^past_(\d)+_", str).match)
    #         target = symbol(str[len_of_header+1:end])
    #         println("Past history, target: ", history, "  ", target)
    #         SYMBOL_TO_FEATURE[sym] = Feature_Past{target, history}()
    #     elseif ismatch(r"^mean_(\d)+_", str)
    #         history = parse(Int, match(r"(\d)+", str).match)
    #         len_of_header = length(match(r"^mean_(\d)+_", str).match)
    #         target = symbol(str[len_of_header+1:end])
    #         println("Mean over history, target: ", history, "  ", target)
    #         SYMBOL_TO_FEATURE[sym] = Feature_Mean_Over_History{target, history}()
    #     elseif ismatch(r"^std_(\d)+_", str)
    #         history = parse(Int, match(r"(\d)+", str).match)
    #         len_of_header = length(match(r"^std_(\d)+_", str).match)
    #         target = symbol(str[len_of_header+1:end])
    #         println("Std over history, target: ", history, "  ", target)
    #         SYMBOL_TO_FEATURE[sym] = Feature_Std_Over_History{target, history}()
    #     elseif ismatch(r"^max_(\d)+_", str)
    #         history = parse(Int, match(r"(\d)+", str).match)
    #         len_of_header = length(match(r"^max_(\d)+_", str).match)
    #         target = symbol(str[len_of_header+1:end])
    #         println("Max over history, target: ", history, "  ", target)
    #         SYMBOL_TO_FEATURE[sym] = Feature_Max_Over_History{target, history}()
    #     elseif ismatch(r"^min_(\d)+_", str)
    #         history = parse(Int, match(r"(\d)+", str).match)
    #         len_of_header = length(match(r"^min_(\d)+_", str).match)
    #         target = symbol(str[len_of_header+1:end])
    #         println("Min over history, target: ", history, "  ", target)
    #         SYMBOL_TO_FEATURE[sym] = Feature_Min_Over_History{target, history}()
    #     end
    # end

    SYMBOL_TO_FEATURE[sym]
end

# ----------------------------------

function generate_basic_feature_functions(
    name::AbstractString,
    sym::Symbol,
    inherent_type::DataType,
    units::AbstractString;
    lowerbound::Float64=-Inf,
    upperbound::Float64=Inf,
    can_be_missing::Bool=false,
    censor_lo::Float64=NaN,
    censor_hi::Float64=NaN,
    history::Int=1,
    )

    for feature in values(SYMBOL_TO_FEATURE)
        @assert(sym != symbol(feature), "symb: $name -> $feature")
    end
    @assert(lowerbound ≤ upperbound)

    feature_name = symbol("Feature_" * name)
    const_name   = symbol(uppercase(name))
    sym_feature  = Meta.quot(sym)

    @eval begin
        export $const_name
        immutable $feature_name <: AbstractFeature end
        const       $const_name  = ($feature_name)()
        units(          ::$feature_name)  = $(units)
        inherent_type(  ::$feature_name)  = $(inherent_type)
        lowerbound(     ::$feature_name)  = $(lowerbound)
        upperbound(     ::$feature_name)  = $(upperbound)
        can_be_missing( ::$feature_name)  = $(can_be_missing)
        history(        ::$feature_name)  = $(history)
        Base.symbol(    ::$feature_name)  = $sym_feature
        SYMBOL_TO_FEATURE[symbol($const_name)] = $const_name
    end
end

# ----------------------------------

generate_basic_feature_functions("PosFt", :posFt, Float64, "m")
Base.get(::Feature_PosFt, rec::SceneRecord, roadway::Roadway, vehicle_index::Int) = FeatureValue(rec[vehicle_index].state.posF.t)

generate_basic_feature_functions("PosFyaw", :posFyaw, Float64, "rad")
Base.get(::Feature_PosFyaw, rec::SceneRecord, roadway::Roadway, vehicle_index::Int) = FeatureValue(rec[vehicle_index].state.posF.ϕ)

generate_basic_feature_functions("Speed", :speed, Float64, "m/s")
Base.get(::Feature_Speed, rec::SceneRecord, roadway::Roadway, vehicle_index::Int) = FeatureValue(rec[vehicle_index].state.v)

generate_basic_feature_functions("VelFs", :velFs, Float64, "m/s")
function Base.get(::Feature_Speed, rec::SceneRecord, roadway::Roadway, vehicle_index::Int)
    veh = rec[vehicle_index]
    FeatureValue(veh.state.v*cos(veh.state.posF.ϕ))
end

generate_basic_feature_functions("VelFt", :velFt, Float64, "m/s")
function Base.get(::Feature_Speed, rec::SceneRecord, roadway::Roadway, vehicle_index::Int)
    veh = rec[vehicle_index]
    FeatureValue(veh.state.v*sin(veh.state.posF.ϕ))
end

generate_basic_feature_functions("MarkerDist_Left", :d_ml, Float64, "m")
function Base.get(::Feature_Speed, rec::SceneRecord, roadway::Roadway, vehicle_index::Int)
    veh = rec[vehicle_index]
    offset = veh.state.posF.t
    lane = roadway[veh.state.posF.roadind.tag]
    FeatureValue(lane.width/2 - offset)
end
generate_basic_feature_functions("MarkerDist_Right", :d_mr, Float64, "m")
function Base.get(::Feature_Speed, rec::SceneRecord, roadway::Roadway, vehicle_index::Int)
    veh = rec[vehicle_index]
    offset = veh.state.posF.t
    lane = roadway[veh.state.posF.roadind.tag]
    FeatureValue(lane.width/2 + offset)
end
generate_basic_feature_functions("RoadEdgeDist_Left", :d_edgel, Float64, "m")
function Base.get(::Feature_Speed, rec::SceneRecord, roadway::Roadway, vehicle_index::Int)
    veh = rec[vehicle_index]
    offset = veh.state.posF.t
    footpoint = get_footpoint(veh)
    seg = roadway[veh.state.posF.roadind.tag.segment]
    lane = seg.lanes[end]
    roadproj = proj(footpoint, lane, roadway)
    curvept = roadway(RoadIndex(roadproj))
    lane = roadway[roadproj.tag]
    FeatureValue(lane.width/2 + abs(curvept.pos - footpoint) - offset)
end
generate_basic_feature_functions("RoadEdgeDist_Right", :d_edger, Float64, "m")
function Base.get(::Feature_Speed, rec::SceneRecord, roadway::Roadway, vehicle_index::Int)
    veh = rec[vehicle_index]
    offset = veh.state.posF.t
    footpoint = get_footpoint(veh)
    seg = roadway[veh.state.posF.roadind.tag.segment]
    lane = seg.lanes[1]
    roadproj = proj(footpoint, lane, roadway)
    curvept = roadway(RoadIndex(roadproj))
    lane = roadway[roadproj.tag]
    FeatureValue(lane.width/2 + abs(curvept.pos - footpoint) + offset)
end

generate_basic_feature_functions( "TimeToCrossing_Right", :ttcr_mr, Float64, "s", lowerbound=0.0, censor_hi=10.0)
function Base.get(::Feature_TimeToCrossing_Right, rec::SceneRecord, roadway::Roadway, vehicle_index::Int)
    d_mr = get(MARKERDIST_RIGHT, rec, roadway, vehicle_index).v
    velFt = get(VELFT, rec, roadway, vehicle_index).v

    if d_mr > 0.0 && velFt < 0.0
        FeatureValue(-d_mr / velFt)
    else
        FeatureValue(Inf)
    end
end
generate_basic_feature_functions( "TimeToCrossing_Left", :ttcr_ml, Float64, "s", lowerbound=0.0, censor_hi=10.0)
function Base.get(::Feature_TimeToCrossing_Left, rec::SceneRecord, roadway::Roadway, vehicle_index::Int)
    d_ml = get(MARKERDIST_RIGHT, rec, roadway, vehicle_index).v
    velFt = get(VELFT, rec, roadway, vehicle_index).v

    if d_ml > 0.0 && velFt < 0.0
        FeatureValue(-d_ml / velFs)
    else
        FeatureValue(Inf)
    end
end
generate_basic_feature_functions( "EstimatedTimeToLaneCrossing", :est_ttcr, Float64, "s", lowerbound=0.0, censor_hi=10.0)
function Base.get(::Feature_EstimatedTimeToLaneCrossing, rec::SceneRecord, roadway::Roadway, vehicle_index::Int)
    ttcr_left = get(TIMETOCROSSING_LEFT, rec, roadway, vehicle_index).v
    ttcr_right = get(TIMETOCROSSING_RIGHT, runlog, sn, colset, frame).v
    FeatureValue(min(ttcr_left, ttcr_right))
end
generate_basic_feature_functions( "A_REQ_StayInLane", :a_req_stayinlane, Float64, "m/s²", can_be_missing=true)
function Base.get(::Feature_A_REQ_StayInLane, rec::SceneRecord, roadway::Roadway, vehicle_index::Int)
    velFt = get(VELFT, rec, roadway, vehicle_index).v

    if velFt > 0.0
        d_mr = get(MARKERDIST_RIGHT, rec, roadway, vehicle_index).v
        if d_mr > 0.0
            return FeatureValue(0.5velFt*velFt / d_mr)
        else
            return FeatureValue(NaN, FeatureState.MISSING)
        end
    else
        d_ml = get(MARKERDIST_LEFT, rec, roadway, vehicle_index)
        if d_ml < 0.0
            return FeatureValue(-0.5velFt*velFt / d_ml)
        else
            return FeatureValue(NaN, FeatureState.MISSING)
        end
    end
end