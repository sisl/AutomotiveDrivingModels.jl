export
    AbstractFeature,
    FeatureValue,
    FeatureState,

    is_feature_valid,
    is_symbol_a_feature,
    allfeatures,
    symbol2feature

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

function generate_feature_functions(
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

generate_feature_functions("PosFt", :posFt, Float64, "m")
function Base.get(::Feature_PosFt, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0)
    FeatureValue(rec[vehicle_index, pastframe].state.posF.t)
end
generate_feature_functions("PosFyaw", :posFyaw, Float64, "rad")
function Base.get(::Feature_PosFyaw, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0)
    FeatureValue(rec[vehicle_index, pastframe].state.posF.ϕ)
end
generate_feature_functions("Speed", :speed, Float64, "m/s")
function Base.get(::Feature_Speed, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0)
    FeatureValue(rec[vehicle_index, pastframe].state.v)
end

generate_feature_functions("VelFs", :velFs, Float64, "m/s")
function Base.get(::Feature_VelFs, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0)
    veh = rec[vehicle_index, pastframe]
    FeatureValue(veh.state.v*cos(veh.state.posF.ϕ))
end
generate_feature_functions("VelFt", :velFt, Float64, "m/s")
function Base.get(::Feature_VelFt, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0)
    veh = rec[vehicle_index, pastframe]
    FeatureValue(veh.state.v*sin(veh.state.posF.ϕ))
end

function get_feature_derivative_backwards(
    f::AbstractFeature,
    rec::SceneRecord,
    roadway::Roadway,
    vehicle_index::Int,
    pastframe::Int=0,
    frames_back::Int=1,
    )

    id = rec[vehicle_index].def.id

    retval = FeatureValue(0.0, FeatureState.INSUF_HIST)
    pastframe2 = pastframe - frames_back
    if pastframe_inbounds(rec, pastframe2)

        veh_index_curr = get_index_of_first_vehicle_with_id(rec, id, pastframe)
        veh_index_prev = get_index_of_first_vehicle_with_id(rec, id, pastframe2)

        if veh_index_prev != 0
            curr = convert(Float64, get(f, rec, roadway, veh_index_curr, pastframe))
            past = convert(Float64, get(f, rec, roadway, veh_index_prev, pastframe2))
            Δt = get_elapsed_time(rec, pastframe2, pastframe)
            retval = FeatureValue((curr - past) / Δt)
        end
    end

    retval
end

generate_feature_functions("TurnRateG", :turnrateG, Float64, "rad/s")
function Base.get(::Feature_TurnRateG, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0; frames_back::Int=1)

    id = rec[vehicle_index].def.id

    retval = FeatureValue(0.0, FeatureState.INSUF_HIST)
    pastframe2 = pastframe - frames_back
    if !pastframe_inbounds(rec, pastframe2)

        veh_index_curr = get_index_of_first_vehicle_with_id(rec, id, pastframe)
        veh_index_prev = get_index_of_first_vehicle_with_id(rec, id, pastframe2)

        if veh_index_prev != 0
            curr = rec[vehicle_index, veh_index_curr].state.posG.θ
            past = rec[vehicle_index, veh_index_prev].state.posG.θ
            Δt = get_elapsed_time(rec, pastframe2, pastframe)
            FeatureValue((curr - past) / Δt)
        end
    end

    retval
end
generate_feature_functions("TurnRateF", :turnrateF, Float64, "rad/s")
function Base.get(::Feature_TurnRateF, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0)
    get_feature_derivative_backwards(POSFYAW, rec, roadway, vehicle_index, pastframe)
end
generate_feature_functions("Acc", :acc, Float64, "m/s^2")
function Base.get(::Feature_Acc, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0)
    get_feature_derivative_backwards(SPEED, rec, roadway, vehicle_index, pastframe)
end
generate_feature_functions("AccFs", :accFs, Float64, "m/s²")
function Base.get(::Feature_AccFs, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0)
    get_feature_derivative_backwards(VELFS, rec, roadway, vehicle_index, pastframe)
end
generate_feature_functions("AccFt", :accFt, Float64, "m/s²")
function Base.get(::Feature_AccFt, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0)
    get_feature_derivative_backwards(VELFT, rec, roadway, vehicle_index, pastframe)
end
generate_feature_functions("Jerk", :jerk, Float64, "m/s³")
function Base.get(::Feature_Jerk, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0)
    get_feature_derivative_backwards(ACC, rec, roadway, vehicle_index, pastframe)
end
generate_feature_functions("JerkFs", :jerkFs, Float64, "m/s³")
function Base.get(::Feature_Jerk, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0)
    get_feature_derivative_backwards(ACCFS, rec, roadway, vehicle_index, pastframe)
end
generate_feature_functions("JerkFt", :jerkFt, Float64, "m/s³")
function Base.get(::Feature_Jerk, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0)
    get_feature_derivative_backwards(ACCFT, rec, roadway, vehicle_index, pastframe)
end

generate_feature_functions("MarkerDist_Left", :d_ml, Float64, "m")
function Base.get(::Feature_MarkerDist_Left, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0)
    veh = rec[vehicle_index, pastframe]
    offset = veh.state.posF.t
    lane = roadway[veh.state.posF.roadind.tag]
    FeatureValue(lane.width/2 - offset)
end
generate_feature_functions("MarkerDist_Right", :d_mr, Float64, "m")
function Base.get(::Feature_MarkerDist_Right, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0)
    veh = rec[vehicle_index, pastframe]
    offset = veh.state.posF.t
    lane = roadway[veh.state.posF.roadind.tag]
    FeatureValue(lane.width/2 + offset)
end
generate_feature_functions("MarkerDist_Left_Left", :d_mll, Float64, "m", can_be_missing=true)
function Base.get(::Feature_MarkerDist_Left_Left, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0)
    #=
    Distance to the left lane marker one lane to the left
    =#

    veh = rec[vehicle_index, pastframe]
    lane = roadway[veh.state.posF.roadind.tag]
    if n_lanes_left(lane, roadway) > 0
        offset = veh.state.posF.t
        lane_left = roadway[LaneTag(lane.tag.segment, lane.tag.lane + 1)]
        FeatureValue(lane.width/2 - offset + lane_left.width)
    else
        FeatureValue(NaN, FeatureState.MISSING) # there is no left lane
    end
end
generate_feature_functions("MarkerDist_Right_Right", :d_mrr, Float64, "m", can_be_missing=true)
function Base.get(::Feature_MarkerDist_Right_Right, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0)
    #=
    Distance to the right lane marker one lane to the right
    =#

    veh = rec[vehicle_index, pastframe]
    lane = roadway[veh.state.posF.roadind.tag]
    if n_lanes_right(lane, roadway) > 0
        offset = veh.state.posF.t
        lane_right = roadway[LaneTag(lane.tag.segment, lane.tag.lane - 1)]
        FeatureValue(lane.width/2 + offset + lane_right.width)
    else
        FeatureValue(NaN, FeatureState.MISSING) # there is no right lane
    end
end
generate_feature_functions("RoadEdgeDist_Left", :d_edgel, Float64, "m")
function Base.get(::Feature_RoadEdgeDist_Left, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0)
    veh = rec[vehicle_index, pastframe]
    offset = veh.state.posF.t
    footpoint = get_footpoint(veh)
    seg = roadway[veh.state.posF.roadind.tag.segment]
    lane = seg.lanes[end]
    roadproj = proj(footpoint, lane, roadway)
    curvept = roadway(RoadIndex(roadproj))
    lane = roadway[roadproj.tag]
    FeatureValue(lane.width/2 + abs(curvept.pos - footpoint) - offset)
end
generate_feature_functions("RoadEdgeDist_Right", :d_edger, Float64, "m")
function Base.get(::Feature_RoadEdgeDist_Right, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0)
    veh = rec[vehicle_index, pastframe]
    offset = veh.state.posF.t
    footpoint = get_footpoint(veh)
    seg = roadway[veh.state.posF.roadind.tag.segment]
    lane = seg.lanes[1]
    roadproj = proj(footpoint, lane, roadway)
    curvept = roadway(RoadIndex(roadproj))
    lane = roadway[roadproj.tag]
    FeatureValue(lane.width/2 + abs(curvept.pos - footpoint) + offset)
end
generate_feature_functions("N_Lane_Right", :n_lane_right, Int, "-", lowerbound=0.0)
function Base.get(::Feature_N_Lane_Right, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0)
    nlr = rec[vehicle_index, pastframe].state.posF.roadind.tag.lane - 1
    FeatureValue(convert(Float64, nlr))
end
generate_feature_functions("N_Lane_Left", :n_lane_left, Int, "-", lowerbound=0.0)
function Base.get(::Feature_N_Lane_Left, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0)
    veh = rec[vehicle_index, pastframe]
    seg = roadway[veh.state.posF.roadind.tag.segment]
    nll = length(seg.lanes) - veh.state.posF.roadind.tag.lane
    FeatureValue(convert(Float64, nll))
end
generate_feature_functions("Has_Lane_Right", :has_lane_right, Bool, "-", lowerbound=0.0, upperbound=1.0)
function Base.get(::Feature_Has_Lane_Right, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0)
    val = get(N_LANE_RIGHT, rec, roadway, vehicle_index, pastframe).v > 0.0
    FeatureValue(convert(Float64, val))
end
generate_feature_functions("Has_Lane_Left", :has_lane_left, Bool, "-", lowerbound=0.0, upperbound=1.0)
function Base.get(::Feature_Has_Lane_Left, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0)
    val = get(N_LANE_LEFT, rec, roadway, vehicle_index, pastframe).v > 0.0
    FeatureValue(convert(Float64, val))
end
generate_feature_functions("LaneCurvature", :curvature, Float64, "1/m")
function Base.get(::Feature_LaneCurvature, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0)
    veh = rec[vehicle_index, pastframe]
    FeatureValue(get_footpoint(veh).k)
end

# Dist_Merge
# Dist_Split

generate_feature_functions("TimeToCrossing_Right", :ttcr_mr, Float64, "s", lowerbound=0.0, censor_hi=10.0)
function Base.get(::Feature_TimeToCrossing_Right, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0)
    d_mr = get(MARKERDIST_RIGHT, rec, roadway, vehicle_index, pastframe).v
    velFt = get(VELFT, rec, roadway, vehicle_index, pastframe).v

    if d_mr > 0.0 && velFt < 0.0
        FeatureValue(-d_mr / velFt)
    else
        FeatureValue(Inf)
    end
end
generate_feature_functions("TimeToCrossing_Left", :ttcr_ml, Float64, "s", lowerbound=0.0, censor_hi=10.0)
function Base.get(::Feature_TimeToCrossing_Left, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0)
    d_ml = get(MARKERDIST_RIGHT, rec, roadway, vehicle_index, pastframe).v
    velFt = get(VELFT, rec, roadway, vehicle_index, pastframe).v

    if d_ml > 0.0 && velFt < 0.0
        FeatureValue(-d_ml / velFs)
    else
        FeatureValue(Inf)
    end
end
generate_feature_functions("EstimatedTimeToLaneCrossing", :est_ttcr, Float64, "s", lowerbound=0.0, censor_hi=10.0)
function Base.get(::Feature_EstimatedTimeToLaneCrossing, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0)
    ttcr_left = get(TIMETOCROSSING_LEFT, rec, roadway, vehicle_index, pastframe).v
    ttcr_right = get(TIMETOCROSSING_RIGHT, rec, roadway, vehicle_index, pastframe).v
    FeatureValue(min(ttcr_left, ttcr_right))
end
generate_feature_functions("A_REQ_StayInLane", :a_req_stayinlane, Float64, "m/s²", can_be_missing=true)
function Base.get(::Feature_A_REQ_StayInLane, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0)
    velFt = get(VELFT, rec, roadway, vehicle_index, pastframe).v

    if velFt > 0.0
        d_mr = get(MARKERDIST_RIGHT, rec, roadway, vehicle_index, pastframe).v
        if d_mr > 0.0
            return FeatureValue(0.5velFt*velFt / d_mr)
        else
            return FeatureValue(NaN, FeatureState.MISSING)
        end
    else
        d_ml = get(MARKERDIST_LEFT, rec, roadway, vehicle_index, pastframe)
        if d_ml < 0.0
            return FeatureValue(-0.5velFt*velFt / d_ml)
        else
            return FeatureValue(NaN, FeatureState.MISSING)
        end
    end
end

generate_feature_functions("Time_Consecutive_Brake", :time_consec_brake, Float64, "s", lowerbound=0.0)
function Base.get(::Feature_Time_Consecutive_Brake, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0)

    prev_accel = get(ACC, rec, roadway, vehicle_index, pastframe)
    if prev_accel ≥ 0.0
        FeatureValue(0.0)
    else
        pastframe_orig = pastframe
        id = rec[vehicle_index, pastframe].def.id
        while pastframe_inbounds(rec, pastframe-1) &&
              get(ACC, rec, roadway, get_index_of_first_vehicle_with_id(rec, id, pastframe-1)) < 0.0

            pastframe -= 1
        end

        FeatureValue(get_elapsed_time(rec, pastframe, pastframe_orig))
    end
end
generate_feature_functions("Time_Consecutive_Accel", :time_consec_accel, Float64, "s", lowerbound=0.0)
function Base.get(::Feature_Time_Consecutive_Accel, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0)

    prev_accel = get(ACC, rec, roadway, vehicle_index, pastframe)
    if prev_accel ≤ 0.0
        FeatureValue(0.0)
    else

        pastframe_orig = pastframe
        id = rec[vehicle_index, pastframe].def.id
        while pastframe_inbounds(rec, pastframe-1) &&
              get(ACC, rec, roadway, get_index_of_first_vehicle_with_id(rec, id, pastframe-1)) > 0.0

            pastframe -= 1
        end

        FeatureValue(get_elapsed_time(rec, pastframe, pastframe_orig))
    end
end
generate_feature_functions("Time_Consecutive_Throttle", :time_consec_throttle, Float64, "s", lowerbound=0.0)
function Base.get(::Feature_Time_Consecutive_Throttle, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0)
    tc_accel = get(TIME_CONSECUTIVE_ACCEL, rec, roadway, vehicle_index, pastframe).v
    tc_brake = get(TIME_CONSECUTIVE_BRAKE, rec, roadway, vehicle_index, pastframe).v
    FeatureValue(tc_accel ≥ tc_brake ? tc_accel : -tc_brake)
end

#############################################
#
# FRONT
#
#############################################

generate_feature_functions("Dist_Front", :d_front, Float64, "m", lowerbound=0.0, can_be_missing=true)
function Base.get(::Feature_Dist_Front, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0;
    neighborfore::NeighborForeResult = get_neighbor_fore_along_lane(get_scene(rec, pastframe), vehicle_index, roadway)
    )

    if neighborfore.ind == 0
        FeatureValue(NaN, FeatureState.MISSING)
    else
        len_ego = rec[vehicle_index, pastframe].def.length
        len_oth = rec[neighborfore.ind, pastframe].def.length
        FeatureValue(neighborfore.Δs - len_ego/2 - len_oth/2)
    end
end

#############################################
#
# FRONT LEFT
#
#############################################

generate_feature_functions("Dist_Front_Left", :d_front_left, Float64, "m", lowerbound=0.0, can_be_missing=true)
function Base.get(::Feature_Dist_Front_Left, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0)

    retval = FeatureValue(NaN, FeatureState.MISSING)

    scene = get_scene(rec, pastframe)
    veh_target = scene[vehicle_index]

    lane = roadway[veh_target.state.posF.roadind.tag]
    if n_lanes_left(lane, roadway) > 0
        lane_left = roadway[LaneTag(lane.tag.segment, lane.tag.lane + 1)]
        roadproj = proj(veh_target.state.posG, lane_left, roadway)
        tag_start = roadproj.tag
        s_base = lane_left[roadproj.curveproj.ind, roadway].s

        neighborfore = get_neighbor_fore_along_lane(scene, roadway, tag_start, s_base,
                                                    index_to_ignore=vehicle_index)

        if neighborfore.ind != 0
            len_ego = rec[vehicle_index, pastframe].def.length
            len_oth = rec[neighborfore.ind, pastframe].def.length
            retval = FeatureValue(neighborfore.Δs - len_ego/2 - len_oth/2)
        end
    end

    retval
end

#############################################
#
# FRONT RIGHT
#
#############################################

generate_feature_functions("Dist_Front_Right", :d_front_right, Float64, "m", lowerbound=0.0, can_be_missing=true)
function Base.get(::Feature_Dist_Front_Right, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0)

    retval = FeatureValue(NaN, FeatureState.MISSING)

    scene = get_scene(rec, pastframe)
    veh_target = scene[vehicle_index]

    lane = roadway[veh_target.state.posF.roadind.tag]
    if n_lanes_right(lane, roadway) > 0
        lane_left = roadway[LaneTag(lane.tag.segment, lane.tag.lane - 1)]
        roadproj = proj(veh_target.state.posG, lane_left, roadway)
        tag_start = roadproj.tag
        s_base = lane_left[roadproj.curveproj.ind, roadway].s
        neighborfore = get_neighbor_fore_along_lane(scene, roadway, tag_start, s_base,
                                                    index_to_ignore=vehicle_index)

        if neighborfore.ind != 0
            len_ego = rec[vehicle_index, pastframe].def.length
            len_oth = rec[neighborfore.ind, pastframe].def.length
            retval = FeatureValue(neighborfore.Δs - len_ego/2 - len_oth/2)
        end
    end

    retval
end