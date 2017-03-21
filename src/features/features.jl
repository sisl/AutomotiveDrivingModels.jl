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
        @assert(sym != Symbol(feature), "symb: $name -> $feature")
    end
    @assert(lowerbound ≤ upperbound)

    feature_name = Symbol("Feature_" * name)
    const_name   = Symbol(uppercase(name))
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
        Base.Symbol(    ::$feature_name)  = $sym_feature
        SYMBOL_TO_FEATURE[Symbol($const_name)] = $const_name
    end
end

# ----------------------------------

generate_feature_functions("PosFt", :posFt, Float64, "m")
function Base.get(::Feature_PosFt, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0)
    FeatureValue(rec[pastframe][vehicle_index].state.posF.t)
end
generate_feature_functions("PosFyaw", :posFyaw, Float64, "rad")
function Base.get(::Feature_PosFyaw, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0)
    FeatureValue(rec[pastframe][vehicle_index].state.posF.ϕ)
end
generate_feature_functions("Speed", :speed, Float64, "m/s")
function Base.get(::Feature_Speed, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0)
    FeatureValue(rec[pastframe][vehicle_index].state.v)
end

generate_feature_functions("VelFs", :velFs, Float64, "m/s")
function Base.get(::Feature_VelFs, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0)
    veh = rec[pastframe][vehicle_index]
    FeatureValue(veh.state.v*cos(veh.state.posF.ϕ))
end
generate_feature_functions("VelFt", :velFt, Float64, "m/s")
function Base.get(::Feature_VelFt, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0)
    veh = rec[pastframe][vehicle_index]
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

    id = rec[pastframe][vehicle_index].id

    retval = FeatureValue(0.0, FeatureState.INSUF_HIST)
    pastframe2 = pastframe - frames_back

    if pastframe_inbounds(rec, pastframe) && pastframe_inbounds(rec, pastframe2)

        veh_index_curr = vehicle_index
        veh_index_prev = findfirst(rec, id, pastframe2)

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

    id = rec[pastframe][vehicle_index].id

    retval = FeatureValue(0.0, FeatureState.INSUF_HIST)
    pastframe2 = pastframe - frames_back
    if pastframe_inbounds(rec, pastframe2)

        veh_index_curr = vehicle_index
        veh_index_prev = findfirst(rec, id, pastframe2)

        if veh_index_prev != 0
            curr = rec[pastframe][veh_index_curr].state.posG.θ
            past = rec[pastframe2][veh_index_prev].state.posG.θ
            Δt = get_elapsed_time(rec, pastframe2, pastframe)
            retval = FeatureValue(deltaangle(past, curr) / Δt)
        end
    end

    retval
end
generate_feature_functions("TurnRateF", :turnrateF, Float64, "rad/s")
function Base.get(::Feature_TurnRateF, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0)
    get_feature_derivative_backwards(POSFYAW, rec, roadway, vehicle_index, pastframe)
end
generate_feature_functions("AngularRateG", :angrateG, Float64, "rad/s²")
function Base.get(::Feature_AngularRateG, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0)
    get_feature_derivative_backwards(TURNRATEG, rec, roadway, vehicle_index, pastframe)
end
generate_feature_functions("AngularRateF", :angrateF, Float64, "rad/s²")
function Base.get(::Feature_AngularRateF, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0)
    get_feature_derivative_backwards(TURNRATEF, rec, roadway, vehicle_index, pastframe)
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
function Base.get(::Feature_JerkFs, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0)
    get_feature_derivative_backwards(ACCFS, rec, roadway, vehicle_index, pastframe)
end
generate_feature_functions("JerkFt", :jerkFt, Float64, "m/s³")
function Base.get(::Feature_JerkFt, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0)
    get_feature_derivative_backwards(ACCFT, rec, roadway, vehicle_index, pastframe)
end
generate_feature_functions("DesiredAngle", :desang, Float64, "rad")
function Base.get(::Feature_DesiredAngle, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0;
    kp_desired_angle::Float64 = 1.0,
    )

    retval = FeatureValue(0.0, FeatureState.INSUF_HIST)
    if pastframe_inbounds(rec, pastframe) && pastframe_inbounds(rec, pastframe-1)

        id = rec[pastframe][vehicle_index].id

        pastϕ = get_state(rec, id, pastframe-1).posF.ϕ
        currϕ = get_state(rec, id, pastframe).posF.ϕ

        Δt = rec.timestep
        expconst = exp(-kp_desired_angle*Δt)
        retval = FeatureValue((currϕ - pastϕ*expconst) / (1.0 - expconst))
    end
    retval
end

generate_feature_functions("MarkerDist_Left", :d_ml, Float64, "m")
Base.get(::Feature_MarkerDist_Left, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0) =
    FeatureValue(get_markerdist_left(rec[pastframe][vehicle_index], roadway))

generate_feature_functions("MarkerDist_Right", :d_mr, Float64, "m")
Base.get(::Feature_MarkerDist_Right, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0) =
    FeatureValue(get_markerdist_right(rec[pastframe][vehicle_index], roadway))

generate_feature_functions("MarkerDist_Left_Left", :d_mll, Float64, "m", can_be_missing=true)
function Base.get(::Feature_MarkerDist_Left_Left, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0)
    #=
    Distance to the left lane marker one lane to the left
    =#

    veh = rec[pastframe][vehicle_index]
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

    veh = rec[pastframe][vehicle_index]
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
    veh = rec[pastframe][vehicle_index]
    offset = veh.state.posF.t
    footpoint = get_footpoint(veh)
    seg = roadway[veh.state.posF.roadind.tag.segment]
    lane = seg.lanes[end]
    roadproj = proj(footpoint, lane, roadway)
    curvept = roadway[RoadIndex(roadproj)]
    lane = roadway[roadproj.tag]
    FeatureValue(lane.width/2 + abs(curvept.pos - footpoint) - offset)
end
generate_feature_functions("RoadEdgeDist_Right", :d_edger, Float64, "m")
function Base.get(::Feature_RoadEdgeDist_Right, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0)
    veh = rec[pastframe][vehicle_index]
    offset = veh.state.posF.t
    footpoint = get_footpoint(veh)
    seg = roadway[veh.state.posF.roadind.tag.segment]
    lane = seg.lanes[1]
    roadproj = proj(footpoint, lane, roadway)
    curvept = roadway[RoadIndex(roadproj)]
    lane = roadway[roadproj.tag]
    FeatureValue(lane.width/2 + abs(curvept.pos - footpoint) + offset)
end
generate_feature_functions("LaneOffsetLeft", :posFtL, Float64, "m", can_be_missing=true)
function Base.get(::Feature_LaneOffsetLeft, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0)
    veh_ego = rec[pastframe][vehicle_index]
    t = veh_ego.state.posF.t
    lane = roadway[veh_ego.state.posF.roadind.tag]
    if n_lanes_left(lane, roadway) > 0
        lane_left = roadway[LaneTag(lane.tag.segment, lane.tag.lane + 1)]
        lane_offset = t - lane.width/2 - lane_left.width/2
        FeatureValue(lane_offset)
    else
        FeatureValue(NaN, FeatureState.MISSING)
    end
end
generate_feature_functions("LaneOffsetRight", :posFtR, Float64, "m", can_be_missing=true)
function Base.get(::Feature_LaneOffsetRight, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0)
    veh_ego = rec[pastframe][vehicle_index]
    t = veh_ego.state.posF.t
    lane = roadway[veh_ego.state.posF.roadind.tag]
    if n_lanes_right(lane, roadway) > 0
        lane_right = roadway[LaneTag(lane.tag.segment, lane.tag.lane - 1)]
        lane_offset = t + lane.width/2 + lane_right.width/2
        FeatureValue(lane_offset)
    else
        FeatureValue(NaN, FeatureState.MISSING)
    end
end
generate_feature_functions("N_Lane_Right", :n_lane_right, Int, "-", lowerbound=0.0)
function Base.get(::Feature_N_Lane_Right, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0)
    nlr = rec[pastframe][vehicle_index].state.posF.roadind.tag.lane - 1
    FeatureValue(convert(Float64, nlr))
end
generate_feature_functions("N_Lane_Left", :n_lane_left, Int, "-", lowerbound=0.0)
function Base.get(::Feature_N_Lane_Left, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0)
    veh = rec[pastframe][vehicle_index]
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
generate_feature_functions("LaneCurvature", :curvature, Float64, "1/m", can_be_missing=true)
function Base.get(::Feature_LaneCurvature, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0)
    veh = rec[pastframe][vehicle_index]
    curvept = roadway[veh.state.posF.roadind]
    val = curvept.k
    if isnan(val)
        FeatureValue(0.0, FeatureState.MISSING)
    else
        FeatureValue(val)
    end
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

    prev_accel = convert(Float64, get(ACC, rec, roadway, vehicle_index, pastframe))
    if prev_accel ≥ 0.0
        FeatureValue(0.0)
    else
        pastframe_orig = pastframe
        id = rec[pastframe][vehicle_index].id
        while pastframe_inbounds(rec, pastframe-1) &&
              get(ACC, rec, roadway, findfirst(rec, id, pastframe-1)) < 0.0

            pastframe -= 1
        end

        FeatureValue(get_elapsed_time(rec, pastframe, pastframe_orig))
    end
end
generate_feature_functions("Time_Consecutive_Accel", :time_consec_accel, Float64, "s", lowerbound=0.0)
function Base.get(::Feature_Time_Consecutive_Accel, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0)

    prev_accel = convert(Float64, get(ACC, rec, roadway, vehicle_index, pastframe))
    if prev_accel ≤ 0.0
        FeatureValue(0.0)
    else

        pastframe_orig = pastframe
        id = rec[pastframe][vehicle_index].id
        while pastframe_inbounds(rec, pastframe-1) &&
              get(ACC, rec, roadway, findfirst(rec, id, pastframe-1)) > 0.0

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
    neighborfore::NeighborLongitudinalResult = get_neighbor_fore_along_lane(rec[pastframe], vehicle_index, roadway),
    censor_hi::Float64=100.0,
    )

    if neighborfore.ind == 0
        FeatureValue(100.0, FeatureState.CENSORED_HI)
    else
        scene = rec[pastframe]
        len_ego = scene[vehicle_index].def.length
        len_oth = scene[neighborfore.ind].def.length
        FeatureValue(neighborfore.Δs - len_ego/2 - len_oth/2)
    end
end
generate_feature_functions("Speed_Front", :v_front, Float64, "m/s", can_be_missing=true)
function Base.get(::Feature_Speed_Front, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0;
    neighborfore::NeighborLongitudinalResult = get_neighbor_fore_along_lane(rec[pastframe], vehicle_index, roadway),
    )

    if neighborfore.ind == 0
        FeatureValue(0.0, FeatureState.MISSING)
    else
        FeatureValue(rec[pastframe][neighborfore.ind].state.v)
    end
end
generate_feature_functions("Timegap", :timegap, Float64, "s", can_be_missing=true)
function Base.get(::Feature_Timegap, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0;
    neighborfore::NeighborLongitudinalResult = get_neighbor_fore_along_lane(rec[pastframe], vehicle_index, roadway),
    censor_hi::Float64 = 10.0,
    )

    v = rec[pastframe][vehicle_index].state.v

    if v ≤ 0.0 || neighborfore.ind == 0
        FeatureValue(censor_hi, FeatureState.CENSORED_HI)
    else
        scene = rec[pastframe]
        len_ego = scene[vehicle_index].def.length
        len_oth = scene[neighborfore.ind].def.length
        Δs = neighborfore.Δs - len_ego/2 - len_oth/2

        if Δs > 0.0
            FeatureValue(Δs / v)
        else
            FeatureValue(0.0) # collision!
        end
    end
end

generate_feature_functions("Inv_TTC", :inv_ttc, Float64, "1/s", can_be_missing=true)
function Base.get(::Feature_Inv_TTC, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0;
    neighborfore::NeighborLongitudinalResult = get_neighbor_fore_along_lane(rec[pastframe], vehicle_index, roadway),
    censor_hi::Float64 = 10.0,
    )


    if neighborfore.ind == 0
        FeatureValue(0.0, FeatureState.MISSING)
    else
        scene = rec[pastframe]
        veh_fore = scene[neighborfore.ind]
        veh_rear = scene[vehicle_index]

        len_ego = veh_fore.def.length
        len_oth = veh_rear.def.length
        Δs = neighborfore.Δs - len_ego/2 - len_oth/2


        Δv = veh_fore.state.v - veh_rear.state.v

        if Δs < 0.0 # collision!
            FeatureValue(censor_hi, FeatureState.CENSORED_HI)
        elseif Δv > 0.0 # front car is pulling away
            FeatureValue(0.0)
        else
            f = -Δv/Δs
            if f > censor_hi
                FeatureValue(f, FeatureState.CENSORED_HI)
            else
                FeatureValue(f)
            end
        end
    end
end
generate_feature_functions("TTC", :ttc, Float64, "s", can_be_missing=true)
function Base.get(::Feature_TTC, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0;
    neighborfore::NeighborLongitudinalResult = get_neighbor_fore_along_lane(rec[pastframe], vehicle_index, roadway),
    censor_hi::Float64 = 10.0,
    inv_ttc::FeatureValue = get(INV_TTC, rec, roadway, vehicle_index, pastframe, neighborfore=neighborfore, censor_hi=censor_hi),
    )

    if inv_ttc.i == FeatureState.MISSING
        # if the value is missing then front car not found and set TTC to censor_hi
        return FeatureValue(censor_hi, FeatureState.MISSING)
    else
        @assert is_feature_valid(inv_ttc) || inv_ttc.i == FeatureState.CENSORED_HI
        return FeatureValue(min(1.0 / inv_ttc.v, censor_hi))
    end
end

#############################################
#
# FRONT LEFT
#
#############################################
generate_feature_functions("Dist_Front_Left", :d_front_left, Float64, "m", lowerbound=0.0, can_be_missing=true)
function Base.get(::Feature_Dist_Front_Left, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0;
    neighborfore::NeighborLongitudinalResult = get_neighbor_fore_along_left_lane(rec[pastframe], vehicle_index, roadway),
    )

    get(DIST_FRONT, rec, roadway, vehicle_index, pastframe, neighborfore=neighborfore)
end

#############################################
#
# FRONT RIGHT
#
#############################################

generate_feature_functions("Dist_Front_Right", :d_front_right, Float64, "m", lowerbound=0.0, can_be_missing=true)
function Base.get(::Feature_Dist_Front_Right, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0;
    neighborfore::NeighborLongitudinalResult = get_neighbor_fore_along_right_lane(rec[pastframe], vehicle_index, roadway),
    )

    get(DIST_FRONT, rec, roadway, vehicle_index, pastframe, neighborfore=neighborfore)
end

#############################################
#
# REAR
#
#############################################

generate_feature_functions("Dist_Rear", :d_rear, Float64, "m", lowerbound=0.0, can_be_missing=true)
function Base.get(::Feature_Dist_Rear, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0;
    neighborrear::NeighborLongitudinalResult = get_neighbor_rear_along_lane(rec[pastframe], vehicle_index, roadway),
    censor_hi::Float64=100.0,
    )

    if neighborrear.ind == 0
        FeatureValue(100.0, FeatureState.CENSORED_HI)
    else
        scene = rec[pastframe]
        len_ego = scene[vehicle_index].def.length
        len_oth = scene[neighborrear.ind].def.length
        FeatureValue(neighborrear.Δs - len_ego/2 - len_oth/2)
    end
end
generate_feature_functions("Speed_Rear", :v_rear, Float64, "m/s", can_be_missing=true)
function Base.get(::Feature_Speed_Rear, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0;
    neighborrear::NeighborLongitudinalResult = get_neighbor_rear_along_lane(rec[pastframe], vehicle_index, roadway),
    )

    if neighborrear.ind == 0
        FeatureValue(0.0, FeatureState.MISSING)
    else
        FeatureValue(rec[pastframe][neighborrear.ind].state.v)
    end
end

#############################################
#
# SCENE WISE
#
#############################################

generate_feature_functions("Is_Colliding", :is_colliding, Bool, "-", lowerbound=0.0, upperbound=1.0)
function Base.get(::Feature_Is_Colliding, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0;
    mem::CPAMemory=CPAMemory(),
    )

    scene = rec[pastframe]
    is_colliding = convert(Float64, get_first_collision(scene, vehicle_index, mem).is_colliding)
    FeatureValue(is_colliding)
end
