
# export AbstractFeature, FeatureTargetLat, FeatureTargetLon
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
    # NA_ALIAS,
    # get_feature,
    # symbol2feature,
    # is_symbol_a_feature,
    # is_feature_na,
    # allfeatures,
    # units,
    # isint,
    # isbool,
    # upperbound,
    # lowerbound,
    # couldna,
    # replace_na,
    # lsymbol,
    # get_feature_derivative_backwards

# ----------------------------------
# types

abstract AbstractFeature
abstract FeatureTargetLat <: AbstractFeature
abstract FeatureTargetLon <: AbstractFeature

# ----------------------------------
# globals

const SYMBOL_TO_FEATURE = Dict{Symbol, AbstractFeature}()
const TIME_THRESHOLD = 10.0 # [s]
const ACCEL_THRESHOLD = 5.0 # [m/s^2]

# ----------------------------------
# feature functions

is_symbol_a_feature(sym::Symbol) = haskey(SYMBOL_TO_FEATURE, sym)
allfeatures() = values(SYMBOL_TO_FEATURE)
is_feature_na(v::Float64) = isnan(v)
function symbol2feature(sym::Symbol)
    if !haskey(SYMBOL_TO_FEATURE, sym)
        str = string(sym)
        if startswith(str, "isclean_")
            target = symbol(str[9:end])
            println("IsClean target: ", target)
            SYMBOL_TO_FEATURE[sym] = Feature_IsClean{target}()
        elseif ismatch(r"^past_(\d)+_", str)
            history = parse(Int, match(r"(\d)+", str).match)
            len_of_header = length(match(r"^past_(\d)+_", str).match)
            target = symbol(str[len_of_header+1:end])
            println("Past history, target: ", history, "  ", target)
            SYMBOL_TO_FEATURE[sym] = Feature_Past{target, history}()
        elseif ismatch(r"^mean_(\d)+_", str)
            history = parse(Int, match(r"(\d)+", str).match)
            len_of_header = length(match(r"^mean_(\d)+_", str).match)
            target = symbol(str[len_of_header+1:end])
            println("Mean over history, target: ", history, "  ", target)
            SYMBOL_TO_FEATURE[sym] = Feature_Mean_Over_History{target, history}()
        elseif ismatch(r"^std_(\d)+_", str)
            history = parse(Int, match(r"(\d)+", str).match)
            len_of_header = length(match(r"^std_(\d)+_", str).match)
            target = symbol(str[len_of_header+1:end])
            println("Std over history, target: ", history, "  ", target)
            SYMBOL_TO_FEATURE[sym] = Feature_Std_Over_History{target, history}()
        elseif ismatch(r"^max_(\d)+_", str)
            history = parse(Int, match(r"(\d)+", str).match)
            len_of_header = length(match(r"^max_(\d)+_", str).match)
            target = symbol(str[len_of_header+1:end])
            println("Max over history, target: ", history, "  ", target)
            SYMBOL_TO_FEATURE[sym] = Feature_Max_Over_History{target, history}()
        elseif ismatch(r"^min_(\d)+_", str)
            history = parse(Int, match(r"(\d)+", str).match)
            len_of_header = length(match(r"^min_(\d)+_", str).match)
            target = symbol(str[len_of_header+1:end])
            println("Min over history, target: ", history, "  ", target)
            SYMBOL_TO_FEATURE[sym] = Feature_Min_Over_History{target, history}()
        end
    end

    SYMBOL_TO_FEATURE[sym]
end

# ----------------------------------
# create_feature_basics

function create_feature_basics(
    name        :: AbstractString,
    sym         :: Symbol,
    lstr        :: LaTeXString,
                :: Type{Float64},
    unit        :: AbstractString,
    minvalue    :: Float64,
    maxvalue    :: Float64,
    can_na      :: Symbol; # ∈ {:can_na, :no_na}
    na_replacement :: Union{Void,Float64} = nothing,
    target      :: Symbol = :none # ∈ {:lat, :lon}
    )

    for feature in values(SYMBOL_TO_FEATURE)
        @assert(sym  != symbol(feature), "symb: $name -> $feature")
        @assert(lstr != lsymbol(feature), "lstr: $name -> $feature")
    end
    @assert(minvalue ≤ maxvalue)
    @assert(can_na == :can_na || can_na == :no_na)

    if target == :lat
        feature_parent = :FeatureTargetLat
    elseif target == :lon
        feature_parent = :FeatureTargetLon
    else
        feature_parent = :AbstractFeature
    end

    feature_name = symbol("Feature_" * name)
    const_name   = symbol(uppercase(name))
    sym_feature  = Meta.quot(sym)
    could_be_na  = can_na == :can_na

    @eval begin
        immutable $feature_name <: $feature_parent end
        const       $const_name  = ($feature_name)()
        units(       ::$feature_name)  = $unit
        isint(       ::$feature_name)  = false
        isbool(      ::$feature_name)  = false
        lowerbound(  ::$feature_name)  = $minvalue
        upperbound(  ::$feature_name)  = $maxvalue
        couldna(     ::$feature_name)  = $could_be_na
        Base.symbol( ::$feature_name)  = $sym_feature
        lsymbol(     ::$feature_name)  = $lstr
        SYMBOL_TO_FEATURE[symbol($const_name)] = $const_name
    end

    if !could_be_na || isa(na_replacement, Void)
        return
    end

    @eval begin
        replace_na(::$feature_name) = $na_replacement
    end
end
function create_feature_basics(
    name        :: AbstractString,
    sym         :: Symbol,
    lstr        :: LaTeXString,
                :: Type{Int},
    unit        :: AbstractString,
    minvalue    :: Float64,
    maxvalue    :: Float64,
    can_na      :: Symbol; # ∈ {:can_na, :no_na}
    na_replacement :: Union{Void,Float64} = nothing
    )

    for feature in values(SYMBOL_TO_FEATURE)
        @assert(sym  != symbol(feature), "symb: $name -> $feature")
        @assert(lstr != lsymbol(feature), "lstr: $name -> $feature")
    end
    @assert(minvalue ≤ maxvalue)
    @assert(can_na == :can_na || can_na == :no_na)

    feature_name = symbol("Feature_" * name)
    const_name   = symbol(uppercase(name))
    sym_feature  = Meta.quot(sym)
    could_be_na  = can_na == :can_na

    @eval begin
        immutable $feature_name <: AbstractFeature end
        const       $const_name  = ($feature_name)()
        units(       ::$feature_name)  = $unit
        isint(       ::$feature_name)  = true
        isbool(      ::$feature_name)  = false
        lowerbound(  ::$feature_name)  = $minvalue
        upperbound(  ::$feature_name)  = $maxvalue
        couldna(     ::$feature_name)  = $could_be_na
        Base.symbol( ::$feature_name)  = $sym_feature
        lsymbol(     ::$feature_name)  = $lstr
        SYMBOL_TO_FEATURE[symbol($const_name)] = $const_name
    end

    if !could_be_na || isa(na_replacement, Void)
        return
    end

    @eval begin
        replace_na(::$feature_name) = $na_replacement
    end
end
function create_feature_basics(
    name        :: AbstractString,
    sym         :: Symbol,
    lstr        :: LaTeXString,
                :: Type{Bool},
    unit        :: AbstractString,
    can_na      :: Symbol; # ∈ {:can_na, :no_na}
    na_replacement :: Union{Void,Float64} = nothing
    )

    for feature in values(SYMBOL_TO_FEATURE)
        @assert(sym  != symbol(feature), "symb: $name -> $feature")
        @assert(lstr != lsymbol(feature), "lstr: $name -> $feature")
    end
    @assert(can_na == :can_na || can_na == :no_na)

    feature_name = symbol("Feature_" * name)
    const_name   = symbol(uppercase(name))
    sym_feature  = Meta.quot(sym)
    could_be_na  = can_na == :can_na

    @eval begin
        immutable $feature_name <: AbstractFeature end
        const       $const_name  = ($feature_name)()
        units(       ::$feature_name)  = $unit
        isint(       ::$feature_name)  = true
        isbool(      ::$feature_name)  = true
        lowerbound(  ::$feature_name)  = 0.0
        upperbound(  ::$feature_name)  = 1.0
        couldna(     ::$feature_name)  = $could_be_na
        Base.symbol( ::$feature_name)  = $sym_feature
        lsymbol(     ::$feature_name)  = $lstr
        SYMBOL_TO_FEATURE[symbol($const_name)] = $const_name
    end

    if !could_be_na || isa(na_replacement, Void)
        return
    end

    @eval begin
        replace_na(::$feature_name) = $na_replacement
    end
end

# ----------------------------------

create_feature_basics("PosFt", :posFt, L"p^F_t", Float64, L"m", -Inf, Inf, :no_na)
Base.get(::Feature_PosFt, runlog::RunLog, ::StreetNetwork, colset::UInt, frame::Integer) =
    (get(runlog, colset, frame, :frenet)::VecSE2).y

create_feature_basics("PosFyaw", :posFyaw, L"\psi", Float64, L"\radian", -Inf, Inf, :no_na)
Base.get(::Feature_PosFyaw, runlog::RunLog, ::StreetNetwork, colset::UInt, frame::Integer) =
    (get(runlog, colset, frame, :frenet)::VecSE2).θ

create_feature_basics("Speed", :speed, L"\|v\|", Float64, L"m/s", 0.0, Inf, :no_na)
function Base.get(::Feature_Speed, runlog::RunLog, ::StreetNetwork, colset::UInt, frame::Integer)
    ratesB = get(runlog, colset, frame, :ratesB)::VecSE2
    _fast_hypot(ratesB)
end

create_feature_basics("VelBx", :velBx, L"v^B_x", Float64, L"m/s", -Inf, Inf, :no_na)
Base.get(::Feature_VelBx, runlog::RunLog, ::StreetNetwork, colset::UInt, frame::Integer) =
    (get(runlog, colset, frame, :ratesB)::VecSE2).x
create_feature_basics("VelBy", :velBy, L"v^B_y", Float64, L"m/s", -Inf, Inf, :no_na)
Base.get(::Feature_VelBy, runlog::RunLog, ::StreetNetwork, colset::UInt, frame::Integer) =
    (get(runlog, colset, frame, :ratesB)::VecSE2).y
create_feature_basics("VelFs", :velFs, L"v^F_s", Float64, L"m/s", -Inf, Inf, :no_na)
Base.get(::Feature_VelFs, runlog::RunLog, ::StreetNetwork, colset::UInt, frame::Integer) =
    (get(runlog, colset, frame, :ratesF)::VecE2).x
create_feature_basics("VelFt", :velFt, L"v^F_t", Float64, L"m/s", -Inf, Inf, :no_na)
Base.get(::Feature_VelFt, runlog::RunLog, ::StreetNetwork, colset::UInt, frame::Integer) =
    (get(runlog, colset, frame, :ratesF)::VecE2).y

create_feature_basics("Scene_Speed_Difference", :scene_speed_diff, L"\delta v_\text{scene}", Float64, L"m/s", -Inf, Inf, :no_na)
function Base.get(::Feature_Scene_Speed_Difference, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    # v_id - v_scene

    # pull average speed
    v_tot = 0.0
    colsets = get_colset_range(runlog, frame)
    for colset_i in colsets
        v_tot += get(POSFYAW, runlog, sn, colset_i, frame)
    end
    v_scene = v_tot / length(colsets)

    v_id = get(POSFYAW, runlog, sn, colset, frame)

    v_id - v_scene
end

create_feature_basics("MarkerDist_Left", :d_ml, L"d_{ml}", Float64, L"m", -Inf, Inf, :no_na)
function Base.get(::Feature_MarkerDist_Left, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    offset = (get(runlog, colset, frame, :frenet)::VecSE2).y
    node = _get_node(runlog, sn, colset, frame)
    node.marker_dist_left - offset
end
create_feature_basics("MarkerDist_Right", :d_mr, L"d_{mr}", Float64, L"m", 0.0, Inf, :no_na)
function Base.get(::Feature_MarkerDist_Right, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    offset = (get(runlog, colset, frame, :frenet)::VecSE2).y
    node = _get_node(runlog, sn, colset, frame)
    node.marker_dist_right + offset
end
create_feature_basics("Dist_From_Centerline", :d_cl, L"d_{cl}", Float64, L"m", 0.0, Inf, :no_na)
function Base.get(::Feature_Dist_From_Centerline, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    (get(runlog, colset, frame, :frenet)::VecSE2).y
end

create_feature_basics( "TimeToCrossing_Right", :ttcr_mr, L"ttcr^{mr}_y", Float64, L"s", 0.0, Inf, :can_na, na_replacement=10.0)
function Base.get(::Feature_TimeToCrossing_Right, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)
    d_mr = get(MARKERDIST_RIGHT, runlog, sn, colset, frame)
    velFt = get(VELFT, runlog, sn, colset, frame)

    if d_mr > 0.0 && velFt < 0.0
        min(-d_mr / velFt, TIME_THRESHOLD)
    else
        NA_ALIAS
    end
end

create_feature_basics( "TimeToCrossing_Left", :ttcr_ml, L"ttcr^{ml}_y", Float64, L"s", 0.0, Inf, :can_na, na_replacement=10.0)
function Base.get(::Feature_TimeToCrossing_Left, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)
    d_ml = get(MARKERDIST_LEFT, runlog, sn, colset, frame)
    velFs = get(VELFS, runlog, sn, colset, frame)

    if d_ml < 0.0 && velFs > 0.0
        min(-d_ml / velFs, TIME_THRESHOLD)
    else
        NA_ALIAS
    end
end

create_feature_basics( "EstimatedTimeToLaneCrossing", :est_ttcr, L"ttcr^\text{est}_y", Float64, L"s", 0.0, Inf, :can_na, na_replacement=10.0)
function Base.get(::Feature_EstimatedTimeToLaneCrossing, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)
    ttcr_left = get(TIMETOCROSSING_LEFT, runlog, sn, colset, frame)
    if !is_feature_na(ttcr_left)
        return ttcr_left
    end
    get(TIMETOCROSSING_RIGHT, runlog, sn, colset, frame)
end

create_feature_basics( "A_REQ_StayInLane", :a_req_stayinlane, L"a^{req}_y", Float64, "m/s2", -Inf, Inf, :can_na, na_replacement=0.0)
function Base.get(::Feature_A_REQ_StayInLane, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)
    velFt = get(VELFT, runlog, sn, colset, frame)

    if velFt > 0.0
        d_mr = get(MARKERDIST_RIGHT, runlog, sn, colset, frame)
        return (d_mr > 0.0) ? min( 0.5velFt*velFt / d_mr, ACCEL_THRESHOLD) : NA_ALIAS
    else
        d_ml = get(MARKERDIST_LEFT, runlog, sn, colset, frame)
        return (d_ml < 0.0) ? min(-0.5velFt*velFt / d_ml, ACCEL_THRESHOLD) : NA_ALIAS
    end
end

create_feature_basics( "Time_Consecutive_Brake", :time_consec_brake, L"t_\text{brake}", Float64, L"s", 0.0, Inf, :no_na)
function Base.get(::Feature_Time_Consecutive_Brake, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer;
    THRESHOLD_BRAKING::Float64=-0.05, # [m/s²]
    )
    #=
    Scan backward to see how long the car has been decelerating
    Returns positive time values
    =#

    prev_accel = get(ACC, runlog, sn, colset, frame)
    if prev_accel > THRESHOLD_BRAKING
        return 0.0
    end

    id = colset2id(runlog, colset, frame)
    frame_past = frame
    frame_jump = N_FRAMES_PER_SIM_FRAME
    while frame_past > frame_jump && idinframe(runlog, id, frame_past-frame_jump) &&
          get(ACC, runlog, sn, id2colset(runlog, id, frame_past-frame_jump), frame_past-frame_jump) ≤ THRESHOLD_BRAKING

        frame_past -= frame_jump
    end

    get_elapsed_time(runlog, frame_past, frame)
end
create_feature_basics( "Time_Consecutive_Accel", :time_consec_accel, L"t_\text{accel}", Float64, L"s", 0.0, Inf, :no_na)
function Base.get(::Feature_Time_Consecutive_Accel, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer;
    THRESHOLD_ACCEL::Float64=0.05, # [m/s²]
    )
    #=
    Scan backward to see how long the car has been accelerating
    Returns positive time values
    =#

    prev_accel = get(ACC, runlog, sn, colset, frame)
    if prev_accel < THRESHOLD_ACCEL
        return 0.0
    end

    id = colset2id(runlog, colset, frame)
    frame_past = frame
    frame_jump = N_FRAMES_PER_SIM_FRAME
    while frame_past > frame_jump && idinframe(runlog, id, frame_past-frame_jump) &&
          get(ACC, runlog, sn, id2colset(runlog, id, frame_past-frame_jump), frame_past-frame_jump) ≥ THRESHOLD_ACCEL

        frame_past -= frame_jump
    end

    get_elapsed_time(runlog, frame_past, frame)
end
create_feature_basics( "Time_Consecutive_Throttle", :time_consec_throttle, L"t_\text{throttle}", Float64, L"s", -Inf, Inf, :no_na)
function Base.get(::Feature_Time_Consecutive_Throttle, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer;
    THRESHOLD_ACCEL::Float64=0.05, # [m/s²]
    THRESHOLD_BRAKING::Float64=-0.05, # [m/s²]
    )
    #=
    Scan backward to see how long the car has been accelerating or decelerating
    Returns a negative value for braking, positive value for accelerating
    =#

    tc_accel = get(TIME_CONSECUTIVE_ACCEL, runlog, sn, colset, frame)
    tc_brake = get(TIME_CONSECUTIVE_BRAKE, runlog, sn, colset, frame)
    tc_accel ≥ tc_brake ? tc_accel : -tc_brake
end

create_feature_basics( "N_Lane_Right", :n_lane_right, L"n^\text{lane}_r", Int, "-", 0.0, 4.0, :no_na)
function Base.get(::Feature_N_Lane_Right, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)
    node = _get_node(runlog, sn, colset, frame)
    convert(Float64, node.n_lanes_right)
end
create_feature_basics( "N_Lane_Left", :n_lane_left, L"n^\text{lane}_l", Int, "-", 0.0, 4.0, :no_na)
function Base.get(::Feature_N_Lane_Left, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)
    node = _get_node(runlog, sn, colset, frame)
    convert(Float64, node.n_lanes_left)
end
create_feature_basics( "Has_Lane_Right", :has_lane_right, L"\exists_{\text{lane}}^\text{r}", Bool, "-", :no_na)
Base.get(::Feature_Has_Lane_Right, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer) =
    Float64(get(N_LANE_RIGHT, runlog, sn , colset, frame) > 0.0)
create_feature_basics( "Has_Lane_Left", :has_lane_left, L"\exists_{\text{lane}}^\text{l}", Bool, "-", :no_na)
Base.get(::Feature_Has_Lane_Left, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer) =
    Float64(get(N_LANE_LEFT, runlog, sn , colset, frame) > 0.0)
create_feature_basics( "LaneCurvature", :curvature, L"\kappa", Float64, "1/m", -Inf, Inf, :no_na)
function Base.get(::Feature_LaneCurvature, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)
    footpoint = get(runlog, colset, frame, :footpoint)::CurvePt
    footpoint.k
end

create_feature_basics("Dist_Merge", :dist_merge, L"d_\text{merge}", Float64, L"m", 0.0, Inf, :no_na)
function Base.get(::Feature_Dist_Merge, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    # the distance from the vehicle's current position to the next merge
    # NOTE(tim): the value is technically thresholded to ~5000m, depending on the StreetNetwork map settings

    node = _get_node(runlog, sn, colset, frame)
    node.d_merge
end
create_feature_basics("Dist_Split", :dist_split, L"d_\text{split}", Float64, L"m", 0.0, Inf, :no_na)
function Base.get(::Feature_Dist_Split, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    # the distance from the vehicle's current position to the next lane split
    # NOTE(tim): the value is technically thresholded to ~5000m, depending on the StreetNetwork map settings

    node = _get_node(runlog, sn, colset, frame)
    node.d_split
end

create_feature_basics("TurnRate", :turnrate, L"\dot{\psi}", Float64, L"\radian\per\second", -Inf, Inf, :no_na)
Base.get(::Feature_TurnRate, runlog::RunLog, ::StreetNetwork, colset::UInt, frame::Integer) =
    (get(runlog, colset, frame, :ratesB)::VecSE2).θ

function get_feature_derivative_backwards(f::AbstractFeature, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)
    frame_prev = frame-N_FRAMES_PER_SIM_FRAME
    colset_prev = colset_in_other_frame(runlog, colset, frame, frame_prev)

    retval = NA_ALIAS
    if colset_prev != COLSET_NULL
        curr = get(f, runlog, sn, colset, frame)::Float64
        past = get(f, runlog, sn, colset_prev, frame_prev)::Float64
        Δt = get_elapsed_time(runlog, frame_prev, frame)
        retval = (curr - past) / Δt
    end

    retval
end

create_feature_basics("Acc", :acc, L"\|a\|", Float64, L"m/s^2", -Inf, Inf, :no_na)
function Base.get(::Feature_Acc, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)
    retval = get_feature_derivative_backwards(SPEED, runlog, sn, colset, frame)
    if is_feature_na(retval)
        retval = 0.0
    end
    retval
end
create_feature_basics("AccBx", :accBx, L"a^B_x", Float64, L"m/s^2", -Inf, Inf, :no_na)
function Base.get(::Feature_AccBx, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    retval = get_feature_derivative_backwards(VELBX, runlog, sn, colset, frame)
    if is_feature_na(retval)
        retval = 0.0
    end
    retval
end
create_feature_basics("AccBy", :accBy, L"a^B_y", Float64, L"m/s^2", -Inf, Inf, :no_na)
function Base.get(::Feature_AccBy, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    retval = get_feature_derivative_backwards(VELBY, runlog, sn, colset, frame)
    if is_feature_na(retval)
        retval = 0.0
    end
    retval
end
create_feature_basics("AccFs", :accFs, L"a^F_s", Float64, L"m/s^2", -Inf, Inf, :no_na)
function Base.get(::Feature_AccFs, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    retval = get_feature_derivative_backwards(VELFS, runlog, sn, colset, frame)
    if is_feature_na(retval)
        retval = 0.0
    end
    retval
end
create_feature_basics("AccFt", :accFt, L"a^F_t", Float64, L"m/s^2", -Inf, Inf, :no_na)
function Base.get(::Feature_AccFt, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    retval = get_feature_derivative_backwards(VELFT, runlog, sn, colset, frame)
    if is_feature_na(retval)
        retval = 0.0
    end
    retval
end
create_feature_basics("Jerk", :jerk, L"j", Float64, L"m/s\cubed", -Inf, Inf, :no_na)
function Base.get(::Feature_Jerk, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)
    retval = get_feature_derivative_backwards(ACCBX, runlog, sn, colset, frame)
    if is_feature_na(retval)
        retval = 0.0
    end
    retval
end

#############################################
#
# Driver Model Features
#
#############################################

create_feature_basics("SUMO", :sumo, L"a_\text{SUMO}", Float64, L"m/s^2", -Inf, Inf, :can_na, na_replacement=0.0)
function Base.get(::Feature_SUMO, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    #=
    SUMO accel prediction
    returns NA if no vehicle in front
    =#

    # TODO: find a better way to configure parameters

    τ = 5.0 # RESPONSE_TIME_AVERAGE
    a_max = 50.0 # ACCEL_MAX
    b_max = 0.342 #-ACCEL_MIN
    v_max = 100.0 # VEL_MAX

    colset_front = get(runlog, colset, frame, :colset_front)::UInt
    if colset_front == COLSET_NULL
        return NA_ALIAS
    end

    d_front = NaN
    try
        d_front = _get_dist_between(runlog, sn, colset, colset_front, frame)
        @assert(!isnan(d_front))
    catch
        return NA_ALIAS
    end

    v = _fast_hypot(get(runlog, colset, frame, :ratesB)::VecSE2)
    v_front = _fast_hypot(get(runlog, colset_front, frame, :ratesB)::VecSE2)
    Δt = estimate_framerate(runlog)

    v_safe = -τ*b_max + sqrt((τ*b_max)^2 + v_front^2 + 2*b_max*d_front)
    v_des = min(min(v_safe, v + a_max*(1 - v/v_max)*Δt), v_max)
    a_sumo = (max(0.0, v_des) - v_safe) / Δt

    return a_sumo
end

create_feature_basics("IDM", :idm, L"a_\text{IDM}", Float64, L"m/s^2", -Inf, Inf, :can_na, na_replacement=0.0)
function Base.get(::Feature_IDM, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    #=
    Intelligent Driver Model accel prediction
    returns NA if no vehicle in front
    =#

    # TODO(tim): configure the parameters in a better way
    a_max = 0.2235743521748731# ACCEL_MAX
    v_max = 100.0# VEL_MAX
    dmn = 1.564476357544317 # MIN_HEADWAY
    d_comf = 2.4401797787331043 # DECEL_COMFORTABLE

    colset_front = get(runlog, colset, frame, :colset_front)::UInt
    if colset_front == COLSET_NULL
        return NA_ALIAS
    end

    d_front = NaN
    try
        d_front = _get_dist_between(runlog, sn, colset, colset_front, frame)
        @assert(!isnan(d_front))
    catch
        return NA_ALIAS
    end

    v = _fast_hypot(get(runlog, colset, frame, :ratesB)::VecSE2)
    v_front = _fast_hypot(get(runlog, colset_front, frame, :ratesB)::VecSE2)
    Δv_front = v_front - v
    Δt = estimate_framerate(runlog)
    iT = get(INV_TIMEGAP_FRONT, runlog, sn, colset, frame)::Float64

    d_des = dmn + v/iT - (v*Δv_front) / (2*sqrt(a_max*d_comf))
    a_idm = a_max * (1.0 - (v/v_max)^4 - (d_des/d_front)^2)

    return a_idm
end

#############################################
#
# FRONT
#
#############################################

create_feature_basics("Has_Front", :has_front, L"\exists_\text{front}", Bool, L"-", :no_na)
function Base.get(::Feature_Has_Front, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    # true if there is a lead vehicle

    colset_front = get(runlog, colset, frame, :colset_front)::UInt
    convert(Float64, colset_front != COLSET_NULL)
end

create_feature_basics("Dist_Front", :d_front, L"d_x^\text{front}", Float64, L"m", 0.0, Inf, :can_na, na_replacement=50.0)
function Base.get(::Feature_Dist_Front, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    colset_front = get(runlog, colset, frame, :colset_front)::UInt
    if colset_front == COLSET_NULL
        return NA_ALIAS
    end

    d_front = NaN
    try
        d_front = _get_dist_between(runlog, sn, colset, colset_front, frame)
        @assert(!isnan(d_front))
    catch
        return NA_ALIAS
    end


    d_front
end

create_feature_basics("D_Y_Front", :d_y_front, L"d_y^\text{front}", Float64, L"m", -5.0, 5.0, :can_na, na_replacement=0.0)
function Base.get(::Feature_D_Y_Front, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    colset_front = get(runlog, colset, frame, :colset_front)::UInt
    if colset_front == COLSET_NULL
        return NA_ALIAS
    end

    dcl_ego = get(DIST_FROM_CENTERLINE, runlog, sn, colset, frame)
    dcl_oth = get(DIST_FROM_CENTERLINE, runlog, sn, colset_front, frame)

    dcl_ego - dcl_oth
end

create_feature_basics("Delta_V_Front", :dv_x_front, L"\Delta v_x^\text{front}", Float64, L"m/s", -Inf, Inf, :can_na, na_replacement=0.0)
function Base.get(::Feature_Delta_V_Front, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    colset_front = get(runlog, colset, frame, :colset_front)::UInt
    if colset_front == COLSET_NULL
        return NA_ALIAS
    end

    v_ego = (get(runlog, colset, frame, :ratesF)::VecE2).x
    v_oth = (get(runlog, colset_front, frame, :ratesF)::VecE2).x

    v_oth - v_ego
end

create_feature_basics("Delta_V_Y_Front", :dv_y_front, L"\Delta v_y^\text{front}", Float64, L"m/s", -Inf, Inf, :can_na, na_replacement=0.0)
function Base.get(::Feature_Delta_V_Y_Front, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    colset_front = get(runlog, colset, frame, :colset_front)::UInt
    if colset_front == COLSET_NULL
        return NA_ALIAS
    end

    v_ego = (get(runlog, colset, frame, :ratesF)::VecE2).y
    v_oth = (get(runlog, colset_front, frame, :ratesF)::VecE2).y

    v_oth - v_ego
end

create_feature_basics("Yaw_Front", :yaw_front, L"\psi^\text{front}", Float64, L"\radian", -Inf, Inf, :can_na, na_replacement=0.0)
function Base.get(::Feature_Yaw_Front, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    colset_front = get(runlog, colset, frame, :colset_front)::UInt
    if colset_front == COLSET_NULL
        return NA_ALIAS
    end

    (get(runlog, colset_front, frame, :frenet)::VecSE2).θ
end

create_feature_basics("Turnrate_Front", :turnrate_front, L"\dot{\psi}^\text{front}", Float64, L"\radian\per\second", -Inf, Inf, :can_na, na_replacement=0.0)
function Base.get(::Feature_Turnrate_Front, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    colset_front = get(runlog, colset, frame, :colset_front)::UInt
    if colset_front == COLSET_NULL
        return NA_ALIAS
    end

    get(TURNRATE, runlog, sn, colset_front, frame)
end

create_feature_basics("Acc_Req_Front", :acc_req_front, L"a_\text{req}^\text{front}", Float64, L"m/s^2", -Inf, Inf, :can_na, na_replacement=0.0)
function Base.get(::Feature_Acc_Req_Front, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    # the const. acceleration required to avoid a collision assuming
    # everyone drives with constant frenet-x velocity

    colset_front = get(runlog, colset, frame, :colset_front)::UInt
    if colset_front == COLSET_NULL
        return NA_ALIAS
    end

    dv = get(DELTA_V_FRONT, runlog, sn, colset, frame)

    if dv ≥ 0.0 # they are pulling away; we are good
        return NA_ALIAS
    end

    dx = get(DIST_FRONT, runlog, sn, colset, frame)

    -dv*dv / (2dx)
end

create_feature_basics("Inv_TTC_Front", :inv_ttc_front, L"ttc_\text{inv}^\text{front}", Float64, L"\per\sec", 0.0, Inf, :can_na, na_replacement=0.0)
function Base.get(::Feature_Inv_TTC_Front, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    # the inverse time to collision with lead vehicle

    colset_front = get(runlog, colset, frame, :colset_front)::UInt
    if colset_front == COLSET_NULL
        return NA_ALIAS
    end

    dv = get(DELTA_V_FRONT, runlog, sn, colset, frame)

    if dv ≥ 0.0 # they are pulling away; we are good
        return NA_ALIAS
    end

    dx = get(DIST_FRONT, runlog, sn, colset, frame)

    -dv / dx
end

create_feature_basics("Inv_Timegap_Front", :inv_timegap_front, L"timegap_\text{inv}^\text{front}", Float64, L"\per\sec", 0.0, Inf, :can_na, na_replacement=0.0)
function Base.get(::Feature_Inv_Timegap_Front, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    # the inverse timegap with lead vehicle
    colset_front = get(runlog, colset, frame, :colset_front)::UInt
    if colset_front == COLSET_NULL
        return NA_ALIAS
    end

    v = (get(runlog, colset, frame, :ratesF)::VecE2).x

    if v ≤ 0.0
        return 0.0
    end

    dx = get(DIST_FRONT, runlog, sn, colset, frame)

    v / dx
end

create_feature_basics("Gaining_On_Front", :gaining_on_front, L"gaining^\text{front}", Bool, L"-", :no_na)
function Base.get(::Feature_Gaining_On_Front, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    colset_front = get(runlog, colset, frame, :colset_front)::UInt
    retval = false
    if colset_front != COLSET_NULL
        v_ego = (get(runlog, colset, frame, :ratesF)::VecE2).x
        v_oth = (get(runlog, colset_front, frame, :ratesF)::VecE2).x
        retval = v_ego > v_oth
    end

    convert(Float64, retval)
end

#############################################
#
# REAR
#
#############################################

create_feature_basics("Has_Rear", :has_rear, L"\exists_\text{rear}", Bool, L"-", :no_na)
function Base.get(::Feature_Has_Rear, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    # true if there is a following vehicle

    colset_rear = get(runlog, colset, frame, :colset_rear)::UInt
    convert(Float64, colset_rear != COLSET_NULL)
end

create_feature_basics("Dist_Rear", :d_rear, L"d_x^\text{rear}", Float64, L"m", 0.0, Inf, :can_na, na_replacement=50.0)
function Base.get(::Feature_Dist_Rear, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    colset_rear = get(runlog, colset, frame, :colset_rear)::UInt
    if colset_rear == COLSET_NULL
        return NA_ALIAS
    end

    d_rear = NaN
    try
        d_rear = _get_dist_between(runlog, sn, colset_rear, colset, frame)
        @assert(!isnan(d_rear))
    catch
        return NA_ALIAS
    end

    d_rear

    #
    # d_rear
end

create_feature_basics("D_Y_Rear", :d_y_rear, L"d_y^\text{rear}", Float64, L"m", -5.0, 5.0, :can_na, na_replacement=0.0)
function Base.get(::Feature_D_Y_Rear, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    colset_rear = get(runlog, colset, frame, :colset_rear)::UInt
    if colset_rear == COLSET_NULL
        return NA_ALIAS
    end

    dcl_ego = get(DIST_FROM_CENTERLINE, runlog, sn, colset, frame)
    dcl_oth = get(DIST_FROM_CENTERLINE, runlog, sn, colset_rear, frame)

    dcl_ego - dcl_oth
end

create_feature_basics("Delta_V_Rear", :dv_x_rear, L"\Delta v_x^\text{rear}", Float64, L"m/s", -Inf, Inf, :can_na, na_replacement=0.0)
function Base.get(::Feature_Delta_V_Rear, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    colset_rear = get(runlog, colset, frame, :colset_rear)::UInt
    if colset_rear == COLSET_NULL
        return NA_ALIAS
    end

    v_ego = (get(runlog, colset, frame, :ratesF)::VecE2).x
    v_oth = (get(runlog, colset_rear, frame, :ratesF)::VecE2).x

    v_oth - v_ego
end

create_feature_basics("Delta_V_Y_Rear", :dv_y_rear, L"\Delta v_y^\text{rear}", Float64, L"m/s", -Inf, Inf, :can_na, na_replacement=0.0)
function Base.get(::Feature_Delta_V_Y_Rear, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    colset_rear = get(runlog, colset, frame, :colset_rear)::UInt
    if colset_rear == COLSET_NULL
        return NA_ALIAS
    end

    v_ego = (get(runlog, colset, frame, :ratesF)::VecE2).y
    v_oth = (get(runlog, colset_rear, frame, :ratesF)::VecE2).y

    v_oth - v_ego
end

create_feature_basics("Yaw_Rear", :yaw_rear, L"\psi^\text{rear}", Float64, L"\radian", -Inf, Inf, :can_na, na_replacement=0.0)
function Base.get(::Feature_Yaw_Rear, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    colset_rear = get(runlog, colset, frame, :colset_rear)::UInt
    if colset_rear == COLSET_NULL
        return NA_ALIAS
    end

    (get(runlog, colset_rear, frame, :frenet)::VecSE2).θ
end

create_feature_basics("Turnrate_Rear", :turnrate_rear, L"\dot{\psi}^\text{rear}", Float64, L"\radian\per\second", -Inf, Inf, :can_na, na_replacement=0.0)
function Base.get(::Feature_Turnrate_Rear, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    colset_rear = get(runlog, colset, frame, :colset_rear)::UInt
    if colset_rear == COLSET_NULL
        return NA_ALIAS
    end

    get(TURNRATE, runlog, sn, colset_rear, frame)
end

create_feature_basics("Acc_Req_Rear", :acc_req_rear, L"a_\text{req}^\text{rear}", Float64, L"m/s^2", -Inf, Inf, :can_na, na_replacement=0.0)
function Base.get(::Feature_Acc_Req_Rear, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    # the const. acceleration required to avoid a collision assuming
    # everyone drives with constant frenet-x velocity

    colset_rear = get(runlog, colset, frame, :colset_rear)::UInt
    if colset_rear == COLSET_NULL
        return NA_ALIAS
    end

    dv = get(DELTA_V_REAR, runlog, sn, colset, frame)

    if dv ≤ 0.0 # they are pulling away; we are good
        return NA_ALIAS
    end

    dx = get(DIST_REAR, runlog, sn, colset, frame)

    dv*dv / (2dx)
end

create_feature_basics("Inv_TTC_Rear", :inv_ttc_rear, L"ttc_\text{inv}^\text{rear}", Float64, L"\per\sec", 0.0, Inf, :can_na, na_replacement=0.0)
function Base.get(::Feature_Inv_TTC_Rear, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    # the inverse time to collision with lead vehicle

    colset_rear = get(runlog, colset, frame, :colset_rear)::UInt
    if colset_rear == COLSET_NULL
        return NA_ALIAS
    end

    dv = get(DELTA_V_REAR, runlog, sn, colset, frame)

    if dv ≤ 0.0 # we are pulling away; we are good
        return NA_ALIAS
    end

    dx = get(DIST_REAR, runlog, sn, colset, frame)

    dv / dx
end

create_feature_basics("Inv_Timegap_Rear", :inv_timegap_rear, L"timegap_\text{inv}^\text{rear}", Float64, L"\per\sec", 0.0, Inf, :can_na, na_replacement=0.0)
function Base.get(::Feature_Inv_Timegap_Rear, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    # the inverse timegap with lead vehicle

    colset_rear = get(runlog, colset, frame, :colset_rear)::UInt
    if colset_rear == COLSET_NULL
        return NA_ALIAS
    end

    v = (get(runlog, colset, frame, :ratesF)::VecE2).x

    if v ≤ 0.0
        return 0.0
    end

    dx = get(DIST_REAR, runlog, sn, colset, frame)

    v / dx
end

create_feature_basics("Rear_Is_Gaining", :gaining_on_rear, L"gaining^\text{rear}", Bool, L"-", :can_na, na_replacement=0.0)
function Base.get(::Feature_Rear_Is_Gaining, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    colset_rear = get(runlog, colset, frame, :colset_rear)::UInt
    retval = false
    if colset_rear != COLSET_NULL
        v_ego = (get(runlog, colset, frame, :ratesF)::VecE2).x
        v_oth = (get(runlog, colset_rear, frame, :ratesF)::VecE2).x
        retval = v_oth > v_ego
    end

    convert(Float64, retval)
end

#############################################
#
# LEFT
#
#############################################

function _get_vehicle_to_left(runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer;
    GAP_LO::Float64 = -20.0, # [m]
    GAP_HI::Float64 = 200.0, # [m]
    GAP_T::Float64 = 1.8,  # [m]
    )

    #=
    Returns the id of the vehicle in the left-neighboring lane if it exists, otherwise returns ID_NULL

    - vehicle must be in the neighboring lane
    - vehicle must be within GAP_LO ≤ s ≤ GAP_HI
    =#

    # 1 - take the local tangent vector to your lane
    # 2 - offset it by 2d_marker_left
    # 3 - project all other vehicles to the linear-assumed lane
    # 4 - retain the vehicle that is closest in s that falls within the lane boundary

    best_colset = COLSET_NULL

    node = _get_node(runlog, sn, colset, frame)
    if node.n_lanes_left > 0
        dcl = 2node.marker_dist_left
        local_tangent_vector = node.pos
        left_lane_tangent = local_tangent_vector + Vec.polar(dcl, local_tangent_vector.θ + π/2)

        bestΔs = Inf

        for colset2 in get_colset_range(runlog, frame)
            if colset2 != colset
                # project vehicle to left lane tangent
                pos = get(runlog, colset2, frame, :inertial)::VecSE2
                pos_rel = inertial2body(pos, left_lane_tangent)
                if GAP_LO ≤ pos_rel.x ≤ GAP_HI && abs(pos_rel.y) ≤ GAP_T && abs(pos_rel.x) ≤ bestΔs
                    bestΔs = abs(pos_rel.x)
                    best_colset = colset2
                end
            end
        end
    end

    best_colset
end

create_feature_basics("Has_Left", :has_left, L"\exists_\text{left}", Bool, L"-", :no_na)
function Base.get(::Feature_Has_Left, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)
    colset_left = _get_vehicle_to_left(runlog, sn, colset, frame)
    convert(Float64, colset_left != COLSET_NULL)
end
create_feature_basics("Dist_Left", :d_left, L"d_x^\text{left}", Float64, L"m", -Inf, Inf, :can_na, na_replacement=50.0)
function Base.get(::Feature_Dist_Left, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    #=
    Returns the relative distance between the left and ego vehicle
        it is positive if left is in front of ego
        it is negative if left is behind the ego vehicle
    =#

    colset_left = _get_vehicle_to_left(runlog, sn, colset, frame)
    if colset_left == COLSET_NULL
        return NA_ALIAS
    end

    pos_ego = get(runlog, colset, frame, :inertial)::VecSE2
    pos_oth = get(runlog, colset_left, frame, :inertial)::VecSE2
    inertial2body(pos_oth, pos_ego).x
end
create_feature_basics("D_Y_Left", :d_y_left, L"d_y^\text{left}", Float64, L"m", -5.0, 5.0, :can_na, na_replacement=0.0)
function Base.get(::Feature_D_Y_Left, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    colset_left = _get_vehicle_to_left(runlog, sn, colset, frame)
    if colset_left == COLSET_NULL
        return NA_ALIAS
    end

    pos_ego = get(runlog, colset, frame, :inertial)::VecSE2
    pos_oth = get(runlog, colset_left, frame, :inertial)::VecSE2
    inertial2body(pos_oth, pos_ego).y
end
create_feature_basics("Delta_V_Left", :dv_x_left, L"\Delta v_x^\text{left}", Float64, L"m/s", -Inf, Inf, :can_na, na_replacement=0.0)
function Base.get(::Feature_Delta_V_Left, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    colset_left = _get_vehicle_to_left(runlog, sn, colset, frame)
    if colset_left == COLSET_NULL
        return NA_ALIAS
    end

    v_ego = (get(runlog, colset, frame, :ratesF)::VecE2).x
    v_oth = (get(runlog, colset_left, frame, :ratesF)::VecE2).x

    v_oth - v_ego
end
create_feature_basics("Delta_V_Y_Left", :dv_y_left, L"\Delta v_y^\text{left}", Float64, L"m/s", -Inf, Inf, :can_na, na_replacement=0.0)
function Base.get(::Feature_Delta_V_Y_Left, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    colset_left = _get_vehicle_to_left(runlog, sn, colset, frame)
    if colset_left == COLSET_NULL
        return NA_ALIAS
    end

    v_ego = (get(runlog, colset, frame, :ratesF)::VecE2).y
    v_oth = (get(runlog, colset_left, frame, :ratesF)::VecE2).y

    v_oth - v_ego
end
create_feature_basics("Yaw_Left", :yaw_left, L"\psi^\text{left}", Float64, L"\radian", -Inf, Inf, :can_na, na_replacement=0.0)
function Base.get(::Feature_Yaw_Left, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    colset_left = _get_vehicle_to_left(runlog, sn, colset, frame)
    if colset_left == COLSET_NULL
        return NA_ALIAS
    end

    (get(runlog, colset_left, frame, :frenet)::VecSE2).θ
end
create_feature_basics("Turnrate_Left", :turnrate_left, L"\dot{\psi}^\text{left}", Float64, L"\radian\per\second", -Inf, Inf, :can_na, na_replacement=0.0)
function Base.get(::Feature_Turnrate_Left, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    colset_left = _get_vehicle_to_left(runlog, sn, colset, frame)
    if colset_left == COLSET_NULL
        return NA_ALIAS
    end

    get(TURNRATE, runlog, sn, colset_left, frame)
end
create_feature_basics("Acc_Req_Left", :acc_req_left, L"a_\text{req}^\text{left}", Float64, L"m/s^2", -Inf, Inf, :can_na, na_replacement=0.0)
function Base.get(::Feature_Acc_Req_Left, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    dv = get(DELTA_V_LEFT, runlog, sn, colset, frame)
    if is_feature_na(dv)
        return NA_ALIAS
    end

    dx = get(DIST_LEFT, runlog, sn, colset, frame)
    if dx ≥ 0.0 && dv ≥ 0.0 || # they are pulling away; we are good
       dx ≤ 0.0 && dv ≤ 0.0 # they are pulling away; we are good
        return NA_ALIAS
    end

    dv*dv / (2dx)
end
create_feature_basics("Inv_TTC_Left", :inv_ttc_left, L"ttc_\text{inv}^\text{left}", Float64, L"\per\sec", 0.0, Inf, :can_na, na_replacement=0.0)
function Base.get(::Feature_Inv_TTC_Left, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    dv = get(DELTA_V_LEFT, runlog, sn, colset, frame)
    if is_feature_na(dv)
        return NA_ALIAS
    end

    dx = get(DIST_LEFT, runlog, sn, colset, frame)
    if dx ≥ 0.0 && dv ≥ 0.0 || # they are pulling away; we are good
       dx ≤ 0.0 && dv ≤ 0.0 # they are pulling away; we are good

       return NA_ALIAS
    end

    dv / dx
end
create_feature_basics("Inv_Timegap_Left", :inv_timegap_left, L"timegap_\text{inv}^\text{left}", Float64, L"\per\sec", 0.0, Inf, :can_na, na_replacement=0.0)
function Base.get(::Feature_Inv_Timegap_Left, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    dx = get(DIST_LEFT, runlog, sn, colset, frame)
    if is_feature_na(dx)
        return NA_ALIAS
    end

    v = (get(runlog, colset, frame, :ratesF)::VecE2).x

    v / dx
end
create_feature_basics("Left_Is_Gaining", :left_is_gaining, L"gaining^\text{left}", Bool, L"-", :can_na, na_replacement=0.0)
function Base.get(::Feature_Left_Is_Gaining, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    retval = false

    dv = get(DELTA_V_LEFT, runlog, sn, colset, frame)
    if !is_feature_na(dv)
        dx = get(DIST_LEFT, runlog, sn, colset, frame)
        if dx ≥ 0.0 && dv ≤ 0.0 ||
           dx ≤ 0.0 && dv ≥ 0.0

           retval = true
        end
    end

    convert(Float64, retval)
end

#############################################
#
# RIGHT
#
#############################################

function _get_vehicle_to_right(runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer;
    GAP_LO::Float64 = -20.0, # [m]
    GAP_HI::Float64 = 200.0, # [m]
    GAP_T::Float64 = 1.8,  # [m]
    )

    #=
    Returns the id of the vehicle in the right-neighboring lane if it exists, otherwise returns ID_NULL

    - vehicle must be in the neighboring lane
    - vehicle must be within GAP_LO ≤ s ≤ GAP_HI
    =#

    # 1 - take the local tangent vector to your lane
    # 2 - offset it by 2d_marker_right
    # 3 - project all other vehicles to the linear-assumed lane
    # 4 - retain the vehicle that is closest in s that falls within the lane boundary

    best_colset = COLSET_NULL

    node = _get_node(runlog, sn, colset, frame)
    if node.n_lanes_right > 0
        dcl = 2node.marker_dist_right
        local_tangent_vector = node.pos
        right_lane_tangent = local_tangent_vector + Vec.polar(dcl, local_tangent_vector.θ - π/2)

        bestΔs = Inf

        for colset2 in get_colset_range(runlog, frame)
            if colset2 != colset
                # project vehicle to right lane tangent
                pos = get(runlog, colset2, frame, :inertial)::VecSE2
                pos_rel = inertial2body(pos, right_lane_tangent)
                if GAP_LO ≤ pos_rel.x ≤ GAP_HI && abs(pos_rel.y) ≤ GAP_T && abs(pos_rel.x) ≤ bestΔs
                    bestΔs = abs(pos_rel.x)
                    best_colset = colset2
                end
            end
        end
    end

    best_colset
end

create_feature_basics("Has_Right", :has_right, L"\exists_\text{right}", Bool, L"-", :no_na)
function Base.get(::Feature_Has_Right, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)
    colset_right = _get_vehicle_to_right(runlog, sn, colset, frame)
    convert(Float64, colset_right != COLSET_NULL)
end
create_feature_basics("Dist_Right", :d_right, L"d_x^\text{right}", Float64, L"m", -Inf, Inf, :can_na, na_replacement=50.0)
function Base.get(::Feature_Dist_Right, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    #=
    Returns the relative distance between the right and ego vehicle
        it is positive if right is in front of ego
        it is negative if right is behind the ego vehicle
    =#

    colset_right = _get_vehicle_to_right(runlog, sn, colset, frame)
    if colset_right == COLSET_NULL
        return NA_ALIAS
    end

    pos_ego = get(runlog, colset, frame, :inertial)::VecSE2
    pos_oth = get(runlog, colset_right, frame, :inertial)::VecSE2
    inertial2body(pos_oth, pos_ego).x
end
create_feature_basics("D_Y_Right", :d_y_right, L"d_y^\text{right}", Float64, L"m", -5.0, 5.0, :can_na, na_replacement=0.0)
function Base.get(::Feature_D_Y_Right, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    colset_right = _get_vehicle_to_right(runlog, sn, colset, frame)
    if colset_right == COLSET_NULL
        return NA_ALIAS
    end

    pos_ego = get(runlog, colset, frame, :inertial)::VecSE2
    pos_oth = get(runlog, colset_right, frame, :inertial)::VecSE2
    inertial2body(pos_oth, pos_ego).y
end
create_feature_basics("Delta_V_Right", :dv_x_right, L"\Delta v_x^\text{right}", Float64, L"m/s", -Inf, Inf, :can_na, na_replacement=0.0)
function Base.get(::Feature_Delta_V_Right, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    colset_right = _get_vehicle_to_right(runlog, sn, colset, frame)
    if colset_right == COLSET_NULL
        return NA_ALIAS
    end

    v_ego = (get(runlog, colset, frame, :ratesF)::VecE2).x
    v_oth = (get(runlog, colset_right, frame, :ratesF)::VecE2).x

    v_oth - v_ego
end
create_feature_basics("Delta_V_Y_Right", :dv_y_right, L"\Delta v_y^\text{right}", Float64, L"m/s", -Inf, Inf, :can_na, na_replacement=0.0)
function Base.get(::Feature_Delta_V_Y_Right, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    colset_right = _get_vehicle_to_right(runlog, sn, colset, frame)
    if colset_right == COLSET_NULL
        return NA_ALIAS
    end

    v_ego = (get(runlog, colset, frame, :ratesF)::VecE2).y
    v_oth = (get(runlog, colset_right, frame, :ratesF)::VecE2).y

    v_oth - v_ego
end
create_feature_basics("Yaw_Right", :yaw_right, L"\psi^\text{right}", Float64, L"\radian", -Inf, Inf, :can_na, na_replacement=0.0)
function Base.get(::Feature_Yaw_Right, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    colset_right = _get_vehicle_to_right(runlog, sn, colset, frame)
    if colset_right == COLSET_NULL
        return NA_ALIAS
    end

    (get(runlog, colset_right, frame, :frenet)::VecSE2).θ
end
create_feature_basics("Turnrate_Right", :turnrate_right, L"\dot{\psi}^\text{right}", Float64, L"\radian\per\second", -Inf, Inf, :can_na, na_replacement=0.0)
function Base.get(::Feature_Turnrate_Right, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    colset_right = _get_vehicle_to_right(runlog, sn, colset, frame)
    if colset_right == COLSET_NULL
        return NA_ALIAS
    end

    get(TURNRATE, runlog, sn, colset_right, frame)
end
create_feature_basics("Acc_Req_Right", :acc_req_right, L"a_\text{req}^\text{right}", Float64, L"m/s^2", -Inf, Inf, :can_na, na_replacement=0.0)
function Base.get(::Feature_Acc_Req_Right, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    dv = get(DELTA_V_RIGHT, runlog, sn, colset, frame)
    if is_feature_na(dv)
        return NA_ALIAS
    end

    dx = get(DIST_RIGHT, runlog, sn, colset, frame)
    if dx ≥ 0.0 && dv ≥ 0.0 || # they are pulling away; we are good
       dx ≤ 0.0 && dv ≤ 0.0 # they are pulling away; we are good
        return NA_ALIAS
    end

    dv*dv / (2dx)
end
create_feature_basics("Inv_TTC_Right", :inv_ttc_right, L"ttc_\text{inv}^\text{right}", Float64, L"\per\sec", 0.0, Inf, :can_na, na_replacement=0.0)
function Base.get(::Feature_Inv_TTC_Right, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    dv = get(DELTA_V_RIGHT, runlog, sn, colset, frame)
    if is_feature_na(dv)
        return NA_ALIAS
    end

    dx = get(DIST_RIGHT, runlog, sn, colset, frame)
    if dx ≥ 0.0 && dv ≥ 0.0 || # they are pulling away; we are good
       dx ≤ 0.0 && dv ≤ 0.0 # they are pulling away; we are good

       return NA_ALIAS
    end

    dv / dx
end
create_feature_basics("Inv_Timegap_Right", :inv_timegap_right, L"timegap_\text{inv}^\text{right}", Float64, L"\per\sec", 0.0, Inf, :can_na, na_replacement=0.0)
function Base.get(::Feature_Inv_Timegap_Right, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    dx = get(DIST_RIGHT, runlog, sn, colset, frame)
    if is_feature_na(dx)
        return NA_ALIAS
    end

    v = (get(runlog, colset, frame, :ratesF)::VecE2).x

    v / dx
end
create_feature_basics("Right_Is_Gaining", :right_is_gaining, L"gaining^\text{right}", Bool, L"-", :can_na, na_replacement=0.0)
function Base.get(::Feature_Right_Is_Gaining, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    retval = false

    dv = get(DELTA_V_RIGHT, runlog, sn, colset, frame)
    if !is_feature_na(dv)
        dx = get(DIST_RIGHT, runlog, sn, colset, frame)
        if dx ≥ 0.0 && dv ≤ 0.0 ||
           dx ≤ 0.0 && dv ≥ 0.0

           retval = true
        end
    end

    convert(Float64, retval)
end

#############################################
#
#
#
#############################################

create_feature_basics("FutureAcceleration", :f_accel, L"a_{t+1}", Float64, L"m/s^2", -Inf, Inf, :can_na, na_replacement=0.0, target=:lon)
function Base.get(::Feature_FutureAcceleration, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    frame_fut = frame + N_FRAMES_PER_SIM_FRAME
    colset_fut = colset_in_other_frame(runlog, colset, frame, frame_fut)

    retval = NA_ALIAS
    if colset_fut != COLSET_NULL
        retval = get(ACCBX, runlog, sn, colset_fut, frame_fut)
    end

    retval
end
create_feature_basics("Future_Delta_Accel", :f_delta_accel, L"\Delta a_{t+1}", Float64, L"m/s^2", -Inf, Inf, :can_na, na_replacement=0.0, target=:lon)
function Base.get(::Feature_Future_Delta_Accel, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    frame_fut = frame + N_FRAMES_PER_SIM_FRAME
    colset_fut = colset_in_other_frame(runlog, colset, frame, frame_fut)

    retval = NA_ALIAS
    if colset_fut != COLSET_NULL
        retval = get(ACCBX, runlog, sn, colset_fut, frame_fut) - get(ACCBX, runlog, sn, colset, frame)
    end

    retval
end
create_feature_basics( "FutureDesiredAngle", :f_des_angle, L"\phi^{\text{des}}", Float64, L"\radian", -convert(Float64, π), convert(Float64, π), :can_na, na_replacement=0.0, target=:lat)
function Base.get(::Feature_FutureDesiredAngle, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    frame_fut = frame + N_FRAMES_PER_SIM_FRAME
    colset_fut = colset_in_other_frame(runlog, colset, frame, frame_fut)

    retval = NA_ALIAS
    if colset_fut != COLSET_NULL
        curϕ = (get(runlog, colset, frame, :frenet)::VecSE2).θ
        futϕ = (get(runlog, colset_fut, frame_fut, :frenet)::VecSE2).θ
        Δt = get_elapsed_time(runlog, frame, frame_fut)
        expconst = exp(-KP_DESIRED_ANGLE*Δt)
        retval = (futϕ - curϕ*expconst) / (1.0 - expconst)
    end

    retval
end
create_feature_basics( "FutureTurnrate", :f_turnrate, L"\dot{\phi}_{t+1}}", Float64, L"\radian\per\second", -Inf, Inf, :can_na, na_replacement=0.0, target=:lat)
function Base.get(::Feature_FutureTurnrate, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    frame_fut = frame + N_FRAMES_PER_SIM_FRAME
    colset_fut = colset_in_other_frame(runlog, colset, frame, frame_fut)

    retval = NA_ALIAS
    if colset_fut != COLSET_NULL
        retval = get(TURNRATE, runlog, sn, colset_fut, frame_fut)
    end

    retval
end
create_feature_basics( "FutureVelFt", :f_velFy, L"v^f_{y, t+1}}", Float64, L"m/s", -Inf, Inf, :can_na, na_replacement=0.0, target=:lat)
function Base.get(::Feature_FutureVelFt, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    frame_fut = frame + N_FRAMES_PER_SIM_FRAME
    colset_fut = colset_in_other_frame(runlog, colset, frame, frame_fut)

    retval = NA_ALIAS
    if colset_fut != COLSET_NULL
        retval = get(VELFT, runlog, sn, colset_fut, frame_fut)
    end

    retval
end

#############################################
#
# Templated Features
#
#############################################

immutable Feature_IsClean{target_symbol} <: AbstractFeature end
units(          ::Feature_IsClean) = "-"
isint(          ::Feature_IsClean) = true
isbool(         ::Feature_IsClean) = true
lowerbound(     ::Feature_IsClean) = 0.0
upperbound(     ::Feature_IsClean) = 1.0
couldna(        ::Feature_IsClean) = false
Base.symbol{F}( ::Feature_IsClean{F}) = symbol("isclean_" * string(F))
lsymbol{F}(     ::Feature_IsClean{F}) = latexstring(replace(L"\texttt{isclean}\left(" * lsymbol(symbol2feature(F)) * L"\right)", "\$", ""))
function Base.get{F}(::Feature_IsClean{F}, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)
    f = symbol2feature(F)
    v = get(f, runlog, sn, colset, frame)::Float64
    convert(Float64, !isnan(v) && !isinf(v))
end

"""
A past feature
 target::Symbol - the symbol for the relevant feature
 history::Int - number of frames previous to extract the feature for

Will be NA if insufficient history
"""
immutable Feature_Past{target, history} <: AbstractFeature end
units{F, H}(       ::Feature_Past{F, H}) = units(symbol2feature(F))
isint{F, H}(       ::Feature_Past{F, H}) = isint(symbol2feature(F))
isbool{F, H}(      ::Feature_Past{F, H}) = isbool(symbol2feature(F))
lowerbound{F, H}(  ::Feature_Past{F, H}) = lowerbound(symbol2feature(F))
upperbound{F, H}(  ::Feature_Past{F, H}) = upperbound(symbol2feature(F))
couldna(           ::Feature_Past)       = true
Base.symbol{F, H}( ::Feature_Past{F, H}) = symbol(@sprintf("past_%d_%s", H, string(F)))
lsymbol{F, H}(     ::Feature_Past{F, H}) = latexstring(replace(L"\texttt{past}\left(" * lsymbol(symbol2feature(F)) * L"\right)_{" * string(H) * L"}", "\$", ""))
function replace_na{F, H}(::Feature_Past{F, H})
    try
        replace_na(symbol2feature(F))
    catch
        0.0
    end
end
function Base.get{F, H}(::Feature_Past{F, H}, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)
    f = symbol2feature(F)

    frame_past = frame-1
    colset_past = colset_in_other_frame(runlog, colset, frame, frame_past)

    retval = NA_ALIAS
    if colset_past != COLSET_NULL
        retval = get(f, runlog, sn, colset_past, frame_past)::Float64
    end

    retval
end

"""
target::Symbol - the symbol for the relevant feature
history::Int - number of frames previous to extract the feature for
               (typically a multiple of N_FRAMES_PER_SIM_FRAME)
"""
immutable Feature_Mean_Over_History{target, history} <: AbstractFeature end
units{F, H}(       ::Feature_Mean_Over_History{F, H}) = units(symbol2feature(F))
isint{F, H}(       ::Feature_Mean_Over_History{F, H}) = false
isbool{F, H}(      ::Feature_Mean_Over_History{F, H}) = false
lowerbound{F, H}(  ::Feature_Mean_Over_History{F, H}) = lowerbound(symbol2feature(F))
upperbound{F, H}(  ::Feature_Mean_Over_History{F, H}) = upperbound(symbol2feature(F))
couldna(           ::Feature_Mean_Over_History)       = true
Base.symbol{F, H}( ::Feature_Mean_Over_History{F, H}) = symbol(@sprintf("mean_%d_%s", H, string(F)))
lsymbol{F, H}(     ::Feature_Mean_Over_History{F, H}) = latexstring(replace(L"\texttt{mean}\left(" * lsymbol(symbol2feature(F)) * L"\right)_{" * string(H) * L"}", "\$", ""))
function replace_na{F, H}(::Feature_Mean_Over_History{F, H})
    try
        replace_na(symbol2feature(F))
    catch
        0.0
    end
end
function Base.get{F, H}(::Feature_Mean_Over_History{F, H}, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)
    f = symbol2feature(F)
    id = colset2id(runlog, colset, frame)

    var = StreamStats.Variance()
    for jump in -H : N_FRAMES_PER_SIM_FRAME : -1

        frame_past = frame + jump

        if frame_inbounds(runlog, frame_past)
            colset_past = id2colset(runlog, id, frame_past)
            v = get(f, runlog, sn, colset_past, frame_past)::Float64
            if !is_feature_na(v)
                update!(var, v)
            end
        end
    end

    if StreamStats.nobs(var) == 0
        return NA_ALIAS
    end
    mean(var)
end

immutable Feature_Std_Over_History{target, history} <: AbstractFeature end
units{F, H}(       ::Feature_Std_Over_History{F, H}) = units(symbol2feature(F))
isint{F, H}(       ::Feature_Std_Over_History{F, H}) = false
isbool{F, H}(      ::Feature_Std_Over_History{F, H}) = false
lowerbound{F, H}(  ::Feature_Std_Over_History{F, H}) = lowerbound(symbol2feature(F))
upperbound{F, H}(  ::Feature_Std_Over_History{F, H}) = upperbound(symbol2feature(F))
couldna(           ::Feature_Std_Over_History)       = true
Base.symbol{F, H}( ::Feature_Std_Over_History{F, H}) = symbol(@sprintf("std_%d_%s", H, string(F)))
lsymbol{F, H}(     ::Feature_Std_Over_History{F, H}) = latexstring(replace(L"\texttt{std}\left(" * lsymbol(symbol2feature(F)) * L"\right)_{" * string(H) * L"}", "\$", ""))
function replace_na{F, H}(::Feature_Std_Over_History{F, H})
    try
        replace_na(symbol2feature(F))
    catch
        0.0
    end
end
function Base.get{F, H}(::Feature_Std_Over_History{F, H}, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)
    f = symbol2feature(F)
    id = colset2id(runlog, colset, frame)

    var = StreamStats.Variance()
    for jump in -H : N_FRAMES_PER_SIM_FRAME : -1

        frame_past = frame + jump

        if frame_inbounds(runlog, frame_past)
            colset_past = id2colset(runlog, id, frame_past)
            v = get(f, runlog, sn, colset_past, frame_past)::Float64
            if !is_feature_na(v)
                update!(var, v)
            end
        end
    end

    if StreamStats.nobs(var) == 0
        return NA_ALIAS
    end
    std(var)
end

immutable Feature_Max_Over_History{target, history} <: AbstractFeature end
units{F, H}(       ::Feature_Max_Over_History{F, H}) = units(symbol2feature(F))
isint{F, H}(       ::Feature_Max_Over_History{F, H}) = false
isbool{F, H}(      ::Feature_Max_Over_History{F, H}) = false
lowerbound{F, H}(  ::Feature_Max_Over_History{F, H}) = lowerbound(symbol2feature(F))
upperbound{F, H}(  ::Feature_Max_Over_History{F, H}) = upperbound(symbol2feature(F))
couldna(           ::Feature_Max_Over_History)       = true
Base.symbol{F, H}( ::Feature_Max_Over_History{F, H}) = symbol(@sprintf("max_%d_%s", H, string(F)))
lsymbol{F, H}(     ::Feature_Max_Over_History{F, H}) = latexstring(replace(L"\texttt{max}\left(" * lsymbol(symbol2feature(F)) * L"\right)_{" * string(H) * L"}", "\$", ""))
function replace_na{F, H}(::Feature_Max_Over_History{F, H})
    try
        replace_na(symbol2feature(F))
    catch
        0.0
    end
end
function Base.get{F, H}(::Feature_Max_Over_History{F, H}, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)
    f = symbol2feature(F)
    id = colset2id(runlog, colset, frame)

    var = StreamStats.Max()
    for jump in -H : N_FRAMES_PER_SIM_FRAME : -1

        frame_past = frame + jump

        if frame_inbounds(runlog, frame_past)
            colset_past = id2colset(runlog, id, frame_past)
            v = get(f, runlog, sn, colset_past, frame_past)::Float64
            if !is_feature_na(v)
                update!(var, v)
            end
        end
    end

    if StreamStats.nobs(var) == 0
        return NA_ALIAS
    end
    maximum(var)
end

immutable Feature_Min_Over_History{target, history} <: AbstractFeature end
units{F, H}(       ::Feature_Min_Over_History{F, H}) = units(symbol2feature(F))
isint{F, H}(       ::Feature_Min_Over_History{F, H}) = false
isbool{F, H}(      ::Feature_Min_Over_History{F, H}) = false
lowerbound{F, H}(  ::Feature_Min_Over_History{F, H}) = lowerbound(symbol2feature(F))
upperbound{F, H}(  ::Feature_Min_Over_History{F, H}) = upperbound(symbol2feature(F))
couldna(           ::Feature_Min_Over_History)       = true
Base.symbol{F, H}( ::Feature_Min_Over_History{F, H}) = symbol(@sprintf("min_%d_%s", H, string(F)))
lsymbol{F, H}(     ::Feature_Min_Over_History{F, H}) = latexstring(replace(L"\texttt{min}\left(" * lsymbol(symbol2feature(F)) * L"\right)_{" * string(H) * L"}", "\$", ""))
function replace_na{F, H}(::Feature_Min_Over_History{F, H})
    try
        replace_na(symbol2feature(F))
    catch
        0.0
    end
end
function Base.get{F, H}(::Feature_Min_Over_History{F, H}, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)
    f = symbol2feature(F)
    id = colset2id(runlog, colset, frame)

    var = StreamStats.Min()
    for jump in -H : N_FRAMES_PER_SIM_FRAME : -1

        frame_past = frame + jump

        if frame_inbounds(runlog, frame_past)
            colset_past = id2colset(runlog, id, frame_past)
            v = get(f, runlog, sn, colset_past, frame_past)::Float64
            if !is_feature_na(v)
                update!(var, v)
            end
        end
    end

    if StreamStats.nobs(var) == 0
        return NA_ALIAS
    end
    minimum(var)
end

############################################
#
# Helper Functions
#
############################################

_fast_hypot(v::VecE2) = sqrt(v.x*v.x + v.y*v.y)
_fast_hypot(v::VecSE2) = sqrt(v.x*v.x + v.y*v.y)

# TODO(tim): move this to StreetNets
function _get_dist_from_A_to_B(tagA::LaneTag, tagB::LaneTag, extindA::Float64, extindB::Float64, sn::StreetNetwork;
    max_dist::Float64 = StreetNetworks.SATURATION_DISTANCE_BETWEEN
    )

    #=
    Obtains the distance travelled along the lane centerline from (tagA, extindA) to (tagB, extindB)
    Returns NaN if B is not found downstream from A
    =#

    lane = get_lane(sn, tagA)
    search_dist = -Curves.curve_at(lane.curve, extindA).s
    finished = false

    while !finished

        if lane.id == tagB # in the same lane
            Δs = search_dist + Curves.curve_at(lane.curve, extindB).s
            return (0.0 ≤ Δs ≤ max_dist) ? Δs : NaN
        else
            # not in the same lane, move downstream
            if has_next_lane(sn, lane)
                search_dist += lane.curve.s[end]
                lane = next_lane(sn, lane)
                finished = (search_dist > max_dist)
            else
                finished = true
            end
        end
    end

    NaN
end

# TODO(tim): move this to StreetNets
function _get_dist_between(tagA::LaneTag, tagB::LaneTag, extindA::Float64, extindB::Float64, sn::StreetNetwork)

    #=
    Obtains the distance travelled along the lane centerline between (tagA, extindA) and (tagB, extindB)
    Returns a positive value if it is found
    Returns NaN if B is not found downstream from A
    =#

    retval = _get_dist_from_A_to_B(tagA, tagB, extindA, extindB, sn)
    if isnan(retval)
        retval = _get_dist_from_A_to_B(tagB, tagA, extindB, extindA, sn)
    end

    retval
end
function _get_dist_between(runlog::RunLog, sn::StreetNetwork, colset::UInt, colset_front::UInt, frame::Int)

    tagA = get(runlog, colset, frame, :lanetag)::LaneTag
    tagB = get(runlog, colset_front, frame, :lanetag)::LaneTag
    extindA = get(runlog, colset, frame, :extind)::Float64
    extindB = get(runlog, colset_front, frame, :extind)::Float64

    _get_dist_between(tagA, tagB, extindA, extindB, sn)
end

function _get_lane(runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)
    lanetag = get(runlog, colset, frame, :lanetag)::LaneTag
    if has_lane(sn, lanetag)
        get_lane(sn, lanetag)
    else
        # project
        pos = get(runlog, colset, frame, :inertial)::VecSE2
        proj = project_point_to_streetmap(pos.x, pos.y, sn)
        @assert(proj.successful)
        proj.lane
    end
end
function _get_node(runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)
    lane = _get_lane(runlog, sn, colset, frame)
    # if lane != LANETAG_NULL
        extind = get(runlog, colset, frame, :extind)::Float64
        sn.nodes[closest_node_to_extind(sn, lane, extind)::Int]
    # else
    #     STREETNODE_NULL
    # end
end

function get_feature(F::AbstractFeature, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    if isa(F, Feature_PosFyaw)
        retval = get(F::Feature_PosFyaw, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_PosFt)
        retval = get(F::Feature_PosFt, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_Speed)
        retval = get(F::Feature_Speed, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_VelBx)
        retval = get(F::Feature_VelBx, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_VelBy)
        retval = get(F::Feature_VelBy, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_VelFs)
        retval = get(F::Feature_VelFs, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_VelFt)
        retval = get(F::Feature_VelFt, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_Scene_Speed_Difference)
        retval = get(F::Feature_Scene_Speed_Difference, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_TurnRate)
        retval = get(F::Feature_TurnRate, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_Acc)
        retval = get(F::Feature_Acc, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_AccFs)
        retval = get(F::Feature_AccFs, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_AccFt)
        retval = get(F::Feature_AccFt, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_AccBx)
        retval = get(F::Feature_AccBx, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_AccBy)
        retval = get(F::Feature_AccBy, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_Jerk)
        retval = get(F::Feature_Jerk, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_MarkerDist_Left)
        retval = get(F::Feature_MarkerDist_Left, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_MarkerDist_Right)
        retval = get(F::Feature_MarkerDist_Right, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_Dist_From_Centerline)
        retval = get(F::Feature_Dist_From_Centerline, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_Dist_Merge)
        retval = get(F::Feature_Dist_Merge, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_Dist_Split)
        retval = get(F::Feature_Dist_Split, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_SUMO)
        retval = get(F::Feature_SUMO, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_IDM)
        retval = get(F::Feature_IDM, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_Has_Front)
        retval = get(F::Feature_Has_Front, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_Dist_Front)
        retval = get(F::Feature_Dist_Front, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_D_Y_Front)
        retval = get(F::Feature_D_Y_Front, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_Delta_V_Front)
        retval = get(F::Feature_Delta_V_Front, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_Delta_V_Y_Front)
        retval = get(F::Feature_Delta_V_Y_Front, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_Yaw_Front)
        retval = get(F::Feature_Yaw_Front, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_Turnrate_Front)
        retval = get(F::Feature_Turnrate_Front, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_Acc_Req_Front)
        retval = get(F::Feature_Acc_Req_Front, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_Inv_TTC_Front)
        retval = get(F::Feature_Inv_TTC_Front, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_Inv_Timegap_Front)
        retval = get(F::Feature_Inv_Timegap_Front, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_Gaining_On_Front)
        retval = get(F::Feature_Gaining_On_Front, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_Has_Rear)
        retval = get(F::Feature_Has_Rear, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_Dist_Rear)
        retval = get(F::Feature_Dist_Rear, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_D_Y_Rear)
        retval = get(F::Feature_D_Y_Rear, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_Delta_V_Rear)
        retval = get(F::Feature_Delta_V_Rear, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_Delta_V_Y_Rear)
        retval = get(F::Feature_Delta_V_Y_Rear, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_Yaw_Rear)
        retval = get(F::Feature_Yaw_Rear, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_Turnrate_Rear)
        retval = get(F::Feature_Turnrate_Rear, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_Acc_Req_Rear)
        retval = get(F::Feature_Acc_Req_Rear, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_Inv_TTC_Rear)
        retval = get(F::Feature_Inv_TTC_Rear, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_Inv_Timegap_Rear)
        retval = get(F::Feature_Inv_Timegap_Rear, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_Rear_Is_Gaining)
        retval = get(F::Feature_Rear_Is_Gaining, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_Has_Left)
        retval = get(F::Feature_Has_Left, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_Dist_Left)
        retval = get(F::Feature_Dist_Left, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_D_Y_Left)
        retval = get(F::Feature_D_Y_Left, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_Delta_V_Left)
        retval = get(F::Feature_Delta_V_Left, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_Delta_V_Y_Left)
        retval = get(F::Feature_Delta_V_Y_Left, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_Yaw_Left)
        retval = get(F::Feature_Yaw_Left, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_Turnrate_Left)
        retval = get(F::Feature_Turnrate_Left, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_Acc_Req_Left)
        retval = get(F::Feature_Acc_Req_Left, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_Inv_TTC_Left)
        retval = get(F::Feature_Inv_TTC_Left, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_Inv_Timegap_Left)
        retval = get(F::Feature_Inv_Timegap_Left, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_Left_Is_Gaining)
        retval = get(F::Feature_Left_Is_Gaining, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_Has_Right)
        retval = get(F::Feature_Has_Right, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_Dist_Right)
        retval = get(F::Feature_Dist_Right, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_D_Y_Right)
        retval = get(F::Feature_D_Y_Right, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_Delta_V_Right)
        retval = get(F::Feature_Delta_V_Right, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_Delta_V_Y_Right)
        retval = get(F::Feature_Delta_V_Y_Right, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_Yaw_Right)
        retval = get(F::Feature_Yaw_Right, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_Turnrate_Right)
        retval = get(F::Feature_Turnrate_Right, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_Acc_Req_Right)
        retval = get(F::Feature_Acc_Req_Right, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_Inv_TTC_Right)
        retval = get(F::Feature_Inv_TTC_Right, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_Inv_Timegap_Right)
        retval = get(F::Feature_Inv_Timegap_Right, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_Right_Is_Gaining)
        retval = get(F::Feature_Right_Is_Gaining, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_TimeToCrossing_Left)
        retval = get(F::Feature_TimeToCrossing_Left, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_TimeToCrossing_Right)
        retval = get(F::Feature_TimeToCrossing_Right, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_EstimatedTimeToLaneCrossing)
        retval = get(F::Feature_EstimatedTimeToLaneCrossing, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_A_REQ_StayInLane)
        retval = get(F::Feature_A_REQ_StayInLane, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_N_Lane_Left)
        retval = get(F::Feature_N_Lane_Left, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_N_Lane_Right)
        retval = get(F::Feature_N_Lane_Right, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_Has_Lane_Right)
        retval = get(F::Feature_Has_Lane_Right, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_Has_Lane_Left)
        retval = get(F::Feature_Has_Lane_Left, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_LaneCurvature)
        retval = get(F::Feature_LaneCurvature, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_Time_Consecutive_Brake)
        retval = get(F::Feature_Time_Consecutive_Brake, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_Time_Consecutive_Accel)
        retval = get(F::Feature_Time_Consecutive_Accel, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_Time_Consecutive_Throttle)
        retval = get(F::Feature_Time_Consecutive_Throttle, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_FutureAcceleration)
        retval = get(F::Feature_FutureAcceleration, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_FutureDesiredAngle)
        retval = get(F::Feature_FutureDesiredAngle, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_FutureTurnrate)
        retval = get(F::Feature_FutureTurnrate, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_FutureVelFt)
        retval = get(F::Feature_FutureVelFt, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_Future_Delta_Accel)
        retval = get(F::Feature_Future_Delta_Accel, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_IsClean)
        retval = get(F::Feature_IsClean, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_Past)
        retval = get(F::Feature_Past, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_Mean_Over_History)
        retval = get(F::Feature_Mean_Over_History, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_Std_Over_History)
        retval = get(F::Feature_Std_Over_History, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_Max_Over_History)
        retval = get(F::Feature_Max_Over_History, runlog, sn, colset, frame)::Float64
    elseif isa(F, Feature_Min_Over_History)
        retval = get(F::Feature_Min_Over_History, runlog, sn, colset, frame)::Float64
    else
        warn("UNKNOWN FEATURE $F")
        retval = NaN
    end

    retval::Float64
end

include("data_preprocessor.jl")

end # module