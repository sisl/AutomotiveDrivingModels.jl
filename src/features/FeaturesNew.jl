module FeaturesNew

using LaTeXStrings

using AutomotiveDrivingModels.CommonTypes
using AutomotiveDrivingModels.Vec
using AutomotiveDrivingModels.Curves
using AutomotiveDrivingModels.RunLogs
using AutomotiveDrivingModels.StreetNetworks

# ----------------------------------
# exports

# export AbstractFeature
# export POSFYAW, POSFT, SPEED, VELBX, VELBY, VELFS, VELFT, SCENE_SPEED_DIFFERENCE
# export TURNRATE, ACC, ACCFS, ACCFT, ACCBX, ACCBY
# export MARKERDIST_LEFT, MARKERDIST_RIGHT, DIST_FROM_CENTERLINE
# export DIST_MERGE, DIST_SPLIT
# export SUMO, IDM,
# export HAS_FRONT, DIST_FRONT, D_Y_FRONT, DELTA_V_FRONT, DELTA_V_Y_FRONT, YAW_FRONT, TURNRATE_FRONT, ACC_REQ_FRONT, INV_TTC_FRONT, INV_TIMEGAP_FRONT, GAINING_ON_FRONT

# INDREAR,  HAS_REAR,  D_X_REAR,  D_Y_REAR,  V_X_REAR,  V_Y_REAR,  YAW_REAR,  TURNRATE_REAR,  A_REQ_REAR,  TTC_REAR,  TIMEGAP_REAR, REAR_IS_GAINING

# export TIMETOCROSSING_LEFT, TIMETOCROSSING_RIGHT, ESTIMATEDTIMETOLANECROSSING, A_REQ_STAYINLANE
# export N_LANE_LEFT, N_LANE_RIGHT, HAS_LANE_RIGHT, HAS_LANE_LEFT, LANECURVATURE
# export FutureAcceleration, FutureDesiredAngle
# export Feature_IsClean
# export NA_ALIAS

# export
#     symbol2feature,
#     is_symbol_a_feature,
#     allfeatures,
#     units,
#     isint,
#     isbool,
#     upperbound,
#     lowerbound,
#     couldna,
#     lsymbol

# ----------------------------------
# constants

const NA_ALIAS = Inf

# ----------------------------------
# types

abstract AbstractFeature

# ----------------------------------
# globals

const SYMBOL_TO_FEATURE = Dict{Symbol, AbstractFeature}()
const TIME_THRESHOLD = 10.0 # [s]
const ACCEL_THRESHOLD = 5.0 # [m/s^2]

# ----------------------------------
# feature functions

symbol2feature(sym::Symbol) = SYMBOL_TO_FEATURE[sym]
is_symbol_a_feature(sym::Symbol) = haskey(SYMBOL_TO_FEATURE, sym)
allfeatures() = values(SYMBOL_TO_FEATURE)

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

create_feature_basics("PosFt", :posFt, L"p^F_t", Float64, L"\metre", -Inf, Inf, :no_na)
Base.get(::Feature_PosFt, runlog::RunLog, ::StreetNetwork, colset::UInt, frame::Integer) =
    (get(runlog, colset, frame, :frenet)::VecSE2).y

create_feature_basics("PosFyaw", :posFyaw, L"\psi", Float64, L"\radian", -Inf, Inf, :no_na)
Base.get(::Feature_PosFyaw, runlog::RunLog, ::StreetNetwork, colset::UInt, frame::Integer) =
    (get(runlog, colset, frame, :frenet)::VecSE2).θ

create_feature_basics("Speed", :speed, L"\|v\|", Float64, L"\metre\per\second", 0.0, Inf, :no_na)
function Base.get(::Feature_Speed, runlog::RunLog, ::StreetNetwork, colset::UInt, frame::Integer)
    ratesB = get(runlog, colset, frame, :ratesB)::VecSE2
    _fast_hypot(ratesB)
end

create_feature_basics("VelBx", :velBx, L"v^B_x", Float64, L"\metre\per\second", -Inf, Inf, :no_na)
Base.get(::Feature_VelBx, runlog::RunLog, ::StreetNetwork, colset::UInt, frame::Integer) =
    (get(runlog, colset, frame, :ratesB)::VecSE2).x
create_feature_basics("VelBy", :velBy, L"v^B_y", Float64, L"\metre\per\second", -Inf, Inf, :no_na)
Base.get(::Feature_VelBy, runlog::RunLog, ::StreetNetwork, colset::UInt, frame::Integer) =
    (get(runlog, colset, frame, :ratesB)::VecSE2).y
create_feature_basics("VelFs", :velFs, L"v^F_s", Float64, L"\metre\per\second", -Inf, Inf, :no_na)
Base.get(::Feature_VelFs, runlog::RunLog, ::StreetNetwork, colset::UInt, frame::Integer) =
    (get(runlog, colset, frame, :ratesF)::VecE2).x
create_feature_basics("VelFt", :velFt, L"v^F_t", Float64, L"\metre\per\second", -Inf, Inf, :no_na)
Base.get(::Feature_VelFt, runlog::RunLog, ::StreetNetwork, colset::UInt, frame::Integer) =
    (get(runlog, colset, frame, :ratesF)::VecE2).y

create_feature_basics("Scene_Speed_Difference", :scene_speed_diff, L"\delta v_\text{scene}", Float64, L"\metre\per\second", -Inf, Inf, :no_na)
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

create_feature_basics("MarkerDist_Left", :d_ml, L"d_{ml}", Float64, L"\metre", -Inf, Inf, :no_na)
function Base.get(::Feature_MarkerDist_Left, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    offset = (get(runlog, colset, frame, :frenet)::VecSE2).y
    node = _get_node(runlog, sn, colset, frame)
    node.marker_dist_left - offset
end
create_feature_basics("MarkerDist_Right", :d_mr, L"d_{mr}", Float64, L"\metre", 0.0, Inf, :no_na)
function Base.get(::Feature_MarkerDist_Right, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    offset = (get(runlog, colset, frame, :frenet)::VecSE2).y
    node = _get_node(runlog, sn, colset, frame)
    node.marker_dist_right + offset
end
create_feature_basics("Dist_From_Centerline", :d_cl, L"d_{cl}", Float64, L"\metre", 0.0, Inf, :no_na)
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
    if !isinf(ttcr_left)
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

create_feature_basics( "N_Lane_Right", :n_lane_right, L"n^\text{lane}_r", Int, "-", 0.0, 10.0, :no_na)
function Base.get(::Feature_N_Lane_Right, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)
    node = _get_node(runlog, sn, colset, frame)
    convert(Float64, node.n_lanes_right)
end
create_feature_basics( "N_Lane_Left", :n_lane_left, L"n^\text{lane}_l", Int, "-", 0.0, 10.0, :no_na)
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

create_feature_basics("Dist_Merge", :dist_merge, L"d_\text{merge}", Float64, L"\metre", 0.0, Inf, :no_na)
function Base.get(::Feature_Dist_Merge, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    # the distance from the vehicle's current position to the next merge
    # NOTE(tim): the value is technically thresholded to ~5000m, depending on the StreetNetwork map settings

    node = _get_node(runlog, sn, colset, frame)
    node.d_merge
end
create_feature_basics("Dist_Split", :dist_split, L"d_\text{split}", Float64, L"\metre", 0.0, Inf, :no_na)
function Base.get(::Feature_Dist_Split, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    # the distance from the vehicle's current position to the next lane split
    # NOTE(tim): the value is technically thresholded to ~5000m, depending on the StreetNetwork map settings

    node = _get_node(runlog, sn, colset, frame)
    node.d_split
end

create_feature_basics("TurnRate", :turnrate, L"\dot{\psi}", Float64, L"\radian\per\second", -Inf, Inf, :no_na)
Base.get(::Feature_TurnRate, runlog::RunLog, ::StreetNetwork, colset::UInt, frame::Integer) =
    (get(runlog, colset, frame, :ratesB)::VecSE2).θ

create_feature_basics("Acc", :acc, L"\|a\|", Float64, L"\metre\per\second\squared", -Inf, Inf, :no_na)
function Base.get(::Feature_Acc, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    # NOTE(tim): defaults to 0.0 if there is no previous frame

    if frame_inbounds(runlog, frame-1)
        curr = get(SPEED, runlog, sn, colset, frame)::Float64
        past = get(SPEED, runlog, sn, colset, frame-1)::Float64
        curr_t = get(runlog, frame,   :time)::Float64
        past_t = get(runlog, frame-1, :time)::Float64
        (curr - past) / (past_t - curr_t)
    else
        0.0
    end
end
create_feature_basics("AccBx", :accBx, L"a^B_x", Float64, L"\metre\per\second\squared", -Inf, Inf, :no_na)
function Base.get(::Feature_AccBx, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    # NOTE(tim): defaults to 0.0 if there is no previous frame

    if frame_inbounds(runlog, frame-1)
        curr = get(VELBX, runlog, sn, colset, frame)::Float64
        past = get(VELBX, runlog, sn, colset, frame-1)::Float64
        curr_t = get(runlog, frame,   :time)::Float64
        past_t = get(runlog, frame-1, :time)::Float64
        (curr - past) / (past_t - curr_t)
    else
        0.0
    end
end
create_feature_basics("AccBy", :accBy, L"a^B_y", Float64, L"\metre\per\second\squared", -Inf, Inf, :no_na)
function Base.get(::Feature_AccBy, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    # NOTE(tim): defaults to 0.0 if there is no previous frame

    if frame_inbounds(runlog, frame-1)
        curr = get(VELBY, runlog, sn, colset, frame)::Float64
        past = get(VELBY, runlog, sn, colset, frame-1)::Float64
        curr_t = get(runlog, frame,   :time)::Float64
        past_t = get(runlog, frame-1, :time)::Float64
        (curr - past) / (past_t - curr_t)
    else
        0.0
    end
end
create_feature_basics("AccFs", :accFs, L"a^F_s", Float64, L"\metre\per\second\squared", -Inf, Inf, :no_na)
function Base.get(::Feature_AccFs, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    # NOTE(tim): defaults to 0.0 if there is no previous frame

    if frame_inbounds(runlog, frame-1)
        curr = get(VELFS, runlog, sn, colset, frame)::Float64
        past = get(VELFS, runlog, sn, colset, frame-1)::Float64
        curr_t = get(runlog, frame,   :time)::Float64
        past_t = get(runlog, frame-1, :time)::Float64
        (curr - past) / (past_t - curr_t)
    else
        0.0
    end
end
create_feature_basics("AccFt", :accFt, L"a^F_t", Float64, L"\metre\per\second\squared", -Inf, Inf, :no_na)
function Base.get(::Feature_AccFt, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    # NOTE(tim): defaults to 0.0 if there is no previous frame

    if frame_inbounds(runlog, frame-1)
        curr = get(VELFT, runlog, sn, colset, frame)::Float64
        past = get(VELFT, runlog, sn, colset, frame-1)::Float64
        curr_t = get(runlog, frame,   :time)::Float64
        past_t = get(runlog, frame-1, :time)::Float64
        (curr - past) / (past_t - curr_t)
    else
        0.0
    end
end

create_feature_basics("SUMO", :sumo, L"a_\text{SUMO}", Float64, L"\metre\per\second\squared", -Inf, Inf, :can_na, na_replacement=0.0)
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

    d_front = _get_dist_between(runlog, sn, colset, colset_front, frame)
    @assert(!isnan(d_front))

    v = _fast_hypot(get(runlog, colset, frame, :ratesB)::VecSE2)
    v_front = _fast_hypot(get(runlog, colset_front, frame, :ratesB)::VecSE2)
    Δt = estimate_framerate(runlog)

    v_safe = -τ*b_max + sqrt((τ*b_max)^2 + v_front^2 + 2*b_max*d_front)
    v_des = min(min(v_safe, v + a_max*(1 - v/v_max)*Δt), v_max)
    a_sumo = (max(0.0, v_des) - v_safe) / Δt

    return a_sumo
end

create_feature_basics("IDM", :idm, L"a_\text{IDM}", Float64, L"\metre\per\second\squared", -Inf, Inf, :can_na, na_replacement=0.0)
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

    d_front = _get_dist_between(runlog, sn, colset, colset_front, frame)
    @assert(!isnan(d_front))

    v = _fast_hypot(get(runlog, colset, frame, :ratesB)::VecSE2)
    v_front = _fast_hypot(get(runlog, colset_front, frame, :ratesB)::VecSE2)
    Δv_front = v_front - v
    Δt = estimate_framerate(runlog)
    T = get(TIMEGAP_FRONT, runlog, sn, colset, frame)::Float64

    d_des = dmn + T*v - (v*Δv_front) / (2*sqrt(a_max*d_comf))
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

create_feature_basics("Dist_Front", :d_front, L"d_x^\text{front}", Float64, L"\metre", 0.0, Inf, :can_na, na_replacement=50.0)
function Base.get(::Feature_Dist_Front, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    colset_front = get(runlog, colset, frame, :colset_front)::UInt
    if colset_front == COLSET_NULL
        return NA_ALIAS
    end

    d_front = _get_dist_between(runlog, sn, colset, colset_front, frame)
    @assert(!isnan(d_front))

    d_front
end

create_feature_basics("D_Y_Front", :d_y_front, L"d_y^\text{front}", Float64, L"\metre", -5.0, 5.0, :can_na, na_replacement=0.0)
function Base.get(::Feature_D_Y_Front, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    colset_front = get(runlog, colset, frame, :colset_front)::UInt
    if colset_front == COLSET_NULL
        return NA_ALIAS
    end

    dcl_ego = get(DIST_FROM_CENTERLINE, runlog, sn, colset, frame)
    dcl_oth = get(DIST_FROM_CENTERLINE, runlog, sn, colset_front, frame)

    dcl_ego - dcl_oth
end

create_feature_basics("Delta_V_Front", :dv_x_front, L"\Delta v_x^\text{front}", Float64, L"\metre\per\second", -Inf, Inf, :can_na, na_replacement=0.0)
function Base.get(::Feature_Delta_V_Front, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    colset_front = get(runlog, colset, frame, :colset_front)::UInt
    if colset_front == COLSET_NULL
        return NA_ALIAS
    end

    v_ego = (get(runlog, colset, frame, :ratesF)::VecSE2).x
    v_oth = (get(runlog, colset_front, frame, :ratesF)::VecSE2).x

    v_oth - v_ego
end

create_feature_basics("Delta_V_Front", :dv_y_front, L"\Delta v_y^\text{front}", Float64, L"\metre\per\second", -Inf, Inf, :can_na, na_replacement=0.0)
function Base.get(::Feature_Delta_V_Front, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    colset_front = get(runlog, colset, frame, :colset_front)::UInt
    if colset_front == COLSET_NULL
        return NA_ALIAS
    end

    v_ego = (get(runlog, colset, frame, :ratesF)::VecSE2).y
    v_oth = (get(runlog, colset_front, frame, :ratesF)::VecSE2).y

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

create_feature_basics("Acc_Req_Front", :acc_req_front, L"a_\text{req}^\text{front}", Float64, L"\metre\per\second\squared", -Inf, Inf, :can_na, na_replacement=0.0)
function Base.get(::Feature_Acc_Req_Front, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    # the const. acceleration required to avoid a collision assuming
    # everyone drives with constant frenet-x velocity

    colset_front = get(runlog, colset, frame, :colset_front)::UInt
    if colset_front == COLSET_NULL
        return NA_ALIAS
    end

    dv = get(DELTA_V_FRONT, runlog, sn, colset, frame)

    if dv >= 0.0 # they are pulling away; we are good
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

    if dv >= 0.0 # they are pulling away; we are good
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

    v = (get(runlog, colset, frame, :ratesF)::VecSE2).x

    if v ≤ 0.0
        return 0.0
    end

    dx = get(DIST_FRONT, runlog, sn, colset, frame)

    v / dx
end

create_feature_basics("Gaining_On_Front", :gaining_on_front, L"gaining^\text{front}", Bool, L"-", :no_na)
function Base.get(::Feature_Gaining_On_Front, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    colset_front = get(runlog, colset, frame, :colset_front)::UInt
    if colset_front == COLSET_NULL
        return NA_ALIAS
    end

    v_ego = (get(runlog, colset, frame, :ratesF)::VecSE2).x
    v_oth = (get(runlog, colset_front, frame, :ratesF)::VecSE2).x

    convert(Float64, v_ego > v_oth)
end

#############################################
#
#
#
#############################################

create_feature_basics("FutureAcceleration", :f_accel, L"a_{t+1}", Float64, L"\metre\per\second\squared", -Inf, Inf, :can_na)
function Base.get(::Feature_FutureAcceleration, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)

    # TODO(tim): should we be using AccBx to propagate the model?

    frame_fut = frame + N_FRAMES_PER_SIM_FRAME
    if frame_inbounds(runlog, frame_fut)
        curr = get(SPEED, runlog, sn, colset, frame)::Float64
        futr = get(SPEED, runlog, sn, colset, frame_fut)::Float64
        curr_t = get(runlog, frame,   :time)::Float64
        futr_t = get(runlog, frame_fut, :time)::Float64
        (futr - curr) / (futr_t - curr_t)
    else
        NA_ALIAS
    end
end
create_feature_basics( "FutureDesiredAngle", :f_des_angle, L"\phi^{\text{des}}", Float64, L"\radian", -convert(Float64, π), convert(Float64, π), :can_na)
function Base.get(::Feature_FutureDesiredAngle, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer;
    KP_DESIRED_ANGLE::Float64=1.0
    )

    frame_fut = frame + N_FRAMES_PER_SIM_FRAME

    if frame_inbounds(runlog, frame_fut)

        curϕ = (get(runlog, colset, frame,   :frenet)::VecSE2).θ
        futϕ = (get(runlog, colset, frame_fut, :frenet)::VecSE2).θ
        curr_t = get(runlog, frame,   :time)::Float64
        futr_t = get(runlog, frame_fut, :time)::Float64

        ΔT = futr_t - curr_t
        expconst = exp(-KP_DESIRED_ANGLE*ΔT)

        (futϕ - curϕ*expconst) / (1.0 - expconst)
    else
        NA_ALIAS
    end
end

immutable Feature_IsClean{target_symbol} <: AbstractFeature end
units(       ::Feature_IsClean) = "-"
isint(       ::Feature_IsClean) = true
isbool(      ::Feature_IsClean) = true
lowerbound(  ::Feature_IsClean) = 0.0
upperbound(  ::Feature_IsClean) = 1.0
couldna(     ::Feature_IsClean) = false
Base.symbol( ::Feature_IsClean) = symbol("isclean_" * string(target_symbol))
lsymbol(     ::Feature_IsClean) = L"\texttt{isclean}\left(" * lsymbol(symbol2feature(target_symbol)) * L"\right)"
function Base.get{F}(::Feature_IsClean{F}, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)
    f = symbol2feature(F)
    v = get(f, runlog, sn, colset, frame)::Float64
    convert(Float64, !isnan(v) && !isinf(v))
end

############################################
#
# Helper Functions
#
############################################


_fast_hypot(v::VecE2) = sqrt(v.x*v.x + v.y*v.y)
_fast_hypot(v::VecSE2) = sqrt(v.x*v.x + v.y*v.y)

function _get_dist_between(tagA::LaneTag, tagB::LaneTag, extindA::Float64, extindB::Float64, sn::StreetNetwork)

    # returns NaN if not in the same lane

end
function _get_dist_between(runlog::RunLog, sn::StreetNetwork, colset::UInt, colset_front::UInt, frame::Int)

    tagA = get(runlog, colset, frame, :lanetag)::LaneTag
    tagB = get(runlog, colset_front, frame, :lanetag)::LaneTag
    extindA = get(runlog, colset, frame, :extind)::Float64
    extindB = get(runlog, colset_front, frame, :extind)::Float64

    _get_dist_between(tagA, tagB, extindA, extindB, sn)
end

function _get_lane(runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)
    get_lane(sn, get(runlog, colset, frame, :lanetag)::LaneTag)
end
function _get_node(runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)
    lane = _get_lane(runlog, sn, colset, frame)
    extind = get(runlog, colset, frame, :extind)::Float64
    sn.nodes[closest_node_to_extind(sn, lane, extind)::Int]
end


include("data_preprocessor.jl")

end # module