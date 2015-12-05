module FeaturesNew

import Base.Meta: quot

using LaTeXStrings

using AutomotiveDrivingModels.Vec
# using AutomotiveDrivingModels.CommonTypes
using AutomotiveDrivingModels.Curves
using AutomotiveDrivingModels.RunLogs
using AutomotiveDrivingModels.StreetNetworks

# ----------------------------------
# exports

export POSFΘ, POSFT, SPEED, VELBX, VELBY, VELFS, VELFT
export TURNRATE, ACC, ACCFX, ACCFY, ACCBX, ACCBY
export MARKERDIST_LEFT, MARKERDIST_RIGHT
export TIMETOCROSSING_LEFT, TIMETOCROSSING_RIGHT, ESTIMATEDTIMETOLANECROSSING, A_REQ_STAYINLANE
export N_LANE_LEFT, N_LANE_RIGHT, HAS_LANE_RIGHT, HAS_LANE_LEFT, LANECURVATURE
export FutureAcceleration, FutureDesiredAngle

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
units(      ::AbstractFeature) = "None"
isint(      ::AbstractFeature) = error("Not implemented!") # true if the feature takes on only integer values
isbool(     ::AbstractFeature) = error("Not implemented!") # true if the feature takes on only boolean values (0,1) (will also have isint = true)
upperbound( ::AbstractFeature) = error("Not implemented!") # max value of the feature
lowerbound( ::AbstractFeature) = error("Not implemented!") # min value of the feature
couldna(    ::AbstractFeature) = true                      # whether the given variable could produce the NA alias
Base.symbol(::AbstractFeature) = error("Not implemented!")
lsymbol(    ::AbstractFeature) = error("Not implemented!")
get(        ::AbstractFeature, ::RunLog, ::StreetNetwork, colset::Integer, frame::Integer) = error("Not implemented!")

allfeatures() = collect(values(SYMBOL_TO_FEATURE)) # array of all features

# ----------------------------------

function create_feature_basics(
    name        :: String,
    sym         :: Symbol,
    lstr        :: LaTeXString,
                :: Type{Float64},
    unit        :: String,
    minvalue    :: Float64,
    maxvalue    :: Float64,
    can_na      :: Symbol, # ∈ {:can_na, :no_na}
    )

    for feature in values(SYMBOL_TO_FEATURE)
        @assert(sym  != symbol(f), "symb: $name -> $feature")
        @assert(lstr != lsymbol(feature), "lstr: $name -> $feature")
    end
    @assert(minvalue ≤ maxvalue)
    @assert(can_na == :can_na || can_na == :no_na)

    feature_name = symbol("Feature_" * name)
    const_name   = symbol(uppercase(name))
    sym_feature  = quot(sym)
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
        sym2ftr[symbol(  $const_name)] = $const_name
    end
end
function create_feature_basics(
    name        :: String,
    sym         :: Symbol,
    lstr        :: LaTeXString,
                :: Type{Int},
    unit        :: String,
    minvalue    :: Float64,
    maxvalue    :: Float64,
    can_na      :: Symbol, # ∈ {:can_na, :no_na}
    )

    for feature in values(SYMBOL_TO_FEATURE)
        @assert(sym  != symbol(f), "symb: $name -> $feature")
        @assert(lstr != lsymbol(feature), "lstr: $name -> $feature")
    end
    @assert(minvalue ≤ maxvalue)
    @assert(can_na == :can_na || can_na == :no_na)

    feature_name = symbol("Feature_" * name)
    const_name   = symbol(uppercase(name))
    sym_feature  = quot(sym)
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
        sym2ftr[symbol(  $const_name)] = $const_name
    end
end
function create_feature_basics(
    name        :: String,
    sym         :: Symbol,
    lstr        :: LaTeXString,
                :: Type{Bool},
    unit        :: String,
    can_na      :: Symbol, # ∈ {:can_na, :no_na}
    )

    for feature in values(SYMBOL_TO_FEATURE)
        @assert(sym  != symbol(f), "symb: $name -> $feature")
        @assert(lstr != lsymbol(feature), "lstr: $name -> $feature")
    end
    @assert(minvalue ≤ maxvalue)
    @assert(can_na == :can_na || can_na == :no_na)

    feature_name = symbol("Feature_" * name)
    const_name   = symbol(uppercase(name))
    sym_feature  = quot(sym)
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
        sym2ftr[symbol(  $const_name)] = $const_name
    end
end

# ----------------------------------

create_feature_basics("PosFΘ", :posFθ, L"\psi", Float64, L"\radian", -Inf, Inf, :no_na)
get(::Feature_POSFθ, runlog::RunLog, ::StreetNetwork, colset::Integer, frame::Integer) =
    (get(runlog, colset, frame, :frenet)::VecSE2).θ
create_feature_basics("PosFt", :posFt, L"p^F_t", Float64, L"\metre", -Inf, Inf, :no_na)
get(::Feature_PosFt, runlog::RunLog, ::StreetNetwork, colset::Integer, frame::Integer) =
    (get(runlog, colset, frame, :frenet)::VecSE2).y

create_feature_basics("Speed", :speed, L"\|v\|", Float64, L"\metre\per\second", 0.0, Inf, :no_na)
function get(::Feature_Speed, runlog::RunLog, ::StreetNetwork, colset::Integer, frame::Integer)
    ratesB = get(runlog, colset, frame, :ratesB)::VecSE2
    sqrt(ratesB.x*ratesB.x + ratesB.y*ratesB.y)
end

create_feature_basics("VelBx", :velBx, L"v^B_x", Float64, L"\metre\per\second", -Inf, Inf, :no_na)
get(::Feature_VelBx, runlog::RunLog, ::StreetNetwork, colset::Integer, frame::Integer) =
    (get(runlog, colset, frame, :ratesB)::VecSE2).x
create_feature_basics("VelBy", :velBy, L"v^B_y", Float64, L"\metre\per\second", -Inf, Inf, :no_na)
get(::Feature_VelBy, runlog::RunLog, ::StreetNetwork, colset::Integer, frame::Integer) =
    (get(runlog, colset, frame, :ratesB)::VecSE2).y
create_feature_basics("VelFs", :velFs, L"v^F_s", Float64, L"\metre\per\second", -Inf, Inf, :no_na)
get(::Feature_VelFs, runlog::RunLog, ::StreetNetwork, colset::Integer, frame::Integer) =
    (get(runlog, colset, frame, :ratesF)::VecE2).x
create_feature_basics("VelFt", :velFt, L"v^F_t", Float64, L"\metre\per\second", -Inf, Inf, :no_na)
get(::Feature_VelFt, runlog::RunLog, ::StreetNetwork, colset::Integer, frame::Integer) =
    (get(runlog, colset, frame, :ratesF)::VecE2).y

create_feature_basics("MarkerDist_Left", :d_ml, L"d_{ml}", Float64, L"\metre", -Inf, Inf, :no_na)
function get(::Feature_MarkerDist_Left, runlog::RunLog, sn::StreetNetwork, colset::Integer, frame::Integer)

    lane_width = _get_lane(runlog, sn, colset, frame).width
    @assert(!isinf(lane_width) && !isnan(lane_width))

    offset = (get(runlog, colset, frame, :frenet)::VecSE2).y
    lane_width/2.0 - offset
end
create_feature_basics("MarkerDist_Right", :d_mr, L"d_{mr}", Float64, L"\metre", 0.0, Inf, :no_na)
function get(::Feature_MarkerDist_Right, runlog::RunLog, sn::StreetNetwork, colset::Integer, frame::Integer)

    lane_width = _get_lane(runlog, sn, colset, frame).width
    @assert(!isinf(lane_width) && !isnan(lane_width))

    offset = (get(runlog, colset, frame, :frenet)::VecSE2).y
    lane_width/2.0 + offset
end

create_feature_basics( "TimeToCrossing_Right", :ttcr_mr, L"ttcr^{mr}_y", Float64, L"s", 0.0, Inf, :can_na)
function get(::Feature_TimeToCrossing_Right, runlog::RunLog, sn::StreetNetwork, colset::Integer, frame::Integer)
    d_mr = get(MARKERDIST_RIGHT, runlog, sn, colset, frame)
    velFt = get(VELFT, runlog, sn, colset, frame)

    if d_mr > 0.0 && velFt < 0.0
        min(-d_mr / velFt, TIME_THRESHOLD)
    else
        NA_ALIAS
    end
end
# replace_na(::Feature_TimeToCrossing_Right, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int) = 10.0 # large threshold value (same value as already set for pos. vel right)

create_feature_basics( "TimeToCrossing_Left", :ttcr_ml, L"ttcr^{ml}_y", Float64, L"s", 0.0, Inf, :can_na)
function get(::Feature_TimeToCrossing_Left, runlog::RunLog, sn::StreetNetwork, colset::Integer, frame::Integer)
    d_ml = get(MARKERDIST_LEFT, runlog, sn, colset, frame)
    velFs = get(VELFS, runlog, sn, colset, frame)

    if d_ml < 0.0 && velFt > 0.0
        min(-d_ml / velFt, TIME_THRESHOLD)
    else
        NA_ALIAS
    end
end
# replace_na(::Feature_TimeToCrossing_Left, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int) = 10.0 # large threshold value (same value as already set for pos. vel left)

create_feature_basics( "EstimatedTimeToLaneCrossing", :est_ttcr, L"ttcr^\text{est}_y", Float64, L"s", 0.0, Inf, :can_na)
function get(::Feature_EstimatedTimeToLaneCrossing, runlog::RunLog, sn::StreetNetwork, colset::Integer, frame::Integer)
    ttcr_left = get(TIMETOCROSSING_LEFT, runlog, sn, colset, frame)
    if !isinf(ttcr_left)
        return ttcr_left
    end
    get(TIMETOCROSSING_RIGHT, runlog, sn, colset, frame)
end
# replace_na(::Feature_EstimatedTimeToLaneCrossing, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int) = 10.0 # large threshold value (same value as already set for pos. vel)

create_feature_basics( "A_REQ_StayInLane", :a_req_stayinlane, L"a^{req}_y", Float64, "m/s2", -Inf, Inf, :can_na)
function get(::Feature_A_REQ_StayInLane, runlog::RunLog, sn::StreetNetwork, colset::Integer, frame::Integer)
    velFt = get(VELFT, runlog, sn, colset, frame)

    if velFt > 0.0
        d_mr = get(D_MR, basics, carind, validfind)
        return (d_mr > 0.0) ? min( 0.5velFt*velFt / d_mr, ACCEL_THRESHOLD) : NA_ALIAS
    else
        d_ml = get(D_ML, basics, carind, validfind)
        return (d_ml < 0.0) ? min(-0.5velFt*velFt / d_ml, ACCEL_THRESHOLD) : NA_ALIAS
    end
end
# replace_na(::Feature_A_REQ_StayInLane, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int) = 0.0 # missing at random; no action

create_feature_basics( "N_LANE_RIGHT", :n_lane_right, L"n^\text{lane}_r", Int, "-", 0.0, 10.0, :no_na)
function get(::Feature_N_LANE_R, runlog::RunLog, sn::StreetNetwork, colset::Integer, frame::Integer)
    node = _get_node(runlog, sn, colset, frame)
    convert(Float64, node.n_lanes_right)
end
create_feature_basics( "N_LANE_LEFT", :n_lane_left, L"n^\text{lane}_r", Int, "-", 0.0, 10.0, :no_na)
function get(::Feature_N_LANE_R, runlog::RunLog, sn::StreetNetwork, colset::Integer, frame::Integer)
    node = _get_node(runlog, sn, colset, frame)
    convert(Float64, node.n_lanes_left)
end
create_feature_basics( "HAS_LANE_RIGHT", :has_lane_right, L"\exists_{\text{lane}}^\text{r}", Bool, "-", :no_na)
get(::Feature_HAS_LANE_RIGHT, runlog::RunLog, sn::StreetNetwork, colset::Integer, frame::Integer) =
    Float64(get(N_LANE_RIGHT, runlog, sn , colset, frame) > 0.0)
create_feature_basics( "HAS_LANE_LEFT", :has_lane_left, L"\exists_{\text{lane}}^\text{l}", Bool, "-", :no_na)
get(::Feature_HAS_LANE_LEFT, runlog::RunLog, sn::StreetNetwork, colset::Integer, frame::Integer) =
    Float64(get(N_LANE_LEFT, runlog, sn , colset, frame) > 0.0)
create_feature_basics( "LaneCurvature", :curvature, L"\kappa", Float64, "1/m", -Inf, Inf, :no_na)
function get(::Feature_LaneCurvature, runlog::RunLog, sn::StreetNetwork, colset::Integer, frame::Integer)
    footpoint = get(runlog, colset, frame, :footpoint)::CurvePt
    footpoint.k
end

create_feature_basics("TurnRate", :turnrate, L"\dot{\psi}", Float64, L"\radian\per\second", -Inf, Inf, :no_na)
get(::Feature_TurnRate, runlog::RunLog, ::StreetNetwork, colset::Integer, frame::Integer) =
    (get(runlog, colset, frame, :ratesB)::VecSE2).θ

create_feature_basics("Acc", :acc, L"\|a\|", Float64, L"\metre\per\second\squared", -Inf, Inf, :no_na)
function _get(::Feature_Acc, runlog::RunLog, sn::StreetNetwork, colset::Integer, frame::Integer)

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
function _get(::Feature_AccBx, runlog::RunLog, sn::StreetNetwork, colset::Integer, frame::Integer)

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
function _get(::Feature_AccBy, runlog::RunLog, sn::StreetNetwork, colset::Integer, frame::Integer)

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
function _get(::Feature_AccFs, runlog::RunLog, sn::StreetNetwork, colset::Integer, frame::Integer)

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
function _get(::Feature_AccFt, runlog::RunLog, sn::StreetNetwork, colset::Integer, frame::Integer)

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

create_feature_basics("FutureAcceleration", :f_accel, L"a^{\text{fut}}", Float64, L"\metre\per\second\squared", -Inf, Inf, :can_na)
function _get(::Feature_FutureAcceleration, runlog::RunLog, sn::StreetNetwork, colset::Integer, frame::Integer)

    # TODO(tim): should we be using AccBx to propagate the model?

    if frame_inbounds(runlog, frame+1)
        curr = get(SPEED, runlog, sn, colset, frame)::Float64
        futr = get(SPEED, runlog, sn, colset, frame+1)::Float64
        curr_t = get(runlog, frame,   :time)::Float64
        futr_t = get(runlog, frame+1, :time)::Float64
        (futr - curr) / (futr_t - curr_t)
    else
        NA_ALIAS
    end
end
create_feature_basics( "FutureDesiredAngle", :f_des_angle, L"\phi^{\text{des}}", Float64, L"\radian", -float64(π), float64(π), :can_na)
function _get(::Feature_FutureDesiredAngle, runlog::RunLog, sn::StreetNetwork, colset::Integer, frame::Integer;
    KP_DESIRED_ANGLE::Float64=1.0
    )

    if frame_inbounds(runlog, frame+1)

        curϕ = (get(runlog, colset, frame,   :frenet)::VecSE2).θ
        futϕ = (get(runlog, colset, frame+1, :frenet)::VecSE2).θ
        curr_t = get(runlog, frame,   :time)::Float64
        futr_t = get(runlog, frame+1, :time)::Float64

        ΔT = futr_t - curr_t
        expconst = exp(-KP_DESIRED_ANGLE*ΔT)

        (futϕ - curϕ*expconst) / (1.0 - expconst)
    else
        NA_ALIAS
    end
end

############################################
#
# Helper Functions
#
############################################

function _get_lane(runlog::RunLog, sn::StreetNetwork, colset::Integer, frame::Integer)
    get_lane(sn, get(runlog, colset, frame, :lanetag)::LaneTag)
end
function _get_node(runlog::RunLog, sn::StreetNetwork, colset::Integer, frame::Integer)
    lane = _get_lane(runlog, sn, colset, frame)
    extind = get(runlog, colset, frame, :extind)::Float64
    closest_node_to_extind(sn, lane, extind)::StreetNode
end # module