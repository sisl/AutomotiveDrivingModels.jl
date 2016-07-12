export
    AbstractFeature,
    FeatureValue,
    FeatureStates,

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

immutable FeatureValue
    v::Float64 # feature value
    i::UInt32 # used to encode
end

baremodule FeatureStates
    # good
    const GOOD        = 0 # value is perfectly A-okay

    # caution
    const INSUF_HIST  = 1 # value best-guess was made due to insufficient history (ex, acceleration set to zero due to one timestamp)

    # bad (in these cases fval.v is typically set to NaN as well)
    const MISSING     = 2 # value is missing (no car in front, etc.)
    const CENSORED_HI = 3 # value is past an operating threshold
    const CENSORED_LO = 4 # value is below an operating threshold
end

is_feature_valid(fval::FeatureValue) = fval.i == FeatureStates.GOOD || fval.i == FeatureStates.INSUF_HIST
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
# function create_feature_basics(
#     name        :: AbstractString,
#     sym         :: Symbol,
#     lstr        :: LaTeXString,
#                 :: Type{Bool},
#     unit        :: AbstractString,
#     can_na      :: Symbol; # ∈ {:can_na, :no_na}
#     na_replacement :: Union{Void,Float64} = nothing
#     )

#     for feature in values(SYMBOL_TO_FEATURE)
#         @assert(sym  != symbol(feature), "symb: $name -> $feature")
#         @assert(lstr != lsymbol(feature), "lstr: $name -> $feature")
#     end
#     @assert(can_na == :can_na || can_na == :no_na)

#     feature_name = symbol("Feature_" * name)
#     const_name   = symbol(uppercase(name))
#     sym_feature  = Meta.quot(sym)
#     could_be_na  = can_na == :can_na

#     @eval begin
#         immutable $feature_name <: AbstractFeature end
#         const       $const_name  = ($feature_name)()
#         units(       ::$feature_name)  = $unit
#         isint(       ::$feature_name)  = true
#         isbool(      ::$feature_name)  = true
#         lowerbound(  ::$feature_name)  = 0.0
#         upperbound(  ::$feature_name)  = 1.0
#         couldna(     ::$feature_name)  = $could_be_na
#         Base.symbol( ::$feature_name)  = $sym_feature
#         lsymbol(     ::$feature_name)  = $lstr
#         SYMBOL_TO_FEATURE[symbol($const_name)] = $const_name
#     end

#     if !could_be_na || isa(na_replacement, Void)
#         return
#     end

#     @eval begin
#         replace_na(::$feature_name) = $na_replacement
#     end
# end

# # ----------------------------------



# end # module