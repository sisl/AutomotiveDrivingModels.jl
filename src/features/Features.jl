##############################################################################
##
##  Features
##  Copyright (c) 2014 Tim wheeler, Robert Bosch Gmbh
##
##  Allows for the extraction of feature values for cars in scenes
##
##############################################################################

module Features

import Base: symbol, get
import Base.Meta: quot

using LaTeXStrings
using DataArrays

using AutomotiveDrivingModels.Vec
using AutomotiveDrivingModels.CommonTypes
using AutomotiveDrivingModels.Curves
using AutomotiveDrivingModels.Trajdata
using AutomotiveDrivingModels.StreetNetworks
# ----------------------------------
# exports

export
	AbstractFeature,
	ExtractedFeatureCache,
	FeatureExtractBasicsPdSet

export YAW, POSFX, POSFY, SPEED, VELFX, VELFY, DELTA_SPEED_LIMIT, TURNRATE, TURNRATE_GLOBAL, ACC, ACCFX, ACCFY, ISEGO
export D_CL, D_ML, D_MR, SCENEVELFX
export TIMETOCROSSING_RIGHT, TIMETOCROSSING_LEFT, ESTIMATEDTIMETOLANECROSSING
export D_MERGE, D_SPLIT, ID
export A_REQ_STAYINLANE, N_LANE_R, N_LANE_L, HAS_LANE_R, HAS_LANE_L, LANECURVATURE
export INDFRONT, HAS_FRONT, D_X_FRONT, D_Y_FRONT, V_X_FRONT, V_Y_FRONT, YAW_FRONT, TURNRATE_FRONT, A_REQ_FRONT, TTC_X_FRONT, TIMEGAP_X_FRONT
export INDREAR,  HAS_REAR,  D_X_REAR,  D_Y_REAR,  V_X_REAR,  V_Y_REAR,  YAW_REAR,  TURNRATE_REAR,  A_REQ_REAR,  TTC_X_REAR,  TIMEGAP_X_REAR
export INDLEFT,             D_X_LEFT,  D_Y_LEFT,  V_X_LEFT,  V_Y_LEFT,  YAW_LEFT,  TURNRATE_LEFT,  A_REQ_LEFT,  TTC_X_LEFT,  TIMEGAP_X_LEFT
export INDRIGHT,            D_X_RIGHT, D_Y_RIGHT, V_X_RIGHT, V_Y_RIGHT, YAW_RIGHT, TURNRATE_RIGHT, A_REQ_RIGHT, TTC_X_RIGHT, TIMEGAP_X_RIGHT
export GAINING_ON_FRONT

export FUTURETURNRATE_250MS, FUTUREACCELERATION_250MS, FUTUREDESIREDANGLE_250MS, FUTUREDESIREDSPEED_250MS, FUTUREACCELCONTROL_250MS
export FUTURETURNRATE_500MS, FUTUREACCELERATION_500MS, FUTUREDESIREDANGLE_500MS, FUTUREDESIREDSPEED_500MS, FUTUREACCELCONTROL_500MS
export FUTUREDELTAY2S, FUTUREDELTAY1S, FUTUREDELTAY_250MS
export TIMETOLANECROSSING, TIMESINCELANECROSSING, LANECHANGEDIR, LANECHANGE2S, LANECHANGE1S, LANECHANGE500MS
export TIME_CONSECUTIVE_BRAKE, TIME_CONSECUTIVE_ACCEL, TIME_CONSECUTIVE_THROTTLE

export OCCUPANCYSCHEDULEGRID_TIME_FRONT,      OCCUPANCYSCHEDULEGRID_ISOCCUPIED_FRONT
export OCCUPANCYSCHEDULEGRID_TIME_FRONTRIGHT, OCCUPANCYSCHEDULEGRID_ISOCCUPIED_FRONTRIGHT
export OCCUPANCYSCHEDULEGRID_TIME_RIGHT,      OCCUPANCYSCHEDULEGRID_ISOCCUPIED_RIGHT
export OCCUPANCYSCHEDULEGRID_TIME_BACKRIGHT,  OCCUPANCYSCHEDULEGRID_ISOCCUPIED_BACKRIGHT
export OCCUPANCYSCHEDULEGRID_TIME_BACK,       OCCUPANCYSCHEDULEGRID_ISOCCUPIED_BACK
export OCCUPANCYSCHEDULEGRID_TIME_BACKLEFT,   OCCUPANCYSCHEDULEGRID_ISOCCUPIED_BACKLEFT
export OCCUPANCYSCHEDULEGRID_TIME_LEFT,       OCCUPANCYSCHEDULEGRID_ISOCCUPIED_LEFT
export OCCUPANCYSCHEDULEGRID_TIME_FRONTLEFT,  OCCUPANCYSCHEDULEGRID_ISOCCUPIED_FRONTLEFT

export      PASTACC250MS,      PASTACC500MS,      PASTACC750MS,      PASTACC1S
export PASTTURNRATE250MS, PASTTURNRATE500MS, PASTTURNRATE750MS, PASTTURNRATE1S
export    PASTVELFY250MS,    PASTVELFY500MS,    PASTVELFY750MS,    PASTVELFY1S
export     PASTD_CL250MS,     PASTD_CL500MS,     PASTD_CL750MS,     PASTD_CL1S

export     MAXACCFX250MS,     MAXACCFX500MS,     MAXACCFX750MS,     MAXACCFX1S,     MAXACCFX1500MS,     MAXACCFX2S,     MAXACCFX2500MS,     MAXACCFX3S,     MAXACCFX4S
export     MAXACCFY250MS,     MAXACCFY500MS,     MAXACCFY750MS,     MAXACCFY1S,     MAXACCFY1500MS,     MAXACCFY2S,     MAXACCFY2500MS,     MAXACCFY3S,     MAXACCFY4S
export  MAXTURNRATE250MS,  MAXTURNRATE500MS,  MAXTURNRATE750MS,  MAXTURNRATE1S,  MAXTURNRATE1500MS,  MAXTURNRATE2S,  MAXTURNRATE2500MS,  MAXTURNRATE3S,  MAXTURNRATE4S
export    MEANACCFX250MS,    MEANACCFX500MS,    MEANACCFX750MS,    MEANACCFX1S,    MEANACCFX1500MS,    MEANACCFX2S,    MEANACCFX2500MS,    MEANACCFX3S,    MEANACCFX4S
export    MEANACCFY250MS,    MEANACCFY500MS,    MEANACCFY750MS,    MEANACCFY1S,    MEANACCFY1500MS,    MEANACCFY2S,    MEANACCFY2500MS,    MEANACCFY3S,    MEANACCFY4S
export MEANTURNRATE250MS, MEANTURNRATE500MS, MEANTURNRATE750MS, MEANTURNRATE1S, MEANTURNRATE1500MS, MEANTURNRATE2S, MEANTURNRATE2500MS, MEANTURNRATE3S, MEANTURNRATE4S
export     STDACCFX250MS,     STDACCFX500MS,     STDACCFX750MS,     STDACCFX1S,     STDACCFX1500MS,     STDACCFX2S,     STDACCFX2500MS,     STDACCFX3S,     STDACCFX4S
export     STDACCFY250MS,     STDACCFY500MS,     STDACCFY750MS,     STDACCFY1S,     STDACCFY1500MS,     STDACCFY2S,     STDACCFY2500MS,     STDACCFY3S,     STDACCFY4S
export  STDTURNRATE250MS,  STDTURNRATE500MS,  STDTURNRATE750MS,  STDTURNRATE1S,  STDTURNRATE1500MS,  STDTURNRATE2S,  STDTURNRATE2500MS,  STDTURNRATE3S,  STDTURNRATE4S

export SUBSET_EMERGENCY, SUBSET_FREE_FLOW, SUBSET_CAR_FOLLOWING, SUBSET_LANE_CROSSING, SUBSET_SUSTAINED_CROSSING
export SUBSET_AT_SIXTYFIVE, SUBSET_AUTO
export Feature_IsClean

export observe, observe!
export description, units, isint, isbool, lowerbound, upperbound, symbol, lsymbol, couldna, get, symbol2feature
export calc_occupancy_schedule_grid, put_occupancy_schedule_grid_in_meta!
export allfeatures
export NA_ALIAS

# ----------------------------------
# constants

const NA_ALIAS                         =  Inf # value assigned to floating point values when not available
const THRESHOLD_DY_CONSIDERED_IN_FRONT =  2.0 # number of meters of horizontal separation which allows two cars to be considered in front of / behind each other
const THRESHOLD_A_REQ                  = 10.0 # maximum required absolute value of required acceleration (m/s^2)
const THRESHOLD_TIME_TO_CROSSING       = 10.0 # maximum time to crossing
const THRESHOLD_TIME_TO_COLLISION      = 10.0 # maximum time to collision
const THRESHOLD_TIMEGAP                = 10.0 # maximum timegap
const THRESHOLD_TIMETOLANECROSSING     = 10.0 # [s]
const THRESHOLD_TIMESINCELANECROSSING  = 10.0 # [s]
const THRESHOLD_TIMECONSECUTIVEACCEL   = 10.0 # [s]
const SPEED_LIMIT                      = 29.06 # [m/s] TODO(tim): remove this and make it a StreetNetwork query
const KP_DESIRED_ANGLE                 = 1.0 # [-] TODO(tim): tune this
const KP_DESIRED_SPEED                 = 0.2 # [-] TODO(tim): tune this
const OCCUPANCY_SCHEDULE_GRID_DIM_S    = 4.0 # [m] longitudinal dimension per cell in the grid
const OCCUPANCY_SCHEDULE_GRID_DIM_D    = 3.7 # [m] lateral dimension per cell in the grid

# ----------------------------------
# types

abstract AbstractFeature


type ExtractedFeatureCache

	# (carid, frameind, feature symbol) -> (value, runid)
	# where runid is an Int associated with a particular run
	dict::Dict{(Int, Int, Symbol), (Float64, Int)}

    ExtractedFeatureCache() = new(Dict{(Int, Int, Symbol), (Float64, Int)}())
end
Base.setindex!(cache::ExtractedFeatureCache, val::(Float64, Int), key::(Int, Int, Symbol)) = cache.dict[key] = val
Base.getindex(cache::ExtractedFeatureCache, key::(Int, Int, Symbol)) = cache.dict[key]

type FeatureExtractBasicsPdSet
	pdset :: PrimaryDataset
	sn :: StreetNetwork
	cache :: ExtractedFeatureCache
	runid :: Int

	function FeatureExtractBasicsPdSet(
		pdset::PrimaryDataset,
		sn::StreetNetwork;
		cache::ExtractedFeatureCache=ExtractedFeatureCache(),
		runid::Int=rand(Int)
		)

		new(pdset, sn, cache, runid)
	end
end

function Base.deepcopy(basics::FeatureExtractBasicsPdSet)
	FeatureExtractBasicsPdSet(
			deepcopy(basics.pdset),
			deepcopy(basics.sn)
		)
end

# key :: (carind, validfind, feature_symbol)
Base.setindex!(basics::FeatureExtractBasicsPdSet, feature_value::Float64, key::(Int, Int, Symbol)) = basics.cache[key] = (feature_value, basics.runid)
Base.getindex(basics::FeatureExtractBasicsPdSet, key::(Int, Int, Symbol)) = basics.cache[key][1]

# ----------------------------------
# globals
sym2ftr = Dict{Symbol,AbstractFeature}()

# ----------------------------------
# abstract feature functions

symbol2feature(sym::Symbol)    = sym2ftr[sym]
# description(::AbstractFeature) = "None"
# units(      ::AbstractFeature) = "None"
# isint(      ::AbstractFeature) = error("Not implemented!") # true if the feature takes on only integer values
# isbool(     ::AbstractFeature) = error("Not implemented!") # true if the feature takes on only boolean values (0,1) (will also have isint = true)
# upperbound( ::AbstractFeature) = error("Not implemented!") # max value of the feature
# lowerbound( ::AbstractFeature) = error("Not implemented!") # min value of the feature
# couldna(    ::AbstractFeature) = true                      # whether the given variable could produce the NA alias
# symbol(     ::AbstractFeature) = error("Not implemented!")
# lsymbol(    ::AbstractFeature) = error("Not implemented!")
# get(        ::AbstractFeature, ::PrimaryDataset, ::Int, ::Int) = error("Not implemented!")

allfeatures() = collect(values(sym2ftr)) # array of all features

function get(F::AbstractFeature, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	cache = basics.cache

	key = (carind, validfind, symbol(F))
	value, runid = get(cache.dict, key, (NaN, 0))
	if runid != basics.runid
		value = _get(F, basics, carind, validfind)::Float64
		cache.dict[key] = (value, basics.runid)
	end

	!isnan(value) || error("value from feature $(symbol(F)) was NaN!")

	value
end

# ----------------------------------

function create_feature_basics(
	name         :: String,
	unit         :: String,
	isint        :: Bool,
	isbool       :: Bool,
	ub           :: Float64,
	lb           :: Float64,
	could_be_na  :: Bool,
	sym          :: Symbol,
	lstr         :: LaTeXString,
	desc         :: String
	)

	for feature in values(sym2ftr)
		@assert(desc != description(feature), "desc: $name -> $feature")
		@assert(sym  != symbol(feature), "symb: $name -> $feature")
		@assert(lstr != lsymbol(feature), "lstr: $name -> $feature")
	end
	@assert(ub >= lb)

	feature_name = symbol("Feature_" * name)
	const_name   = symbol(uppercase(name))
	sym_feature  = quot(sym)

	@eval begin
		immutable $feature_name <: AbstractFeature end
		const       $const_name  = ($feature_name)()
		description( ::$feature_name)  = $desc
		units(       ::$feature_name)  = $unit
		isint(       ::$feature_name)  = $isint
		isbool(      ::$feature_name)  = $isbool
		upperbound(  ::$feature_name)  = $ub
		lowerbound(  ::$feature_name)  = $lb
		couldna(     ::$feature_name)  = $could_be_na
		symbol(      ::$feature_name)  = $sym_feature
		lsymbol(     ::$feature_name)  = $lstr
		sym2ftr[symbol(  $const_name)] = $const_name
	end
end
function create_feature_basics_boolean( name::String, could_be_na::Bool, sym::Symbol, lstr::LaTeXString, desc::String )

	create_feature_basics(name, "-", true, true, 1.0, 0.0, could_be_na, sym, lstr, desc)
end

function observe!{F<:AbstractFeature}(
    observations::Dict{Symbol,Float64},
    basics::FeatureExtractBasicsPdSet,
    carind::Int,
    validfind::Int,
    features::Vector{F}
    )

    for f in features
        val = get(f, basics, carind, validfind)::Float64
        observations[symbol(f)] = val
    end
    observations
end
function observe!{F<:AbstractFeature}(
	observations::Vector{Float64},
    basics::FeatureExtractBasicsPdSet,
    carind::Int,
    validfind::Int,
    features::Vector{F}
    )

    for (i,f) in enumerate(features)
        observations[i] = get(f, basics, carind, validfind)::Float64
    end
    observations
end
function observe{F<:AbstractFeature}(
    basics::FeatureExtractBasicsPdSet,
    carind::Int,
    validfind::Int,
    features::Vector{F}
    )

    observations = Dict{Symbol,Float64}()
    observe!(observations, basics, carind, validfind, features)
end

# ----------------------------------
# inherent features

create_feature_basics("Yaw", "rad", false, false, Inf, -Inf, false, :yaw, L"\psi", "angle relative to the closest lane")
get(::Feature_Yaw, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int) = get(basics.pdset, :posFyaw, carind, validfind)::Float64

create_feature_basics("PosFx", "m", false, false, Inf, -Inf, false, :posFx, L"p^F_x", "x position in the frenet frame")
get(::Feature_PosFx, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int) = get(basics.pdset, :posFx, carind, validfind)::Float64

create_feature_basics("PosFy", "m", false, false, Inf, -Inf, false, :posFy, L"p^F_y", "y position in the frenet frame")
get(::Feature_PosFy, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int) = get(basics.pdset, :posFy, carind, validfind)::Float64

create_feature_basics("Speed", "m/s", false, false, Inf, -Inf, false, :speed, L"\|v\|", "speed")
_get(::Feature_Speed, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int) = get_speed(basics.pdset, carind::Integer, validfind)

create_feature_basics("Delta_Speed_Limit", "m/s", false, false, Inf, -Inf, false, :delta_speed_limit, L"Δv_{\text{limit}}", "difference between current speed and speed limit")
function get(::Feature_Delta_Speed_Limit, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	speed = get_speed(basics.pdset, carind::Integer, validfind)
	SPEED_LIMIT - speed
end

create_feature_basics("VelFx", "m/s", false, false, Inf, -Inf, false, :velFx, L"v^F_x", "velocity along the lane centerline")
get(::Feature_VelFx, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int) = get(basics.pdset, :velFx, carind, validfind)::Float64

create_feature_basics("VelFy", "m/s", false, false, Inf, -Inf, false, :velFy, L"v^F_y", "velocity perpendicular to the lane centerline")
get(::Feature_VelFy, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int) = get(basics.pdset, :velFy, carind, validfind)::Float64

create_feature_basics_boolean("IsEgo", false, :isego, L"1_{ego}", "whether the car is the ego car")
get(::Feature_IsEgo, ::FeatureExtractBasicsPdSet, carind::Int, ::Int) = carind == CARIND_EGO

create_feature_basics( "D_CL", "m", false, false, Inf, -Inf, false, :d_cl, L"d_{cl}", "distance to the closest centerline")
get(::Feature_D_CL, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int) = get(basics.pdset, :d_cl, carind, validfind)::Float64

create_feature_basics( "D_ML", "m", false, false, Inf, 0.0, true, :d_ml, L"d_{ml}", "lateral distance between center of car and the left lane marker")
function get(::Feature_D_ML, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	d_ml = get(basics.pdset, :d_ml, carind, validfind)
	isa(d_ml, NAtype) ? NA_ALIAS : d_ml
end

create_feature_basics( "D_MR", "m", false, false, Inf, 0.0, true, :d_mr, L"d_{mr}", "lateral distance (strictly positive) between center of car and the right lane marker")
function get(::Feature_D_MR, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	d_mr = get(basics.pdset, :d_mr, carind, validfind)::Float64
	return isa(d_mr, NAtype) ? NA_ALIAS : d_mr
end

create_feature_basics( "TimeToCrossing_Right", "s", false, false, Inf, 0.0, true, :ttcr_mr, L"ttcr^{mr}_y", "time to cross the right marking of assigned lane")
function get(::Feature_TimeToCrossing_Right, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	d_mr = get(D_MR, basics, carind, validfind)
	velFy = get(VELFY, basics, carind, validfind)
	d_mr > 0.0 && velFy < 0.0 ? min(-d_mr / velFy, THRESHOLD_TIME_TO_CROSSING) : NA_ALIAS
end

create_feature_basics( "TimeToCrossing_Left", "s", false, false, Inf, 0.0, true, :ttcr_ml, L"ttcr^{ml}_y", "time to cross the left marking of assigned lane")
function get(::Feature_TimeToCrossing_Left, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	d_ml = get(D_ML, basics, carind, validfind)
	velFy = get(VELFY, basics, carind, validfind)
	d_ml < 0.0 && velFy > 0.0 ? min(-d_ml / velFy,THRESHOLD_TIME_TO_CROSSING) : NA_ALIAS
end

create_feature_basics( "EstimatedTimeToLaneCrossing", "s", false, false, Inf, 0.0, true, :est_ttcr, L"ttcr^\text{est}_y", "time to crossing the lane boundary in the direction of travel")
function get(::Feature_EstimatedTimeToLaneCrossing, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	ttcr_left = get(TIMETOCROSSING_LEFT, basics, carind, validfind)
	if !isinf(ttcr_left)
		return ttcr_left
	end
	get(TIMETOCROSSING_RIGHT, basics, carind, validfind)
end

create_feature_basics( "A_REQ_StayInLane", "m/s2", false, false, Inf, -Inf, true, :a_req_stayinlane, L"a^{req}_y", "acceleration required to stay in the current lane")
function get(::Feature_A_REQ_StayInLane, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	velFy = get(VELFY, basics, carind, validfind)

	if velFy > 0.0
		d_mr = get(D_MR, basics, carind, validfind)
		return d_mr > 0.0 ? min( 0.5velFy*velFy / d_mr, THRESHOLD_A_REQ) : NA_ALIAS
	else
		d_ml = get(D_ML, basics, carind, validfind)
		return d_ml < 0.0 ? min(-0.5velFy*velFy / d_ml, THRESHOLD_A_REQ) : NA_ALIAS
	end
end

create_feature_basics( "N_LANE_L", "-", true, false, 10.0, 0.0, false, :n_lane_left, L"nlane_l", "number of lanes on the left side of this vehicle")
get(::Feature_N_LANE_L, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int) = float64(get(basics.pdset, :nll, carind, validfind))

create_feature_basics( "N_LANE_R", "-", true, false, 10.0, 0.0, false, :n_lane_right, L"nlane_r", "number of lanes on the right side of this vehicle")
get(::Feature_N_LANE_R, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int) = float64(get(basics.pdset, :nlr, carind, validfind))

create_feature_basics_boolean( "HAS_LANE_R", false, :has_lane_right, L"\exists_{\text{lane}}^\text{r}", "whether at least one lane exists to right")
get(::Feature_HAS_LANE_R, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int) = float64(get(N_LANE_R, basics, carind, validfind) > 0.0)

create_feature_basics_boolean( "HAS_LANE_L", false, :has_lane_left, L"\exists_{\text{lane}}^\text{l}", "whether at least one lane exists to the left")
get(::Feature_HAS_LANE_L, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int) = float64(get(N_LANE_L, basics, carind, validfind) > 0.0)

###########################
# OCCUPANCY SCHEDULE GRID #
###########################

# TODO(tim): Move these to StreetNetwork
function _calc_jumps_required_to_reach_projectable_lane_segment_downstream(
	sn    :: StreetNetwork,
	lane  :: StreetLane, # lane we want to find the connected component closest to the given pt
	posGx :: Float64,
	posGy :: Float64;
	max_dist :: Float64 = 2000.0, # [m] maximum distance to search downstream for curve for a valid segment to project (posGx, posGy) to
	)

	#=
	RETURNS
		njumps :: Int = the number of times next_lane() must be called to reach the given lane
		                returns 0 if the current lane is correct
		                returns -1 if no connecting lane is found
	=#

	current_lane = lane
	search_dist = -lane.curve.s[end]
	found_lane = false
	njumps = 0

	finished = false
	while !finished

		extind = Curves.closest_point_extind_to_curve(current_lane.curve, posGx, posGy)

		if !is_extind_at_curve_end(current_lane.curve, extind)
			return njumps
		elseif has_next_lane(sn, current_lane)
			search_dist += current_lane.curve.s[end]
			current_lane = next_lane(sn, current_lane)
			njumps += 1
			finished = search_dist > max_dist
		else
			finished = true
		end
	end

	-1
end
function _calc_jumps_required_to_reach_projectable_lane_segment_upstream(
	sn    :: StreetNetwork,
	lane  :: StreetLane, # lane we want to find the connected component closest to the given pt
	posGx :: Float64,
	posGy :: Float64;
	max_dist :: Float64 = 2000.0, # [m] maximum distance to search upstream for curve for a valid segment to project (posGx, posGy) to
	)

	#=
	RETURNS
		njumps :: Int = the number of times prev_lane() must be called to reach the given lane
		                returns 0 if the current lane is correct
		                returns -1 if no connecting lane is found
	=#

	current_lane = lane
	search_dist = -lane.curve.s[end]
	found_lane = false
	njumps = 0

	finished = false
	while !finished

		extind = Curves.closest_point_extind_to_curve(current_lane.curve, posGx, posGy)

		if !is_extind_at_curve_end(current_lane.curve, extind)
			return njumps
		elseif has_prev_lane(sn, current_lane)
			search_dist += current_lane.curve.s[end]
			current_lane = prev_lane(sn, current_lane)
			njumps += 1
			finished = search_dist > max_dist
		else
			finished = true
		end
	end

	-1
end
function _calc_jumps_required_to_reach_projectable_lane_segment(
	sn    :: StreetNetwork,
	lane  :: StreetLane, # lane we want to find the connected component closest to the given pt
	posGx :: Float64,
	posGy :: Float64;
	max_dist_downstream :: Float64 = 2000.0, # [m] maximum distance to search downstream for curve for a valid segment to project (posGx, posGy) to
	max_dist_upstream :: Float64 = max_dist_downstream # [m] maximum distance to search upstream the curve for a valid segment to project (posGx, posGy) to
	)

	#=
	RETURNS (njumps, success)
		njumps :: Int = the number of times prev_lane() or next_lane() must be called to reach the given lane
		                if 0 it means the original lane is it, positive corresponds to next_lane(),
		                negative to prev_lane()
		success :: Bool = true if a lane was found
	=#

	# search downstream first
	njumps = _calc_jumps_required_to_reach_projectable_lane_segment_downstream(sn, lane, posGx, posGy, max_dist=max_dist_downstream)
	if njumps ≥ 0
		return (njumps, true)
	end

	# now search upstream
	njumps = _calc_jumps_required_to_reach_projectable_lane_segment_upstream(sn, lane, posGx, posGy, max_dist=max_dist_upstream)
	if njumps ≥ 0
		return (-njumps, true)
	else
		return (0, false)
	end
end

function _jump_downstream_lane(sn::StreetNetwork, lane::StreetLane, njumps::Int)
	retval = lane
	for i = 1 : njumps
		retval = next_lane(sn, retval)
	end
	retval
end
function _jump_upstream_lane(sn::StreetNetwork, lane::StreetLane, njumps::Int)
	retval = lane
	for i = 1 : njumps
		retval = prev_lane(sn, retval)
	end
	retval
end
function _jump_along_lane(sn::StreetNetwork, lane::StreetLane, njumps::Int)

	# njumps is number of jumps to perform
	# we assume that the lanes actually exist (will throw an exception otherwise)
	# positive indicates use of next_lane()
	# negative indicates use of prev_lane()

	if njumps > 0
		_jump_downstream_lane(sn, lane, njumps)
	elseif njumps < 0
		_jump_upstream_lane(sn, lane, -njumps)
	else
		lane
	end
end

function _calc_num_real_roots_of_quadratic(val::Float64; absolute_tolerance_single_root::Float64 = eps(Float64))
	# val = b^2 - 4ac
	if isapprox(val, 0.0, atol = absolute_tolerance_single_root)
		1
	elseif val > 0.0
		2
	else
		0
	end
end
function _calc_num_real_roots_of_quadratic(a::Float64, b::Float64, c::Float64;
	absolute_tolerance_single_root::Float64 = eps(Float64))
	_calc_num_real_roots_of_quadratic(b*b - 4*a*c,
		absolute_tolerance_single_root=absolute_tolerance_single_root)
end

function _quadratic_roots_default_to_smallest_positive(a::Float64, b::Float64, c::Float64)

	val = b*b - 4*a*c
	if val ≥ 0.0
		val_lo = (-b - sqrt(val)) / (2a)
		val_hi = (-b + sqrt(val)) / (2a)

		if val_lo < 0.0
			return val_hi
		else
			return val_lo
		end
	else
		return NaN
	end
end

type OccupancyScheduleGridCell
	occupied :: Bool
	time     :: Float64 # if occupied; time to exit, if unoccupied, time to entry

	OccupancyScheduleGridCell(occupied::Bool=false, time::Float64=Inf) = new(occupied, time)
end
immutable OccupancyScheduleGrid
	F  :: OccupancyScheduleGridCell # front
	FR :: OccupancyScheduleGridCell # front-right
	R  :: OccupancyScheduleGridCell # right
	BR :: OccupancyScheduleGridCell # back-right
	B  :: OccupancyScheduleGridCell # back
	BL :: OccupancyScheduleGridCell # back-left
	L  :: OccupancyScheduleGridCell # left
	FL :: OccupancyScheduleGridCell # front-left

	function OccupancyScheduleGrid()
		new(
			OccupancyScheduleGridCell(),
			OccupancyScheduleGridCell(),
			OccupancyScheduleGridCell(),
			OccupancyScheduleGridCell(),
			OccupancyScheduleGridCell(),
			OccupancyScheduleGridCell(),
			OccupancyScheduleGridCell(),
			OccupancyScheduleGridCell()
		)
	end
end
function _project_pt_to_lane_frenet(
	sn     :: StreetNetwork,
	origin :: TilePoint2DProjectionResult, # curve & location we are projecting against
	posGx  :: Float64, # location we are projecting to the curve
	posGy  :: Float64;
	max_dist :: Float64 = 1000.0 # [m]
	)

	#=
	RETURNS: (s, d)
		s :: Float64 = separation distance along the curve between (posGx, posGy) and the origin
						positive if downstream, negative if upstream
		d :: Float64 = separation distance perpendicular to the curve between (posGx, posGy) and the origin
						positive if to the left, negative if to the right
						Δd = 0.0 means it lies on the lane centerline

		Both values will be NaN if no such lane can be found
	=#

	lane = get_lane(origin)
	njumps, successful = _calc_jumps_required_to_reach_projectable_lane_segment(sn, lane, posGx, posGy, max_dist_downstream=max_dist)

	if successful

		origin_s = origin.curvept.s

		if njumps == 0

			extind = Curves.closest_point_extind_to_curve(lane.curve, posGx, posGy)
			curvept = Curves.curve_at(lane.curve, extind)
			local_s, d = Curves.pt_to_frenet_xy(curvept, posGx, posGy)

			s = local_s - origin_s
			return (s,d)
		elseif njumps > 0

			search_dist = 0.0
			for i = 1 : njumps
				search_dist += lane.curve.s[end]
				lane = next_lane(sn, lane)
			end

			extind = Curves.closest_point_extind_to_curve(lane.curve, posGx, posGy)
			curvept = Curves.curve_at(lane.curve, extind)
			local_s, d = Curves.pt_to_frenet_xy(curvept, posGx, posGy)

			s = search_dist + local_s - origin_s
			return (s,d)
		elseif njumps < 0

			search_dist = 0.0
			for i = 1 : njumps
				lane = next_lane(sn, lane)
				search_dist += lane.curve.s[end]
			end

			extind = Curves.closest_point_extind_to_curve(lane.curve, posGx, posGy)
			curvept = Curves.curve_at(lane.curve, extind)
			local_s, d = Curves.pt_to_frenet_xy(curvept, posGx, posGy)

			s = search_dist + origin_s - local_s
			return (s,d)
		end
	else
		return (NaN, NaN)
	end
end
function _get_smallest_nonan_value(time_depart_forward::Float64, time_depart_backward::Float64)

	is_valid_time_depart_forward = isnan(time_depart_forward)
	is_valid_time_depart_backward = isnan(time_depart_backward)

	if is_valid_time_depart_backward
		if is_valid_time_depart_forward
			return min(time_depart_forward, time_depart_backward)
		else
			return time_depart_backward
		end
	else
		if is_valid_time_depart_forward
			return time_depart_forward
		else
			return Inf
		end
	end
end

function _update_time_to_enter_cell!(cell::OccupancyScheduleGridCell, time_enter::Float64)
	if !cell.occupied
		if !isnan(time_enter)
			cell.time = time_enter
		else
			cell.time = min(cell.time, time_enter)
		end
	end
	nothing
end
function _update_time_to_depart_cell!(cell::OccupancyScheduleGridCell, time_depart_forward::Float64, time_depart_backward::Float64)
	thetime = _get_smallest_nonan_value(time_depart_forward, time_depart_backward)

	if cell.occupied
		cell.time = max(cell.time, thetime)
	else
		cell.occupied = true
	end
	nothing
end
function calc_occupancy_schedule_grid(basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)

	#=
	Computes an occupancy schedule grid for the given vehicle
	Computes the follow values for the eight locations
	  - Whether the cell is occupied or not
	  - Time until a vehicle enters the cell (if it is unoccupied, 0.0 otherwise)
	  - Time until a vehicle departs the cell (if it is occupied, 0.0)
	  - Combined time value, max(T_enter, T_depart)
	The locations are the grid cells in the cardinal and ordinal directions
	  {front, front-right, right, back-right, back, back-left, left, front-left}
	All calculations are performed assuming constant relative acceleration
	=#

	grid = OccupancyScheduleGrid()
	pdset = basics.pdset
	sn = basics.sn

	# Grid frame is centered on the target vehicle
	#    and is oriented with the lane

	frameind = validfind2frameind(pdset, validfind)
	grid_center_x = get(pdset, :posGx, carind, frameind, validfind)::Float64
	grid_center_y = get(pdset, :posGy, carind, frameind, validfind)::Float64
	acc_us = get(ACCFX, basics, carind, validfind)
	vel_us = get(VELFX, basics, carind, validfind)
	proj_center = project_point_to_streetmap(grid_center_x, grid_center_y, sn)

	if proj_center.successful

		# project entire scene to the frame
		# figure out each occupancy schedule grid position

		nothercarsinframe = get_num_other_cars_in_frame(pdset, validfind)
		for loop_carind = -1 : nothercarsinframe-1 # this includes the ego vehicle

			if loop_carind == carind
				continue
			end

			carPosGx = get(pdset, :posGx, loop_carind, frameind, validfind)::Float64
			carPosGy = get(pdset, :posGy, loop_carind, frameind, validfind)::Float64
			s, d = _project_pt_to_lane_frenet(sn, proj_center, carPosGx, carPosGy)

			acc_rel = get(ACCFX, basics, loop_carind, validfind) - acc_us
			vel_rel = get(VELFX, basics, loop_carind, validfind) - vel_us

			time_to_reach_neg_2 = _quadratic_roots_default_to_smallest_positive(acc_rel/2,vel_rel,s + 1.5*OCCUPANCY_SCHEDULE_GRID_DIM_D)
			time_to_reach_neg_1 = _quadratic_roots_default_to_smallest_positive(acc_rel/2,vel_rel,s + 0.5*OCCUPANCY_SCHEDULE_GRID_DIM_D)
			time_to_reach_pos_1 = _quadratic_roots_default_to_smallest_positive(acc_rel/2,vel_rel,s - 0.5*OCCUPANCY_SCHEDULE_GRID_DIM_D)
			time_to_reach_pos_2 = _quadratic_roots_default_to_smallest_positive(acc_rel/2,vel_rel,s - 1.5*OCCUPANCY_SCHEDULE_GRID_DIM_D)

			if OCCUPANCY_SCHEDULE_GRID_DIM_D ≤ d ≤ 2*OCCUPANCY_SCHEDULE_GRID_DIM_D
				# LEFT

				if s > 1.5*OCCUPANCY_SCHEDULE_GRID_DIM_D
					# car is in front of FRONT_LEFT
					_update_time_to_enter_cell!(grid.FL, time_to_reach_pos_2)
					_update_time_to_enter_cell!(grid.L,  time_to_reach_pos_1)
					_update_time_to_enter_cell!(grid.BL, time_to_reach_neg_1)
				elseif s > 0.5*OCCUPANCY_SCHEDULE_GRID_DIM_D
					# car is in FRONT_LEFT
					_update_time_to_depart_cell!(grid.FL, time_to_reach_pos_2, time_to_reach_pos_1)
					_update_time_to_enter_cell!(grid.L,  time_to_reach_pos_1)
					_update_time_to_enter_cell!(grid.BL, time_to_reach_neg_1)
				elseif s > -0.5*OCCUPANCY_SCHEDULE_GRID_DIM_D
					# car is in LEFT
					_update_time_to_enter_cell!(grid.FL,  time_to_reach_pos_1)
					_update_time_to_depart_cell!(grid.L, time_to_reach_pos_1, time_to_reach_neg_1)
					_update_time_to_enter_cell!(grid.BL, time_to_reach_neg_1)
				elseif s > -1.5*OCCUPANCY_SCHEDULE_GRID_DIM_D
					# car is in BEHIND_LEFT
					_update_time_to_enter_cell!(grid.FL,  time_to_reach_pos_1)
					_update_time_to_enter_cell!(grid.L,  time_to_reach_neg_1)
					_update_time_to_depart_cell!(grid.BL, time_to_reach_neg_1, time_to_reach_neg_2)
				else
					# car is in behind of BEHIND_LEFT
					_update_time_to_enter_cell!(grid.FL, time_to_reach_pos_1)
					_update_time_to_enter_cell!(grid.L,  time_to_reach_neg_1)
					_update_time_to_enter_cell!(grid.BL, time_to_reach_neg_2)
				end
			elseif -OCCUPANCY_SCHEDULE_GRID_DIM_D ≤ d ≤ OCCUPANCY_SCHEDULE_GRID_DIM_D
				# CENTER

				if s > 1.5*OCCUPANCY_SCHEDULE_GRID_DIM_D
					# car is in front of FRONT
					_update_time_to_enter_cell!(grid.F, time_to_reach_pos_2)
					_update_time_to_enter_cell!(grid.B, time_to_reach_neg_1)
				elseif s > 0.5*OCCUPANCY_SCHEDULE_GRID_DIM_D
					# car is in FRONT
					_update_time_to_depart_cell!(grid.F, time_to_reach_pos_2, time_to_reach_pos_1)
					_update_time_to_enter_cell!(grid.B, time_to_reach_neg_1)
				elseif s > -0.5*OCCUPANCY_SCHEDULE_GRID_DIM_D
					# car is in CENTER
					_update_time_to_enter_cell!(grid.F,  time_to_reach_pos_1)
					_update_time_to_enter_cell!(grid.B, time_to_reach_neg_1)
				elseif s > -1.5*OCCUPANCY_SCHEDULE_GRID_DIM_D
					# car is in BEHIND
					_update_time_to_enter_cell!(grid.F,  time_to_reach_pos_1)
					_update_time_to_depart_cell!(grid.B, time_to_reach_neg_1, time_to_reach_neg_2)
				else
					# car is in behind of BEHIND
					_update_time_to_enter_cell!(grid.F, time_to_reach_pos_1)
					_update_time_to_enter_cell!(grid.B, time_to_reach_neg_2)
				end
			elseif -2*OCCUPANCY_SCHEDULE_GRID_DIM_D ≤ d ≤ -OCCUPANCY_SCHEDULE_GRID_DIM_D
				# RIGHT

				if s > 1.5*OCCUPANCY_SCHEDULE_GRID_DIM_D
					# car is in front of FRONT_RIGHT
					_update_time_to_enter_cell!(grid.FR, time_to_reach_pos_2)
					_update_time_to_enter_cell!(grid.R,  time_to_reach_pos_1)
					_update_time_to_enter_cell!(grid.BR, time_to_reach_neg_1)
				elseif s > 0.5*OCCUPANCY_SCHEDULE_GRID_DIM_D
					# car is in FRONT_RIGHT
					_update_time_to_depart_cell!(grid.FR, time_to_reach_pos_2, time_to_reach_pos_1)
					_update_time_to_enter_cell!(grid.R,  time_to_reach_pos_1)
					_update_time_to_enter_cell!(grid.BR, time_to_reach_neg_1)
				elseif s > -0.5*OCCUPANCY_SCHEDULE_GRID_DIM_D
					# car is in RIGHT
					_update_time_to_enter_cell!(grid.FR,  time_to_reach_pos_1)
					_update_time_to_depart_cell!(grid.R, time_to_reach_pos_1, time_to_reach_neg_1)
					_update_time_to_enter_cell!(grid.BR, time_to_reach_neg_1)
				elseif s > -1.5*OCCUPANCY_SCHEDULE_GRID_DIM_D
					# car is in BEHIND_RIGHT
					_update_time_to_enter_cell!(grid.FR,  time_to_reach_pos_1)
					_update_time_to_enter_cell!(grid.R,  time_to_reach_neg_1)
					_update_time_to_depart_cell!(grid.BR, time_to_reach_neg_1, time_to_reach_neg_2)
				else
					# car is in behind of BEHIND_RIGHT
					_update_time_to_enter_cell!(grid.FR, time_to_reach_pos_1)
					_update_time_to_enter_cell!(grid.R,  time_to_reach_neg_1)
					_update_time_to_enter_cell!(grid.BR, time_to_reach_neg_2)
				end
			end
		end
	end

	grid
end
function put_occupancy_schedule_grid_in_meta!(basics::FeatureExtractBasicsPdSet, grid::OccupancyScheduleGrid, carind::Int, validfind::Int)

	# checkmeta(carind, validfind, :osg_isoccupied_f) # need symbol so Dict is initialized

	basics[(carind, validfind, :osg_isoccupied_f)]  = float(grid.F.occupied)
	basics[(carind, validfind, :osg_isoccupied_fr)] = float(grid.FR.occupied)
	basics[(carind, validfind, :osg_isoccupied_r)]  = float(grid.R.occupied)
	basics[(carind, validfind, :osg_isoccupied_br)] = float(grid.BR.occupied)
	basics[(carind, validfind, :osg_isoccupied_b)]  = float(grid.B.occupied)
	basics[(carind, validfind, :osg_isoccupied_bl)] = float(grid.BL.occupied)
	basics[(carind, validfind, :osg_isoccupied_l)]  = float(grid.L.occupied)
	basics[(carind, validfind, :osg_isoccupied_fl)] = float(grid.FL.occupied)

	basics[(carind, validfind, :osg_time_f)]  = grid.F.time
	basics[(carind, validfind, :osg_time_fr)] = grid.FR.time
	basics[(carind, validfind, :osg_time_r)]  = grid.R.time
	basics[(carind, validfind, :osg_time_br)] = grid.BR.time
	basics[(carind, validfind, :osg_time_b)]  = grid.B.time
	basics[(carind, validfind, :osg_time_bl)] = grid.BL.time
	basics[(carind, validfind, :osg_time_l)]  = grid.L.time
	basics[(carind, validfind, :osg_time_fl)] = grid.FL.time
end

create_feature_basics_boolean( "OccupancyScheduleGrid_IsOccupied_Front",      false, :osg_isoccupied_f,  L"OCC_\text{osg}^\text{f}",  "whether f cell is occupied in occupancy schedule grid")
function _get(::Feature_OccupancyScheduleGrid_IsOccupied_Front, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	grid = calc_occupancy_schedule_grid(basics, carind, validfind)
	put_occupancy_schedule_grid_in_meta!(basics, grid, carind, validfind)
	basics[(carind, validfind, :osg_isoccupied_f)]
end
create_feature_basics_boolean( "OccupancyScheduleGrid_IsOccupied_FrontRight", false, :osg_isoccupied_fr, L"OCC_\text{osg}^\text{fr}", "whether fr cell is occupied in occupancy schedule grid")
function _get(::Feature_OccupancyScheduleGrid_IsOccupied_FrontRight, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	grid = calc_occupancy_schedule_grid(basics, carind, validfind)
	put_occupancy_schedule_grid_in_meta!(basics, grid, carind, validfind)
	basics[(carind, validfind, :osg_isoccupied_fr)]
end
create_feature_basics_boolean( "OccupancyScheduleGrid_IsOccupied_Right",      false, :osg_isoccupied_r,  L"OCC_\text{osg}^\text{r}",  "whether r cell is occupied in occupancy schedule grid")
function _get(::Feature_OccupancyScheduleGrid_IsOccupied_Right, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	grid = calc_occupancy_schedule_grid(basics, carind, validfind)
	put_occupancy_schedule_grid_in_meta!(basics, grid, carind, validfind)
	basics[(carind, validfind, :osg_isoccupied_r)]
end
create_feature_basics_boolean( "OccupancyScheduleGrid_IsOccupied_BackRight",  false, :osg_isoccupied_br, L"OCC_\text{osg}^\text{br}", "whether br cell is occupied in occupancy schedule grid")
function _get(::Feature_OccupancyScheduleGrid_IsOccupied_BackRight, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	grid = calc_occupancy_schedule_grid(basics, carind, validfind)
	put_occupancy_schedule_grid_in_meta!(basics, grid, carind, validfind)
	basics[(carind, validfind, :osg_isoccupied_br)]
end
create_feature_basics_boolean( "OccupancyScheduleGrid_IsOccupied_Back",       false, :osg_isoccupied_b,  L"OCC_\text{osg}^\text{b}",  "whether b cell is occupied in occupancy schedule grid")
function _get(::Feature_OccupancyScheduleGrid_IsOccupied_Back, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	grid = calc_occupancy_schedule_grid(basics, carind, validfind)
	put_occupancy_schedule_grid_in_meta!(basics, grid, carind, validfind)
	basics[(carind, validfind, :osg_isoccupied_b)]
end
create_feature_basics_boolean( "OccupancyScheduleGrid_IsOccupied_BackLeft",   false, :osg_isoccupied_bl, L"OCC_\text{osg}^\text{bl}", "whether bl cell is occupied in occupancy schedule grid")
function _get(::Feature_OccupancyScheduleGrid_IsOccupied_BackLeft, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	grid = calc_occupancy_schedule_grid(basics, carind, validfind)
	put_occupancy_schedule_grid_in_meta!(basics, grid, carind, validfind)
	basics[(carind, validfind, :osg_isoccupied_bl)]
end
create_feature_basics_boolean( "OccupancyScheduleGrid_IsOccupied_Left",       false, :osg_isoccupied_l,  L"OCC_\text{osg}^\text{l}",  "whether l cell is occupied in occupancy schedule grid")
function _get(::Feature_OccupancyScheduleGrid_IsOccupied_Left, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	grid = calc_occupancy_schedule_grid(basics, carind, validfind)
	put_occupancy_schedule_grid_in_meta!(basics, grid, carind, validfind)
	basics[(carind, validfind, :osg_isoccupied_l)]
end
create_feature_basics_boolean( "OccupancyScheduleGrid_IsOccupied_FrontLeft",  false, :osg_isoccupied_fl, L"OCC_\text{osg}^\text{fl}", "whether fl cell is occupied in occupancy schedule grid")
function _get(::Feature_OccupancyScheduleGrid_IsOccupied_FrontLeft, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	grid = calc_occupancy_schedule_grid(basics, carind, validfind)
	put_occupancy_schedule_grid_in_meta!(basics, grid, carind, validfind)
	basics[(carind, validfind, :osg_isoccupied_fl)]
end

create_feature_basics( "OccupancyScheduleGrid_Time_Front",      "s", false, false, Inf, 0.0, true, :osg_time_f,  L"T_\text{osg}^\text{f}",  "time to entry or exit of cell f in the occupancy schedule grid")
function _get(::Feature_OccupancyScheduleGrid_Time_Front, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	grid = calc_occupancy_schedule_grid(basics, carind, validfind)
	put_occupancy_schedule_grid_in_meta!(basics, grid, carind, validfind)
	basics[(carind, validfind, :osg_time_f)]
end
create_feature_basics( "OccupancyScheduleGrid_Time_FrontRight", "s", false, false, Inf, 0.0, true, :osg_time_fr, L"T_\text{osg}^\text{fr}", "time to entry or exit of cell fr in the occupancy schedule grid")
function _get(::Feature_OccupancyScheduleGrid_Time_FrontRight, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	grid = calc_occupancy_schedule_grid(basics, carind, validfind)
	put_occupancy_schedule_grid_in_meta!(basics, grid, carind, validfind)
	basics[(carind, validfind, :osg_time_fr)]
end
create_feature_basics( "OccupancyScheduleGrid_Time_Right",      "s", false, false, Inf, 0.0, true, :osg_time_r,  L"T_\text{osg}^\text{r}",  "time to entry or exit of cell r in the occupancy schedule grid")
function _get(::Feature_OccupancyScheduleGrid_Time_Right, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	grid = calc_occupancy_schedule_grid(basics, carind, validfind)
	put_occupancy_schedule_grid_in_meta!(basics, grid, carind, validfind)
	basics[(carind, validfind, :osg_time_r)]
end
create_feature_basics( "OccupancyScheduleGrid_Time_BackRight",  "s", false, false, Inf, 0.0, true, :osg_time_br, L"T_\text{osg}^\text{br}", "time to entry or exit of cell br in the occupancy schedule grid")
function _get(::Feature_OccupancyScheduleGrid_Time_BackRight, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	grid = calc_occupancy_schedule_grid(basics, carind, validfind)
	put_occupancy_schedule_grid_in_meta!(basics, grid, carind, validfind)
	basics[(carind, validfind, :osg_time_br)]
end
create_feature_basics( "OccupancyScheduleGrid_Time_Back",       "s", false, false, Inf, 0.0, true, :osg_time_b,  L"T_\text{osg}^\text{b}",  "time to entry or exit of cell b in the occupancy schedule grid")
function _get(::Feature_OccupancyScheduleGrid_Time_Back, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	grid = calc_occupancy_schedule_grid(basics, carind, validfind)
	put_occupancy_schedule_grid_in_meta!(basics, grid, carind, validfind)
	basics[(carind, validfind, :osg_time_b)]
end
create_feature_basics( "OccupancyScheduleGrid_Time_BackLeft",   "s", false, false, Inf, 0.0, true, :osg_time_bl, L"T_\text{osg}^\text{bl}", "time to entry or exit of cell bl in the occupancy schedule grid")
function _get(::Feature_OccupancyScheduleGrid_Time_BackLeft, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	grid = calc_occupancy_schedule_grid(basics, carind, validfind)
	put_occupancy_schedule_grid_in_meta!(basics, grid, carind, validfind)
	basics[(carind, validfind, :osg_time_bl)]
end
create_feature_basics( "OccupancyScheduleGrid_Time_Left",       "s", false, false, Inf, 0.0, true, :osg_time_l,  L"T_\text{osg}^\text{l}",  "time to entry or exit of cell l in the occupancy schedule grid")
function _get(::Feature_OccupancyScheduleGrid_Time_Left, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	grid = calc_occupancy_schedule_grid(basics, carind, validfind)
	put_occupancy_schedule_grid_in_meta!(basics, grid, carind, validfind)
	basics[(carind, validfind, :osg_time_l)]
end
create_feature_basics( "OccupancyScheduleGrid_Time_FrontLeft",  "s", false, false, Inf, 0.0, true, :osg_time_fl, L"T_\text{osg}^\text{fl}", "time to entry or exit of cell fl in the occupancy schedule grid")
function _get(::Feature_OccupancyScheduleGrid_Time_FrontLeft, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	grid = calc_occupancy_schedule_grid(basics, carind, validfind)
	put_occupancy_schedule_grid_in_meta!(basics, grid, carind, validfind)
	basics[(carind, validfind, :osg_time_fl)]
end

#########
# FRONT #
#########

function _get_carind_front_and_dist(basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int;
	max_dist_front::Float64 = 100.0
	)

	best_carind = -2
	best_dist   = max_dist_front # start at max
	best_ΔpFy   = 0.0

	sn = basics.sn
	pdset = basics.pdset

	frameind = validfind2frameind(pdset, validfind)

	ncarsinframe = get_num_other_cars_in_frame(pdset, validfind)
	cars_to_check = Set{Int}()
	for i in -1 : (ncarsinframe-1)
		push!(cars_to_check, i)
	end

	lanetags = Array(LaneTag, ncarsinframe+1)
	for cind in cars_to_check
		lanetags[cind+2] = get(pdset, :lanetag, cind, frameind, validfind)::LaneTag
	end

	posFx = get(pdset, :posFx,  carind, frameind, validfind)::Float64
	posFy = get(pdset, :d_cl,  carind, frameind, validfind)::Float64
	active_lanetag = lanetags[carind+2]
	active_lane = get_lane(sn, active_lanetag)
	search_dist = 0.0

	delete!(cars_to_check, carind)

	finished = false
	while !finished
		to_remove = Set{Int}()
		for target_carind in cars_to_check
			if active_lanetag == lanetags[target_carind+2]
				target_posFx = get(pdset, :posFx, target_carind, frameind, validfind)

				# only accept cars that are in front of us
				# and better than what we already have

				target_dist = target_posFx - posFx + search_dist
				if 0.0 < target_dist < best_dist

					target_posFy = get(pdset, :d_cl, target_carind, frameind, validfind)
					ΔpFy = target_posFy - posFy
					if abs(ΔpFy) < THRESHOLD_DY_CONSIDERED_IN_FRONT
						best_carind, best_dist = target_carind, target_dist
						best_ΔpFy = ΔpFy
					end
				end

				push!(to_remove, target_carind)
			end
		end
		if best_carind != -2
			break
		end

		for target_carind in to_remove
			delete!(cars_to_check, target_carind)
		end
		if isempty(cars_to_check)
			break
		end

		# move to next lane
		if has_next_lane(sn, active_lane)
			search_dist += active_lane.curve.s[end]
			active_lane = next_lane(sn, active_lane)
			active_lanetag = active_lane.id
			finished = search_dist-posFx > best_dist
		else
			finished = true
		end
	end

	if best_carind != -2
		return (best_carind, best_dist, best_ΔpFy)
	else
		return (-2, NaN, NaN)
	end
end

create_feature_basics( "IndFront", "-", true, false, Inf, -1.0, true, :ind_front, L"i_{front}", "index of the closest car in front")
function _get(::Feature_IndFront, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int;
	max_dist_front :: Float64 = 100.0
	)

	(best_carind, best_dist, best_ΔpFy) = _get_carind_front_and_dist(basics, carind, validfind, max_dist_front=max_dist_front)

	pdset = basics.pdset

	if best_carind != -2
		frameind = validfind2frameind(pdset, validfind)
		basics[(carind, validfind, :d_x_front)] = best_dist
		basics[(carind, validfind, :d_y_front)] = best_ΔpFy
		basics[(carind, validfind, :v_x_front)] = get(pdset, :velFx,   best_carind, frameind, validfind) - get(pdset, :velFx, carind, frameind, validfind)
		basics[(carind, validfind, :v_y_front)] = get(pdset, :velFy,   best_carind, frameind, validfind) - get(pdset, :velFy, carind, frameind, validfind)
		basics[(carind, validfind, :yaw_front)] = get(pdset, :posFyaw, best_carind, frameind, validfind)
		basics[(carind, validfind, :turnrate_front)] = get(TURNRATE, basics, best_carind, validfind)
		return float64(best_carind)
	else
		basics[(carind, validfind, :d_x_front)] = NA_ALIAS
		basics[(carind, validfind, :d_y_front)] = NA_ALIAS
		basics[(carind, validfind, :v_x_front)] = NA_ALIAS
		basics[(carind, validfind, :v_y_front)] = NA_ALIAS
		basics[(carind, validfind, :yaw_front)] = NA_ALIAS
		basics[(carind, validfind, :turnrate_front)] = NA_ALIAS
		return NA_ALIAS
	end
end

create_feature_basics_boolean( "HAS_FRONT", false, :has_front, L"\exists_{fo}", "whether there is a car in front")
get( ::Feature_HAS_FRONT, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int) = float64(get(INDFRONT, basics, carind, validfind) != NA_ALIAS)

create_feature_basics( "D_X_FRONT", "m", false, false, Inf, 0.0, true, :d_x_front, L"d_{x,fo}", "longitudinal distance to the closest vehicle in the same lane in front")
function get( ::Feature_D_X_FRONT, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	get(INDFRONT, basics, carind, validfind)
	basics[(carind, validfind, :d_x_front)]
end

create_feature_basics( "D_Y_FRONT", "m", false, false, Inf, -Inf, true, :d_y_front, L"d_{y,fo}", "lateral distance to the closest vehicle in the same lane in front")
function get( ::Feature_D_Y_FRONT, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	get(INDFRONT, basics, carind, validfind)
	basics[(carind, validfind, :d_y_front)]
end

create_feature_basics( "V_X_FRONT", "m/s", false, false, Inf, -Inf, true, :v_x_front, L"v^{rel}_{x,fo}", "relative x velocity of the vehicle in front of you")
function get( ::Feature_V_X_FRONT, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	get(INDFRONT, basics, carind, validfind)
	basics[(carind, validfind, :v_x_front)]
end

create_feature_basics( "V_Y_FRONT", "m/s", false, false, Inf, -Inf, true, :v_y_front, L"v^{rel}_{y,fo}", "relative y velocity of the vehicle in front of you")
function get( ::Feature_V_Y_FRONT, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	get(INDFRONT, basics, carind, validfind)
	basics[(carind, validfind, :v_y_front)]
end

create_feature_basics( "YAW_FRONT", "rad", false, false, float64(pi), float64(-pi), true, :yaw_front, L"\psi_{fo}", "yaw of the vehicle in front of you")
function get( ::Feature_YAW_FRONT, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	get(INDFRONT, basics, carind, validfind)
	basics[(carind, validfind, :yaw_front)]
end

create_feature_basics( "TURNRATE_FRONT", "rad/s", false, false, Inf, -Inf, true, :turnrate_front, L"\dot{\psi}_{fo}", "turnrate of the vehicle in front of you")
function get( ::Feature_TURNRATE_FRONT, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	get(INDFRONT, basics, carind, validfind)
	basics[(carind, validfind, :turnrate_front)]
end

create_feature_basics( "A_REQ_FRONT", "m/s2", false, false, 0.0, -Inf, true, :a_req_front, L"a^{req}_{x,fo}", "const acceleration required to prevent collision with car in front assuming constant velocity")
function get( ::Feature_A_REQ_FRONT, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)

	ind_front = get(INDFRONT, basics, carind, validfind)
	if ind_front == NA_ALIAS
		return NA_ALIAS
	end

	dx = get(D_X_FRONT, basics, carind, validfind) # distance between cars
	dv = get(V_X_FRONT, basics, carind, validfind) # v_front - v_back

	if dv >= 0.0 # they are pulling away; we are good
		return NA_ALIAS
	end

	-min(dv*dv / (2dx), THRESHOLD_A_REQ)
end

create_feature_basics_boolean( "Gaining_On_Front", true, :gaining_on_front, L"1\{v_\text{ego} > v_\text{front}\}", "whether the car will collide with front if no action is taken and both const. vel")
function get( ::Feature_Gaining_On_Front, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	get(INDFRONT, basics, carind, validfind)
	ΔV = basics[(carind, validfind, :v_x_front)]
	float64(ΔV < 0.0)
end

create_feature_basics( "TTC_X_FRONT", "s", false, false, Inf, 0.0, true, :ttc_x_front, L"ttc_{x,fo}", "time to collision with car in front assuming constant velocities")
function get( ::Feature_TTC_X_FRONT, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)

	ind_front = get(INDFRONT, basics, carind, validfind)
	if ind_front == NA_ALIAS
		return NA_ALIAS
	end

	dx = get(D_X_FRONT, basics, carind, validfind) # distance between cars
	dv = get(V_X_FRONT, basics, carind, validfind) # v_front - v_back

	if dv >= 0.0 # they are pulling away; we are good
		return NA_ALIAS
	end

	min(-dx / dv, THRESHOLD_TIME_TO_COLLISION)
end

create_feature_basics( "Timegap_X_FRONT", "s", false, false, Inf, 0.0, false, :timegap_x_front, L"\tau_{x,fo}", "timegap between cars")
function get( ::Feature_Timegap_X_FRONT, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	ind_front = get(INDFRONT, basics, carind, validfind)
	if ind_front == NA_ALIAS
		return Features.THRESHOLD_TIMEGAP
	end

	dx = get(D_X_FRONT, basics, carind, validfind) # distance between cars
	v  = get(basics.pdset, :velFx, carind, validfind) # our current velocity

	if v <= 0.0
		return Features.THRESHOLD_TIMEGAP
	end

	min(dx / v, THRESHOLD_TIMEGAP)
end

# ----------------------------------
# REAR

create_feature_basics( "IndRear", "-", true, false, Inf, -2.0, true, :ind_rear, L"i_{rear}", "index of the closest car behind")
function _get(::Feature_IndRear, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)

	best_carind = -2
	best_dist   = 100.0 # start at max
	best_ΔpFy   = 0.0

	sn = basics.sn
	pdset = basics.pdset

	frameind = validfind2frameind(pdset, validfind)

	ncarsinframe = get_num_other_cars_in_frame(pdset, validfind)
	cars_to_check = Set([-1 : (ncarsinframe-1)])

	lanetags = Array(LaneTag, ncarsinframe+1)
	for cind in cars_to_check
		lanetags[cind+2] = get(pdset, :lanetag, cind, frameind, validfind)::LaneTag
	end

	posFx = get(pdset, :posFx, carind, frameind, validfind)::Float64
	posFy = get(pdset, :posFy, carind, frameind, validfind)::Float64
	active_lanetag = lanetags[carind+2]
	active_lane = get_lane(sn, active_lanetag)
	search_dist = 0.0

	delete!(cars_to_check, carind)

	finished = false
	while !finished
		to_remove = Set{Int}()
		for target_carind in cars_to_check
			if active_lanetag == lanetags[target_carind+2]
				target_posFx = get(pdset, :posFx, target_carind, frameind, validfind)

				target_dist = posFx - target_posFx + search_dist

				if 0.0 < target_dist < best_dist
					target_posFy = get(pdset, :posFy, target_carind, frameind, validfind)
					ΔpFy = target_posFy - posFy
					if abs(ΔpFy) < THRESHOLD_DY_CONSIDERED_IN_FRONT
						best_carind, best_dist = target_carind, target_dist
						best_ΔpFy = ΔpFy
					end
				end

				push!(to_remove, target_carind)
			end
		end
		if best_carind != -2 || search_dist > best_dist
			break
		end

		for target_carind in to_remove
			delete!(cars_to_check, target_carind)
		end
		if isempty(cars_to_check)
			break
		end

		if has_prev_lane(sn, active_lane)
			active_lane = prev_lane(sn, active_lane)
			active_lanetag = active_lane.id
			search_dist += active_lane.curve.s[end]
		else
			finished = true
		end
	end

	if best_carind != -2
		basics[(carind, validfind, :d_x_rear)] = best_dist
		basics[(carind, validfind, :d_y_rear)] = best_ΔpFy
		basics[(carind, validfind, :v_x_rear)] = get(pdset, :velFx,   best_carind, frameind, validfind) - get(pdset, :velFx, carind, frameind, validfind)
		basics[(carind, validfind, :v_y_rear)] = get(pdset, :velFy,   best_carind, frameind, validfind) - get(pdset, :velFy, carind, frameind, validfind)
		basics[(carind, validfind, :yaw_rear)] = get(pdset, :posFyaw, best_carind, frameind, validfind)
		basics[(carind, validfind, :turnrate_rear)] = get(TURNRATE, basics, best_carind, validfind)
		return float64(best_carind)
	else
		basics[(carind, validfind, :d_x_rear)] = NA_ALIAS
		basics[(carind, validfind, :d_y_rear)] = NA_ALIAS
		basics[(carind, validfind, :v_x_rear)] = NA_ALIAS
		basics[(carind, validfind, :v_y_rear)] = NA_ALIAS
		basics[(carind, validfind, :yaw_rear)] = NA_ALIAS
		basics[(carind, validfind, :turnrate_rear)] = NA_ALIAS
		return NA_ALIAS
	end
end

create_feature_basics_boolean( "HAS_REAR", false, :has_rear, L"\exists_{re}", "whether there is a car behind")
get( ::Feature_HAS_REAR, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int) = float64(get(INDREAR, basics, carind, validfind) != NA_ALIAS)

create_feature_basics( "D_X_REAR", "m", false, false, Inf, 0.0, true, :d_x_rear, L"d_{x,re}", "longitudinal distance to the closest vehicle in the same lane in rear")
function get( ::Feature_D_X_REAR, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	get(INDREAR, basics, carind, validfind)
	basics[(carind, validfind, :d_x_rear)]
end

create_feature_basics( "D_Y_REAR", "m", false, false, Inf, -Inf, true, :d_y_rear, L"d_{y,re}", "lateral distance to the closest vehicle in the same lane in rear")
function get( ::Feature_D_Y_REAR, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	get(INDREAR, basics, carind, validfind)
	basics[(carind, validfind, :d_y_rear)]
end

create_feature_basics( "V_X_REAR", "m/s", false, false, Inf, -Inf, true, :v_x_rear, L"v^{rel}_{x,re}", "relative x velocity of the vehicle behind you")
function get( ::Feature_V_X_REAR, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	get(INDREAR, basics, carind, validfind)
	basics[(carind, validfind, :v_x_rear)]
end

create_feature_basics( "V_Y_REAR", "m/s", false, false, Inf, -Inf, true, :v_y_rear, L"v^{rel}_{y,re}", "relative y velocity of the vehicle behind you")
function get( ::Feature_V_Y_REAR, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	get(INDREAR, basics, carind, validfind)
	basics[(carind, validfind, :v_y_rear)]
end

create_feature_basics( "YAW_REAR", "rad", false, false, float64(pi), float64(-pi), true, :yaw_rear, L"\psi^{rel}_{re}", "yaw of the vehicle behind you")
function get( ::Feature_YAW_REAR, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	get(INDREAR, basics, carind, validfind)
	basics[(carind, validfind, :yaw_rear)]
end

create_feature_basics( "TURNRATE_REAR", "rad/s", false, false, Inf, -Inf, true, :turnrate_rear, L"\dot{\psi}^{rel}_{re}", "turnrate of the vehicle behind you")
function get( ::Feature_TURNRATE_REAR, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	get(INDREAR, basics, carind, validfind)
	basics[(carind, validfind, :turnrate_rear)]
end

create_feature_basics( "A_REQ_REAR", "m/s2", false, false, Inf, 0.0, true, :a_req_rear, L"a^{req}_{x,re}", "const acceleration required to prevent collision with car behind assuming constant velocity")
function get( ::Feature_A_REQ_REAR, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)

	ind_rear = get(INDREAR, basics, carind, validfind)
	if ind_rear == NA_ALIAS
		return NA_ALIAS
	end

	dx = get(D_X_REAR, basics, carind, validfind) # distance between cars
	dv = get(V_X_REAR, basics, carind, validfind) # v_front - v_back

	if dv <= 0.0
		return NA_ALIAS
	end

	min(dv*dv / (2dx), THRESHOLD_A_REQ)
end

create_feature_basics( "TTC_X_REAR", "s", false, false, Inf, 0.0, true, :ttc_x_rear, L"ttc_{x,re}", "time to collision with rear car assuming constant velocities")
function get( ::Feature_TTC_X_REAR, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)

	ind_rear = get(INDREAR, basics, carind, validfind)
	if ind_rear == NA_ALIAS
		return NA_ALIAS
	end

	dx = get(D_X_REAR, basics, carind, validfind) # distance between cars
	dv = get(V_X_REAR, basics, carind, validfind) # v_them - v_us

	if dv <= 0.0
		return NA_ALIAS
	end

	min(dx / dv, THRESHOLD_TIME_TO_COLLISION)
end

create_feature_basics( "Timegap_X_REAR", "s", false, false, Inf, 0.0, true, :timegap_x_rear, L"\tau_{x,re}", "timegap with rear car")
function get( ::Feature_Timegap_X_REAR, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)

	ind_rear = get(INDREAR, basics, carind, validfind)
	if ind_rear == NA_ALIAS
		return THRESHOLD_TIMEGAP
	end

	dx = get(D_X_REAR, basics, carind, validfind) # distance between cars
	v  = get(VELFX,    basics, carind, validfind)

	if v <= 0.0
		return THRESHOLD_TIMEGAP
	end

	min(dx / v, THRESHOLD_TIMEGAP)
end

# ----------------------------------
# LEFT

create_feature_basics( "IndLeft", "-", true, false, Inf, -2.0, true, :ind_left, L"i_{\text{left}}", "index of the closest car in the left-hand lane")
function _get(::Feature_IndLeft, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)

	# Finds the car that is closest in physical distance to the car in the left lane

	# TODO(tim): may still have an issue with computing distances across curve boundaries

	best_carind = -2
	best_dist   = 50.0
	best_ΔpFy   = NA_ALIAS

	pdset = basics.pdset
	sn = basics.sn

	frameind = validfind2frameind(pdset, validfind)

	if get(pdset, :nll, carind, frameind, validfind)::Int8 > 0

		ncarsinframe = get_num_other_cars_in_frame(pdset, validfind)
		cars_to_check = Set([-1 : (ncarsinframe-1)])

		lanetags = Array(LaneTag, ncarsinframe+1)
		for cind in cars_to_check
			lanetags[cind+2] = get(pdset, :lanetag, cind, frameind, validfind)::LaneTag
		end

		current_lanetag = lanetags[carind+2]
		current_lane    = get_lane(sn, current_lanetag)
		current_lane_next = has_next_lane(sn, current_lane) ? next_lane(sn, current_lane) : current_lane
		current_lane_prev = has_prev_lane(sn, current_lane) ? prev_lane(sn, current_lane) : current_lane

		posGx = get(pdset, :posGx,   carind, frameind, validfind)::Float64
		posGy = get(pdset, :posGy,   carind, frameind, validfind)::Float64
		posGθ = get(pdset, :posGyaw, carind, frameind, validfind)::Float64

		rayEx = posGx + cos(posGθ)
		rayEy = posGy + sin(posGθ)

		# project the current location to the tilemap, but accept only lanes to the left of current location & not the current lane
		function f_filter(curve_pt::CurvePt, lane::StreetLane)
			is_pt_left_of_ray(curve_pt.x, curve_pt.y, posGx, posGy, rayEx, rayEy) &&
				!(current_lane      === lane) &&
				!(current_lane_next === lane) &&
				!(current_lane_prev === lane)
		end
		proj = project_point_to_streetmap(posGx, posGy, sn, f_filter)

		if proj.successful

			posFx, posFy = pt_to_frenet_xy(proj.curvept, posGx, posGy)

			left_lane = proj.lane
			left_lanetag = left_lane.id

			delete!(cars_to_check, carind)

			pq = Collections.PriorityQueue()
			Collections.enqueue!(pq, (left_lane, left_lanetag, true, 0.0), 0.0)

			finished = false
			while !finished && !isempty(pq)
				to_remove = Set{Int}()

				active_lane, active_lanetag, is_forward, search_dist = Collections.dequeue!(pq)

				# if ( is_forward && search_dist > abs(best_dist)) ||
				#    (!is_forward && search_dist > (abs(best_dist) + active_lane.curve.s[end]))
				#    break
				# end

				for target_carind in cars_to_check
					if active_lanetag == lanetags[target_carind+2]
						target_posFx = get(pdset, :posFx, target_carind, frameind, validfind)

						target_dist = is_forward ? target_posFx - posFx + search_dist :
						                           posFx - target_posFx + search_dist

						if abs(target_dist) < abs(best_dist)
							best_carind, best_dist = target_carind, target_dist
							target_posFy = get(pdset, :posFy, target_carind, frameind, validfind)
							best_ΔpFy = target_posFy - posFy
						end

						push!(to_remove, target_carind)
					end
				end

				for target_carind in to_remove
					delete!(cars_to_check, target_carind)
				end
				if isempty(cars_to_check)
					break
				end

				if is_forward && has_next_lane(sn, active_lane)
					next_search_dist = search_dist + active_lane.curve.s[end]
					next_active_lane = next_lane(sn, active_lane)
					Collections.enqueue!(pq, (next_active_lane, next_active_lane.id, true, next_search_dist), next_search_dist)
				end
				if (!is_forward || isapprox(search_dist, 0.0)) && has_prev_lane(sn, active_lane)
					prev_active_lane = prev_lane(sn, active_lane)
					prev_search_dist = search_dist + prev_active_lane.curve.s[end]
					Collections.enqueue!(pq, (prev_active_lane, prev_active_lane.id, false, prev_search_dist), prev_search_dist)
				end
			end
		end
	end

	if best_carind != -2
		basics[(carind, validfind, :d_x_left)] = best_dist # NOTE(tim): + if other car in front, - if other car behind
		basics[(carind, validfind, :d_y_left)] = best_ΔpFy
		basics[(carind, validfind, :v_x_left)] = get(pdset, :velFx,   best_carind, frameind, validfind) - get(pdset, :velFx, carind, frameind, validfind)
		basics[(carind, validfind, :v_y_left)] = get(pdset, :velFy,   best_carind, frameind, validfind) - get(pdset, :velFy, carind, frameind, validfind)
		basics[(carind, validfind, :yaw_left)] = get(pdset, :posFyaw, best_carind, frameind, validfind)
		basics[(carind, validfind, :turnrate_left)] = get(TURNRATE, basics, best_carind, validfind)
		return float64(best_carind)
	else
		basics[(carind, validfind, :d_x_left)] = NA_ALIAS
		basics[(carind, validfind, :d_y_left)] = NA_ALIAS
		basics[(carind, validfind, :v_x_left)] = NA_ALIAS
		basics[(carind, validfind, :v_y_left)] = NA_ALIAS
		basics[(carind, validfind, :yaw_left)] = NA_ALIAS
		basics[(carind, validfind, :turnrate_left)] = NA_ALIAS
		return NA_ALIAS
	end
end

create_feature_basics( "D_X_LEFT", "m", false, false, Inf, -Inf, true, :d_x_left, L"d_{x,lf}", "longitudinal distance to the closest vehicle in the left lane")
function get( ::Feature_D_X_LEFT, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	get(INDLEFT, basics, carind, validfind)
	basics[(carind, validfind, :d_x_left)]
end

create_feature_basics( "D_Y_LEFT", "m", false, false, Inf, -Inf, true, :d_y_left, L"d_{y,lf}", "lateral distance to the closest vehicle in the left lane")
function get( ::Feature_D_Y_LEFT, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	get(INDLEFT, basics, carind, validfind)
	basics[(carind, validfind, :d_y_left)]
end

create_feature_basics( "V_X_LEFT", "m/s", false, false, Inf, -Inf, true, :v_x_left, L"v^{rel}_{x,lf}", "relative x velocity of the closest vehicle in left lane")
function get( ::Feature_V_X_LEFT, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	get(INDLEFT, basics, carind, validfind)
	basics[(carind, validfind, :v_x_left)]
end

create_feature_basics( "V_Y_LEFT", "m/s", false, false, Inf, -Inf, true, :v_y_left, L"v^{rel}_{y,le}", "relative y velocity of the closest vehicle in left lane")
function get( ::Feature_V_Y_LEFT, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	get(INDLEFT, basics, carind, validfind)
	basics[(carind, validfind, :v_y_left)]
end

create_feature_basics( "YAW_LEFT", "rad", false, false, float64(pi), float64(-pi), true, :yaw_left, L"\psi_{le}", "yaw of the closest vehicle in left lane")
function get( ::Feature_YAW_LEFT, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	get(INDLEFT, basics, carind, validfind)
	basics[(carind, validfind, :yaw_left)]
end

create_feature_basics( "TURNRATE_LEFT", "rad/s", false, false, Inf, -Inf, true, :turnrate_left, L"\dot{\psi}_{le}", "turnrate of the closest vehicle in left lane")
function get( ::Feature_TURNRATE_LEFT, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	get(INDLEFT, basics, carind, validfind)
	basics[(carind, validfind, :turnrate_left)]
end

create_feature_basics( "A_REQ_LEFT", "m/s2", false, false, 0.0, -Inf, true, :a_req_left, L"a^{req}_{x,le}", "const acceleration (+ to left) required to prevent collision with car to left assuming constant velocity")
function get( ::Feature_A_REQ_LEFT, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)

	ind_left = get(INDLEFT, basics, carind, validfind)
	if ind_left == NA_ALIAS
		return NA_ALIAS
	end

	dx = get(D_X_LEFT, basics, carind, validfind) # distance between cars
	dv = get(V_X_LEFT, basics, carind, validfind) # v_other - v_me

	if (dx > 0.0 && dv > 0.0) || (dx < 0.0 && dv < 0.0)
		return NA_ALIAS
	end

	if dx > 0.0
		-min(dv*dv / (2*dx), THRESHOLD_A_REQ)
	else
		min(dv*dv / (2*abs(dx)), THRESHOLD_A_REQ)
	end
end

create_feature_basics( "TTC_X_LEFT", "s", false, false, Inf, 0.0, true, :ttc_x_left, L"ttc_{x,le}", "time to collision with left car assuming constant velocities")
function get( ::Feature_TTC_X_LEFT, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)

	ind_left = get(INDLEFT, basics, carind, validfind)
	if ind_left == NA_ALIAS
		return NA_ALIAS
	end

	dx = get(D_X_LEFT, basics, carind, validfind) # distance between cars
	dv = get(V_X_LEFT, basics, carind, validfind) # v_other - v_me

	if (dx > 0.0 && dv > 0.0) || (dx < 0.0 && dv < 0.0)
		return NA_ALIAS
	end

	min(abs(dx / dv), THRESHOLD_TIME_TO_COLLISION)
end

create_feature_basics( "Timegap_X_LEFT", "s", false, false, Inf, 0.0, true, :timegap_x_left, L"\tau_{x,le}", "timegap with left car")
function get( ::Feature_Timegap_X_LEFT, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)

	ind_left = get(INDLEFT, basics, carind, validfind)
	if ind_left == NA_ALIAS
		return THRESHOLD_TIMEGAP
	end

	dx = get(D_X_LEFT, basics, carind, validfind) # distance between cars
	v  = get(VELFX,    basics, carind, validfind)

	if (dx > 0.0 && v > 0.0) || (dx < 0.0 && v < 0.0)
		return THRESHOLD_TIMEGAP
	end

	min(abs(dx / v), THRESHOLD_TIMEGAP)
end

# ----------------------------------
# RIGHT

create_feature_basics( "IndRight", "-", true, false, Inf, -2.0, true, :ind_right, L"i_{right}", "index of the closest car in the right-hand lane")
function _get(::Feature_IndRight, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)

	# TODO(tim): may still have an issue with computing distances across curve boundaries

	best_carind = -2
	best_dist   = 50.0
	best_ΔpFy   = NA_ALIAS

	sn = basics.sn
	pdset = basics.pdset

	frameind = validfind2frameind(pdset, validfind)

	if get(pdset, :nlr, carind, frameind, validfind)::Int8 > 0

		ncarsinframe = get_num_other_cars_in_frame(pdset, validfind)
		cars_to_check = Set([-1 : (ncarsinframe-1)])

		lanetags = Array(LaneTag, ncarsinframe+1)
		for cind in cars_to_check
			lanetags[cind+2] = get(pdset, :lanetag, cind, frameind, validfind)::LaneTag
		end

		current_lanetag = lanetags[carind+2]
		current_lane    = get_lane(sn, current_lanetag)
		current_lane_next = has_next_lane(sn, current_lane) ? next_lane(sn, current_lane) : current_lane
		current_lane_prev = has_prev_lane(sn, current_lane) ? prev_lane(sn, current_lane) : current_lane

		posGx = get(pdset, :posGx,   carind, frameind, validfind)::Float64
		posGy = get(pdset, :posGy,   carind, frameind, validfind)::Float64
		posGθ = get(pdset, :posGyaw, carind, frameind, validfind)::Float64

		rayEx = posGx + cos(posGθ)
		rayEy = posGy + sin(posGθ)

		function f_filter(curve_pt::CurvePt, lane::StreetLane)
			!is_pt_left_of_ray(curve_pt.x, curve_pt.y, posGx, posGy, rayEx, rayEy) &&
				!(current_lane      === lane) &&
				!(current_lane_next === lane) &&
				!(current_lane_prev === lane)
		end
		proj = project_point_to_streetmap(posGx, posGy, sn, f_filter)

		if proj.successful

			posFx, posFy = pt_to_frenet_xy(proj.curvept, posGx, posGy)


			right_lane = proj.lane
			right_lanetag = right_lane.id

			delete!(cars_to_check, carind)

			pq = Collections.PriorityQueue()
			Collections.enqueue!(pq, (right_lane, right_lanetag, true, 0.0), 0.0)

			finished = false
			while !finished && !isempty(pq)
				to_remove = Set{Int}()

				active_lane, active_lanetag, is_forward, search_dist = Collections.dequeue!(pq)

				for target_carind in cars_to_check
					if active_lanetag == lanetags[target_carind+2]
						target_posFx = get(pdset, :posFx, target_carind, frameind, validfind)

						target_dist = is_forward ? target_posFx - posFx + search_dist :
						                           posFx - target_posFx + search_dist

						if abs(target_dist) < abs(best_dist)
							best_carind, best_dist = target_carind, target_dist
							target_posFy = get(pdset, :posFy, target_carind, frameind, validfind)
							best_ΔpFy = target_posFy - posFy
						end

						push!(to_remove, target_carind)
					end
				end

				for target_carind in to_remove
					delete!(cars_to_check, target_carind)
				end
				if isempty(cars_to_check)
					break
				end

				if is_forward && has_next_lane(sn, active_lane)
					next_search_dist = search_dist + active_lane.curve.s[end]
					next_active_lane = next_lane(sn, active_lane)
					Collections.enqueue!(pq, (next_active_lane, next_active_lane.id, true, next_search_dist), next_search_dist)
				end
				if (!is_forward || isapprox(search_dist, 0.0)) && has_prev_lane(sn, active_lane)
					prev_active_lane = prev_lane(sn, active_lane)
					prev_search_dist = search_dist + prev_active_lane.curve.s[end]
					Collections.enqueue!(pq, (prev_active_lane, prev_active_lane.id, false, prev_search_dist), prev_search_dist)
				end
			end
		end
	end

	if best_carind != -2
		basics[(carind, validfind, :d_x_right)] = best_dist # NOTE(tim): + if other car in front, - if other car behind
		basics[(carind, validfind, :d_y_right)] = best_ΔpFy
		basics[(carind, validfind, :v_x_right)] = get(pdset, :velFx,   best_carind, frameind, validfind) - get(pdset, :velFx, carind, frameind, validfind)
		basics[(carind, validfind, :v_y_right)] = get(pdset, :velFy,   best_carind, frameind, validfind) - get(pdset, :velFy, carind, frameind, validfind)
		basics[(carind, validfind, :yaw_right)] = get(pdset, :posFyaw, best_carind, frameind, validfind)
		basics[(carind, validfind, :turnrate_right)] = get(TURNRATE, basics, best_carind, validfind)
		return float64(best_carind)
	else
		basics[(carind, validfind, :d_x_right)] = NA_ALIAS
		basics[(carind, validfind, :d_y_right)] = NA_ALIAS
		basics[(carind, validfind, :v_x_right)] = NA_ALIAS
		basics[(carind, validfind, :v_y_right)] = NA_ALIAS
		basics[(carind, validfind, :yaw_right)] = NA_ALIAS
		basics[(carind, validfind, :turnrate_right)] = NA_ALIAS
		return NA_ALIAS
	end
end

create_feature_basics( "D_X_RIGHT", "m", false, false, Inf, -Inf, true, :d_x_right, L"d_{x,ri}", "longitudinal distance to the closest vehicle in the right lane")
function get( ::Feature_D_X_RIGHT, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	get(INDRIGHT, basics, carind, validfind)
	basics[(carind, validfind, :d_x_right)]
end

create_feature_basics( "D_Y_RIGHT", "m", false, false, Inf, -Inf, true, :d_y_right, L"d_{y,ri}", "lateral distance to the closest vehicle in the right lane")
function get( ::Feature_D_Y_RIGHT, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	get(INDRIGHT, basics, carind, validfind)
	basics[(carind, validfind, :d_y_right)]
end

create_feature_basics( "V_X_RIGHT", "m/s", false, false, Inf, -Inf, true, :v_x_right, L"v^{rel}_{x,ri}", "relative x velocity of the closest vehicle in right lane")
function get( ::Feature_V_X_RIGHT, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	get(INDRIGHT, basics, carind, validfind)
	basics[(carind, validfind, :v_x_right)]
end

create_feature_basics( "V_Y_RIGHT", "m/s", false, false, Inf, -Inf, true, :v_y_right, L"v^{rel}_{y,ri}", "relative y velocity of the closest vehicle in right lane")
function get( ::Feature_V_Y_RIGHT, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	get(INDRIGHT, basics, carind, validfind)
	basics[(carind, validfind, :v_y_right)]
end

create_feature_basics( "YAW_RIGHT", "rad", false, false, float64(pi), float64(-pi), true, :yaw_right, L"\psi_{ri}", "yaw of the closest vehicle in right lane")
function get( ::Feature_YAW_RIGHT, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
    get(INDRIGHT, basics, carind, validfind)
	basics[(carind, validfind, :yaw_right)]
end

create_feature_basics( "TURNRATE_RIGHT", "rad", false, false, Inf, -Inf, true, :turnrate_right, L"\dot{\psi}_{ri}", "turnrate of the closest vehicle in right lane")
function get( ::Feature_TURNRATE_RIGHT, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	get(INDRIGHT, basics, carind, validfind)
	basics[(carind, validfind, :turnrate_right)]
end

create_feature_basics( "A_REQ_RIGHT", "m/s2", false, false, Inf, 0.0, true, :a_req_right, L"a^{req}_{x,ri}", "const acceleration (+ to left) required to prevent collision with car to right assuming constant velocity")
function get( ::Feature_A_REQ_RIGHT, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)

	ind_right = get(INDRIGHT, basics, carind, validfind)
	if ind_right == NA_ALIAS
		return NA_ALIAS
	end

	dx = get(D_X_RIGHT, basics, carind, validfind) # distance between cars
	dv = get(V_X_RIGHT, basics, carind, validfind) # v_other - v_me

	if (dx > 0.0 && dv > 0.0) || (dx < 0.0 && dv < 0.0)
		return NA_ALIAS
	end

	if dx > 0.0
		-min(dv*dv / (2*dx), THRESHOLD_A_REQ)
	else
		min(dv*dv / (2*abs(dx)), THRESHOLD_A_REQ)
	end
end

create_feature_basics( "TTC_X_RIGHT", "s", false, false, Inf, 0.0, true, :ttc_x_right, L"ttc_{x,ri}", "time to collision with right car assuming constant velocities")
function get( ::Feature_TTC_X_RIGHT, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)

	ind_right = get(INDRIGHT, basics, carind, validfind)
	if ind_right == NA_ALIAS
		return NA_ALIAS
	end

	dx = get(D_X_RIGHT, basics, carind, validfind) # distance between cars
	dv = get(V_X_RIGHT, basics, carind, validfind) # v_other - v_me

	if (dx > 0.0 && dv > 0.0) || (dx < 0.0 && dv < 0.0)
		return NA_ALIAS
	end

	min(abs(dx / dv), THRESHOLD_TIME_TO_COLLISION)
end

create_feature_basics( "Timegap_X_RIGHT", "s", false, false, Inf, 0.0, true, :timegap_x_right, L"\tau_{x,ri}", "timegap with right car")
function get( ::Feature_Timegap_X_RIGHT, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	ind_right = get(INDRIGHT, basics, carind, validfind)
	if ind_right == NA_ALIAS
		return THRESHOLD_TIMEGAP
	end

	dx = get(D_X_RIGHT, basics, carind, validfind) # distance between cars
	v  = get(VELFX,     basics, carind, validfind)

	if (dx > 0.0 && v > 0.0) || (dx < 0.0 && v < 0.0)
		return THRESHOLD_TIMEGAP
	end

	min(abs(dx / v), THRESHOLD_TIMEGAP)
end

# ----------------------------------

create_feature_basics( "SceneVelFx", "m", false, false, Inf, -Inf, false, :scene_velFx, L"\|v\|_{scene}", "average velocity along the lane across all cars in the scene")
function _get( ::Feature_SceneVelFx, basics::FeatureExtractBasicsPdSet, ::Int, validfind::Int)
	total = 1
	val = gete(basics.pdset, :velFx, validfind2frameind(basics.pdset, validfind))
	for carind = 0 : get_num_other_cars_in_frame(basics.pdset, validfind)-1
		val += getc(basics.pdset, :velFx, carind, validfind)
		total += 1
	end
	val / total
end

create_feature_basics( "TurnRate", "rad/s", false, false, Inf, -Inf, false, :turnrate, L"\dot{\psi}", "turn rate")
function _get(::Feature_TurnRate, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	# NOTE(tim): defaults to 0.0

	lookback = DEFAULT_FRAME_PER_SEC
	pdset = basics.pdset

	validfind_past = jumpframe(pdset, validfind, -lookback)
	if validfind_past == 0 # Does not exist
		return 0.0
	end

	if carind == CARIND_EGO
		frameind_curr = validfind2frameind(pdset, validfind)
		frameind_past = validfind2frameind(pdset, validfind_past)
		curr = gete(pdset, :posFyaw, frameind_curr)
		past = gete(pdset, :posFyaw, frameind_past)
		return deltaangle(curr, past) / (DEFAULT_SEC_PER_FRAME*lookback)
	end

	carid = carind2id(pdset, carind, validfind)
	if idinframe(pdset, carid, validfind_past)
		farind = carid2ind(pdset, carid, validfind_past)
		curr = getc(pdset, :posFyaw, carind, validfind)
		past = getc(pdset, :posFyaw, farind, validfind_past)
		return deltaangle(curr, past) / (DEFAULT_SEC_PER_FRAME*lookback)
	end
	0.0
end

create_feature_basics( "TurnRate_Global", "rad/s", false, false, Inf, -Inf, false, :turnrate_global, L"\dot{\psi}^G", "turn rate in the global frame")
function _get(::Feature_TurnRate_Global, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	# NOTE(tim): defaults to 0.0

	lookback = DEFAULT_FRAME_PER_SEC
	pdset = basics.pdset

	validfind_past = jumpframe(pdset, validfind, -lookback)
	if validfind_past == 0 # Does not exist
		return 0.0
	end

	if carind == CARIND_EGO
		frameind_curr = validfind2frameind(pdset, validfind)
		frameind_past = validfind2frameind(pdset, validfind_past)
		curr = gete(pdset, :posGyaw, frameind_curr)
		past = gete(pdset, :posGyaw, frameind_past)
		return deltaangle(curr, past) / (DEFAULT_SEC_PER_FRAME*lookback)
	end

	carid = carind2id(pdset, carind, validfind)
	if idinframe(pdset, carid, validfind_past)
		farind = carid2ind(pdset, carid, validfind_past)
		curr = getc(pdset, :posGyaw, carind, validfind)
		past = getc(pdset, :posGyaw, farind, validfind_past)
		return deltaangle(curr, past) / (DEFAULT_SEC_PER_FRAME*lookback)
	end
	0.0
end

create_feature_basics( "AccFx", "m/s2", false, false, Inf, -Inf, false, :accFx, L"a^F_x", "instantaneous acceleration along the lane")
function _get(::Feature_AccFx, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	# NOTE(tim): defaults to 0.0

	lookback = DEFAULT_FRAME_PER_SEC
	pdset = basics.pdset

	validfind_past = jumpframe(pdset, validfind, -lookback) # look back five frames (for quarter-second lookback)
	if validfind_past == 0 # Does not exist
		return 0.0
	end

	if carind == CARIND_EGO
		frameind_curr = validfind2frameind(pdset, validfind)
		frameind_past = validfind2frameind(pdset, validfind_past)
		curr = gete(pdset, :velFx, frameind_curr)
		past = gete(pdset, :velFx, frameind_past)
		return (curr - past) / (DEFAULT_SEC_PER_FRAME*lookback)
	end

	carid = carind2id(pdset, carind, validfind)
	if idinframe(pdset, carid, validfind_past)

		carind_past = carid2ind(pdset, carid, validfind_past)

		curr = getc(pdset, :velFx, carind,      validfind)
		past = getc(pdset, :velFx, carind_past, validfind_past)
		return (curr - past) / (DEFAULT_SEC_PER_FRAME*lookback)
	end

	0.0
end

create_feature_basics( "AccFy", "m/s2", false, false, Inf, -Inf, false, :accFy, L"a^F_y", "instantaneous acceleration perpendicular to the lane")
function _get(::Feature_AccFy, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	# NOTE(tim): defaults to 0.0

	lookback = DEFAULT_FRAME_PER_SEC
	pdset = basics.pdset

	validfind_past = jumpframe(pdset, validfind, -lookback)
	if validfind_past == 0 # Does not exist
		return 0.0
	end

	if carind == CARIND_EGO
		frameind_curr = validfind2frameind(pdset, validfind)
		frameind_past = validfind2frameind(pdset, validfind_past)
		curr = gete(pdset, :velFy, frameind_curr)
		past = gete(pdset, :velFy, frameind_past)
		return (curr - past) / (DEFAULT_SEC_PER_FRAME*lookback)
	end

	carid = carind2id(pdset, carind, validfind)
	if idinframe(pdset, carid, validfind_past)
		farind = carid2ind(pdset, carid, validfind_past)
		curr = getc(pdset, :velFy, carind, validfind)
		past = getc(pdset, :velFy, farind, validfind_past)
		return (curr - past) / (DEFAULT_SEC_PER_FRAME*lookback)
	end
	0.0
end

create_feature_basics( "Acc", "m/s2", false, false, Inf, -Inf, false, :acc, L"a", "instantaneous ego longitudinal acceleration")
function _get(::Feature_Acc, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	# NOTE(tim): defaults to 0.0

	pdset = basics.pdset
	lookback = DEFAULT_FRAME_PER_SEC

	validfind_past = int(jumpframe(pdset, validfind, -lookback))
	if validfind_past == 0 # Does not exist
		return 0.0
	end

	if carind == CARIND_EGO
		curr = get(SPEED, basics, carind, validfind)
		past = get(SPEED, basics, carind, validfind_past)
		return (curr - past) / (DEFAULT_SEC_PER_FRAME*lookback)
	end

	carid = carind2id(pdset, carind, validfind)
	if idinframe(pdset, carid, validfind_past)
		farind = int(carid2ind(pdset, carid, validfind_past))
		curr = get(SPEED, basics, carind, validfind)
		past = get(SPEED, basics, farind, validfind_past)
		return (curr - past) / (DEFAULT_SEC_PER_FRAME*lookback)
	end
	0.0
end

# ----------------------------------

create_feature_basics( "D_Merge", "m", false, false, Inf, 0.0, true, :d_merge, L"d_{merge}", "the distance along the lane until it merges")
function get(::Feature_D_Merge, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	d_merge = get(basics.pdset, :d_merge, carind, validfind)
	return isa(d_merge, NAtype) ? NA_ALIAS : d_merge
end

create_feature_basics( "D_Split", "m", false, false, Inf, 0.0, true, :d_split, L"d_{split}", "the distance along the lane until it splits")
function get(::Feature_D_Split, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	d_merge = get(basics.pdset, :d_split, carind, validfind)
	return isa(d_merge, NAtype) ? NA_ALIAS : d_merge
end

# ----------------------------------
# Future

create_feature_basics( "FutureTurnRate_250ms", "rad/s", false, false, Inf, -Inf, true, :f_turnrate_250ms, L"\dot{\psi}^{\text{fut}}_{250ms}", "the average rate of heading change over the next quarter second")
function _get(::Feature_FutureTurnRate_250ms, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)

	const lookahead = 5
	timestep = lookahead * DEFAULT_SEC_PER_FRAME

	pdset = basics.pdset

	futrvfind = jumpframe(pdset, validfind, lookahead)
	if futrvfind == 0 # Does not exist
		return NA_ALIAS
	end

	if carind == CARIND_EGO
		frameind = validfind2frameind(pdset, validfind)
		cur = gete(pdset, :posFyaw, frameind)::Float64
		fut = gete(pdset, :posFyaw, frameind+lookahead)::Float64
		return deltaangle(fut, cur) / timestep
	end

	carid = carind2id(pdset, carind, validfind)
	if idinframe(pdset, carid, futrvfind)
		farind = carid2ind(pdset, carid, futrvfind)
		cur = getc(pdset, :posFyaw, carind, validfind)::Float64
		fut = getc(pdset, :posFyaw, farind, futrvfind)::Float64
		return deltaangle(fut, cur) / timestep
	end
	NA_ALIAS
end

create_feature_basics( "FutureTurnRate_500ms", "rad/s", false, false, Inf, -Inf, true, :f_turnrate_500ms, L"\dot{\psi}^{\text{fut}}_{500ms}", "the average rate of heading change over the next half second")
function _get(::Feature_FutureTurnRate_500ms, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)

	const lookahead = 10
	timestep = lookahead * DEFAULT_SEC_PER_FRAME

	pdset = basics.pdset

	futrvfind = jumpframe(pdset, validfind, lookahead)
	if futrvfind == 0 # Does not exist
		return NA_ALIAS
	end

	if carind == CARIND_EGO
		frameind = validfind2frameind(pdset, validfind)
		cur = gete(pdset, :posFyaw, frameind)::Float64
		fut = gete(pdset, :posFyaw, frameind+lookahead)::Float64
		return deltaangle(fut, cur) / timestep
	end

	carid = carind2id(pdset, carind, validfind)
	if idinframe(pdset, carid, futrvfind)
		farind = carid2ind(pdset, carid, futrvfind)
		cur = getc(pdset, :posFyaw, carind, validfind)::Float64
		fut = getc(pdset, :posFyaw, farind, futrvfind)::Float64
		return deltaangle(fut, cur) / timestep
	end
	NA_ALIAS
end

create_feature_basics( "FutureAcceleration_250ms", "m/s2", false, false, Inf, -Inf, true, :f_accel_250ms, L"a^{\text{fut}}_{250ms}", "the average rate of speed change over the next quarter second")
function _get(::Feature_FutureAcceleration_250ms, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	const lookahead = 5

	pdset = basics.pdset

	futrvfind = jumpframe(pdset, validfind, lookahead)
	if futrvfind == 0 # Does not exist
		return NA_ALIAS
	end

	timestep  = lookahead * DEFAULT_SEC_PER_FRAME
	if carind == CARIND_EGO
		frameind = validfind2frameind(pdset, validfind)
		curx = gete(pdset, :velFx, frameind)::Float64
		cury = gete(pdset, :velFy, frameind)::Float64
		cur  = hypot(curx, cury)
		futx = gete(pdset, :velFx, frameind + lookahead)::Float64
		futy = gete(pdset, :velFy, frameind + lookahead)::Float64
		fut  = hypot(futx, futy)
		return (fut - cur) / timestep
	end

	carid = carind2id(pdset, carind, validfind)
	if idinframe(pdset, carid, futrvfind)
		farind = carid2ind(pdset, carid, futrvfind)
		curx = getc(pdset, :velFx, carind, validfind)::Float64
		cury = getc(pdset, :velFy, carind, validfind)::Float64
		cur  = hypot(curx, cury)
		futx = getc(pdset, :velFx, farind, futrvfind)::Float64
		futy = getc(pdset, :velFy, farind, futrvfind)::Float64
		fut  = hypot(futx, futy)
		return (fut - cur) / timestep
	end
	NA_ALIAS
end

create_feature_basics( "FutureAcceleration_500ms", "m/s2", false, false, Inf, -Inf, true, :f_accel_500ms, L"a^{\text{fut}}_{500ms}", "the average rate of speed change over the next half second")
function _get(::Feature_FutureAcceleration_500ms, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)

	const lookahead = 10

	pdset = basics.pdset

	futrvfind = jumpframe(pdset, validfind, lookahead)
	if futrvfind == 0 # Does not exist
		return NA_ALIAS
	end

	timestep  = lookahead * DEFAULT_SEC_PER_FRAME
	if carind == CARIND_EGO
		frameind = validfind2frameind(pdset, validfind)
		curx = gete(pdset, :velFx, frameind)::Float64
		cury = gete(pdset, :velFy, frameind)::Float64
		cur  = hypot(curx, cury)
		futx = gete(pdset, :velFx, frameind + lookahead)::Float64
		futy = gete(pdset, :velFy, frameind + lookahead)::Float64
		fut  = hypot(futx, futy)
		return (fut - cur) / timestep
	end

	carid = carind2id(pdset, carind, validfind)
	if idinframe(pdset, carid, futrvfind)
		farind = carid2ind(pdset, carid, futrvfind)
		curx = getc(pdset, :velFx, carind, validfind)::Float64
		cury = getc(pdset, :velFy, carind, validfind)::Float64
		cur  = hypot(curx, cury)
		futx = getc(pdset, :velFx, farind, futrvfind)::Float64
		futy = getc(pdset, :velFy, farind, futrvfind)::Float64
		fut  = hypot(futx, futy)
		return (fut - cur) / timestep
	end
	NA_ALIAS
end

create_feature_basics( "FutureDesiredAngle_250ms", "rad", false, false, float64(π), -float64(π), true, :f_des_angle_250ms, L"\phi^{\text{des}}_{250ms}", "the inferred desired heading angle over the next quarter second")
function _get(::Feature_FutureDesiredAngle_250ms, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)

	const lookahead = 5

	pdset = basics.pdset

	futrvfind = jumpframe(pdset, validfind, lookahead)
	if futrvfind == 0 # Does not exist
		return NA_ALIAS
	end

	curϕ = futϕ = 0.0
	if carind == CARIND_EGO
		frameind = validfind2frameind(pdset, validfind)
		curϕ = gete(pdset, :posFyaw, frameind)::Float64
		futϕ = gete(pdset, :posFyaw, frameind + lookahead)::Float64
	else
		carid = carind2id(pdset, carind, validfind)
		if idinframe(pdset, carid, futrvfind)
			farind = carid2ind(pdset, carid, futrvfind)
			curϕ = getc(pdset, :posFyaw, carind, validfind)::Float64
			futϕ = getc(pdset, :posFyaw, farind, futrvfind)::Float64
		else
			return NA_ALIAS
		end
	end

	T  = lookahead * DEFAULT_SEC_PER_FRAME
	ex = exp(-KP_DESIRED_ANGLE*T)

	(futϕ - curϕ*ex) / (1.0 - ex)
end

create_feature_basics( "FutureDesiredAngle_500ms", "rad", false, false, float64(π), -float64(π), true, :f_des_angle_500ms, L"\phi^{\text{des}}_{500ms}", "the inferred desired heading angle over the next half second")
function _get(::Feature_FutureDesiredAngle_500ms, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)

	const lookahead = 10

	pdset = basics.pdset

	futrvfind = jumpframe(pdset, validfind, lookahead)
	if futrvfind == 0 # Does not exist
		return NA_ALIAS
	end

	curϕ = futϕ = 0.0
	if carind == CARIND_EGO
		frameind = validfind2frameind(pdset, validfind)
		curϕ = gete(pdset, :posFyaw, frameind)::Float64
		futϕ = gete(pdset, :posFyaw, frameind + lookahead)::Float64
	else
		carid = carind2id(pdset, carind, validfind)
		if idinframe(pdset, carid, futrvfind)
			farind = carid2ind(pdset, carid, futrvfind)
			curϕ = getc(pdset, :posFyaw, carind, validfind)::Float64
			futϕ = getc(pdset, :posFyaw, farind, futrvfind)::Float64
		else
			return NA_ALIAS
		end
	end

	T  = lookahead * DEFAULT_SEC_PER_FRAME
	ex = exp(-KP_DESIRED_ANGLE*T)

	(futϕ - curϕ*ex) / (1.0 - ex)
end

function _get_futuredesired_speed(basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int, lookahead::Int)

	pdset = basics.pdset

	futrvfind = int(jumpframe(pdset, validfind, lookahead))
	if futrvfind == 0 # Does not exist
		return NA_ALIAS
	end

	carid = carind2id(pdset, carind, validfind)
	carind_fut = carid2ind_or_negative_two_otherwise(pdset, carid, futrvfind)

	if carind_fut == -2
		return NA_ALIAS
	end

	curv = get(SPEED, basics, carind, validfind)::Float64
	futv = get(SPEED, basics, carind_fut, futrvfind)::Float64
	if isinf(futv) || isinf(curv)
		return NA_ALIAS
	end

	T = lookahead * DEFAULT_SEC_PER_FRAME
	ex = exp(-KP_DESIRED_SPEED*T)

	(futv - curv*ex) / (1.0 - ex) # - curv
end
create_feature_basics( "FutureDesiredSpeed_250ms", "m/s", false, false, Inf, -Inf, true, :f_des_speed_250ms, L"|v|^{\text{des}}_{250ms}", "the inferred desired speed over the next quarter second")
_get(::Feature_FutureDesiredSpeed_250ms, pdset::PrimaryDataset, ::StreetNetwork, carind::Int, validfind::Int) =
  _get_futuredesired_speed(pdset, carind, validfind,  5)
create_feature_basics( "FutureDesiredSpeed_500ms", "m/s", false, false, Inf, -Inf, true, :f_des_speed_500ms, L"|v|^{\text{des}}_{500ms}", "the inferred desired speed over the next half second")
_get(::Feature_FutureDesiredSpeed_500ms, pdset::PrimaryDataset, ::StreetNetwork, carind::Int, validfind::Int) =
	_get_futuredesired_speed(pdset, carind, validfind, 10)

function _get_futureaccel_control(pdset::PrimaryDataset, carind::Int, validfind::Int, lookahead::Int)

	futrvfind = int(jumpframe(pdset, validfind, lookahead))
	if futrvfind == 0 # Does not exist
		return NA_ALIAS
	end

	carid = carind2id(pdset, carind, validfind)
	carind_fut = carid2ind_or_negative_two_otherwise(pdset, carid, futrvfind)

	if carind_fut == -2
		return NA_ALIAS
	end

	curv = get(SPEED, pdset, carind, validfind)::Float64
	futv = get(SPEED, pdset, carind_fut, futrvfind)::Float64
	if isinf(futv) || isinf(curv)
		return NA_ALIAS
	end

	T = lookahead * DEFAULT_SEC_PER_FRAME

	a = (futv - curv) / T
	a / (SPEED_LIMIT - curv) # Kp = a / (v_des - v)
end
function _get_futureaccel_control(basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int, lookahead::Int)

	pdset = basics.pdset

	futrvfind = int(jumpframe(pdset, validfind, lookahead))
	if futrvfind == 0 # Does not exist
		return NA_ALIAS
	end

	carid = carind2id(pdset, carind, validfind)
	carind_fut = carid2ind_or_negative_two_otherwise(pdset, carid, futrvfind)

	if carind_fut == -2
		return NA_ALIAS
	end

	curv = get(SPEED, basics, carind, validfind)::Float64
	futv = get(SPEED, basics, carind_fut, futrvfind)::Float64
	if isinf(futv) || isinf(curv)
		return NA_ALIAS
	end

	T = lookahead * DEFAULT_SEC_PER_FRAME

	a = (futv - curv) / T
	a / (SPEED_LIMIT - curv) # Kp = a / (v_des - v)
end
create_feature_basics( "FutureAccelControl_250ms", "m/s", false, false, Inf, -Inf, true, :f_acc_control_250ms, L"K^a_{250ms}", "the inferred speed control constant over the next quarter second")
_get(::Feature_FutureAccelControl_250ms, pdset::PrimaryDataset, ::StreetNetwork, carind::Int, validfind::Int) =
	_get_futureaccel_control(pdset, carind, validfind,  5)
create_feature_basics( "FutureAccelControl_500ms", "m/s", false, false, Inf, -Inf, true, :f_acc_control_500ms, L"K^a_{500ms}", "the inferred speed control constant over the next half second")
_get(::Feature_FutureAccelControl_500ms, pdset::PrimaryDataset, ::StreetNetwork, carind::Int, validfind::Int) =
	_get_futureaccel_control(pdset, carind, validfind, 10)

create_feature_basics( "ID", "-", true, false, Inf, -1.0, false, :id, L"\text{id}", "the corresponding carid, -1 for the ego car")
get(::Feature_ID, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int) = float64(carind2id(basics.pdset, carind, validfind))

create_feature_basics( "LaneCurvature", "1/m", false, false, Inf, -Inf, false, :curvature, L"\kappa", "the local lane curvature")
get(::Feature_LaneCurvature, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int) = get(basics.pdset, :curvature, carind, validfind)::Float64

create_feature_basics( "TimeToLaneCrossing", "s", false, false, THRESHOLD_TIMETOLANECROSSING, 0.0, false, :timetolanecrossing, L"t_{\text{crossing}}^{+}", "the time until the next lane crossing")
function _get(::Feature_TimeToLaneCrossing, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	# scan forward until the car is no longer in the same lane

	pdset = basics.pdset
	sn = basics.sn

	frameind = validfind2frameind(pdset, validfind)
	t_orig = gete(pdset, :time, frameind)::Float64

	cur_lanetag = get(pdset, :lanetag, carind, frameind, validfind)::LaneTag
	cur_lane = get_lane(sn, cur_lanetag)
	cur_frameind = frameind

	finished = false
	while !finished
		cur_frameind += 1
		cur_validfind = frameind2validfind(pdset, cur_frameind)
		if cur_validfind != 0 && indinframe(pdset, carind, cur_validfind)
			Δt = gete(pdset, :time, cur_frameind) - t_orig
			if Δt > THRESHOLD_TIMETOLANECROSSING
				return THRESHOLD_TIMETOLANECROSSING
			end

			fut_lanetag = get(pdset, :lanetag, carind, cur_frameind, cur_validfind)::LaneTag
			if fut_lanetag != cur_lanetag
				if same_tile(cur_lanetag, fut_lanetag) || !has_next_lane(sn, cur_lane)
					return Δt
				else
					cur_lane = next_lane(sn, cur_lane)
					cur_lanetag = cur_lane.id
					if fut_lanetag != cur_lanetag
						return Δt
					end
				end
			end
		else
			return THRESHOLD_TIMETOLANECROSSING
		end
	end

	error("INVALID CODEPATH")
	return THRESHOLD_TIMETOLANECROSSING
end

create_feature_basics( "TimeSinceLaneCrossing", "s", false, false, THRESHOLD_TIMESINCELANECROSSING, 0.0, false, :timesincelanecrossing, L"t_{\text{crossing}}^{-}", "the time since the last lane crossing")
function _get(::Feature_TimeSinceLaneCrossing, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	# scan backward until the car is no longer in the same lane
	# NOTE(tim): returns positive time values

	pdset = basics.pdset
	sn = basics.sn

	frameind = validfind2frameind(pdset, validfind)
	t_orig = gete(pdset, :time, frameind)::Float64

	cur_lanetag = get(pdset, :lanetag, carind, frameind, validfind)::LaneTag
	cur_lane = get_lane(sn, cur_lanetag)
	cur_frameind = frameind

	finished = false
	while !finished
		cur_frameind -= 1
		cur_validfind = frameind2validfind(pdset, cur_frameind)
		if cur_validfind != 0 && indinframe(pdset, carind, cur_validfind)
			Δt = t_orig - gete(pdset, :time, cur_frameind)
			if Δt > THRESHOLD_TIMESINCELANECROSSING
				return THRESHOLD_TIMESINCELANECROSSING
			end
			past_lanetag = get(pdset, :lanetag, carind, cur_frameind, cur_validfind)::LaneTag
			if past_lanetag != cur_lanetag
				if same_tile(cur_lanetag, past_lanetag) || !has_prev_lane(sn, cur_lane)
					return Δt
				else
					cur_lane = prev_lane(sn, cur_lane)
					cur_lanetag = cur_lane.id
					if past_lanetag != cur_lanetag
						return Δt
					end
				end
			end
		else
			return THRESHOLD_TIMESINCELANECROSSING
		end
	end

	error("INVALID CODEPATH")
	return THRESHOLD_TIMESINCELANECROSSING
end

create_feature_basics( "Time_Consecutive_Brake", "s", false, false, THRESHOLD_TIMECONSECUTIVEACCEL, 0.0, false, :time_consecutive_brake, L"t_\text{brake}", "the consecutive time during which the car has been decelerating")
function _get(::Feature_Time_Consecutive_Brake, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	# scan backward until the car is no longer braking
	# NOTE(tim): returns positive time values

	const THRESHOLD_BRAKING = -0.05 # [m/s²]

	pdset = basics.pdset

	frameind = validfind2frameind(pdset, validfind)
	t_orig = gete(pdset, :time, frameind)::Float64

	past_validfind = int(jumpframe(pdset, validfind, -1))
	if past_validfind == 0
		basics[(carind, validfind, :time_consecutive_accel)] = 0.0
		return 0.0 # default
	end

	cur_accel = get(ACC, basics, carind, validfind)
	if cur_accel > THRESHOLD_BRAKING
		return 0.0
	end

	basics[(carind, validfind, :time_consecutive_accel)] = 0.0

	past_frameind = frameind
	finished = false
	while !finished
		past_frameind -= 1
		past_validfind = int(jumpframe(pdset, past_validfind, -1))
		if past_validfind != 0
			past_accel = _get(ACC, basics, carind, past_validfind)
			if past_accel > THRESHOLD_BRAKING
				return t_orig - gete(pdset, :time, past_frameind+1)
			end

			Δt = t_orig - gete(pdset, :time, past_frameind)
			if Δt > THRESHOLD_TIMECONSECUTIVEACCEL
				return THRESHOLD_TIMECONSECUTIVEACCEL
			end
		else
			return t_orig - gete(pdset, :time, past_frameind+1)
		end
	end

	error("INVALID CODEPATH")
	return 0.0
end

create_feature_basics( "Time_Consecutive_Accel", "s", false, false, THRESHOLD_TIMECONSECUTIVEACCEL, 0.0, false, :time_consecutive_accel, L"t_\text{accel}", "the consecutive time during which the car has been accelerating")
function _get(::Feature_Time_Consecutive_Accel, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
	# scan backward until the car is no longer braking
	# NOTE(tim): returns positive time values

	pdset = basics.pdset

	const THRESHOLD_ACCELERATION = 0.05 # [m/s²]

	frameind = validfind2frameind(pdset, validfind)
	t_orig = gete(pdset, :time, frameind)::Float64

	past_validfind = int(jumpframe(pdset, validfind, -1))
	if past_validfind == 0
		basics[(carind, validfind, :time_consecutive_brake)] = 0.0
		return 0.0 # default
	end

	cur_accel = get(ACC, basics, carind, validfind)
	if cur_accel < THRESHOLD_ACCELERATION
		return 0.0
	end

	basics[(carind, validfind, :time_consecutive_brake)] = 0.0

	past_frameind = frameind
	finished = false
	while !finished
		past_frameind -= 1
		past_validfind = int(jumpframe(pdset, past_validfind, -1))
		if past_validfind != 0

			past_accel = _get(ACC, basics, carind, past_validfind)
			if past_accel < THRESHOLD_ACCELERATION
				return t_orig - gete(pdset, :time, past_frameind+1)
			end

			Δt = t_orig - gete(pdset, :time, past_frameind)
			if Δt > THRESHOLD_TIMECONSECUTIVEACCEL
				return THRESHOLD_TIMECONSECUTIVEACCEL
			end
		else
			return t_orig - gete(pdset, :time, past_frameind+1)
		end
	end

	error("INVALID CODEPATH")
	return 0.0
end

create_feature_basics( "Time_Consecutive_Throttle", "s", false, false, THRESHOLD_TIMECONSECUTIVEACCEL, 0.0, false, :time_consecutive_throttle, L"t_\text{throttle}", "a union between time_consecutive_accel and time_consecutive_brake")
function _get(::Feature_Time_Consecutive_Throttle, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)

	# returns a positive value if t_consec_accel
	# returns a negative value if t_consec_brake

	t_consec_accel = get(TIME_CONSECUTIVE_ACCEL, basics, carind, validfind)
	if t_consec_accel > 0
		return t_consec_accel
	end
	-get(TIME_CONSECUTIVE_BRAKE, basics, carind, validfind)
end

# ----------------------------------
# submodels

create_feature_basics_boolean( "Subset_Emergency", false, :subset_emergency, L"\mathcal{D}_\text{emerg}", "subset of data for emergency behavior")
function _get(::Feature_Subset_Emergency, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int;
	threshold_acc :: Float64 = 2.0,
	threshold_turnrate :: Float64 = 0.05
	)

	# emergency
	# - events close to a collision
	# - can be found when the driver does a big accel?
	# - definition
	#   - |acc| > threshold
	#           or
	#   - |turnrate| > threshold

	a = get(ACC,      basics, carind, validfind)
	ω = get(TURNRATE, basics, carind, validfind)

	retval = abs(a) > threshold_acc ||
	         abs(ω) < threshold_turnrate

	float64(retval)
end

create_feature_basics_boolean( "Subset_Free_Flow", false, :subset_free_flow, L"\mathcal{D}_\text{free}", "subset of data for free flow")
function _get(::Feature_Subset_Free_Flow, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int;
	threshold_timegap_front :: Float64 = 3.0,
	threshold_d_v_front     :: Float64 = 0.5
	)

	ΔT = get(TIMEGAP_X_FRONT, basics, carind, validfind)
	dv = get(      V_X_FRONT, basics, carind, validfind)

	retval =     ΔT > threshold_timegap_front ||
	             dv > threshold_d_v_front

	float64(retval)
end

create_feature_basics_boolean( "Subset_Car_Following", false, :subset_car_following, L"\mathcal{D}_\text{follow}", "subset of data for car following")
function _get(::Feature_Subset_Car_Following, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int;
	threshold_timegap_front :: Float64 = 3.0,
	threshold_d_v_front     :: Float64 = 0.5
	)

	# following
	# - when there is a car in front of you
	# - you typically modulate your speed to match theirs and maintain a certain distance
	#   - distance is proportional to speed?
	#   - maintain a time gap?
	# - interesting:
	#   - what form does the timegap take?
	#   - how rigid is the tracking?
	#   - what sort of delays are there?
	# - definition
	#   - timegap_front < threshold
	#   - d_v_front < threshold

	ΔT = get(TIMEGAP_X_FRONT, basics, carind, validfind)
	dv = get(      V_X_FRONT, basics, carind, validfind)

	retval =    !(ΔT > threshold_timegap_front ||
	              dv > threshold_d_v_front)

	float64(retval)
end

create_feature_basics_boolean( "Subset_Lane_Crossing", false, :subset_lane_crossing, L"\mathcal{D}_\text{crossing}", "subset of data near lane crossing")
function _get(::Feature_Subset_Lane_Crossing, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int;
	max_time_to_lane_change::Float64 = 2.0, # [s]
	max_time_since_lane_change::Float64 = 2.0
	)

	# lane change
	# - when the driver changes lanes
	# - interesting:
	#   - what causes a driver to change lanes?
	# - definition
	#   - a lane crossing has occurred

	time_to_lane_change = get(TIMETOLANECROSSING, basics, carind, validfind)
	time_since_lane_change = get(TIMESINCELANECROSSING, basics, carind, validfind)

	retval = time_to_lane_change    < max_time_to_lane_change ||
	         time_since_lane_change < max_time_since_lane_change

	float64(retval)
end

create_feature_basics_boolean( "Subset_Sustained_Crossing", false, :subset_sustained_crossing, L"\mathcal{D}_\text{sustained}", "subset of data with a sustained lane crossing")
function _get(::Feature_Subset_Sustained_Crossing, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int;
	max_time_to_lane_change     :: Float64 = 2.0, # [s]
	max_time_since_lane_change  :: Float64 = 2.0, # [s]
	min_abs_d_cl_at_extrema     :: Float64 = 1.0, # [m]
	req_time_d_cl_threshold_met :: Float64 = 0.1, # [s] amount of time at extrema of segment that must meet d_cl requirements
	)

	had_lane_crossing = _get(SUBSET_LANE_CROSSING, basics, carind, validfind,
							max_time_to_lane_change = max_time_to_lane_change,
							max_time_since_lane_change = max_time_since_lane_change)

	if isapprox(had_lane_crossing, 0.0)
		return float64(false)
	end

	pdset = basics.pdset
	carid = carind2id(pdset, carind, validfind)
	frameind = validfind2frameind(pdset, validfind)
	t = gete(pdset, :time, frameind)::Float64

	time_to_lane_crossing    = get(TIMETOLANECROSSING,    basics, carind, validfind)
	time_since_lane_crossing = get(TIMESINCELANECROSSING, basics, carind, validfind)

	if time_to_lane_crossing < time_since_lane_crossing
		Δt = time_to_lane_crossing
	else
		Δt = -time_since_lane_crossing
	end

	the_closest_validfind = Trajdata.closest_validfind(pdset, t + Δt)
	@assert(the_closest_validfind != 0)
	closest_frameind = validfind2frameind(pdset, the_closest_validfind)
	@assert(closest_frameind != 0)

	frameind, vind = closest_frameind, the_closest_validfind
	dir  = convert(typeof(closest_frameind), sign(Δt))
	t    = gete(pdset, :time, closest_frameind)::Float64
	dt   = 0.0
	carind_fut = carid2ind_or_negative_two_otherwise(pdset, carid, vind)

	while dt < req_time_d_cl_threshold_met && carind_fut != -2
		d_cl = get(pdset, :d_cl, carind_fut, frameind, vind)::Float64
		if abs(d_cl) > min_abs_d_cl_at_extrema
			return float64(false)
		end

		frameind += dir
		vind  = frameind2validfind(pdset, frameind)
		while vind == 0
			frameind += dir
			vind  = frameind2validfind(pdset, frameind)
		end

		carind_fut = carid2ind_or_negative_two_otherwise(pdset, carid, vind)
		dt = abs(gete(pdset, :time, frameind)::Float64 - t)
	end

	float64(true)
end

create_feature_basics_boolean( "Subset_At_SixtyFive", false, :subset_at_sixtyfive, L"\mathcal{D}_{v=65\text{mph}}", "subset of data where the car is close to 65 mph")
function _get(::Feature_Subset_At_SixtyFive, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int;
	speed_tolerance :: Float64 = 2.235 # [m/s²]
	)

	V = get(SPEED, basics, carind, validfind)
	float64(abs(V - SPEED_LIMIT) < speed_tolerance)
end

create_feature_basics_boolean( "Subset_Auto", false, :subset_auto, L"s_\text{auto}", "subset of data where the car is in autonomous mode")
function _get(::Feature_Subset_Auto, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)

	if carind != CARIND_EGO
		return float64(false)
	end

	status = gete_validfind(basics.pdset, :control_status, validfind)::Int
	return float64(status == Trajdata.CONTROL_STATUS_AUTO)
end

immutable Feature_IsClean{target_symbol} <: AbstractFeature end
description( ::Feature_IsClean) = "Whether the given feature is nan or inf"
units(       ::Feature_IsClean) = "-"
isint(       ::Feature_IsClean) = true
isbool(      ::Feature_IsClean) = true
upperbound(  ::Feature_IsClean) = 1.0
lowerbound(  ::Feature_IsClean) = 0.0
couldna(     ::Feature_IsClean) = false
symbol(      ::Feature_IsClean) = symbol("isclean_" * string(target_symbol))
function get{F}(::Feature_IsClean{F}, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)
    f = symbol2feature(F)
    v = get(f, basics, carind, validfind)
    !isnan(v) && !isinf(v)
end

# ----------------------------------

function ticks_to_time_string( ticks::Int )
	n_secs = ticks//DEFAULT_FRAME_PER_SEC
	unit = "s"
	if den(n_secs) != 1
		unit = "ms"
		n_secs *= 1000
	end
	@assert(den(n_secs) == 1)
	(num(n_secs), unit)
end

function is_pt_left_of_ray(x::Float64, y::Float64, raySx::Float64, raySy::Float64, rayEx::Float64, rayEy::Float64)

  	(y-raySy)*(rayEx-raySx) > (x-raySx)*(rayEy-raySy)
end
function is_pt_left_of_ray(x::Float64, y::Float64, rX::Float64, rY::Float64, rθ::Float64)

	is_pt_left_of_ray(x, y, rX, rY, rX + cos(rθ), rY + sin(rθ))
end

function replace_featuresymbols_in_string_with_latex(str::String)

	strs = map(k->string(k), keys(sym2ftr))
	lstr = map(f->lsymbol(f), values(sym2ftr))

	p = sortperm(strs, by=length, rev=true)

	for i in p
		str = replace(str, strs[i], lstr[i])
	end
	str
end
function print_feature_table()
	for f in values(sym2ftr)
		println(string(f), " & ", lsymbol(f), " & ", units(f), " & ", description(f))
	end
end

# ----------------------------------

tick_list = [5,10,15,20,30,40,50,60,80]
tick_list_short = [5,10,15,20]

include("template_past_accel.jl")
include("template_past_turnrate.jl")
include("template_past_velFy.jl")
include("template_past_d_cl.jl")

include("template_max_accel.jl")
include("template_max_turnrate.jl")
include("template_mean_accel.jl")
include("template_mean_turnrate.jl")
include("template_std_accel.jl")
include("template_std_turnrate.jl")

end # end module