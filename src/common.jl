export StraightRoadway, PointSE2, Vehicle, VehicleInitialConditions
export LOG_COL_X, LOG_COL_Y, LOG_COL_ϕ, LOG_COL_V, LOG_COL_A, LOG_COL_T, LOG_NCOLS_PER_CAR
export CAR_LENGTH, CAR_WIDTH
export onroad, lanecenters, laneborders
export ncars, nframes, logindexbase, create_log

# a log is represented as a Matrix{Float64}
const LOG_COL_X = 1
const LOG_COL_Y = 2
const LOG_COL_ϕ = 3
const LOG_COL_V = 4
const LOG_COL_A = 5 # acceleration input
const LOG_COL_T = 6 # turnrate input
const LOG_COL_logprobweight_A = 7 # logP of action A given observations and em
const LOG_COL_logprobweight_T = 8
const LOG_COL_em = 9 # the em model used in this step
const LOG_NCOLS_PER_CAR = 9

const CAR_LENGTH = 4.6 # [m]
const CAR_WIDTH  = 2.0 # [m]

immutable StraightRoadway
	nlanes    :: Int
	lanewidth :: Float64 # [m]
end

type PointSE2
	# a point in the 2D special euclidean group
	# represents a position and rotation

	x :: Float64
	y :: Float64
	ϕ :: Float64 # [rad]
end

type Vehicle
	pos    :: PointSE2 # [m,m,rad]
	speed  :: Float64  # [m/s]
	length :: Float64  # [m]
	width  :: Float64  # [m]
end

type VehicleInitialConditions
	posFx    :: Float64 # [m]
	posFy    :: Float64 # [m]
	yaw      :: Float64 # [rad]
	speed    :: Float64 # [m/s]
	turnrate :: Float64 # [rad/s]
	accel    :: Float64 # [m/s²]

	VehicleInitialConditions() = new(0.0,0.0,0.0,0.0,0.0,0.0)
	function VehicleInitialConditions(posFx::Float64, posFy::Float64, yaw::Float64,
		speed::Float64; turnrate::Float64 = 0.0, accel::Float64=0.0
		)

		new(posFx, posFy, yaw, speed, turnrate, accel)
	end
	function VehicleInitialConditions(
		posFx::Float64, posFy::Float64,    yaw::Float64,
		speed::Float64, turnrate::Float64, accel::Float64
		)

		new(posFx, posFy, yaw, speed, turnrate, accel)
	end
end

type VehicleTrace
	# records the trace of a vehicle during a simulation, 
	# including the history before the simulation begins
	#  columns are {x, y, ϕ, v, accel, turnrate, logP_A, logP_T, em_choice}
	#  actions at time t produce state at time t+1

	log :: Matrix{Float64}
	history :: Int # the index of the first frame (history = 1 indicates frame 1 is the initial state)
end

onroad(posFy::Real, road::StraightRoadway) = -0.5road.lanewidth < posFy <  (road.nlanes - 0.5)*road.lanewidth
lanecenters(road::StraightRoadway) = [0:(road.nlanes-1)].*road.lanewidth # [0,w,2w,...]
laneborders(road::StraightRoadway) = [0:road.nlanes].*road.lanewidth - road.lanewidth/2

create_log(ncars::Integer, nframes::Integer) = Array(Float64, nframes, ncars*LOG_NCOLS_PER_CAR)
ncars(log::Matrix{Float64}) = div(size(log,2), LOG_NCOLS_PER_CAR)
nframes(log::Matrix{Float64}) = size(log, 1)
logindexbase(carind::Int) = LOG_NCOLS_PER_CAR*carind-LOG_NCOLS_PER_CAR # the index to which LOG_COL_* is added