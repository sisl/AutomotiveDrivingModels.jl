export StraightRoadway, PointSE2, Vehicle, Scene
export LOG_COL_X, LOG_COL_Y, LOG_COL_ϕ, LOG_COL_V, LOG_COL_A, LOG_COL_T, LOG_NCOLS_PER_CAR
export CAR_LENGTH, CAR_WIDTH
export onroad, lanecenters
export ncars, nframes, logindexbase, create_log

# a log is represented as a Matrix{Float64}
#  columns are car_1_x, car_1_y, car_1_ϕ, car_1_v, car_1_a, car_1_t, car_2_x, ...
#  actions at time t produce state at time t+1
const LOG_COL_X = 1
const LOG_COL_Y = 2
const LOG_COL_ϕ = 3
const LOG_COL_V = 4
const LOG_COL_A = 5 # acceleration input
const LOG_COL_T = 6 # turnrate input
const LOG_NCOLS_PER_CAR = 6

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

typealias Scene Vector{Vehicle}

onroad(posFy::Real, road::StraightRoadway) = -0.5road.lanewidth < posFy <  (road.nlanes - 0.5)*road.lanewidth
lanecenters(road::StraightRoadway) = [0:(road.nlanes-1)].*road.lanewidth # [0,w,2w,...]

create_log(ncars::Integer, nframes::Integer) = Array(Float64, nframes, ncars*LOG_NCOLS_PER_CAR)
ncars(log::Matrix{Float64}) = div(size(log,2), LOG_NCOLS_PER_CAR)
nframes(log::Matrix{Float64}) = size(log, 1)
logindexbase(carind::Int) = LOG_NCOLS_PER_CAR*carind-LOG_NCOLS_PER_CAR # the index to which LOG_COL_* is added