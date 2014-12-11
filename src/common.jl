export LOG_COL_X, LOG_COL_Y, LOG_COL_ϕ, LOG_COL_V, LOG_NCOLS_PER_CAR
export ncars, logindexbase, onroad

# a log is represented as a Matrix{Float64}
#  columns are car_1_x, car_1_y, car_1_ϕ, car_1_v, car_2_x, ...
const LOG_COL_X    = 1
const LOG_COL_Y    = 2
const LOG_COL_ϕ    = 3
const LOG_COL_V    = 4
const LOG_NCOLS_PER_CAR = 4

immutable Roadway
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

function onroad(posFy::Real, road::Roadway)
	
	-road.lanewidth < posFy <  (road.nlanes - 0.5)*road.lanewidth
end

ncars(log::Matrix{Float64}) = div(size(log,2), LOG_NCOLS_PER_CAR)
logindexbase(carind::Int) = LOG_NCOLS_PER_CAR*cind-LOG_NCOLS_PER_CAR # the index to which LOG_COL_* is added