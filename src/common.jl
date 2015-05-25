export StraightRoadway, PointSE2, Vehicle, VehicleTrace
export LOG_COL_X, LOG_COL_Y, LOG_COL_ϕ, LOG_COL_V, LOG_COL_A, LOG_COL_T, LOG_NCOLS_PER_CAR,
	   LOG_COL_logprobweight_A, LOG_COL_logprobweight_T, LOG_COL_em
export is_onroad, get_lanecenters, get_laneborders
export get_ncars, get_nframes, calc_logindexbase, create_log, create_trace
export calc_required_bytes_to_allocate, allocate_simlog_for_traces, allocate_simlogs_for_all_traces

export
		INPUT_EMSTATS_FOLDER,
		TRACE_DIR,
		OUTPUT_FOLDER_DIRICHLET,
		OUTPUT_FOLDER_CATEGORICAL,

		CAR_LENGTH,
		CAR_WIDTH,
		DEFAULT_LANE_WIDTH

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
const DEFAULT_LANE_WIDTH = 3.7

const INPUT_EMSTATS_FOLDER      = "/media/tim/DATAPART1/Data/Bosch/processed/plots/graph_feature_selection_NEW/"
const TRACE_DIR                 = "/media/tim/DATAPART1/Data/Bosch/processed/traces/"
const OUTPUT_FOLDER_DIRICHLET   = "/media/tim/DATAPART1/Data/Bosch/processed/plots/sim/dirichlet/"
const OUTPUT_FOLDER_CATEGORICAL = "/media/tim/DATAPART1/Data/Bosch/processed/plots/sim/categorical/"

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

immutable VehicleTrace
	# records the trace of a vehicle during a simulation, 
	# including the history before the simulation begins
	#  columns are {x, y, ϕ, v, accel, turnrate, logP_A, logP_T, em_choice}
	#  actions at time t produce state at time t+1

	log :: Matrix{Float64}
	history :: Int # the index of the first frame (history = 1 indicates frame 1 is the initial state)

	function VehicleTrace(nframes::Integer, history::Integer) 
		@assert(history > 0)
		@assert(history ≤ nframes)

		simlog = Array(Float64, nframes, LOG_NCOLS_PER_CAR)
		new(simlog, history)
	end
end

is_onroad(posFy::Real, road::StraightRoadway) = -0.5road.lanewidth < posFy <  (road.nlanes - 0.5)*road.lanewidth
get_lanecenters(road::StraightRoadway) = [0:(road.nlanes-1)].*road.lanewidth # [0,w,2w,...]
get_laneborders(road::StraightRoadway) = [0:road.nlanes].*road.lanewidth - road.lanewidth/2

get_ncars(log::Matrix{Float64}) = div(size(log,2), LOG_NCOLS_PER_CAR)
get_nframes(log::Matrix{Float64}) = size(log, 1)
calc_logindexbase(carind::Int) = LOG_NCOLS_PER_CAR*carind-LOG_NCOLS_PER_CAR # the index to which LOG_COL_* is added

create_log(ncars::Integer, nframes::Integer) = Array(Float64, nframes, ncars*LOG_NCOLS_PER_CAR)

function fill_log_with_trace_complete!(simlog::Matrix{Float64}, trace::VehicleTrace, carind::Int, startframe::Int)
	# perform a direct copy of the trace

	nframes_simlog = size(simlog, 1)
	@assert(nframes_simlog ≤ size(trace.log, 1))
	@assert(size(trace.log, 2) == LOG_NCOLS_PER_CAR)
	@assert(startframe == trace.history)

	baseind = calc_logindexbase(carind)

    for j = 1 : LOG_NCOLS_PER_CAR
    	for i = 1 : nframes_simlog
    		simlog[i,baseind+j] = trace.log[i,j]
    	end
    end

	simlog
end
function fill_log_with_trace_partial!(simlog::Matrix{Float64}, trace::VehicleTrace, carind::Int, startframe::Int)
	# only copy the history and perform no override of the rest

	m = size(simlog, 1)
	@assert(m ≥ size(trace.log, 1))
	@assert(size(trace.log, 1) ≥ trace.history)
	@assert(size(trace.log, 2) == LOG_NCOLS_PER_CAR)
	@assert(startframe == trace.history)

	baseind = calc_logindexbase(carind)

    for j = 1 : LOG_NCOLS_PER_CAR
    	for i = 1 : startframe
    		simlog[i,baseind+j] = trace.log[i,j]
    	end
    end

	simlog
end

function calc_required_bytes_to_allocate(traces::Vector{VehicleTrace}, nframes_total::Int)
    @assert(nframes_total > 0)
    ncars = length(traces)
    ncars * nframes_total * LOG_NCOLS_PER_CAR * sizeof(Float64)
end
function calc_required_bytes_to_allocate(tracesets::Vector{Vector{VehicleTrace}}, nframes_total::Int)
    total_bytes_to_allocate = 0
    for traces in tracesets
        total_bytes_to_allocate += calc_required_bytes_to_allocate(traces, nframes_total)
    end
    total_bytes_to_allocate
end
function allocate_simlog_for_traces(traces::Vector{VehicleTrace}, nframes_total::Int)
    @assert(nframes_total > 0)
    simlog = create_log(length(traces), nframes_total)
end
function allocate_simlogs_for_all_traces(tracesets::Vector{Vector{VehicleTrace}}, nframes_total::Int)

    #=
    allocate enough simlogs to support ALL traces
        this is do-able for 2500 logs ~ 40 MB
        and still doable for 25,000 logs ~ 400 MB
        AND kinda doable for 250,000 logs ~ 4 GB
    =#

    simlogs = Array(Matrix{Float64}, length(tracesets))
    for (i,traces) in enumerate(tracesets)
        simlogs[i] = allocate_simlog_for_traces(traces, nframes_total)
    end
    simlogs
end
