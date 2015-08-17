export
    StraightRoadway,
    PointSE2,
    Vehicle,
    VehicleTrace,
    PdsetSegment,

    INPUT_EMSTATS_FOLDER,
    TRACE_DIR,
    OUTPUT_FOLDER_DIRICHLET,
    OUTPUT_FOLDER_CATEGORICAL,

    CAR_LENGTH,
    CAR_WIDTH,
    DEFAULT_LANE_WIDTH,
    DEFAULT_SEC_PER_FRAME,
    SPEED_65MPH,

    LOG_COL_X,
    LOG_COL_Y,
    LOG_COL_ϕ,
    LOG_COL_V,
    LOG_COL_ACTION_LAT,
    LOG_COL_ACTION_LON,
    LOG_COL_LOGL_LAT,
    LOG_COL_LOGL_LON,
    LOG_NCOLS_PER_CAR,

    is_onroad,
    get_lanecenter,
    get_lanecenters,
    get_laneborders,
    
    get_ncars,
    get_nframes,
    get_horizon,
    get_history,
    get_max_nframes,

    calc_logindexbase,
    create_log,
    create_and_fill_log,

    pull_vehicle,
    pull_vehicle!,
    place_vehicle!,

    fill_log_with_trace!,

    calc_required_bytes_to_allocate,
    allocate_simlog_for_traces,
    allocate_simlogs_for_all_traces,

    get_max_vehicle_count,

    estimate_history,

    translate!,
    translate_so_is_at_loc_at_frame!

# a log is represented as a Matrix{Float64}
const LOG_COL_X = 1
const LOG_COL_Y = 2
const LOG_COL_ϕ = 3
const LOG_COL_V = 4
const LOG_COL_ACTION_LAT = 5 # the lateral action input
const LOG_COL_ACTION_LON = 6 # the longitudinal action input
const LOG_COL_LOGL_LAT = 7 # loglikelihood of lateral action
const LOG_COL_LOGL_LON = 8 # loglikelihood of longitudinal action
const LOG_NCOLS_PER_CAR = 8

const CAR_LENGTH = 4.6 # [m]
const CAR_WIDTH  = 2.0 # [m]
const DEFAULT_LANE_WIDTH = 3.25 # [m]
const DEFAULT_SEC_PER_FRAME = 0.25 # [s]
const SPEED_65MPH = 29.06 # [m/s]

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

    PointSE2() = new()
    PointSE2(x::Float64, y::Float64, ϕ::Float64=0.0) = new(x,y,ϕ)
end

type Vehicle
	pos    :: PointSE2 # [m,m,rad]
	speed  :: Float64  # [m/s]
	length :: Float64  # [m]
	width  :: Float64  # [m]

    Vehicle() = new(PointSE2(), 0.0, CAR_LENGTH, CAR_WIDTH)
    function Vehicle(
        pos::PointSE2,
        speed::Float64,
        length::Float64=CAR_LENGTH,
        width::Float64=CAR_WIDTH
        )

        new(pos, speed, length, width)
    end
    function Vehicle(
        x::Float64,
        y::Float64,
        ϕ::Float64,
        speed::Float64,
        length::Float64=CAR_LENGTH,
        width::Float64=CAR_WIDTH
        )

        new(PointSE2(x,y,ϕ), speed, length, width)
    end
end

immutable VehicleTrace
	# records the trace of a vehicle during a simulation, 
	# including the history before the simulation begins
	#  columns are the same ones as in simlog
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

immutable PdsetSegment
    pdset_id        :: Int
    streetnet_id    :: Int
    carid           :: Int # the active vehicle
    validfind_start :: Int # the starting validfind (does not count any sort of history)
    validfind_end   :: Int # the ending validfind
end

function Base.show(io::IO, veh::Vehicle)
    println("PdsetSegment")
    @printf("\tpdset_id: %d\n", veh.pos)
    @printf("\tspeed:    %d\n", veh.speed)
    @printf("\tlength:   %d\n", veh.length)
    @printf("\twidth:    %d\n", veh.width)
end
function Base.show(io::IO, seg::PdsetSegment)
    println("PdsetSegment")
    @printf("\tpdset_id:        %d\n", seg.pdset_id)
    @printf("\tstreetnet_id:    %d\n", seg.streetnet_id)
    @printf("\tcarid:           %d\n", seg.carid)
    @printf("\tvalidfind_start: %d\n", seg.validfind_start)
    @printf("\tvalidfind_end:   %d\n", seg.validfind_end)
end

is_onroad(posFy::Real, road::StraightRoadway) = -0.5road.lanewidth < posFy <  (road.nlanes - 0.5)*road.lanewidth
get_lanecenter(road::StraightRoadway, laneindex::Int) = road.lanewidth * (laneindex-1)
get_lanecenters(road::StraightRoadway) = [0:(road.nlanes-1)].*road.lanewidth # [0,w,2w,...]
get_laneborders(road::StraightRoadway) = [0:road.nlanes].*road.lanewidth - road.lanewidth/2

get_ncars(simlog::Matrix{Float64}) = div(size(simlog,2), LOG_NCOLS_PER_CAR)
get_nframes(simlog::Matrix{Float64}) = size(simlog, 1)
get_nframes(trace::VehicleTrace) = size(trace.log,1)
get_nframes(seg::PdsetSegment) = seg.validfind_end - seg.validfind_start + 1
get_horizon(trace::VehicleTrace) = size(trace.log,1) - trace.history
get_horizon(seg::PdsetSegment) = seg.validfind_end - seg.validfind_start # NOTE(tim): no +1 as the first frame does not count in horizon
get_history(trace::VehicleTrace) = trace.history

function get_max_nframes(traces::Vector{VehicleTrace})
    nframes = -1
    for trace in traces
        n = get_nframes(trace) 
        if n > nframes
            nframes = n
        end
    end
    nframes
end

calc_logindexbase(carind::Int) = LOG_NCOLS_PER_CAR*carind-LOG_NCOLS_PER_CAR # the index to which LOG_COL_* is added
create_log(ncars::Integer, nframes::Integer) = Array(Float64, nframes, ncars*LOG_NCOLS_PER_CAR)
function create_and_fill_log(traces::Vector{VehicleTrace}, nframes::Integer=get_max_nframes(traces))

    @assert(!isempty(traces))
    simlog = create_log(length(traces), nframes)
    fill!(simlog, NaN)

    for (carind,trace) in enumerate(traces)
        fill_log_with_trace!(simlog, 1, trace, 1, get_nframes(trace), carind)
    end

    simlog
end


pull_vehicle(simlog::Matrix{Float64}, baseind::Int, frameind::Int) = pull_vehicle!(Vehicle(), simlog, baseind, frameind)
function pull_vehicle!(veh::Vehicle, simlog::Matrix{Float64}, baseind::Int, frameind::Int)
    veh.pos.x = simlog[frameind, baseind + LOG_COL_X]
    veh.pos.y = simlog[frameind, baseind + LOG_COL_Y]
    veh.pos.ϕ = simlog[frameind, baseind + LOG_COL_ϕ]
    veh.speed = simlog[frameind, baseind + LOG_COL_V]
    veh
end
function place_vehicle!(simlog::Matrix{Float64}, baseind::Int, frameind::Int, veh::Vehicle)
    simlog[frameind, baseind + LOG_COL_X] = veh.pos.x
    simlog[frameind, baseind + LOG_COL_Y] = veh.pos.y
    simlog[frameind, baseind + LOG_COL_ϕ] = veh.pos.ϕ
    simlog[frameind, baseind + LOG_COL_V] = veh.speed
    simlog
end
function place_vehicle!(simlog::Matrix{Float64}, baseind::Int, frameind::Int, simloghistory::Int, trace::VehicleTrace)
    framedelta = frameind - simloghistory
    traceindex = trace.history + framedelta

    for j = 1 : LOG_NCOLS_PER_CAR
        simlog[frameind, baseind+j] = trace.log[traceindex, j]
    end
    simlog
end

function fill_log_with_trace!(
    simlog::Matrix{Float64},
    index_in_simlog_where_we_start::Int,
    trace::VehicleTrace,
    index_in_trace_where_we_start::Int,
    index_in_trace_where_we_end::Int,
    carind::Int,
    )

    @assert(size(trace.log, 2) == LOG_NCOLS_PER_CAR)

    baseind = calc_logindexbase(carind)

    index_in_simlog = index_in_simlog_where_we_start - 1
    for index_in_trace = index_in_trace_where_we_start : index_in_trace_where_we_end
        index_in_simlog += 1

        for j = 1 : LOG_NCOLS_PER_CAR
            simlog[index_in_simlog,baseind+j] = trace.log[index_in_trace,j]
        end
    end

    simlog
end
# function fill_log_with_trace_complete!(simlog::Matrix{Float64}, trace::VehicleTrace, carind::Int, startframe::Int)
# 	# perform a direct copy of the trace
# 	@assert(startframe == trace.history)
#     fill_log_with_trace!(simlog, startframe, trace, startframe, size(simlog, 1), carind)
# end
# function fill_log_with_trace_partial!(simlog::Matrix{Float64}, trace::VehicleTrace, carind::Int, startframe::Int)
# 	# only copy the history and perform no override of the rest
#     @assert(startframe == trace.history)
#     fill_log_with_trace!(simlog, 1, trace, 1, startframe, carind)
# end

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

function get_max_vehicle_count(tracesets::Vector{Vector{VehicleTrace}})
    hi = 0
    for traceset in tracesets
        hi = max(hi, length(traceset))
    end
    hi
end

function estimate_history(simlog::Matrix{Float64})
    # history should be where ego X position is 0.0
    # returns -1 if it was not found

    for i = 1 : get_nframes(simlog)
        if isapprox(simlog[i, LOG_COL_X], 0.0)
            return i
        end
    end
    return -1
end

function translate!(trace::VehicleTrace, Δx::Float64, Δy::Float64)

    trace.log[:,LOG_COL_X] += Δx
    trace.log[:,LOG_COL_Y] += Δy

    trace
end
function translate_so_is_at_loc_at_frame!(trace::VehicleTrace, x::Float64, y::Float64, frameind::Int)

    x_t = trace.log[frameind, LOG_COL_X]
    y_t = trace.log[frameind, LOG_COL_Y]
    Δx = x - x_t
    Δy = y - y_t
    translate!(trace, Δx, Δy)
end