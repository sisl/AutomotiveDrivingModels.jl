module CommonTypes

export
    Vehicle,
    PdsetSegment,
    RunLogSegment,

    DEFAULT_CAR_LENGTH,
    DEFAULT_CAR_WIDTH,
    DEFAULT_LANE_WIDTH,
    DEFAULT_FRAME_PER_SEC,
    DEFAULT_SEC_PER_FRAME,
    N_FRAMES_PER_SIM_FRAME,

    get_nframes,
    get_horizon,
    get_nsimframes,

    continuous_segments,
    near_continuous_segments

using AutomotiveDrivingModels.Vec

#############################################################################

const DEFAULT_CAR_LENGTH = 4.6 # [m]
const DEFAULT_CAR_WIDTH  = 2.0 # [m]
const DEFAULT_LANE_WIDTH = 3.25 # [m]

const DEFAULT_FRAME_PER_SEC = 20 # [frame / s]
const DEFAULT_SEC_PER_FRAME = 1.0 / DEFAULT_FRAME_PER_SEC # [s]
const N_FRAMES_PER_SIM_FRAME = 5

#############################################################################
# Vehicle

type Vehicle
    pos    :: VecSE2 # [m,m,rad] (center of the vehicle)
    speed  :: Float64  # [m/s]
    length :: Float64  # [m]
    width  :: Float64  # [m]

    Vehicle() = new(VecSE2(), 0.0, DEFAULT_CAR_LENGTH, DEFAULT_CAR_WIDTH)
    function Vehicle(
        pos::VecSE2,
        speed::Float64,
        length::Float64=DEFAULT_CAR_LENGTH,
        width::Float64=DEFAULT_CAR_WIDTH
        )

        new(pos, speed, length, width)
    end
    function Vehicle(
        x::Float64,
        y::Float64,
        ϕ::Float64,
        speed::Float64,
        length::Float64=DEFAULT_CAR_LENGTH,
        width::Float64=DEFAULT_CAR_WIDTH
        )

        new(VecSE2(x,y,ϕ), speed, length, width)
    end
end
function Base.show(io::IO, veh::Vehicle)
    println(io, "PdsetSegment")
    @printf(io, "\tpdset_id: %d\n", veh.pos)
    @printf(io, "\tspeed:    %d\n", veh.speed)
    @printf(io, "\tlength:   %d\n", veh.length)
    @printf(io, "\twidth:    %d\n", veh.width)
end

#############################################################################
# PdsetSegment

immutable PdsetSegment
    pdset_id        :: Int
    streetnet_id    :: Int
    carid           :: Int # the active vehicle
    validfind_start :: Int # the starting validfind (does not count any sort of history)
    validfind_end   :: Int # the ending validfind
end
function Base.(:(==))(a::PdsetSegment, b::PdsetSegment)
    a.pdset_id == b.pdset_id &&
    a.streetnet_id == b.streetnet_id &&
    a.carid == b.carid &&
    a.validfind_start == b.validfind_start &&
    a.validfind_end == b.validfind_end
end
function Base.show(io::IO, seg::PdsetSegment)
    println(io, "PdsetSegment")
    @printf(io, "\tpdset_id:        %d\n", seg.pdset_id)
    @printf(io, "\tstreetnet_id:    %d\n", seg.streetnet_id)
    @printf(io, "\tcarid:           %d\n", seg.carid)
    @printf(io, "\tvalidfind_start: %d\n", seg.validfind_start)
    @printf(io, "\tvalidfind_end:   %d\n", seg.validfind_end)
end

get_nframes(seg::PdsetSegment) = seg.validfind_end - seg.validfind_start + 1
get_horizon(seg::PdsetSegment) = seg.validfind_end - seg.validfind_start # NOTE(tim): no +1 as the first frame does not count in horizon
function get_nsimframes(seg::PdsetSegment, nframes_per_simframe::Int=N_FRAMES_PER_SIM_FRAME)
    # NOTE(tim): this includes the first frame
    h = get_horizon(seg)
    int(h / nframes_per_simframe) + 1
end

#############################################################################

function continuous_segments(arr::AbstractVector{Bool})

    # INPUT: boolean or bit array
    # OUTPUT: set of continuous segments
    #
    # ex: [T,T,T,F,F,T,T]
    #     [(1,3)(6,7)]

    ind = 1
    N = length(arr)
    segmentset = Tuple{Int,Int}[] # (frameind_lo, frameind_hi)
    while ind <= N
        curseg_1 = findnext(arr, true, ind)
        if curseg_1 == 0
            break
        end

        ind = curseg_1 + 1
        curseg_2 = findnext(arr, false, ind)

        if curseg_2 == 0
            curseg_2 = N
        else
            ind = curseg_2
            curseg_2 -= 1
        end

        push!(segmentset, (curseg_1, curseg_2))
    end
    segmentset
end
function near_continuous_segments(arr::Vector{Int}, tol::Int)
    # this assumes frames are evenly spaced
    # arr :: list of integer values, sorted in increasing order
    # tol :: tolerance in jump between segments
    # outputs list of segments by index of arr

    # example:
    #   arr = [1,2,3,5,10,11,12]
    #   tol = 2
    #   output: [(1,4),(5,7)]

    segmentset = Tuple{Int,Int}[] # (lo,hi)
    prev_val, prev_ind = arr[1], 1
    curseg_1 = 1
    for (ind,val) in enumerate(arr)
        Δ = val - prev_val
        @assert(Δ >= 0)
        if Δ > tol # start a new segment
            push!(segmentset, (curseg_1, prev_ind))
            curseg_1 = ind
        end
        prev_ind = ind
        prev_val = val
    end

    push!(segmentset, (curseg_1, length(arr)))
    segmentset
end

#############################################################################

end