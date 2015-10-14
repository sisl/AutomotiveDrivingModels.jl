module CommonTypes

export
    Vehicle,
    PdsetSegment,

    DEFAULT_CAR_LENGTH,
    DEFAULT_CAR_WIDTH,
    DEFAULT_LANE_WIDTH,
    DEFAULT_FRAME_PER_SEC,
    DEFAULT_SEC_PER_FRAME,
    N_FRAMES_PER_SIM_FRAME,

    get_nframes,
    get_horizon,
    get_nsimframes

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
function ==(a::PdsetSegment, b::PdsetSegment)
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

end