module RunLogs

using DataFrames
using DataArrays

using AutomotiveDrivingModels.Vec

import AutomotiveDrivingModels: LaneTag, CurvePt

export
    AgentClass,                         # enum containing classes of agents
    ContextClass,                       # enum for context classes
    ControlStatus,                      # enum containing control status defintions
    AgentDefinition,                    # type definition of an agent
    RunLogHeader,                       # type definition of a RunLog header
    RunLog,                             # type definition of all data from a continuous run
    LaneTag,                            # reexport from StreetNetworks

    ID_NULL,                            # null value for non-existant UID
    ID_EGO,                             # ego UID
    COLSET_NULL,                        # null value for non-existant column set index

    # create_empty_runlog               # generate a new one
    # expand_rowcount!,                 # increase the number of rows
    push_agent!,                        # add a new agent to the set
    push_columnset!,                    # adds a new column set (with id's = ID_NULL) and returns runlog
    extract_subset,                     # pull a subregion as a new runlog

    nframes,                            # number of frames
    ncolsets,                           # max number of colsets
    frame_inbounds,                     # whether the given frame is within [0,nframes(runlog)]

    # Base.get                          # pull item from runlog
    set!,                               # set the entry in the runlog

    idinframe,                          # whether the given id is in the frame
    idinframes,                         # whether the given id is in all the frames

    colset2id,                          # get the id of the vehicle in the given colset
    id2colset,                          # get the colset of the vehicle with the given id, returns COLSET_NULL on fail

    estimate_framerate,                 # estimate the framerate using an average
    get_frame,                          # obtains a frame based on the time
    get_elapsed_time,                   # elapsed time between validfinds

    nactors,                            # number of column sets with actors (id ≠ ID_NULL)
    get_max_nactors,                    # maximum nactors over all frames in runlog

    get_first_vacant_colset,            # returns index of first vacant colset
    get_first_vacant_colset!,           # exands the number of colsets if necessary
    frames_contain_id                   # returns true if all of the frames in the range contain the id

    # copy_trace!,
    # copy_vehicle!,

    # remove_car_from_frame!,
    # remove_cars_from_frame!,
    # remove_car!,

    # extract_continuous_segments,
    # spanning_frames ,
    # get_frames_contining_id

##########################################################
#
#
#
##########################################################

baremodule AgentClass
    const CAR        = 0x01
    const MOTORCYCLE = 0x02
    const TRUCK      = 0x03
end

baremodule ControlStatus
    const UNKNOWN             = 0x00 #
    const START               = 0x01 # vehicle is in start mode
    const ACTIVE              = 0x02 # vehicle is in active mode
    const ACTIVE_REINT        = 0x03 #
    const CONTROL_ACTIVE      = 0x04 #
    const AUTO                = 0x05 # autonomous driving mode
    const HANDOVER_TO_DRIVER  = 0x06 # system -> driver handover
    const HANDOVER_TO_SYSTEM  = 0x07 # driver -> system handover
    const PREPARE_ACTUATORS   = 0x08 #
    const READY               = 0x09 #
    const WAITING             = 0x10 #
    const FAILURE             = 0x11 #
    const INIT                = 0x12 #
    const PASSIVE             = 0x13 #
    const VIRES_AUTO          = 0x99 # system was simulated by Vires
end

baremodule ContextClass
    const NULL                = 0x0000
    const FREEFLOW            = 0x0001
    const FOLLOWING           = 0x0002
    const LANECHANGE          = 0x0004
end

immutable AgentDefinition
    class  :: UInt8   # agent class ∈ {motorcycle, car, ...} (see AgentClass)
    id     :: UInt    # unique identification number
    length :: Float64 # bounding box length [m]
    width  :: Float64 # bounding box width [m]


    function AgentDefinition(class::UInt8, uid::UInt, length::Real, width::Real)
        new(class, uid, length, width)
    end
    function AgentDefinition(class::UInt8, uid::UInt)
        length = NaN
        width = NaN
        if class == AgentClass.CAR
            length = 4.6
            width = 2.0
        elseif class == AgentClass.MOTORCYCLE
            length = 3.0
            width = 1.0
        elseif class == AgentClass.TRUCK
            length = 5.0
            width = 2.5
        end
        new(class, uid, length, width)
    end
end

type RunLogHeader
    map_name::AbstractString
    driver_name::AbstractString
end

type RunLog
    header::RunLogHeader
    agents::Dict{UInt, AgentDefinition} # Id -> AgentDefinition

    framedata::DataFrame # the data that is not repeated for each frame (time, environment)
    colsets::Vector{DataFrame} # each dataframe contains the same columns
                               # colsets is indexed by `colset`, the column set index

    function RunLog(nframes::Integer, header::RunLogHeader, ncolsets::Integer=0)

        agents = Dict{UInt, AgentDefinition}()
        framedata = _create_framedata(nframes)
        colsets = Array(DataFrame, ncolsets)

        for i in 1 : ncolsets
            colsets[i] = _create_columnset(nframes)
        end

        new(header, agents, framedata, colsets)
    end
end

const ID_NULL = typemax(UInt)
const ID_EGO = one(UInt)
const COLSET_NULL = zero(UInt)

# BODY
# ----------------------------------------
# EACH FRAME CONTAINS THESE VALUES ONCE
#
# Float64 time            [s] unix epoch
# UInt16  environment     [0] environment mask
# ----------------------------------------
# THESE COLUMNS ARE REPEATED FOR N VEHICLES (each entry in RunLog.colsets)
#
# UInt        id          [-] id of the vehicle in this column set for this frame, ID_NULL if none
# UInt        next_colset [-] column set index for this vehicle in the next frame, COLSET_NULL if none
# UInt        prev_colset [-] column set index for this vehicle in the previous frame, COLSET_NULL if none
# VecSE2      inertial    (x,y,ϕ) [m,m,rad] vehicle position in inertial coordinates
# VecSE2      frenet      (s,d,θ) [m,m,rad] vehicle position in frenet coordinates
# VecSE2      ratesB      [m/s, m/s, rad/2] linear and angular rates in the body frame
# VecS2       ratesF      [m/s, m/s] linear rates in the lane-relative Frenet frame (relative to closest lane centerline)
# Float64     extind      [-] extended index of the closest centerline (see Curves.jl)
# CurvePt     footpoint   [-] point on the closest centerline
# LaneTag     lanetag     [-] assigned lane in given map
# UInt        idfront     [-] colset of the vehicle in front of this one, COLSET_NULL if none
# UInt        idrear      [-] colset of the vehicle behind this one, COLSET_NULL if none
# UInt16      behavior    [0] context class

function _create_framedata(nframes::Integer)
    DataFrame(
        time = DataArray(Float64, nframes),
        environment = DataArray(UInt16, nframes),
    )
end
function _create_columnset(nframes::Integer)
    DataFrame(
        id = fill(ID_NULL, nframes),
        next_colset = fill(COLSET_NULL, nframes),
        prev_colset = fill(COLSET_NULL, nframes),
        inertial = DataArray(VecSE2, nframes),
        frenet = DataArray(VecSE2, nframes),
        ratesB = DataArray(VecSE2, nframes),
        ratesF = DataArray(VecE2, nframes),
        extind = DataArray(Float64, nframes),
        footpoint = DataArray(CurvePt, nframes),
        lanetag = DataArray(LaneTag, nframes),
        idfront = fill(COLSET_NULL, nframes),
        idrear = fill(COLSET_NULL, nframes),
        behavior = fill(ContextClass.NULL, nframes),
    )
end

function push_agent!(runlog::RunLog, agent::AgentDefinition)
    @assert(!haskey(runlog.agents, agent.id))
    runlog.agents[agent.id] = agent
    runlog
end
function push_columnset!(runlog::RunLog)
    df = _create_columnset(nframes(runlog))
    push!(runlog.colsets, df)
end
function extract_subset(runlog::RunLog, frame_start::Int, frame_end::Int)

    N = validfind_end - validfind_start + 1
    max_nactors = get_max_nactors(runlog, frame_start:frame_end)
    n_col_colset = size(runlog.colsets[1], 2)

    header = deepcopy(runlog.header)
    agents = Dict{UInt, PlayerDefinition}()
    framedata = _create_framedata(N)
    colsets = Array(DataFrame, max_nactors)

    for (i,frame) in enumerate(frame_start:frame_end)

        framedata[i,:time] = runlog.framedata[frame,:time]
        framedata[i,:environment] = runlog.framedata[frame,:environment]

        colset = 1
        while df[frame, :id] != ID_NULL
            # copy over new agents
            id = df[frame, :id]
            if !in(id, agents)
                agents[id] = deepcopy(runlog.agents[id])
            end

            colset_new = colsets[i]
            colset_old = runlog.colsets[i]

            # copy over colset data
            for j in 1 : n_col_colset
                colset_new[i,j] = colsets_old[i,j]
            end

            colset += 1
        end
    end

    RunLog(header, agents, framedata, colsets)
end

nframes(runlog::RunLog) = nrow(runlog.colsets[1])
ncolsets(runlog::RunLog) = convert(UInt, length(runlog.colsets))
frame_inbounds(runlog::RunLog, frame::Integer) = (1 ≤ frame ≤ nframes(runlog))

Base.get(runlog::RunLog, frame::Integer, sym::Symbol) = runlog.framedata[frame, sym]
Base.get(runlog::RunLog, colset::UInt, frame::Integer, sym::Symbol) = runlog.colsets[colset][frame, sym]
function Base.get{I<:Integer}(runlog::RunLog, frames::AbstractVector{I}, sym::Symbol)
    retval = Array(eltype(runlog.framedata[sym]), length(frames))
    for (i,frame) in enumerate(frames)
        retval[i] = runlog.framedata[frame, sym]
    end
    retval
end
function Base.get{I<:Integer}(runlog::RunLog, colset::UInt, frames::AbstractVector{I}, sym::Symbol)
    df = runlog.colsets[colset]
    retval = Array(eltype(df[sym]), length(frames))
    for (i,frame) in enumerate(frames)
        retval[i] = df[frame, sym]
    end
    retval
end

function set!(runlog::RunLog, colset::UInt, frame::Integer, sym::Symbol, value::Any)
    runlog.colsets[colset][frame, sym] = value
    runlog
end
function set!{I<:Integer}(runlog::RunLog, colset::UInt, frames::AbstractVector{I}, sym::Symbol, values::AbstractVector)

    df = runlog.colsets[colset]
    for (frame,value) in zip(frames, values)
        df[frame, sym] = value
    end

    runlog
end
function set!(
    runlog::RunLog,
    colset::UInt,
    frame::Integer,
    id        :: UInt,
    inertial  :: VecSE2,
    frenet    :: VecSE2,
    ratesB    :: VecSE2,
    extind    :: Float64,
    footpoint :: CurvePt,
    lanetag   :: LaneTag,
    idfront   :: UInt = COLSET_NULL,
    idrear    :: UInt = COLSET_NULL,
    behavior  :: UInt16 = ContextClass.NULL,
    )

    #=
    Sets all attributes for a vehicle's frame
    Note that next_colset, prev_colset, and ratesF are automatically computed
    Note that front and rear are >not< computed
    =#

    df = runlog.colsets[colset]

    next_colset = (frame < nframes(runlog)) ? id2colset(runlog, id, frame+1) : COLSET_NULL
    prev_colset = (frame > 1) ? id2colset(runlog, id, frame-1) : COLSET_NULL

    speed = hypot(ratesB)
    ratesF = polar(speed, frenet.θ)

    df[frame, :id         ] = id
    df[frame, :next_colset] = next_colset
    df[frame, :prev_colset] = prev_colset
    df[frame, :inertial   ] = inertial
    df[frame, :frenet     ] = frenet
    df[frame, :ratesB     ] = ratesB
    df[frame, :ratesF     ] = ratesF
    df[frame, :extind     ] = extind
    df[frame, :footpoint  ] = footpoint
    df[frame, :lanetag    ] = lanetag
    df[frame, :idfront    ] = idfront
    df[frame, :idrear     ] = idrear
    df[frame, :behavior   ] = behavior

    runlog
end
function set!(runlog::RunLog, frame::Integer, sym::Symbol, value::Any)
    runlog.framedata[frame, sym] = value
    runlog
end
function set!{I<:Integer}(runlog::RunLog, frames::AbstractVector{I}, sym::Symbol, values::AbstractVector)

    df = runlog.framedata
    for (frame,value) in zip(frames, values)
        df[frame, sym] = value
    end

    runlog
end


function idinframe(runlog::RunLog, id::UInt, frame::Integer)
    # whether the given id is in the frame

    for df in runlog.colsets
        if df[frame, :id] == id
            return true
        end
    end
    false
end
function idinframes(runlog::RunLog, id::UInt, frames::AbstractVector{Int64})
    # returns true if all of the frames contain the given id

    for frame in frames
        if !idinframe(runlog, id, frame)
            return false
        end
    end
    true
end

colset2id(runlog::RunLog, colset::UInt, frame::Integer) = get(runlog, colset, frame, :id)::UInt
function id2colset(runlog::RunLog, id::UInt, frame::Integer)

    #=
    Find the colset for the given id in the frame
    NOTE: does not check whether frame is valid
    Returns COLSET_NULL if non-existent
    =#

    for colset in 1 : nactors(runlog, frame)
        if colset2id(runlog, convert(UInt, colset), frame) == id
            return colset
        end
    end
    return COLSET_NULL
end

function estimate_framerate(runlog::RunLog)

    N = nframes(runlog)
    t0 = get(runlog, 1, :time)::Float64
    tf = get(runlog, N, :time)::Float64

    (tf - t0) / (N-1)
end
function get_elapsed_time(runlog::RunLog, frameA::Integer, frameB::Integer)
    t1 = get(runlog, frameA, :time)::Float64
    t2 = get(runlog, frameB, :time)::Float64
    t2 - t1
end
function get_frame( runlog::RunLog, time::Float64, mode::Symbol=:nearest)

    #=
    returns the frame that is closest to the given time
    uses the fact that Δt is more or less consistent
    mode = :nearest rounds to the nearest frame
    mode = :up rounds to the nearest frame above, if it exists, otherwise throws an error
    mode = :down rounds to the nearest frame below, if it exists, otherwise throws an error
    =#

    N = nframes(runlog)
    t0 = get(runlog, 1, :time)::Float64
    tf = get(runlog, N, :time)::Float64
    Δt = (tf - t0) / (N-1)

    frame = clamp(round(Int, 1.0 + (time - t0) / Δt, RoundNearestTiesAway), 1, N)
    t  = get(runlog, frame, :time)::Float64

    # now do a search to ensure we are actually the closest we can be to that time
    t⁺ = (frame < N) ? get(runlog, frame+1, :time)::Float64 : t
    if abs(t⁺ - time) < abs(t - time)
        t, frame = t⁺, frame+1
        t⁺ = (frame < N) ? get(runlog, frame+1, :time)::Float64 : t
        while abs(t⁺ - time) < abs(t-time)
            t, frame = t⁺, frame+1
            t⁺ = (frame < N) ? get(runlog, frame+1, :time)::Float64 : t
        end
        return frame
    end

    t⁻ = (frame > 1) ? get(runlog, frame+1, :time)::Float64 : t
    if abs(t⁻ - time) < abs(t-time)
        t, frame = t⁻, frame-1
        t⁻ = (frame > 1) ? get(runlog, frame+1, :time)::Float64 : t
        while abs(t⁻ - time) < abs(t-time)
            t, frame = t⁻, frame-1
            t⁻ = (frame > 1) ? get(runlog, frame+1, :time)::Float64 : t
        end
        return frame
    end

    frame
end

function nactors(runlog::RunLog, frame::Integer)
    # number of column sets with actors (id ≠ ID_NULL)

    retval = 0
    for df in runlog.colsets
        retval += (df[frame, :id] != ID_NULL)
    end
    retval
end
function get_max_nactors(runlog::RunLog)
    # maximum nactors over all frames in runlog

    retval = 0
    for frame in 1 : nframes(runlog)
        retval = max(retval, nactors(runlog, frame))
    end
    retval
end
function get_max_nactors{I<:Integer}(runlog::RunLog, frames::AbstractVector{I})
    # maximum nactors over all frames in runlog

    retval = 0
    for frame in frames
        retval = max(retval, nactors(runlog, frame))
    end
    retval
end

# Base.show(runlog::RunLog) # this should just list some basic statics
# Base.print(runlog::RunLog) # print everything

# Base.write(io::IO, runlog::RunLog) # export to a csv format similar to that of ViresCSV
# Base.read(io::IO, runlog::RunLog) # read from a csv format similar to that of ViresCSV

# Base.delete!(runlog::RunLog, colset::UInt, frame::Integer) = # Removes the player in the given frame and returns the runlog
    # this function should then shift all actors above colset down to fill the gap
    # note that this requires us to update indnext and indprev for the previous and next frames
# Base.delete!(runlog::RunLog, id::UInt) # remove the given id from all frames in runlog and return the runlog
    # this function should then shift all actors above colset down to fill the gap
    # note that this requires us to update indnext and indprev for the previous and next frames

function get_first_vacant_colset(runlog::RunLog, id::UInt, frame::Integer)
    for (colset, df) in enumerate(runlog.colsets)
        if df[frame, :id] == ID_NULL
            return convert(UInt, colset)
        end
    end
    COLSET_NULL
end
function get_first_vacant_colset!(runlog::RunLog, id::UInt, frame::Integer)
    colset = get_first_vacant_colset(runlog, id, frame)::UInt
    if colset == COLSET_NULL
        push_columnset!(runlog)
        colset = ncolsets(runlog)::UInt
    end
    colset
end

# function extract_column_set_values!(
#     runlog::RunLog,
#     mgr::OdrManagerLite,
#     id::UInt,
#     frame::Integer,
#     inertial::AbstractCoord, # inertial position
#     ratesB::AbstractCoord, # linear and angular rates in the body frame
#     )
#
#     #=
#     1 - use add_id!() to obtain the colset DONE
#     2 - compute the features DONE
#          - id is given DONE
#          - inertial is given DONE
#          - ratesB is given DONE
#          - ratesF need to be computed
#          - lanecoord needs to be computed
#          - behavior, idfront, and idrear # tackle these later (just use 0 for now)
#          - indprev needs to be computed (use id2ind_zero_on_fail())
#          - indnext also needs to be computed (use id2ind_zero_on_fail())
#     3 - insert them into the body using set!()
#          - note that you need to convert all Coord-likes into their immutable versions
#            convert(CoordIm, inertial)
#     4 - return the modified runlog
#     =#
#
#     set_pos(mgr, inertial)
#     inertial2lane(mgr)
#     lane_pos = get_lanepos(mgr)
#     foot_pos = get_footpoint(mgr)
#
#     lane_heading = foot_pos.h
#     posFθ = inertial.h - lane_heading
#     velFs, velFt = get_lane_relative_velocity(mgr, posFθ, ratesB.x, ratesB.y)
#     ratesF = CoordIm(velFs, velFt, ratesB.z, ratesB.h, ratesB.p, ratesB.r)
#
#     colset = add_id!(runlog, id, frame)
#     set!(runlog, colset, frame, :id, convert(UInt, id))
#     set!(runlog, colset, frame, :inertial, convert(CoordIm, inertial))
#     set!(runlog, colset, frame, :ratesB, convert(CoordIm, ratesB))
#     set!(runlog, colset, frame, :ratesF, ratesF)
#     set!(runlog, colset, frame, :lanecoord, convert(LaneCoordIm, lane_pos))
#
#     runlog
# end

let
    function all_fields_are_bits(df::DataFrame)
        for j in 1:size(df,2)
            @assert(isbits(eltype(df[j])))
        end
    end

    # ensure that all field in RunLog's framedata are immutable
    all_fields_are_bits(_create_framedata(1))

    # ensure that all fields in RunLog's colset are immutable
    all_fields_are_bits(_create_columnset(1))
end

end # module