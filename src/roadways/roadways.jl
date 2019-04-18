"""
    LaneBoundary
Data structure to represent lanes boundaries such as double yellow lines.
# Fields
    - `style::Symbol` ∈ :solid, :broken, :double
    - `color::Symbol` ∈ :yellow, white
"""
struct LaneBoundary
    style::Symbol # ∈ :solid, :broken, :double
    color::Symbol # ∈ :yellow, white
end
const NULL_BOUNDARY = LaneBoundary(:unknown, :unknown)

#######################
"""
    SpeedLimit
Datastructure to represent a speed limit
# Fields 
- `lo::Float64` [m/s] lower speed limit 
- `hi::Float64` [m/s] higher speed limit
"""
struct SpeedLimit
    lo::Float64 # [m/s]
    hi::Float64 # [m/s]
end
const DEFAULT_SPEED_LIMIT = SpeedLimit(-Inf, Inf)

#######################
"""
    LaneTag 
An identifier for a lane. The lane object can be retrieved by indexing the roadway by the lane tag:
```julia
tag = LaneTag(1, 2) # second lane segment 1
lane = roadway[tag] # returns a Lane object
```
# Fields 
- `segment::Int64` segment id
- `lane::Int64` index in segment.lanes of this lane
"""
struct LaneTag
    segment::Int64 # segment id
    lane::Int64 # index in segment.lanes of this lane
end
Base.show(io::IO, tag::LaneTag) = @printf(io, "LaneTag(%d, %d)", tag.segment, tag.lane)
const NULL_LANETAG = LaneTag(0,0)

#######################################

"""
    RoadIndex{I <: Integer, T <: Real}
A data structure to index points in a roadway. Calling `roadway[roadind]` will return the point
associated to the road index.
# Fields 
- `ind::CurveIndex{I,T}` the index of the point in the curve
- `tag::LaneTag` the lane tag of the point
"""
struct RoadIndex{I <: Integer, T <: Real}
    ind::CurveIndex{I, T}
    tag::LaneTag
end
const NULL_ROADINDEX = RoadIndex(CurveIndex(-1,NaN), LaneTag(-1,-1))

Base.show(io::IO, r::RoadIndex) = @printf(io, "RoadIndex({%d, %3.f}, {%d, %d})", r.ind.i, r.ind.t, r.tag.segment, r.tag.lane)
Base.write(io::IO, r::RoadIndex) = @printf(io, "%d %.6f %d %d", r.ind.i, r.ind.t, r.tag.segment, r.tag.lane)

#######################################

"""
    LaneConnection{I <: Integer, T <: Real}
Data structure to specify the connection of a lane. It connects `mylane` to the point `target`. 
`target` would typically be the starting point of a new lane.
- `downstream::Bool`
- `mylane::CurveIndex{I,T}`
- `target::RoadIndex{I,T}`
"""
struct LaneConnection{I <: Integer, T <: Real}
    downstream::Bool # if true, mylane → target, else target → mylane
    mylane::CurveIndex{I, T}
    target::RoadIndex{I, T}
end

Base.show(io::IO, c::LaneConnection) = print(io, "LaneConnection(", c.downstream ? "D" : "U", ", ", c.mylane, ", ", c.target)
function Base.write(io::IO, c::LaneConnection)
    @printf(io, "%s (%d %.6f) ", c.downstream ? "D" : "U", c.mylane.i, c.mylane.t)
    write(io, c.target)
end
function Base.parse(::Type{LaneConnection}, line::AbstractString)
    cleanedline = replace(line, r"(\(|\))" => "")
    tokens = split(cleanedline, ' ')

    @assert(tokens[1] == "D" || tokens[1] == "U")
    downstream = tokens[1] == "D"
    mylane = CurveIndex(parse(Int, tokens[2]), parse(Float64, tokens[3]))
    target = RoadIndex(
                CurveIndex(parse(Int, tokens[4]), parse(Float64, tokens[5])),
                LaneTag(parse(Int, tokens[6]), parse(Int, tokens[7]))
            )
    LaneConnection{Int64, Float64}(downstream, mylane, target)
end

#######################################

const DEFAULT_LANE_WIDTH = 3.0 # [m]

"""
    Lane 
A driving lane on a roadway. It identified by a `LaneTag`. A lane is defined by a curve which
represents a center line and a width. In addition it has attributed like speed limit. 
A lane can be connected to other lane in the roadway, the connection are specified in the exits
and entrances fields.

# Fields
- `tag::LaneTag`
- `curve::Curve`
- `width::Float64`  [m]
- `speed_limit::SpeedLimit`
- `boundary_left::LaneBoundary`
- `boundary_right::LaneBoundary`
- `exits::Vector{LaneConnection} # list of exits; put the primary exit (at end of lane) first`
- `entrances::Vector{LaneConnection} # list of entrances; put the primary entrance (at start of lane) first`
"""
mutable struct Lane{T <: Real} 
    tag            :: LaneTag
    curve          :: Curve{T}
    width          :: Float64 # [m]
    speed_limit    :: SpeedLimit
    boundary_left  :: LaneBoundary
    boundary_right :: LaneBoundary
    exits          :: Vector{LaneConnection{Int64, T}} # list of exits; put the primary exit (at end of lane) first
    entrances      :: Vector{LaneConnection{Int64, T}} # list of entrances; put the primary entrance (at start of lane) first
end
function Lane(
    tag::LaneTag,
    curve::Curve{T};
    width::Float64 = DEFAULT_LANE_WIDTH,
    speed_limit::SpeedLimit = DEFAULT_SPEED_LIMIT,
    boundary_left::LaneBoundary = NULL_BOUNDARY,
    boundary_right::LaneBoundary = NULL_BOUNDARY,
    exits::Vector{LaneConnection{Int64, T}} = LaneConnection{Int64,T}[],
    entrances::Vector{LaneConnection{Int64, T}} = LaneConnection{Int64,T}[],
    next::RoadIndex=NULL_ROADINDEX,
    prev::RoadIndex=NULL_ROADINDEX,
    ) where T

    lane = Lane{T}(tag, 
                curve,
                width,
                speed_limit,
                boundary_left,
                boundary_right,
                exits,
                entrances)

    if next != NULL_ROADINDEX
        pushfirst!(lane.exits, LaneConnection(true, curveindex_end(lane.curve), next))
    end
    if prev != NULL_ROADINDEX
        pushfirst!(lane.entrances, LaneConnection(false, CURVEINDEX_START, prev))
    end

    return lane
end

"""
    has_next(lane::Lane)
returns true if the end of the lane is connected to another lane (i.e. if it has an exit lane)
"""
has_next(lane::Lane) = !isempty(lane.exits) && lane.exits[1].mylane == curveindex_end(lane.curve)

"""
    has_prev(lane::Lane)
returns true if another lane is connected to the beginning of that lane. (i.e. if it has an entrance lane)
"""
has_prev(lane::Lane) = !isempty(lane.entrances) && lane.entrances[1].mylane == CURVEINDEX_START

"""
    is_in_exits(lane::Lane, target::LaneTag)
returns true if `target` is in the exit lanes of `lane`.
"""
is_in_exits(lane::Lane, target::LaneTag) = findfirst(lc->lc.target.tag == target, lane.exits) != nothing

"""
    is_in_entrances(lane::Lane, target::LaneTag)
returns true if `target` is in the entrances lanes of `lane`.
"""
is_in_entrances(lane::Lane, target::LaneTag) = findfirst(lc->lc.target.tag == target, lane.entrances) != nothing

"""
    connect!(source::Lane, dest::Lane)
connect two lanes to each other. Useful for roadway construction.
"""
function connect!(source::Lane, dest::Lane)
    # place these at the front

    cindS = curveindex_end(source.curve)
    cindD = CURVEINDEX_START

    pushfirst!(source.exits,   LaneConnection(true,  cindS, RoadIndex(cindD, dest.tag)))
    pushfirst!(dest.entrances, LaneConnection(false, cindD, RoadIndex(cindS, source.tag)))
    (source, dest)
end
function connect!(source::Lane, cindS::CurveIndex, dest::Lane, cindD::CurveIndex)
    push!(source.exits,   LaneConnection(true,  cindS, RoadIndex(cindD, dest.tag)))
    push!(dest.entrances, LaneConnection(false, cindD, RoadIndex(cindS, source.tag)))
    (source, dest)
end

#######################################
"""
    RoadSegment{T}
a list of lanes forming a single road with a common direction
# Fields
- `id::Int64`
- `lanes::Vector{Lane{T}}` lanes are stored right to left
"""
mutable struct RoadSegment{T<:Real}
    id::Int64
    lanes::Vector{Lane{T}} 
end
RoadSegment{T}(id::Int64) where T = RoadSegment{T}(id, Lane{T}[])

#######################################

"""
    Roadway
The main datastructure to represent road network, it consists of a list of `RoadSegment`
# Fields
- `segments::Vector{RoadSegment}`
"""
mutable struct Roadway{T<:Real}
    segments::Vector{RoadSegment{T}}
end
Roadway{T}() where T = Roadway{T}(RoadSegment{T}[]) 
Roadway() = Roadway{Float64}()

Base.show(io::IO, roadway::Roadway) = @printf(io, "Roadway")
"""
    Base.write(io::IO, ::MIME"text/plain", roadway::Roadway)
write all the roadway information to a text file
"""
function Base.write(io::IO, ::MIME"text/plain", roadway::Roadway)
    # writes to a text file

    println(io, "ROADWAY")
    println(io, length(roadway.segments)) # number of segments
    for seg in roadway.segments
        println(io, seg.id)
        println(io, "\t", length(seg.lanes)) # number of lanes
        for (i,lane) in enumerate(seg.lanes)
            @assert(lane.tag.lane == i)
            @printf(io, "\t%d\n", i)
            @printf(io, "\t\t%.3f\n", lane.width)
            @printf(io, "\t\t%.3f %.3f\n", lane.speed_limit.lo, lane.speed_limit.hi)
            println(io, "\t\t", lane.boundary_left.style, " ", lane.boundary_left.color)
            println(io, "\t\t", lane.boundary_right.style, " ", lane.boundary_right.color)
            println(io, "\t\t", length(lane.exits) + length(lane.entrances))
            for conn in lane.exits
                print(io, "\t\t\t"); write(io, conn); print(io, "\n")
            end
            for conn in lane.entrances
                print(io, "\t\t\t"); write(io, conn); print(io, "\n")
            end
            println(io, "\t\t", length(lane.curve))
            for pt in lane.curve
                @printf(io, "\t\t\t(%.4f %.4f %.6f) %.4f %.8f %.8f\n", pt.pos.x, pt.pos.y, pt.pos.θ, pt.s, pt.k, pt.kd)
            end
        end
    end
end

"""
    Base.read(io::IO, ::MIME"text/plain", ::Type{Roadway})
extract roadway information from a text file and returns a roadway object.
"""
function Base.read(io::IO, ::MIME"text/plain", ::Type{Roadway})
    lines = readlines(io)
    line_index = 1
    if occursin("ROADWAY", lines[line_index])
        line_index += 1
    end

    function advance!()
        line = strip(lines[line_index])
        line_index += 1
        line
    end

    nsegs = parse(Int, advance!())
    roadway = Roadway{Float64}(Array{RoadSegment{Float64}}(undef, nsegs))
    for i_seg in 1:nsegs
        segid = parse(Int, advance!())
        nlanes = parse(Int, advance!())
        seg = RoadSegment(segid, Array{Lane{Float64}}(undef, nlanes))
        for i_lane in 1:nlanes
            @assert(i_lane == parse(Int, advance!()))
            tag = LaneTag(segid, i_lane)
            width = parse(Float64, advance!())

            tokens = split(advance!(), ' ')
            speed_limit = SpeedLimit(parse(Float64, tokens[1]), parse(Float64, tokens[2]))

            tokens = split(advance!(), ' ')
            boundary_left = LaneBoundary(Symbol(tokens[1]), Symbol(tokens[2]))

            tokens = split(advance!(), ' ')
            boundary_right = LaneBoundary(Symbol(tokens[1]), Symbol(tokens[2]))

            exits = LaneConnection{Int64, Float64}[]
            entrances = LaneConnection{Int64, Float64}[]
            n_conns = parse(Int, advance!())
            for i_conn in 1:n_conns
                conn = parse(LaneConnection, advance!())
                conn.downstream ? push!(exits, conn) : push!(entrances, conn)
            end

            npts = parse(Int, advance!())
            curve = Array{CurvePt{Float64}}(undef, npts)
            for i_pt in 1:npts
                line = advance!()
                cleanedline = replace(line, r"(\(|\))" => "")
                tokens = split(cleanedline, ' ')
                x = parse(Float64, tokens[1])
                y = parse(Float64, tokens[2])
                θ = parse(Float64, tokens[3])
                s = parse(Float64, tokens[4])
                k = parse(Float64, tokens[5])
                kd = parse(Float64, tokens[6])
                curve[i_pt] = CurvePt(VecSE2(x,y,θ), s, k, kd)
            end

            seg.lanes[i_lane] = Lane(tag, curve, width=width, speed_limit=speed_limit,
                                     boundary_left=boundary_left,
                                     boundary_right=boundary_right,
                                     entrances=entrances, exits=exits)
        end
        roadway.segments[i_seg] = seg
    end

    roadway
end


"""
    lane[ind::CurveIndex, roadway::Roadway]
Accessor for lanes based on a CurveIndex.
Note that we extend the definition of a CurveIndex,
previously ind.i ∈ [1, length(curve)-1], to:

    ind.i ∈ [0, length(curve)]

where 1 ≤ ind.i ≤ length(curve)-1 is as before, but if the index
is on the section between two lanes, we use:

    ind.i = length(curve), ind.t ∈ [0,1] for the region between curve[end] → next
    ind.i = 0,             ind.t ∈ [0,1] for the region between prev → curve[1]
"""
function Base.getindex(lane::Lane{T}, ind::CurveIndex{I, T}, roadway::Roadway{T}) where {I<:Integer,T<:Real}
    if ind.i == 0
        pt_lo = prev_lane_point(lane, roadway)
        pt_hi = lane.curve[1]
        s_gap = norm(VecE2(pt_hi.pos - pt_lo.pos))
        pt_lo = CurvePt{T}(pt_lo.pos, -s_gap, pt_lo.k, pt_lo.kd)
        return lerp(pt_lo, pt_hi, ind.t)
    elseif ind.i < length(lane.curve)
        return lane.curve[ind]
    else
        pt_hi = next_lane_point(lane, roadway)
        pt_lo = lane.curve[end]
        s_gap = norm(VecE2(pt_hi.pos - pt_lo.pos))
        pt_hi = CurvePt{T}(pt_hi.pos, pt_lo.s + s_gap, pt_hi.k, pt_hi.kd)
        return lerp( pt_lo, pt_hi, ind.t)
    end
end

"""
    Base.getindex(roadway::Roadway, segid::Int)
returns the segment associated with id `segid`
"""
function Base.getindex(roadway::Roadway, segid::Int)
    for seg in roadway.segments
        if seg.id == segid
            return seg
        end
    end
    error("Could not find segid $segid in roadway")
end

"""
    Base.getindex(roadway::Roadway, tag::LaneTag)
returns the lane identified by the tag `LaneTag`
"""
function Base.getindex(roadway::Roadway, tag::LaneTag)
    seg = roadway[tag.segment]
    seg.lanes[tag.lane]
end

"""
    is_between_segments_lo(ind::CurveIndex)
"""
is_between_segments_lo(ind::CurveIndex) = ind.i == 0

"""
    is_between_segments_hi(ind::CurveIndex, curve::Curve)
"""
is_between_segments_hi(ind::CurveIndex, curve::Curve) = ind.i == length(curve)

"""
    is_between_segments(ind::CurveIndex, curve::Curve)
"""
is_between_segments(ind::CurveIndex, curve::Curve) = is_between_segments_lo(ind) || is_between_segments_hi(ind, curve)

"""
    next_lane(lane::Lane, roadway::Roadway)
returns the lane connected to the end `lane`. If `lane` has several exits, it returns the first one
"""
next_lane(lane::Lane, roadway::Roadway) = roadway[lane.exits[1].target.tag]

"""
    prev_lane(lane::Lane, roadway::Roadway)
returns the lane connected to the beginning `lane`. If `lane` has several entrances, it returns the first one
"""
prev_lane(lane::Lane, roadway::Roadway) = roadway[lane.entrances[1].target.tag]

"""
    next_lane_point(lane::Lane, roadway::Roadway) 
returns the point of connection between `lane` and its first exit
"""
next_lane_point(lane::Lane, roadway::Roadway) = roadway[lane.exits[1].target]

"""
    prev_lane_point(lane::Lane, roadway::Roadway)
returns the point of connection between `lane` and its first entrance
"""
prev_lane_point(lane::Lane, roadway::Roadway) = roadway[lane.entrances[1].target]

"""
    has_segment(roadway::Roadway, segid::Int)
returns true if `segid` is in `roadway`.
"""
function has_segment(roadway::Roadway, segid::Int)
    for seg in roadway.segments
        if seg.id == segid
            return true
        end
    end
    false
end

"""
    has_lanetag(roadway::Roadway, tag::LaneTag)
returns true if `roadway` contains a lane identified by `tag`
"""
function has_lanetag(roadway::Roadway, tag::LaneTag)
    if !has_segment(roadway, tag.segment)
        return false
    end
    seg = roadway[tag.segment]
    1 ≤ tag.lane ≤ length(seg.lanes)
end

"""
    RoadProjection{I <: Integer, T <: Real}
represents the projection of a point on the roadway
# Fields
- `curveproj::CurveProjection{I, T}`
- `tag::LaneTag`
"""
struct RoadProjection{I <: Integer, T <: Real}
    curveproj::CurveProjection{I, T}
    tag::LaneTag
end

function get_closest_perpendicular_point_between_points(A::VecSE2{T}, B::VecSE2{T}, Q::VecSE2{T};
    tolerance::T = 0.01, # acceptable error in perpendicular component
    max_iter::Int = 50, # maximum number of iterations
    ) where T

    # CONDITIONS: a < b, either f(a) < 0 and f(b) > 0 or f(a) > 0 and f(b) < 0
    # OUTPUT: value which differs from a root of f(x)=0 by less than TOL

    a = convert(T, 0.0)
    b = convert(T, 1.0)

    f_a = inertial2body(Q, A).x
    f_b = inertial2body(Q, B).x

    if sign(f_a) == sign(f_b) # both are wrong - use the old way
        t = get_lerp_time_unclamped(A, B, Q)
        t = clamp(t, 0.0, 1.0)
        return (t, lerp(A,B,t))
    end

    iter = 1
    while iter ≤ max_iter
        c = (a+b)/2 # new midpoint
        footpoint = lerp(A, B, c)
        f_c = inertial2body(Q, footpoint).x
        if abs(f_c) < tolerance # solution found
            return (c, footpoint)
        end
        if sign(f_c) == sign(f_a)
            a, f_a = c, f_c
        else
            b = c
        end
        iter += 1
    end

    # Maximum number of iterations passed
    # This will occur when we project with a point that is not actually in the range,
    # and we converge towards one edge

    if a == 0.0
        return (a, A)
    elseif b == 1.0
        return (b, B)
    else
        warn("get_closest_perpendicular_point_between_points - should not happen")
        c = (a+b)/2 # should not happen
        return (c, lerp(A,B,c))
    end
end

"""
    proj(posG::VecSE2{T}, lane::Lane, roadway::Roadway; move_along_curves::Bool=true) where T <: Real
Return the RoadProjection for projecting posG onto the lane.
This will automatically project to the next or prev curve as appropriate.
if `move_along_curves` is false, will only project to lane.curve
"""
function Vec.proj(posG::VecSE2{T}, lane::Lane{T}, roadway::Roadway{T};
    move_along_curves::Bool = true, # if false, will only project to lane.curve
    ) where T <: Real
    curveproj = proj(posG, lane.curve)
    rettag = lane.tag

    if curveproj.ind == CurveIndex(1,zero(T)) && has_prev(lane)
        pt_lo = prev_lane_point(lane, roadway)
        pt_hi = lane.curve[1]

        t = get_lerp_time_unclamped(pt_lo, pt_hi, posG)
        if t ≤ 0.0 && move_along_curves
            return proj(posG, prev_lane(lane, roadway), roadway)
        elseif t < 1.0 # for t == 1.0 we use the actual end of the lane
            @assert(!move_along_curves || 0.0 ≤ t < 1.0)

            # t was computed assuming a constant angle
            # this is not valid for the large distances and angle disparities between lanes
            # thus we now use a bisection search to find the appropriate location

            t, footpoint = get_closest_perpendicular_point_between_points(pt_lo.pos, pt_hi.pos, posG)

            ind = CurveIndex(0, t)
            curveproj = get_curve_projection(posG, footpoint, ind)
        end
    elseif curveproj.ind == curveindex_end(lane.curve) && has_next(lane)
        pt_lo = lane.curve[end]
        pt_hi = next_lane_point(lane, roadway)

        t = get_lerp_time_unclamped(pt_lo, pt_hi, posG)

        if t ≥ 1.0 && move_along_curves
             # for t == 1.0 we use the actual start of the lane
            return proj(posG, next_lane(lane, roadway), roadway)
        elseif t ≥ 0.0
            @assert(!move_along_curves || 0.0 ≤ t ≤ 1.0)

            # t was computed assuming a constant angle
            # this is not valid for the large distances and angle disparities between lanes
            # thus we now use a bisection search to find the appropriate location

            t, footpoint = get_closest_perpendicular_point_between_points(pt_lo.pos, pt_hi.pos, posG)

            ind = CurveIndex(length(lane.curve), t)
            curveproj = get_curve_projection(posG, footpoint, ind)
        end
    end
    RoadProjection(curveproj, rettag)
end

"""
    proj(posG::VecSE2{T}, seg::RoadSegment, roadway::Roadway) where T <: Real
Return the RoadProjection for projecting posG onto the segment.
Tries all of the lanes and gets the closest one
"""
function Vec.proj(posG::VecSE2{T}, seg::RoadSegment, roadway::Roadway) where T <: Real

    best_dist2 = Inf
    best_proj = RoadProjection{Int64, T}(CurveProjection(CurveIndex(-1,convert(T, -1)), convert(T, NaN), convert(T, NaN)),
                               NULL_LANETAG)

    for lane in seg.lanes
        roadproj = proj(posG, lane, roadway)
        footpoint = roadway[roadproj.tag][roadproj.curveproj.ind, roadway]
        dist2 = norm(VecE2(posG - footpoint.pos))
        if dist2 < best_dist2
            best_dist2 = dist2
            best_proj = roadproj
        end
    end

    best_proj
end

"""
    proj(posG::VecSE2{T}, seg::RoadSegment, roadway::Roadway) where T <: Real
Return the RoadProjection for projecting posG onto the roadway.
Tries all of the lanes and gets the closest one
"""
function Vec.proj(posG::VecSE2{T}, roadway::Roadway) where T <: Real

    best_dist2 = Inf
    best_proj = RoadProjection(CurveProjection(CurveIndex(-1,convert(T,-1.0)), 
                                               convert(T, NaN), 
                                               convert(T, NaN)),
                                NULL_LANETAG)

    for seg in roadway.segments
        for lane in seg.lanes
            roadproj = proj(posG, lane, roadway, move_along_curves=false)
            targetlane = roadway[roadproj.tag]
            footpoint = targetlane[roadproj.curveproj.ind, roadway]
            dist2 = normsquared(VecE2(posG - footpoint.pos))
            if dist2 < best_dist2
                best_dist2 = dist2
                best_proj = roadproj
            end
        end
    end

    best_proj
end

############################################

RoadIndex(roadproj::RoadProjection) = RoadIndex(roadproj.curveproj.ind, roadproj.tag)

"""
    Base.getindex(roadway::Roadway, roadind::RoadIndex)
returns the CurvePt on the roadway associated to `roadind`
"""
function Base.getindex(roadway::Roadway, roadind::RoadIndex)
    lane = roadway[roadind.tag]
    lane[roadind.ind, roadway]
end

"""
    move_along(roadind::RoadIndex, road::Roadway, Δs::Float64)
Return the RoadIndex at ind's s position + Δs
"""
function move_along(roadind::RoadIndex{I, T}, 
                    roadway::Roadway, 
                    Δs::Float64, 
                    depth::Int=0) where {I <: Integer, T <: Real}

    lane = roadway[roadind.tag]
    curvept = lane[roadind.ind, roadway]

    if curvept.s + Δs < 0.0
        if has_prev(lane)
            pt_lo = prev_lane_point(lane, roadway)
            pt_hi = lane.curve[1]
            s_gap = norm(VecE2(pt_hi.pos - pt_lo.pos))

            if curvept.s + Δs < -s_gap
                lane_prev = prev_lane(lane, roadway)
                curveind = curveindex_end(lane_prev.curve)
                roadind = RoadIndex{I,T}(curveind, lane_prev.tag)
                return move_along(roadind, roadway, Δs + curvept.s + s_gap, depth+1)
            else # in the gap between lanes
                t = (s_gap + curvept.s + Δs) / s_gap
                curveind = CurveIndex(0, t)
                RoadIndex{I,T}(curveind, lane.tag)
            end

        else # no prev lane, return the beginning of this one
            curveind = CurveIndex(1, 0.0)
            return RoadIndex{I,T}(curveind, roadind.tag)
        end
    elseif curvept.s + Δs > lane.curve[end].s
        if has_next(lane)
            pt_lo = lane.curve[end]
            pt_hi = next_lane_point(lane, roadway)
            s_gap = norm(VecE2(pt_hi.pos - pt_lo.pos))

            if curvept.s + Δs ≥ pt_lo.s + s_gap # extends beyond the gap
                curveind = lane.exits[1].target.ind
                roadind = RoadIndex{I,T}(curveind, lane.exits[1].target.tag)
                return move_along(roadind, roadway, Δs - (lane.curve[end].s + s_gap - curvept.s))
            else # in the gap between lanes
                t = (Δs - (lane.curve[end].s - curvept.s)) / s_gap
                curveind = CurveIndex(0, t)
                RoadIndex{I,T}(curveind, lane.exits[1].target.tag)
            end
        else # no next lane, return the end of this lane
            curveind = curveindex_end(lane.curve)
            return RoadIndex{I,T}(curveind, roadind.tag)
        end
    else
        if roadind.ind.i == 0
            ind = get_curve_index(CurveIndex(1,0.0), lane.curve, curvept.s+Δs)
        elseif roadind.ind.i == length(lane.curve)
            ind = get_curve_index(curveindex_end(lane.curve), lane.curve, curvept.s+Δs)
        else
            ind = get_curve_index(roadind.ind, lane.curve, Δs)
        end
        RoadIndex{I,T}(ind, roadind.tag)
    end
end

"""
    n_lanes_right(lane::Lane, roadway::Roadway)
returns the number of lanes to the right of `lane`
"""
n_lanes_right(lane::Lane, roadway::Roadway) = lane.tag.lane - 1

"""
    n_lanes_left(lane::Lane, roadway::Roadway)
returns the number of lanes to the left of `lane`
"""
function n_lanes_left(lane::Lane, roadway::Roadway)
    seg = roadway[lane.tag.segment]
    length(seg.lanes) - lane.tag.lane
end

