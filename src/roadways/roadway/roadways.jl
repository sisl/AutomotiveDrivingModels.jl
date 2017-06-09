
immutable LaneTag
    segment :: Int # segment id
    lane    :: Int # index in segment.lanes of this lane
end
Base.show(io::IO, tag::LaneTag) = @printf(io, "LaneTag(%d, %d)", tag.segment, tag.lane)
hash(id::LaneTag, h::UInt=zero(UInt)) = hash(id.segment, hash(id.lane, h))
Base.:(==)(a::LaneTag, b::LaneTag) = a.segment == b.segment && a.lane == b.lane
const NULL_LANETAG = LaneTag(0,0)

#######################################

immutable RoadIndex
    ind::CurveIndex
    tag::LaneTag
end
const NULL_ROADINDEX = RoadIndex(CurveIndex(-1,NaN), LaneTag(-1,-1))

Base.show(io::IO, r::RoadIndex) = @printf(io, "RoadIndex({%d, %3.f}, {%d, %d})", r.ind.i, r.ind.t, r.tag.segment, r.tag.lane)
Base.write(io::IO, r::RoadIndex) = @printf(io, "%d %.6f %d %d", r.ind.i, r.ind.t, r.tag.segment, r.tag.lane)

#######################################

immutable LaneConnection
    downstream::Bool # if true, mylane → target, else target → mylane
    mylane::CurveIndex
    target::RoadIndex
end

Base.show(io::IO, c::LaneConnection) = print(io, "LaneConnection(", c.downstream ? "D" : "U", ", ", c.mylane, ", ", c.target)
function Base.write(io::IO, c::LaneConnection)
    @printf(io, "%s (%d %.6f) ", c.downstream ? "D" : "U", c.mylane.i, c.mylane.t)
    write(io, c.target)
end
function Base.parse(::Type{LaneConnection}, line::String)
    cleanedline = replace(line, r"(\(|\))", "")
    tokens = split(cleanedline, ' ')

    @assert(tokens[1] == "D" || tokens[1] == "U")
    downstream = tokens[1] == "D"
    mylane = CurveIndex(parse(Int, tokens[2]), parse(Float64, tokens[3]))
    target = RoadIndex(
                CurveIndex(parse(Int, tokens[4]), parse(Float64, tokens[5])),
                LaneTag(parse(Int, tokens[6]), parse(Int, tokens[7]))
            )
    LaneConnection(downstream, mylane, target)
end

#######################################

const DEFAULT_LANE_WIDTH = 3.0 # [m]

type Lane
    tag            :: LaneTag
    curve          :: Curve
    width          :: Float64 # [m]
    speed_limit    :: SpeedLimit
    boundary_left  :: LaneBoundary
    boundary_right :: LaneBoundary
    exits          :: Vector{LaneConnection} # list of exits; put the primary exit (at end of lane) first
    entrances      :: Vector{LaneConnection} # list of entrances; put the primary entrance (at start of lane) first

    function Lane(
        tag::LaneTag,
        curve::Curve;
        width::Float64 = DEFAULT_LANE_WIDTH,
        speed_limit::SpeedLimit = DEFAULT_SPEED_LIMIT,
        boundary_left::LaneBoundary = NULL_BOUNDARY,
        boundary_right::LaneBoundary = NULL_BOUNDARY,
        exits::Vector{LaneConnection} = LaneConnection[],
        entrances::Vector{LaneConnection} = LaneConnection[],
        next::RoadIndex=NULL_ROADINDEX,
        prev::RoadIndex=NULL_ROADINDEX,
        )

        retval = new()
        retval.tag = tag
        retval.curve = curve
        retval.width = width
        retval.speed_limit = speed_limit
        retval.boundary_left = boundary_left
        retval.boundary_right = boundary_right
        retval.exits = exits
        retval.entrances = entrances

        if next != NULL_ROADINDEX
            unshift!(retval.exits, LaneConnection(true, curveindex_end(retval.curve), next))
        end
        if prev != NULL_ROADINDEX
            unshift!(retval.entrances, LaneConnection(false, CURVEINDEX_START, prev))
        end

        retval
    end
end

has_next(lane::Lane) = !isempty(lane.exits) && lane.exits[1].mylane == curveindex_end(lane.curve)
has_prev(lane::Lane) = !isempty(lane.entrances) && lane.entrances[1].mylane == CURVEINDEX_START

is_in_exits(lane::Lane, target::LaneTag) = findfirst(lc->lc.target.tag == target, lane.exits) != 0
is_in_entrances(lane::Lane, target::LaneTag) = findfirst(lc->lc.target.tag == target, lane.entrances) != 0


function connect!(source::Lane, dest::Lane)
    # place these at the front

    cindS = curveindex_end(source.curve)
    cindD = CURVEINDEX_START

    unshift!(source.exits,   LaneConnection(true,  cindS, RoadIndex(cindD, dest.tag)))
    unshift!(dest.entrances, LaneConnection(false, cindD, RoadIndex(cindS, source.tag)))
    (source, dest)
end
function connect!(source::Lane, cindS::CurveIndex, dest::Lane, cindD::CurveIndex)
    push!(source.exits,   LaneConnection(true,  cindS, RoadIndex(cindD, dest.tag)))
    push!(dest.entrances, LaneConnection(false, cindD, RoadIndex(cindS, source.tag)))
    (source, dest)
end

#######################################

type RoadSegment
    # a list of lanes forming a single road with a common direction
    id::Int
    lanes::Vector{Lane} # lanes are stored right to left
    RoadSegment(id::Int, lanes::Vector{Lane}=Lane[]) = new(id, lanes)
end

#######################################

type Roadway
    segments::Vector{RoadSegment}
    Roadway(segments::Vector{RoadSegment}=RoadSegment[]) = new(segments)
end

Base.show(io::IO, roadway::Roadway) = @printf(io, "Roadway")
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
function Base.read(io::IO, ::MIME"text/plain", ::Type{Roadway})
    lines = readlines(io)
    line_index = 1
    if contains(lines[line_index], "ROADWAY")
        line_index += 1
    end

    function advance!()
        line = strip(lines[line_index])
        line_index += 1
        line
    end

    nsegs = parse(Int, advance!())
    roadway = Roadway(Array(RoadSegment, nsegs))
    for i_seg in 1:nsegs
        segid = parse(Int, advance!())
        nlanes = parse(Int, advance!())
        seg = RoadSegment(segid, Array(Lane, nlanes))
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

            exits = LaneConnection[]
            entrances = LaneConnection[]
            n_conns = parse(Int, advance!())
            for i_conn in 1:n_conns
                conn = parse(LaneConnection, advance!())
                conn.downstream ? push!(exits, conn) : push!(entrances, conn)
            end

            npts = parse(Int, advance!())
            curve = Array(CurvePt, npts)
            for i_pt in 1:npts
                line = advance!()
                cleanedline = replace(line, r"(\(|\))", "")
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
function Base.getindex(lane::Lane, ind::CurveIndex, roadway::Roadway)
    if ind.i == 0
        pt_lo = prev_lane_point(lane, roadway)
        pt_hi = lane.curve[1]
        s_gap = abs(pt_hi.pos - pt_lo.pos)
        pt_lo = CurvePt(pt_lo.pos, -s_gap, pt_lo.k, pt_lo.kd)
        lerp(pt_lo, pt_hi, ind.t)
    elseif ind.i < length(lane.curve)
        lane.curve[ind]
    else
        pt_hi = next_lane_point(lane, roadway)
        pt_lo = lane.curve[end]
        s_gap = abs(pt_hi.pos - pt_lo.pos)
        pt_hi = CurvePt(pt_hi.pos, pt_lo.s + s_gap, pt_hi.k, pt_hi.kd)
        lerp( pt_lo, pt_hi, ind.t)
    end
end
function Base.getindex(roadway::Roadway, segid::Int)
    for seg in roadway.segments
        if seg.id == segid
            return seg
        end
    end
    error("Could not find segid $segid in roadway")
end
function Base.getindex(roadway::Roadway, tag::LaneTag)
    seg = roadway[tag.segment]
    seg.lanes[tag.lane]
end

is_between_segments_lo(ind::CurveIndex) = ind.i == 0
is_between_segments_hi(ind::CurveIndex, curve::Curve) = ind.i == length(curve)
is_between_segments(ind::CurveIndex, curve::Curve) = is_between_segments_lo(ind) || is_between_segments_hi(ind, curve)

next_lane(lane::Lane, roadway::Roadway) = roadway[lane.exits[1].target.tag]
prev_lane(lane::Lane, roadway::Roadway) = roadway[lane.entrances[1].target.tag]
next_lane_point(lane::Lane, roadway::Roadway) = roadway[lane.exits[1].target]
prev_lane_point(lane::Lane, roadway::Roadway) = roadway[lane.entrances[1].target]

function has_segment(roadway::Roadway, segid::Int)
    for seg in roadway.segments
        if seg.id == segid
            return true
        end
    end
    false
end
function has_lanetag(roadway::Roadway, tag::LaneTag)
    if !has_segment(roadway, tag.segment)
        return false
    end
    seg = roadway[tag.segment]
    1 ≤ tag.lane ≤ length(seg.lanes)
end

immutable RoadProjection
    curveproj::CurveProjection
    tag::LaneTag
end

function get_closest_perpendicular_point_between_points(A::VecSE2, B::VecSE2, Q::VecSE2;
    tolerance::Float64 = 0.01, # acceptable error in perpendicular component
    max_iter::Int = 50, # maximum number of iterations
    )

    # CONDITIONS: a < b, either f(a) < 0 and f(b) > 0 or f(a) > 0 and f(b) < 0
    # OUTPUT: value which differs from a root of f(x)=0 by less than TOL

    a = 0.0
    b = 1.0

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
        return (0.0, A)
    elseif b == 1.0
        return (1.0, B)
    else
        warn("get_closest_perpendicular_point_between_points - should not happen")
        c = (a+b)/2 # should not happen
        return (c, lerp(A,B,c))
    end
end

"""
    proj(posG::VecSE2, lane::Lane, roadway::Roadway)
Return the RoadProjection for projecting posG onto the lane.
This will automatically project to the next or prev curve as appropriate.
"""
function Vec.proj(posG::VecSE2, lane::Lane, roadway::Roadway;
    move_along_curves::Bool = true, # if false, will only project to lane.curve
    )
    curveproj = proj(posG, lane.curve)
    rettag = lane.tag

    if curveproj.ind == CurveIndex(1,0.0) && has_prev(lane)
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
    proj(posG::VecSE2, seg::RoadSegment, roadway::Roadway)
Return the RoadProjection for projecting posG onto the segment.
Tries all of the lanes and gets the closest one
"""
function Vec.proj(posG::VecSE2, seg::RoadSegment, roadway::Roadway)

    best_dist2 = Inf
    best_proj = RoadProjection(CurveProjection(CurveIndex(-1,-1), NaN, NaN), NULL_LANETAG)

    for lane in seg.lanes
        roadproj = proj(posG, lane, roadway)
        footpoint = roadway[roadproj.tag][roadproj.curveproj.ind, roadway]
        dist2 = abs2(posG - footpoint.pos)
        if dist2 < best_dist2
            best_dist2 = dist2
            best_proj = roadproj
        end
    end

    best_proj
end

"""
    proj(posG::VecSE2, seg::RoadSegment, roadway::Roadway)
Return the RoadProjection for projecting posG onto the roadway.
Tries all of the lanes and gets the closest one
"""
function Vec.proj(posG::VecSE2, roadway::Roadway)

    best_dist2 = Inf
    best_proj = RoadProjection(CurveProjection(CurveIndex(-1,-1), NaN, NaN), NULL_LANETAG)

    for seg in roadway.segments
        for lane in seg.lanes
            roadproj = proj(posG, lane, roadway, move_along_curves=false)
            targetlane = roadway[roadproj.tag]
            footpoint = targetlane[roadproj.curveproj.ind, roadway]
            dist2 = abs2(posG - footpoint.pos)
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

function Base.getindex(roadway::Roadway, roadind::RoadIndex)
    lane = roadway[roadind.tag]
    lane[roadind.ind, roadway]
end

"""
    move_along(roadind::RoadIndex, road::Roadway, Δs::Float64)
Return the RoadIndex at ind's s position + Δs
"""
function move_along(roadind::RoadIndex, roadway::Roadway, Δs::Float64, depth::Int=0)

    lane = roadway[roadind.tag]
    curvept = lane[roadind.ind, roadway]

    if curvept.s + Δs < 0.0
        if has_prev(lane)
            pt_lo = prev_lane_point(lane, roadway)
            pt_hi = lane.curve[1]
            s_gap = abs(pt_hi.pos - pt_lo.pos)

            if curvept.s + Δs < -s_gap
                lane_prev = prev_lane(lane, roadway)
                curveind = curveindex_end(lane_prev.curve)
                roadind = RoadIndex(curveind, lane_prev.tag)
                return move_along(roadind, roadway, Δs + curvept.s + s_gap, depth+1)
            else # in the gap between lanes
                t = (s_gap + curvept.s + Δs) / s_gap
                curveind = CurveIndex(0, t)
                RoadIndex(curveind, lane.tag)
            end

        else # no prev lane, return the beginning of this one
            curveind = CurveIndex(1, 0.0)
            return RoadIndex(curveind, roadind.tag)
        end
    elseif curvept.s + Δs > lane.curve[end].s
        if has_next(lane)
            pt_lo = lane.curve[end]
            pt_hi = next_lane_point(lane, roadway)
            s_gap = abs(pt_hi.pos - pt_lo.pos)

            if curvept.s + Δs ≥ pt_lo.s + s_gap # extends beyond the gap
                curveind = lane.exits[1].target.ind
                roadind = RoadIndex(curveind, lane.exits[1].target.tag)
                return move_along(roadind, roadway, Δs - (lane.curve[end].s + s_gap - curvept.s))
            else # in the gap between lanes
                t = (Δs - (lane.curve[end].s - curvept.s)) / s_gap
                curveind = CurveIndex(0, t)
                RoadIndex(curveind, lane.exits[1].target.tag)
            end
        else # no next lane, return the end of this lane
            curveind = curveindex_end(lane.curve)
            return RoadIndex(curveind, roadind.tag)
        end
    else
        if roadind.ind.i == 0
            ind = get_curve_index(CurveIndex(1,0.0), lane.curve, curvept.s+Δs)
        elseif roadind.ind.i == length(lane.curve)
            ind = get_curve_index(curveindex_end(lane.curve), lane.curve, curvept.s+Δs)
        else
            ind = get_curve_index(roadind.ind, lane.curve, Δs)
        end
        RoadIndex(ind, roadind.tag)
    end
end


n_lanes_right(lane::Lane, roadway::Roadway) = lane.tag.lane - 1
function n_lanes_left(lane::Lane, roadway::Roadway)
    seg = roadway[lane.tag.segment]
    length(seg.lanes) - lane.tag.lane
end

############################################

function _fit_curve(
    pts::AbstractMatrix{Float64}, # 2×n
    desired_distance_between_samples::Real;
    max_iterations::Int=50,
    epsilon::Float64=1e-4,
    n_intervals_in_arclen::Int=100,
    )

    @assert(size(pts, 1) == 2)

    spline_coeffs = fit_cubic_spline(pts)

    L = calc_curve_length(spline_coeffs[1], spline_coeffs[2], n_intervals_per_segment=n_intervals_in_arclen)
    n = round(Int, L/desired_distance_between_samples, RoundNearestTiesUp)+1

    s_arr = collect(linspace(0.0,L,n))
    t_arr = calc_curve_param_given_arclen(spline_coeffs[1], spline_coeffs[2], s_arr,
        curve_length=L, max_iterations=max_iterations, epsilon=epsilon, n_intervals_in_arclen=n_intervals_in_arclen)

    x_arr = sample_spline(spline_coeffs[1], t_arr)
    y_arr = sample_spline(spline_coeffs[2], t_arr)
    θ_arr = sample_spline_theta(spline_coeffs[1], spline_coeffs[2], t_arr)

    κ_arr = sample_spline_curvature(spline_coeffs[1], spline_coeffs[2], t_arr)
    κd_arr = sample_spline_derivative_of_curvature(spline_coeffs[1], spline_coeffs[2], t_arr)

    @assert(!any(s->isnan(s), s_arr))
    @assert(!any(s->isnan(s), x_arr))
    @assert(!any(s->isnan(s), y_arr))
    @assert(!any(s->isnan(s), θ_arr))

    curve = Array(CurvePt, n)
    for i in 1 : n
        pos = VecSE2(x_arr[i], y_arr[i], θ_arr[i])
        curve[i] = CurvePt(pos, s_arr[i], κ_arr[i], κd_arr[i])
    end
    curve
end

"""
    read_dxf(io::IO, ::Type{Roadway})
Return a Roadway generated from a DXF file

    Layers with names such as seg001 will contain LWPOLYLINEs.
    Each LWPOLYLINE corresponds to a lane centerline, which together
    are all neighbored.
"""
function read_dxf(io::IO, ::Type{Roadway};
    dist_threshold_lane_connect::Float64 = 0.25, # [m]
    desired_distance_between_curve_samples::Float64 = 1.0; # [m]
    )

    lines = readlines(io)

    i = findfirst(lines, "ENTITIES\n")
    i != 0 || error("ENTITIES section not found")

    ###################################################
    # Pull pts for each lane
    lane_pts_dict = Dict{LaneTag, Vector{VecE2}}()

    i = findnext(lines, "LWPOLYLINE\n", i)
    while i != 0
        i = findnext(lines, "  8\n", i)
        if i != 0 # segment identifier found in LWPOLYLINE

            if ismatch(r"(?<=seg)(\d*)", lines[i+1])
                segid = parse(Int, match(r"(?<=seg)(\d*)", lines[i+1]).match)

                    i = findnext(lines, "AcDbPolyline\n", i)
                i != 0 || error("AcDbPolyline not found in LWPOLYLINE!")
                i = findnext(lines, " 90\n", i)
                i != 0 || error("Number of vertices not found in AcDbPolyline!")

                N = parse(Int, lines[i+1])
                N > 0 || error("Empty line segment!")

                pts = Array(VecE2, N)

                i = findnext(lines, " 10\n", i)
                i != 0 || error("First point not found in AcDbPolyline!")

                for j in 1 : N
                    x = parse(Float64, lines[i+1])
                    y = parse(Float64, lines[i+3])
                    i += 4
                    pts[j] = VecE2(x,y)
                end

                laneid = 1
                for tag in keys(lane_pts_dict)
                    if tag.segment == segid
                        laneid += 1
                    end
                end
                lane_pts_dict[LaneTag(segid, laneid)] = pts
            end

            i = findnext(lines, "LWPOLYLINE\n", i)
        end
    end

    ###################################################
    # Shift pts to connect to previous / next pts
    lane_next_dict = Dict{LaneTag, Tuple{VecE2, LaneTag}}()
    lane_prev_dict = Dict{LaneTag, Tuple{VecE2, LaneTag}}()

    for (tag, pts) in lane_pts_dict
        # see if can connect to next
        best_tag = NULL_LANETAG
        best_ind = -1
        best_sq_dist = dist_threshold_lane_connect
        for (tag2, pts2) in lane_pts_dict
            if tag2.segment != tag.segment
                for (ind,pt) in enumerate(pts2)
                    sq_dist = abs2(pt - pts[end])
                    if sq_dist < best_sq_dist
                        best_sq_dist = sq_dist
                        best_ind = ind
                        best_tag = tag2
                    end
                end
            end
        end
        if best_tag != NULL_LANETAG
            # remove our last pt and set next to pt to their pt
            pop!(pts)
            lane_next_dict[tag] = (lane_pts_dict[best_tag][best_ind], best_tag)
            if best_ind == 1 # set connect prev as well
                lane_prev_dict[best_tag] = (pts[end], tag)
            end
        end
    end
    for (tag, pts) in lane_pts_dict
        # see if can connect to prev
        if !haskey(lane_prev_dict, tag)
            best_tag = NULL_LANETAG
            best_ind = -1
            best_sq_dist = dist_threshold_lane_connect
            for (tag2, pts2) in lane_pts_dict
                if tag2.segment != tag.segment
                    for (ind,pt) in enumerate(pts2)
                        sq_dist = abs2(pt - pts[1])
                        if sq_dist < best_sq_dist
                            best_sq_dist = sq_dist
                            best_ind = ind
                            best_tag = tag2
                        end
                    end
                end
            end
            if best_tag != NULL_LANETAG
                # connect 'em
                shift!(pts)
                lane_prev_dict[tag] = (lane_pts_dict[best_tag][best_ind], best_tag)
            end
        end
    end

    ###################################################
    # Build the roadway
    retval = Roadway()
    for (tag, pts) in lane_pts_dict
        if !has_segment(retval, tag.segment)
            push!(retval.segments, RoadSegment(tag.segment))
        end
    end

    lane_new_dict = Dict{LaneTag, LaneTag}() # old -> new tag
    for seg in retval.segments

        # pull lanetags for this seg
        lanetags = LaneTag[]
        for tag in keys(lane_pts_dict)
            if tag.segment == seg.id
                push!(lanetags, tag)
            end
        end

        # sort the lanes such that the rightmost lane is lane 1
        # do this by taking the first lane,
        # then project each lane's midpoint to the perpendicular at the midpoint

        @assert(!isempty(lanetags))
        proj_positions = Array(Float64, length(lanetags))

        first_lane_pts = lane_pts_dict[lanetags[1]]
        n = length(first_lane_pts)
        lo = first_lane_pts[div(n,2)]
        hi = first_lane_pts[div(n,2)+1]
        midpt_orig = (lo + hi)/2
        dir = polar(1.0, atan2(hi - lo) + π/2) # direction perpendicular (left) of lane

        for (i,tag) in enumerate(lanetags)
            pts = lane_pts_dict[tag]
            n = length(pts)
            midpt = (pts[div(n,2)] + pts[div(n,2)+1])/2
            proj_positions[i] = proj(midpt - midpt_orig, dir, Float64)
        end

        for (i,j) in enumerate(sortperm(proj_positions))

            tag = lanetags[j]
            boundary_left = i == length(proj_positions) ? LaneBoundary(:solid, :white) : LaneBoundary(:broken, :white)
            boundary_right = i == 1 ? LaneBoundary(:solid, :white) : LaneBoundary(:broken, :white)

            pts = lane_pts_dict[tag]
            pt_matrix = Array(Float64, 2, length(pts))
            for (k,P) in enumerate(pts)
                pt_matrix[1,k] = P.x
                pt_matrix[2,k] = P.y
            end

            println("fitting curve ", length(pts), "  "); tic()
            curve = _fit_curve(pt_matrix, desired_distance_between_curve_samples)
            toc()

            tag_new = LaneTag(seg.id, length(seg.lanes)+1)
            lane = Lane(tag_new, curve,
                        boundary_left = boundary_left,
                        boundary_right = boundary_right)
            push!(seg.lanes, lane)
            lane_new_dict[tag] = tag_new
        end
    end

    ###################################################
    # Connect the lanes
    for (tag_old, tup) in lane_next_dict
        next_pt, next_tag_old = tup
        lane = retval[lane_new_dict[tag_old]]
        next_tag_new = lane_new_dict[next_tag_old]
        dest = retval[next_tag_new]
        roadproj = proj(VecSE2(next_pt, 0.0), dest, retval)

        # println("connecting $(lane.tag) to $(dest.tag)")

        cindS = curveindex_end(lane.curve)
        cindD = roadproj.curveproj.ind

        if cindD == CURVEINDEX_START # a standard connection
            connect!(lane, dest)
            # remove any similar connection from lane_prev_dict
            if haskey(lane_prev_dict, next_tag_old) && lane_prev_dict[next_tag_old][2] == tag_old
                delete!(lane_prev_dict, next_tag_old)
            end
        else
            # otherwise connect as before
            unshift!(lane.exits,  LaneConnection(true,  cindS, RoadIndex(cindD, dest.tag)))
            push!(dest.entrances, LaneConnection(false, cindD, RoadIndex(cindS, lane.tag)))
        end
    end
    for (tag_old, tup) in lane_prev_dict
        prev_pt, prev_tag_old = tup
        lane = retval[lane_new_dict[tag_old]]
        prev_tag_new = lane_new_dict[prev_tag_old]
        prev = retval[prev_tag_new]
        roadproj = proj(VecSE2(prev_pt, 0.0), prev, retval)

        # println("connecting $(lane.tag) from $(prev.tag)")

        cindS = roadproj.curveproj.ind
        cindD = CURVEINDEX_START

        if cindS == curveindex_end(prev) # a standard connection
            @assert(!has_prev(prev))
            connect!(prev, lane)
        else
            # a standard connection
            push!(prev.exits,  LaneConnection(true,  cindS, RoadIndex(cindD, lane.tag)))
            unshift!(lane.entrances, LaneConnection(false, cindD, RoadIndex(cindS, prev.tag)))
        end
    end

    retval
end

