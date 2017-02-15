export
        RoadType,
        roadtype_unknown,
        roadtype_rural,
        roadtype_motorway,
        roadtype_town,
        roadtype_low_speed,
        roadtype_pedestrian,
        roadtype_bicycle,

        RoadMarkType,
        roadmarktype_none,
        roadmarktype_solid,
        roadmarktype_broken,
        roadmarktype_solid_solid,
        roadmarktype_solid_broken,
        roadmarktype_broken_solid,
        roadmarktype_broken_broken,
        roadmarktype_botts_dots,
        roadmarktype_grass,
        roadmarktype_curb,

        RoadMarkWeight,
        roadmarkweight_standard,
        roadmarkweight_bold,

        LaneType,
        lanetype_none,
        lanetype_driving,
        lanetype_stop,
        lanetype_shoulder,
        lanetype_biking,
        lanetype_sidewalk,
        lanetype_border,
        lanetype_restricted,
        lanetype_parking,
        lanetype_bidirectional,
        lanetype_median,
        lanetype_special1,
        lanetype_special2,
        lanetype_special3,
        lanetype_roadworks,
        lanetype_tram,
        lanetype_rail,
        lanetype_entry,
        lanetype_exit,
        lanetype_offramp,
        lanetype_onramp,

        RoadTypeRecord,
        RoadLinkRecord,

        LaneGeometry,
        LaneGeometries,
        LaneGeometryRecord,
        GeomLine,
        GeomArc,

        Offset,
        CubicOffset,
        PiecewiseOffset,
        Offsets,
        OffsetRecord,

        DEFAULT_MARK_WIDTH,
        RoadMark,
        LaneSpeedRecord,
        Lane,
        LaneSection,

        n_lanes_left,
        n_lanes_right,

        Road,
        n_successors,
        n_predecessors,
        get_border_point,
        get_center_point


@enum RoadType roadtype_unknown=0 roadtype_rural=1 roadtype_motorway=2 roadtype_town=3 roadtype_low_speed=4 roadtype_pedestrian=5 roadtype_bicycle=6

const DEFAULT_OFFSET_TOLERANCE = 1e-8

function _get_based_on_s(sequential::Vector, s::Real)
    # NOTE: only works on things with an 's' field
    N = length(sequential)
    @assert N ≥ 1
    i = 1
    while i < N && sequential[i+1].s ≤ s
        i += 1
    end
    return sequential[i]
end

immutable RoadTypeRecord
    s::Float64 # starting location
    roadtype::RoadType
    speed_limit::Float64 # [m/s]
end
function RoadTypeRecord(;
    s::Float64 = 0.0,
    roadtype::RoadType = roadtype_unknown,
    speed_limit::Float64 = NaN,
    )
    RoadTypeRecord(s, roadtype, speed_limit)
end

immutable RoadLinkRecord
    id::UInt32 # ID of the linked element
    at_start::Bool # whether it contacts at the start (or end otherwise)
end

abstract LaneGeometry
immutable GeomLine <: LaneGeometry
end
immutable GeomArc <: LaneGeometry
    curvature::Float64 # [1/m], constant curvature throughout the element
                       # positive indicates a left-hand turn
end
typealias LaneGeometries Union{GeomLine,GeomArc}

immutable LaneGeometryRecord{G<:LaneGeometries}
    posG::VecSE2 # start position in global coordinates (x,y,θ)
    length::Float64 # length of the element's reference line
    geo::G
    s::Float64 # start position (s-coordinate)

    function LaneGeometryRecord(posG::VecSE2, length::Float64, geo, s::Float64)
        isfinite(posG) || throw(ArgumentError("LaneGeometryRecord posG must be finite"))
        isfinite(length) || throw(ArgumentError("LaneGeometryRecord length must be finite"))
        length > 0  || throw(ArgumentError("LaneGeometryRecord length must be positive"))
        isfinite(s) || throw(ArgumentError("LaneGeometryRecord s must be finite"))
        s ≥ 0 || throw(ArgumentError("LaneGeometryRecord starting s must be nonnegative"))
        new(posG, length, geo, s)
    end
end
LaneGeometryRecord{G<:LaneGeometries}(posG::VecSE2, length::Real, geo::G) = LaneGeometryRecord{G}(posG, convert(Float64, length), geo, 0.0)
LaneGeometryRecord{G<:LaneGeometries}(posG::VecSE2, length::Real, geo::G, s::Real) = LaneGeometryRecord{G}(posG, convert(Float64, length), geo, convert(Float64, s))


Base.length(lanegeo::LaneGeometryRecord) = lanegeo.length

"""
    get(lanegeo, s)

Returns the VecSE2 in the global frame corresponding to the given s.
Note: s is relative to the start of the overall reference line
Throws an error if s is out of bounds
"""
function Base.get(lanegeo::LaneGeometryRecord{GeomLine}, s::Float64; offset_tol::Float64 = DEFAULT_OFFSET_TOLERANCE)
    Δs = s - lanegeo.s
    -offset_tol ≤ Δs ≤ lanegeo.length + offset_tol || throw(DomainError())
    θ = lanegeo.posG.θ
    return lanegeo.posG + polar(Δs, θ)
end
function Base.get(lanegeo::LaneGeometryRecord{GeomArc}, s::Float64; offset_tol::Float64 = DEFAULT_OFFSET_TOLERANCE)
    Δs = s - lanegeo.s
    -offset_tol ≤ Δs ≤ lanegeo.length + offset_tol || throw(DomainError())
    κ = lanegeo.geo.curvature
    P = lanegeo.posG
    θ = P.θ
    if isapprox(κ, 0.0)
        # use a straight segment
        return P + polar(Δs, θ)
    else
        r = 1 / κ
        C = P + polar(r, θ + π/2) # center of rotation
        ϕ = Δs / r # rotation about C
        return C + polar(r, θ+ϕ-π/2, ϕ)
    end
end

"""
    proj(posG, lanegeo, Float64)

Returns a tuple (s,⟂) where
s is the s-value of the closest footpoint of posG onto the lanegeo.
    It is the absolute s value, ie not necessarily 0 at the start of the lanegeo

⟂ is true if the projection posG is perpendicular to the lane geo (it is false when off either end).
    If posG is exactly on the line it is considered perpendicular.
"""
function Vec.proj(posG::Union{VecE2, VecSE2}, lanegeo::LaneGeometryRecord{GeomLine}, ::Type{Float64};
    # perp_tolerance::Float64=1e-8,
    )

    isfinite(posG) || throw(ArgumentError("posG must be finite"))

    C = convert(VecE2, lanegeo.posG) # origin
    P = convert(VecE2, posG)
    D = P - C # 2D position wrt to start of lanegeo
    Q = polar(1.0, lanegeo.posG.θ) # line unit ray vector
    Δs = proj(D, Q, Float64) # get distance along

    if Δs < 0.0
        return (lanegeo.s, false)
    elseif Δs > lanegeo.length
        return (lanegeo.s + lanegeo.length, false)
    else
        # F = C+Δs*Q # footpoint
        # ⟂ = isapprox(dot(Q, P-F), 0.0, atol=perp_tolerance) # whether it is perpendicular
        return (lanegeo.s + Δs, true)
    end
end
function Vec.proj(posG::Union{VecE2, VecSE2}, lanegeo::LaneGeometryRecord{GeomArc}, ::Type{Float64};
    # perp_tolerance::Float64=1e-8,
    critical_tolerance::Float64=1e-8,
    warn_on_critical::Bool=false,
    )

    isfinite(posG) || throw(ArgumentError("posG must be finite"))

    κ = lanegeo.geo.curvature
    if isapprox(κ, 0.0) # treat it as a line
        return proj(posG, LaneGeometryRecord(lanegeo.posG, lanegeo.length, GeomLine(), lanegeo.s))
    end

    r = 1/κ

    O = convert(VecE2, lanegeo.posG) # origin
    P = convert(VecE2, posG)
    C = O + polar(r, lanegeo.posG.θ + π/2)
    CP = P-C

    if isapprox(abs2(CP), 0.0, atol=critical_tolerance) # on the C point
        # This is a critical location, and any ψ is valid
        # We will default to s = lanegeo.s
        return (lanegeo.s, true)
    end

    ψ₀ = atan2(O-C)
    ψ = atan2(CP) - ψ₀ # directed angle from O to P about C
    Δs = r*ψ

    if Δs < 0.0
        return (lanegeo.s, false)
    elseif Δs > lanegeo.length
        return (lanegeo.s + lanegeo.length, false)
    else
        # F = C + polar(r, ψ + ψ₀) # footpoint
        # ⟂ = isapprox(dot(Q, P-F), 0.0, atol=perp_tolerance) # whether it is perpendicular
        return (lanegeo.s + Δs, true)
    end


    Δs = proj(D, Q, Float64) # get distance along
end

abstract Offset
immutable CubicOffset <: Offset
    # offset = a + b⋅Δs + c⋅Δs² + d⋅Δs³
    a::Float64
    b::Float64
    c::Float64
    d::Float64
end
immutable PiecewiseOffset <: Offset
    # TODO THIS
end
typealias Offsets Union{CubicOffset,PiecewiseOffset}
immutable OffsetRecord{O<:Offsets}
    s::Float64 # start position (s-coordinate)
    offset::O
end

function _get_offset(O::CubicOffset, Δs::Float64)
    a, b, c, d = O.a, O.b, O.c, O.d
    return a + Δs*(b + Δs*(c + Δs*d))
end
function _get_offset(O::PiecewiseOffset, Δs::Float64)
    error("NOT IMPLEMENTED")
end

function _get_orientation(O::CubicOffset, Δs::Float64)
    b, c, d = O.b, O.c, O.d
    dt = b + Δs*(2c + Δs*3d)
    atan2(dt, Δs)
end
function _get_orientation(O::PiecewiseOffset, Δs::Float64)
    error("NOT IMPLEMENTED")
end

@enum RoadMarkType roadmarktype_none=0 roadmarktype_solid=1 roadmarktype_broken=2 roadmarktype_solid_solid=3 roadmarktype_solid_broken=4 roadmarktype_broken_solid=5 roadmarktype_broken_broken=6 roadmarktype_botts_dots=7 roadmarktype_grass=8 roadmarktype_curb=9
@enum RoadMarkWeight roadmarkweight_standard=0 roadmarkweight_bold=1
@enum LaneType lanetype_none=0 lanetype_driving=1 lanetype_stop=2 lanetype_shoulder=3 lanetype_biking=4 lanetype_sidewalk=5 lanetype_border=6 lanetype_restricted=7 lanetype_parking=8 lanetype_bidirectional=9 lanetype_median=10 lanetype_special1=11 lanetype_special2=12 lanetype_special3=13 lanetype_roadWorks=14 lanetype_tram=15 lanetype_rail=16 lanetype_entry=17 lanetype_exit=18 lanetype_offramp=19 lanetype_onramp=20

const DEFAULT_MARK_WIDTH = 0.1 # [m]

immutable RoadMark
    s::Float64 # start position relative to the position of the preceeding LaneSection
    marktype::RoadMarkType
    weight::RoadMarkWeight # (the center of the marking is on the border)
    color::Colorant # typically white, blue, green, red, or yellow
    width::Float64 # [m]
    allows_lanechange_pos::Bool # allows lane changes in direction of positive ids
    allows_lanechange_neg::Bool # allows lane changes in direction of negative ids
end
function RoadMark(s::Float64;
    marktype::RoadMarkType = roadmarktype_solid,
    weight::RoadMarkWeight = roadmarkweight_standard,
    color::Colorant = colorant"white",
    width::Float64 = DEFAULT_MARK_WIDTH,
    allows_lanechange_pos::Bool = false,
    allows_lanechange_neg::Bool = false,
    )

    RoadMark(s, marktype, weight, color, width, allows_lanechange_pos, allows_lanechange_neg)
end

immutable LaneSpeedRecord
    s::Float64 # start position relative to the position of the preceeding LaneSection
    speed_limit::Float64 # [m/s]
end

type Lane
    lanetype::LaneType
    borders::Vector{RoadMark}
    border_offsets::Vector{OffsetRecord} # distance from refline to marking; for lane 0 this is the offset of the center
    speeds::Vector{LaneSpeedRecord}
    next::Int # id of next lane, 0 if none
    prev::Int # id of prev lane, 0 if none
    # TODO: material type / friction ?
    # TODO: access record?
    # TODO: rule record?
end

type LaneSection
    center::Lane
    lanes_left::Vector{Lane} # positive id
    lanes_right::Vector{Lane} # negative id
    s::Float64 # start position [m]
end
function LaneSection(center::Lane;
    lanes_left::Vector{Lane} = Lane[],
    lanes_right::Vector{Lane} = Lane[],
    s::Float64 = 0.0,
    )

    return LaneSection(center, lanes_left, lanes_right, s)
end

n_lanes_left(section::LaneSection) = length(section.lanes_left)
n_lanes_right(section::LaneSection) = length(section.lanes_right)

"""
    section[id]

Returns the Lane for the given id.
Positive ids are left, negative ids right, and 0 is the center.
"""
function Base.getindex(section::LaneSection, id::Int)
    if id == 0
        return section.center
    elseif id > 0
        return section.lanes_left[id]
    else
        return section.lanes_right[-id]
    end
end

"""
    get_offset(lane, s)

Returns the offset of the lane outer border at s.
Note that the returned t already has the correct sign:
    postive indicates towards the left
"""
function get_offset(lane::Lane, s::Float64)
    border_offset = _get_based_on_s(lane.border_offsets, s) # get the relevant offset
    return _get_offset(border_offset.offset, s)
end
function get_orientation(lane::Lane, s::Float64)
    border_offset = _get_based_on_s(lane.border_offsets, s)
    return _get_orientation(border_offset.offset, s)
end
function get_offset_and_orientation(lane::Lane, s::Float64)
    border_offset = _get_based_on_s(lane.border_offsets, s)
    t = _get_offset(border_offset.offset, s)
    θ = _get_orientation(border_offset.offset, s)
    return (t,θ)
end


type Road
    name::String
    length::Float64
    id::UInt32 # starting at 0
    types::Vector{RoadTypeRecord} # in order
    predecessors::Vector{RoadLinkRecord}
    successors::Vector{RoadLinkRecord}
    refline::Vector{LaneGeometryRecord}
    sections::Vector{LaneSection}
end
function Road(;
    name="UNNAMED",
    id::UInt32=zero(UInt32),
    )

    return Road(name, 0.0, id, RoadTypeRecord[], RoadLinkRecord[], RoadLinkRecord[], LaneGeometryRecord[], LaneSection[])
end

Base.length(road::Road) = road.length

"""
    get(road, RoadTypeRecord, s)

Obtain the type record valid for the given s ∈ [0, road.length]
Any s values out of bounds will be clamped
"""
Base.get(road::Road, ::Type{RoadTypeRecord}, s::Float64) = _get_based_on_s(road.types, s)

"""
    get(road, LaneGeometryRecord, s)

Obtain the LaneGeometryRecord for the reference line containing s ∈ [0, road.length]
Any s values out of bounds will be clamped
"""
Base.get(road::Road, ::Type{LaneGeometryRecord}, s::Float64) = _get_based_on_s(road.refline, s)

"""
    get(road, LaneSection, s)

Obtain the LaneSection containing s ∈ [0, road.length]
Any s values out of bounds will be clamped
"""
Base.get(road::Road, ::Type{LaneSection}, s::Float64) = _get_based_on_s(road.sections, s)

n_successors(road::Road) = length(road.successors)
n_predecessors(road::Road) = length(road.predecessors)


"""
    get(road, VecSE2, s)

Obtain the position in global coordinates for the reference line containing s ∈ [0, road.length]
Any s values out of bounds will be clamped
"""
Base.get(road::Road, ::Type{VecSE2}, s::Float64) = get(get(road, LaneGeometryRecord, s), s)

"""
    get_border_point(road, section, lane, s)

Obtain the position in global coordinates for the lane border containing at s
Any s values out of bounds will be clamped
"""
function get_border_point(road::Road, section::LaneSection, lane::Lane, s::Float64)
    t,θ = get_offset_and_orientation(lane, section.s - s)
    P = get(road, VecSE2, s) # footpoint
    return P + polar(t, P.θ + π/2, θ)
end

"""
    get_center_point(road, section, lane, s)

Obtain the position in global coordinates for the lane centerline containing at s
Any s values out of bounds will be clamped
"""
function get_center_point(road::Road, section::LaneSection, laneid::Int, s::Float64)
    if laneid == 0
        return get_border_point(road, section, section[laneid], s)
    elseif laneid > 0
        outer = get_border_point(road, section, section[laneid], s)
        inner = get_border_point(road, section, section[laneid-1], s)
        return lerp(inner, outer, 0.5)
    else
        outer = get_border_point(road, section, section[laneid], s)
        inner = get_border_point(road, section, section[laneid+1], s)
        return lerp(inner, outer, 0.5)
    end
end

"""
    push!(road, lanegeo)

Append the lane geometry record to the reference line and increment the road's length as appropriate
"""
function Base.push!(road::Road, lanegeo::LaneGeometryRecord)
    @assert isapprox(lanegeo.s, road.length) # starts at end
    road.length += lanegeo.length # add to total road length
    push!(road.refline, lanegeo)
    return road
end

"""
    push!(road, length, geo)

Construct and append the lane geometry record to the reference line and increment the road's length as appropriate
Only works for non-empty reference lines
"""
function Base.push!{G<:LaneGeometries}(road::Road, length::Float64, geo::G)
    !isempty(road.refline) || error("Cannot append to an empty road refline as we need a starting position")
    posG = get(road, VecSE2, road.length) # starting pos is end of road
    lanegeo = LaneGeometryRecord(posG, length, geo, road.length)
    return push!(road, lanegeo)
end