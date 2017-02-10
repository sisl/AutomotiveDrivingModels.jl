export
        RoadType,
        roadtype_unknown,
        roadtype_rural,
        roadtype_motorway,
        roadtype_town,
        roadtype_low_speed,
        roadtype_pedestrian,
        roadtype_bicycle,

        RoadTypeRecord,
        RoadLinkRecord,

        LaneGeometry,
        LaneGeometries,
        LaneGeometryRecord,
        GeomLine,
        GeomArc,

        Road,
        n_successors,
        n_predecessors


@enum RoadType roadtype_unknown=0 roadtype_rural=1 roadtype_motorway=2 roadtype_town=3 roadtype_low_speed=4 roadtype_pedestrian=5 roadtype_bicycle=6

const DEFAULT_OFFSET_TOLERANCE = 1e-8

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

    function SpacingRecord(offset, s::Float64=0.0)
        s ≥ 0 || throw(ArgumentError("SpacingRecord starting s must be nonnegative"))
        new(s, offset)
    end
end

function _get_offset(O::CubicOffset, Δs::Float64)
    a, b, c, d = O.a, O.b, O.c, O.d
    return a + Δs*(b + Δs*(c + Δs*d))
end
function _get_offset(O::PiecewiseOffset, Δs::Float64)
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
    )

    new(s, marktype, weight, color)
end

@enum LaneType lanetype_unknown=0

immutable LaneSpeedRecord
    s::Float64 # start position relative to the position of the preceeding LaneSection
    speed_limit::Float64 # [m/s]
end

type Lane
    lanetype::LaneType
    lanetype::LaneType
    border::RoadMark
    border_offset::Vector{OffsetRecord} # distance from ref to marking; for lane 0 this is the offset of the center
    speeds::Vector{LaneSpeedRecord}
    next::Int # id of next lane, 0 if none
    prev::Int # id of prev lane, 0 if none
    # TODO: material type / friction ?
    # TODO: access record?
    # TODO: rule record?
end

type LaneSection
    s::Float64 # start position [m]
    left_lanes::Vector{Lane} # positive id
    right_lanes::Vector{Lane} # negative id
    center::Lane # just used for its type?
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

    return Road(name, 0.0, id, RoadTypeRecord[], RoadLinkRecord[], RoadLinkRecord[], LaneGeometryRecord[])
end

Base.length(road::Road) = road.length

"""
    get(road, RoadTypeRecord, s)

Obtain the type record valid for the given s ∈ [0, road.length]
Any s values out of bounds will be clamped
"""
function Base.get(road::Road, ::Type{RoadTypeRecord}, s::Float64)
    N = length(road.types)
    @assert N ≥ 1
    i = 1
    while i < N && road.types[i+1].s ≤ s
        i += 1
    end
    return road.types[i]
end

"""
    get_offset_at(road, s)

Returns the
"""
function Base.get(road::Road, ::Type{RoadTypeRecord}, s::Float64)
    N = length(road.types)
    @assert N ≥ 1
    i = 1
    while i < N && road.types[i+1].s ≤ s
        i += 1
    end
    return road.types[i]
end

n_successors(road::Road) = length(road.successors)
n_predecessors(road::Road) = length(road.predecessors)

"""
    get(road, LaneGeometryRecord, s)

Obtain the LaneGeometryRecord for the reference line containing s ∈ [0, road.length]
Any s values out of bounds will be clamped
"""
function Base.get(road::Road, ::Type{LaneGeometryRecord}, s::Float64)
    N = length(road.refline)
    @assert N ≥ 1
    i = 1
    while i < N && road.refline[i+1].s ≤ s
        i += 1
    end
    return road.refline[i]
end

"""
    get(road, VecSE2, s)

Obtain the position in global coordinates for the reference line containing s ∈ [0, road.length]
Any s values out of bounds will be clamped
"""
Base.get(road::Road, ::Type{VecSE2}, s::Float64) = get(get(road, LaneGeometryRecord, s), s)

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