using Vec
using Base.Test

@enum RoadType roadtype_unknown=0 roadtype_rural=1 roadtype_motorway=2 roadtype_town=3 roadtype_low_speed=4 roadtype_pedestrian=5 roadtype_bicycle=6

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
function Base.get(lanegeo::LaneGeometryRecord{GeomLine}, s::Float64)
    Δs = s - lanegeo.s
    0 ≤ Δs ≤ lanegeo.length || throw(DomainError())
    θ = lanegeo.posG.θ
    return lanegeo.posG + polar(Δs, θ)
end
function Base.get(lanegeo::LaneGeometryRecord{GeomArc}, s::Float64)
    Δs = s - lanegeo.s
    0 ≤ Δs ≤ lanegeo.length || throw(DomainError())
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

let
    @test_throws ArgumentError LaneGeometryRecord(VecSE2(NaN,0.0,0.0), 1.0, GeomLine())
    @test_throws ArgumentError LaneGeometryRecord(VecSE2(Inf,0.0,0.0), 1.0, GeomLine())
    @test_throws ArgumentError LaneGeometryRecord(VecSE2(0.0,0.0,0.0), NaN, GeomLine())
    @test_throws ArgumentError LaneGeometryRecord(VecSE2(0.0,0.0,0.0),-1.0, GeomLine())
    @test_throws ArgumentError LaneGeometryRecord(VecSE2(0.0,0.0,0.0), 1.0, GeomLine(), -1.0)
    @test_throws ArgumentError LaneGeometryRecord(VecSE2(0.0,0.0,0.0), 1.0, GeomLine(),  NaN)
    @test_throws ArgumentError LaneGeometryRecord(VecSE2(0.0,0.0,0.0), 1.0, GeomLine(),  Inf)

    lanegeo = LaneGeometryRecord(VecSE2(0.0,0.0,0.0), 1.0, GeomLine())
    @test isapprox(get(lanegeo, 0.0), VecSE2(0.0,0.0,0.0))
    @test isapprox(get(lanegeo, 1.0), VecSE2(1.0,0.0,0.0))
    @test isapprox(get(lanegeo, 0.2), VecSE2(0.2,0.0,0.0))
    @test_throws DomainError get(lanegeo, 1.2)

    lanegeo = LaneGeometryRecord(VecSE2(1.0,1.0,0.0), 1.0, GeomLine())
    @test isapprox(get(lanegeo, 0.0), VecSE2(1.0,1.0,0.0))
    @test isapprox(get(lanegeo, 1.0), VecSE2(2.0,1.0,0.0))
    @test isapprox(get(lanegeo, 0.2), VecSE2(1.2,1.0,0.0))

    θ = 1.0
    lanegeo = LaneGeometryRecord(VecSE2(1.0,1.0,θ), 1.0, GeomLine())
    @test isapprox(get(lanegeo, 0.0), VecSE2(1.0,1.0,θ))
    @test isapprox(get(lanegeo, 1.0), VecSE2(1.0+cos(θ),1.0+sin(θ),θ))
    @test isapprox(get(lanegeo, 0.2), VecSE2(1.0+0.2cos(θ),1.0+0.2sin(θ),θ))

    lanegeo = LaneGeometryRecord(VecSE2(0.0,0.0,0.0), 1.0, GeomArc(0.0))
    @test isapprox(get(lanegeo, 0.0), VecSE2(0.0,0.0,0.0))
    @test isapprox(get(lanegeo, 1.0), VecSE2(1.0,0.0,0.0))
    @test isapprox(get(lanegeo, 0.2), VecSE2(0.2,0.0,0.0))
    @test_throws DomainError get(lanegeo, 1.2)

    lanegeo = LaneGeometryRecord(VecSE2(1.0,1.0,0.0), 1.0, GeomArc(0.0))
    @test isapprox(get(lanegeo, 0.0), VecSE2(1.0,1.0,0.0))
    @test isapprox(get(lanegeo, 1.0), VecSE2(2.0,1.0,0.0))
    @test isapprox(get(lanegeo, 0.2), VecSE2(1.2,1.0,0.0))

    θ = 1.0
    lanegeo = LaneGeometryRecord(VecSE2(1.0,1.0,θ), 1.0, GeomArc(0.0))
    @test isapprox(get(lanegeo, 0.0), VecSE2(1.0,1.0,θ))
    @test isapprox(get(lanegeo, 1.0), VecSE2(1.0+cos(θ),1.0+sin(θ),θ))
    @test isapprox(get(lanegeo, 0.2), VecSE2(1.0+0.2cos(θ),1.0+0.2sin(θ),θ))

    lanegeo = LaneGeometryRecord(VecSE2(1.0,1.0,θ), 1.0, GeomArc(1.0))
    @test isapprox(get(lanegeo, 0.0), VecSE2(1.0,1.0,θ))
    @test isapprox(get(lanegeo, 1.0), VecSE2(1.068, 1.956, 2.000), atol=1e-3)
    @test isapprox(get(lanegeo, 0.2), VecSE2(1.091, 1.178, 1.200), atol=1e-3)

    lanegeo = LaneGeometryRecord(VecSE2(1.0,1.0,θ), 1.0, GeomArc(-1.0))
    @test isapprox(get(lanegeo, 0.0), VecSE2(1.0,1.0,θ))
    @test isapprox(get(lanegeo, 1.0), VecSE2(1.841, 1.460, 0.000), atol=1e-3)
    @test isapprox(get(lanegeo, 0.2), VecSE2(1.124, 1.156, 0.800), atol=1e-3)
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

let
    lanegeo = LaneGeometryRecord(VecSE2(0.0,0.0,0.0), 1.0, GeomLine())
    (s, ⟂) = proj(VecSE2(0.0,0.0), lanegeo, Float64); @test isapprox(s,0.0); @test ⟂
    (s, ⟂) = proj(VecSE2(0.0,-1.0), lanegeo, Float64); @test isapprox(s,0.0); @test ⟂
    (s, ⟂) = proj(VecSE2(0.0, 1.0), lanegeo, Float64); @test isapprox(s,0.0); @test ⟂
    (s, ⟂) = proj(VecSE2(0.25, 0.0), lanegeo, Float64); @test isapprox(s,0.25); @test ⟂
    (s, ⟂) = proj(VecSE2(0.25,-1.0), lanegeo, Float64); @test isapprox(s,0.25); @test ⟂
    (s, ⟂) = proj(VecSE2(0.25, 1.0), lanegeo, Float64); @test isapprox(s,0.25); @test ⟂
    (s, ⟂) = proj(VecSE2(1.0, 0.0), lanegeo, Float64); @test isapprox(s,1.00); @test ⟂
    (s, ⟂) = proj(VecSE2(1.0,-1.0), lanegeo, Float64); @test isapprox(s,1.00); @test ⟂
    (s, ⟂) = proj(VecSE2(1.0, 1.0), lanegeo, Float64); @test isapprox(s,1.00); @test ⟂
    (s, ⟂) = proj(VecE2(1.25, 0.0), lanegeo, Float64); @test isapprox(s,1.00); @test !⟂
    (s, ⟂) = proj(VecE2(1.25,-1.0), lanegeo, Float64); @test isapprox(s,1.00); @test !⟂

    lanegeo = LaneGeometryRecord(VecSE2(1.0,2.0,π/4), 2.0, GeomLine())
    (s, ⟂) = proj(VecSE2(3.0,2.0), lanegeo, Float64); @test isapprox(s,sqrt(2)); @test ⟂
    (s, ⟂) = proj(VecSE2(1.0,4.0), lanegeo, Float64); @test isapprox(s,sqrt(2)); @test ⟂

    lanegeo = LaneGeometryRecord(VecSE2(0.0,0.0,0.0), π/2, GeomArc(1.0))
    (s, ⟂) = proj(VecSE2(0.0,0.0), lanegeo, Float64); @test isapprox(s,0.0); @test ⟂
    (s, ⟂) = proj(VecSE2(0.0,-1.0), lanegeo, Float64); @test isapprox(s,0.0); @test ⟂
    (s, ⟂) = proj(VecSE2(0.0, 1.0), lanegeo, Float64); @test isapprox(s,0.0); @test ⟂
    (s, ⟂) = proj(VecSE2(0.0, 2.0), lanegeo, Float64); @test isapprox(s,π/2); @test !⟂
    (s, ⟂) = proj(VecSE2(-1.0, -2.0), lanegeo, Float64); @test isapprox(s,0.0); @test !⟂
    (s, ⟂) = proj(VecSE2(0.0,1.0) + polar(1.1, 0.4-π/2), lanegeo, Float64); @test isapprox(s,0.4); @test ⟂
    (s, ⟂) = proj(VecSE2(0.0,1.0) + polar(0.9, 0.4-π/2), lanegeo, Float64); @test isapprox(s,0.4); @test ⟂
end

type Road
    name::String
    length::Float64
    id::UInt32 # starting at 0
    types::Vector{RoadTypeRecord} # in order
    predecessors::Vector{RoadLinkRecord}
    successors::Vector{RoadLinkRecord}
    refline::Vector{LaneGeometryRecord}
    # sections::Vector{LaneSection}
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

let
    types =  [RoadTypeRecord(0.0, roadtype_unknown, 1.0),
              RoadTypeRecord(1.0, roadtype_unknown, 2.0),
              RoadTypeRecord(5.0, roadtype_unknown, 3.0),
              RoadTypeRecord(7.0, roadtype_unknown, 4.0),
             ]
    road = Road("", NaN, zero(UInt32), types, RoadLinkRecord[], RoadLinkRecord[], LaneGeometryRecord[])
    @test get(road, RoadTypeRecord,-1.0) === types[1]
    @test get(road, RoadTypeRecord, 0.0) === types[1]
    @test get(road, RoadTypeRecord, 0.5) === types[1]
    @test get(road, RoadTypeRecord, 1.5) === types[2]
    @test get(road, RoadTypeRecord, 5.0) === types[3]
    @test get(road, RoadTypeRecord, 5.5) === types[3]
    @test get(road, RoadTypeRecord, 7.0) === types[4]
    @test get(road, RoadTypeRecord, 7.5) === types[4]
end

let
    lane_geometry_records = [
        LaneGeometryRecord(VecSE2(0.0,0.0,0.0), 1.0, GeomLine()),
        LaneGeometryRecord(VecSE2(1.0,0.0,0.0), π/2, GeomArc(1.0), 1.0),
        LaneGeometryRecord(VecSE2(2.0,1.0,π/2), 1.0, GeomLine(), 1.0+π/2),
    ]
    road = Road("", 2+π/2, zero(UInt32), RoadTypeRecord[], RoadLinkRecord[], RoadLinkRecord[], lane_geometry_records)
    @test isapprox(get(road, VecSE2, 0.0), VecSE2(0.0,0.0,0.0))
    @test isapprox(get(road, VecSE2, 0.5), VecSE2(0.5,0.0,0.0))
    @test isapprox(get(road, VecSE2, 1.0), VecSE2(1.0,0.0,0.0))
    @test isapprox(get(road, VecSE2, 1.0+π/4), VecSE2(1.0+cos(π/4),1-sin(π/4),π/4), atol=1e-6)

    # this tests push - should get the same as above
    road = Road()
    push!(road, lane_geometry_records[1])
    push!(road, lane_geometry_records[2].length, lane_geometry_records[2].geo)
    push!(road, lane_geometry_records[3].length, lane_geometry_records[3].geo)
    @test isapprox(get(road, VecSE2, 0.0), VecSE2(0.0,0.0,0.0))
    @test isapprox(get(road, VecSE2, 0.5), VecSE2(0.5,0.0,0.0))
    @test isapprox(get(road, VecSE2, 1.0), VecSE2(1.0,0.0,0.0))
    @test isapprox(get(road, VecSE2, 1.0+π/4), VecSE2(1.0+cos(π/4),1-sin(π/4),π/4), atol=1e-6)
end