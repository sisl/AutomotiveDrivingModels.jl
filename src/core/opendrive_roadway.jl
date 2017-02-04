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
        LaneGeometryRecord(VecSE2(1.0,0.0,0.0), π/4, GeomArc(1.0), 1.0),
        LaneGeometryRecord(VecSE2(2.0,1.0,π/4), 1.0, GeomLine(), 1.0+π/4),
    ]
    road = Road("", 2+π/4, zero(UInt32), RoadTypeRecord[], RoadLinkRecord[], RoadLinkRecord[], lane_geometry_records)
    @test isapprox(get(road, VecSE2, 0.0), VecSE2(0.0,0.0,0.0))
    @test isapprox(get(road, VecSE2, 0.5), VecSE2(0.5,0.0,0.0))
    @test isapprox(get(road, VecSE2, 1.0), VecSE2(1.0,0.0,0.0))
    # NOT CONVINCED ABOUT THIS TEST!
    # println(get(road, VecSE2, 1.0+π/8))
    # println(VecSE2(1.0+cos(π/8),0.0+sin(π/8),π/8))
    # @test isapprox(get(road, VecSE2, 1.0+π/8), VecSE2(1.0+cos(π/8),0.0+sin(π/8),π/8), atol=1e-6)
end
