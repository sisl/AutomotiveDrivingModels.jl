
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
    posG::Float64 # start position in global coordinates (x,y,θ)
    s::Float64 # start position (s-coordinate)
    length::Float64 # length of the element's reference line
    geo::G
end
Base.length(lanegeo::LaneGeometryRecord) = lanegeo.length

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

using Base.Test

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
