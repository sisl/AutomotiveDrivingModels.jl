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