let
    roadway = Wraparound(Straight1DRoadway(10.0))
    @test get_headway(1.0, 2.0, roadway) == 1.0
    @test get_headway(1.0, 1.0, roadway) == 0.0
    @test get_headway(2.0, 1.0, roadway) == 9.0
    @test get_headway(1.0, 10.5, roadway) == 9.5
    @test get_headway(1.0, 11.5, roadway) == 0.5
    @test get_headway(1.0, -8.0, roadway) == 1.0

    @test mod_position_to_roadway(0.0, roadway) == 0.0
    @test mod_position_to_roadway(1.0, roadway) == 1.0
    @test mod_position_to_roadway(-1.0, roadway) == 9.0
    @test mod_position_to_roadway(11.0, roadway) == 1.0
end

let
    curve = get_test_curve1()
    roadway = Wraparound(curve)
    @test get_headway(1.0, 2.0, roadway) == 1.0
    @test get_headway(1.0, 1.0, roadway) == 0.0
    @test get_headway(2.0, 1.0, roadway) == 2.0
    @test get_headway(1.0, 3.5, roadway) == 2.5
    @test get_headway(1.0, 4.5, roadway) == 0.5
    @test get_headway(1.0, -1.0, roadway) == 1.0

    @test mod_position_to_roadway(0.0,roadway) == 0.0
    @test mod_position_to_roadway(1.0,roadway) == 1.0
    @test mod_position_to_roadway(-1.0,roadway) == 2.0
    @test mod_position_to_roadway(4.0,roadway) == 1.0
end