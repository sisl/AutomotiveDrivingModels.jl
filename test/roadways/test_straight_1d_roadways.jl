let
    roadway = Straight1DRoadway(10.0)
    @test get_headway(1.0, 2.0, roadway) == 1.0
    @test get_headway(1.0, 1.0, roadway) == 0.0
    @test isinf(get_headway(2.0, 1.0, roadway))

    @test_throws ErrorException get_headway(200.0, 1.0, roadway)
    @test_throws ErrorException get_headway(-200.0, 1.0, roadway)
    @test_throws ErrorException get_headway(1.0, 200.0, roadway)
    @test_throws ErrorException get_headway(1.0, -200.0, roadway)
end