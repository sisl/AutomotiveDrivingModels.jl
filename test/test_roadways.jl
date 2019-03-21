@testset "1d roadway" begin 
    roadway = StraightRoadway(20.0)
    s = 10.0 
    @test mod_position_to_roadway(s, roadway) == s
    s = 25.0 
    @test mod_position_to_roadway(s, roadway) == 5.0 
    s = 45.0
    @test mod_position_to_roadway(s, roadway) == 5.0
    s = -5.0 
    @test mod_position_to_roadway(s, roadway) == 15.0
    s_rear = 10.0 
    s_fore = 15.0
    @test get_headway(s_rear, s_fore, roadway) == 5.0
    s_fore = 25.0
    @test get_headway(s_rear, s_fore, roadway) == 15.0
    s_fore = 5.0 
    @test get_headway(s_rear, s_fore, roadway) == 15.0
end