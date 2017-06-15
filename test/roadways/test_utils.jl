let
    lb = LaneBoundary(:solid, :yellow)
    sl = SpeedLimit(0.0, 5.0)
end

let
    tag = LaneTag(1, 2)
    @test tag.segment == 1
    @test tag.lane == 2
    @test LaneTag(1,2) == tag
    @test LaneTag(2,1) != tag
end