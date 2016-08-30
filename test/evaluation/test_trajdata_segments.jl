let
    trajdata = get_test_trajdata()
    seg = TrajdataSegment(1, 1, 1, 2)
    show(IOBuffer(), seg)

    @test seg == TrajdataSegment(1, 1, 1, 2)
    @test nsteps(seg) == 1
    @test nframes(seg) == 2

    segments = pull_continuous_segments(trajdata, 999)
    @test TrajdataSegment(999, 1, 1, 2) in segments
    @test TrajdataSegment(999, 2, 1, 2) in segments
end