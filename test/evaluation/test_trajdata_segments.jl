let
    trajdata = get_test_trajdata()
    seg = TrajdataSegment(1, 1, 1, 2)
    show(seg)

    @test seg == TrajdataSegment(1, 1, 1, 2)
    @test nsteps(seg) == 1
    @test nframes(seg) == 2
end