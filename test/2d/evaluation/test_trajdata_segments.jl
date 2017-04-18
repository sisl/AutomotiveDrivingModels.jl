let
    roadway = get_test_roadway()
    trajdata = get_test_trajdata(roadway)
    seg = TrajdataSegment(1, 1, 1, 2)
    show(IOBuffer(), seg)

    @test seg == TrajdataSegment(1, 1, 1, 2)
    @test nsteps(seg) == 1
    @test nframes(seg) == 2

    srand(0)
    for i in 1 : 10
        seg2 = sample_random_subinterval(seg, 1)
        @test nframes(seg2) == 1
        @test seg2.frame_lo ≥ seg.frame_lo
        @test seg2.frame_hi ≤ seg.frame_hi
        @test seg2.egoid == seg.egoid
        @test seg2.trajdata_index == seg.trajdata_index
    end

    segments = pull_continuous_segments(trajdata, 999)
    @test TrajdataSegment(999, 1, 1, 2) in segments
    @test TrajdataSegment(999, 2, 1, 2) in segments
end