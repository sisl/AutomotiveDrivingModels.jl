let
    trajdatas = [get_test_trajdata(), get_test_trajdata()]
    segments = [TrajdataSegment(1, 2, 1, 2), TrajdataSegment(1, 2, 1, 2)]
    evaldata = EvaluationData(trajdatas, segments)

    evaldata2 = create_evaldata_with_random_subsegments_of_equal_length(evaldata, nsegs=2, nframes=1)
    @test length(evaldata2.segments) == 2
    @test evaldata2.trajdatas === evaldata.trajdatas
    for seg in evaldata2.segments
        @test nframes(seg) == 1
    end

    segments = [TrajdataSegment(1, 2, 1, 2), TrajdataSegment(3, 4, 10, 20)]
    evaldata = EvaluationData(trajdatas, segments)
    evaldata2 = create_evaldata_with_random_subsegments_of_equal_length(evaldata, foldset_match([1,2], 1), nsegs=2, nframes=1)
    @test length(evaldata2.segments) == 2
    @test evaldata2.trajdatas === evaldata.trajdatas
    @test evaldata2.segments[1].trajdata_index == evaldata.segments[1].trajdata_index
    @test evaldata2.segments[1].egoid == evaldata.segments[1].egoid
    @test evaldata2.segments[2].trajdata_index == evaldata.segments[1].trajdata_index
    @test evaldata2.segments[2].egoid == evaldata.segments[1].egoid

    metrics = TraceMetricExtractor[RootWeightedSquareError(SPEED, 2.0)]
    metric_df = allocate_metrics_dataframe(metrics, 10)
    @test ncol(metric_df) == 3
    @test nrow(metric_df) == 10
    @test names(metric_df) == [:time, :logl, symbol(metrics[1])]

    rec = pull_record(segments[1], evaldata)
    @test get_scene(rec, 0)[1].def == get_vehicle(trajdatas[1], 1, 2).def
    @test get_scene(rec, 0)[1].state.posG == get_vehicle(trajdatas[1], 1, 2).state.posG
end