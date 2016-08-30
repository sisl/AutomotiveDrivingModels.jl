let
    trajdatas = [get_test_trajdata(), get_test_trajdata()]
    segments = [TrajdataSegment(1, 2, 1, 2), TrajdataSegment(1, 2, 1, 2)]
    evaldata = EvaluationData(trajdatas, segments)

    metrics = TraceMetricExtractor[RootWeightedSquareError(SPEED, 2.0)]
    metric_df = allocate_metrics_dataframe(metrics, 10)
    @test ncol(metric_df) == 1
    @test nrow(metric_df) == 10
    @test names(metric_df) == [symbol(metrics[1])]

    rec = pull_record(segments[1], evaldata)
    @test get_scene(rec, 0)[1].def == get_vehicle(trajdatas[1], 1, 2).def
    @test get_scene(rec, 0)[1].state.posG == get_vehicle(trajdatas[1], 1, 2).state.posG
end