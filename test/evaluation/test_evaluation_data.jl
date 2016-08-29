let
    trajdatas = [get_test_trajdata(), get_test_trajdata()]
    segments = [TrajdataSegment(1, 2, 1, 2), TrajdataSegment(1, 2, 1, 2)]
    evaldata = EvaluationData(trajdatas, segments)
end