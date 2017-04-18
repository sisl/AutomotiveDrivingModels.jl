export
    TraceMetricExtractor,

    get_score,
    reset!,
    extract!,

    extract_log_likelihood,
    extract_sum_square_jerk,

    RootWeightedSquareError,
    SumSquareJerk,
    EmergentKLDivergence

include("metrics.jl")