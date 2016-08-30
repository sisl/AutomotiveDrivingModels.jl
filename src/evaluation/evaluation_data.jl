export
    EvaluationData,

    allocate_metrics_dataframe,
    calc_trace_metrics!

type EvaluationData
    trajdatas::Vector{Trajdata}
    segments::Vector{TrajdataSegment}
end
EvaluationData() = EvaluationData(Trajdata[], TrajdataSegment[])

pull_record(seg::TrajdataSegment, evaldata::EvaluationData) = pull_record(seg, evaldata.trajdatas[seg.trajdata_index])

function allocate_metrics_dataframe{M<:TraceMetricExtractor}(metrics::Vector{M}, nfolds::Int)
    df = DataFrame()
    for m in metrics
        df[symbol(m)] = Array(Float64, nfolds)
    end
    df
end

function calc_trace_metrics!(
    metrics_df::DataFrame,
    model::DriverModel,
    metrics::Vector{TraceMetricExtractor},
    evaldata::EvaluationData,
    foldset_seg_test::FoldSet; # should match the test segments in evaldata
    n_simulations_per_trace::Int = 10,
    df_index::Int = foldset_seg_test.fold, # row in metrics_df to write to
    )

    # reset metrics
    for metric in metrics
        reset!(metric)
    end

    # simulate traces and perform online metric extraction
    scene = Scene()
    for seg_index in foldset_seg_test

        seg = evaldata.segments[seg_index]
        trajdata = evaldata.trajdatas[seg.trajdata_index]

        for sim_index in 1 : n_simulations_per_trace

            rec_orig = pull_record(seg, evaldata) # TODO - make efficient
            rec_sim = deepcopy(rec_orig)

            time_start = get_time(trajdata, seg.frame_lo)
            time_end = get_time(trajdata, seg.frame_hi)

            simulate!(rec_sim, model, seg.egoid, trajdata,
                      time_start, time_end, scene=scene)

            for metric in metrics
                extract!(metric, rec_orig, rec_sim, trajdata.roadway, seg.egoid)
            end
        end
    end

    # compute metric scores
    for metric in metrics
        metrics_df[df_index, symbol(metric)] = get_score(metric)
    end

    metrics_df
end