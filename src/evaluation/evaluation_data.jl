export
    EvaluationData,

    get_trajdata,
    pull_record,

    allocate_metrics_dataframe,
    calc_trace_metrics!,

    create_evaldata_with_random_subsegments_of_equal_length

type EvaluationData
    trajdatas::Vector{Trajdata}
    segments::Vector{TrajdataSegment}
end
EvaluationData() = EvaluationData(Trajdata[], TrajdataSegment[])

get_trajdata(evaldata::EvaluationData, seg::TrajdataSegment) = evaldata.trajdatas[seg.trajdata_index]
pull_record(seg::TrajdataSegment, evaldata::EvaluationData, prime_history::Int=0) = pull_record(seg, get_trajdata(evaldata, seg), prime_history)

"""
    create_evaldata_with_random_subsegments_of_equal_length(evaldata::EvaluationData; nsegs::Int=1, nframes::Int=101)
"""
function create_evaldata_with_random_subsegments_of_equal_length(evaldata::EvaluationData; nsegs::Int=1, nframes::Int=101)
    segments = Array(TrajdataSegment, nsegs)
    i = 0
    while i < nsegs
        seg = evaldata.segments[rand(1:length(evaldata.segments))]
        if AutoCore.nframes(seg) ≥ nframes
            segments[i+=1] = sample_random_subinterval(seg, nframes)
        end
    end

    EvaluationData(evaldata.trajdatas, segments)
end
function create_evaldata_with_random_subsegments_of_equal_length(evaldata::EvaluationData, foldset::FoldSet; nsegs::Int=1, nframes::Int=101)
    segments = Array(TrajdataSegment, nsegs)
    indeces = collect(foldset)
    i = 0
    while i < nsegs
        seg = evaldata.segments[indeces[rand(1:length(indeces))]]
        if AutoCore.nframes(seg) ≥ nframes
            segments[i+=1] = sample_random_subinterval(seg, nframes)
        end
    end

    EvaluationData(evaldata.trajdatas, segments)
end

function allocate_metrics_dataframe{M<:TraceMetricExtractor}(
    metrics::Vector{M},
    nrows::Int;
    )

    df = DataFrame()
    df[:time] = Array(String, nrows)
    df[:logl] = Array(Float64, nrows)
    for m in metrics
        df[Symbol(m)] = Array(Float64, nrows)
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
    row::Int = foldset_seg_test.fold, # row in metrics_df to write to
    prime_history::Int = 0,
    calc_logl::Bool = true,
    )

    # reset metrics
    for metric in metrics
        reset!(metric)
    end

    logl = 0.0
    n_traces = 0

    # simulate traces and perform online metric extraction
    scene = Scene()
    for seg_index in foldset_seg_test

        seg = evaldata.segments[seg_index]
        trajdata = evaldata.trajdatas[seg.trajdata_index]

        rec_orig = pull_record(seg, evaldata) # TODO - make efficient
        rec_sim = deepcopy(rec_orig)

        time_start = get_time(trajdata, seg.frame_lo)
        time_end = get_time(trajdata, seg.frame_hi)

        logl += extract_log_likelihood(model, rec_orig, trajdata.roadway, seg.egoid, prime_history=prime_history)
        n_traces += 1

        for sim_index in 1 : n_simulations_per_trace

            simulate!(rec_sim, model, seg.egoid, trajdata,
                      time_start, time_end, scene=scene, prime_history=prime_history)

            for metric in metrics
                extract!(metric, rec_orig, rec_sim, trajdata.roadway, seg.egoid)
            end
        end
    end

    # compute metric scores
    for metric in metrics
        metrics_df[row, Symbol(metric)] = get_score(metric)
    end
    metrics_df[row, :time] = string(now())
    metrics_df[row, :logl] = logl/n_traces

    metrics_df
end