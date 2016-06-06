export
    EvaluationData,
    allocate_runlogs_for_simulation,

    BehaviorMetric,
    BehaviorFrameMetric,
    BehaviorTraceMetric,

    EmergentKLDivMetric,
    RootWeightedSquareError,

    SumSquareJerk,
    JerkSignInversions,
    LagOneAutocorrelation,

    get_name,
    get_score,
    reset!,
    extract!,

    # extract_frame_metrics,
    # extract_trace_metrics,
    # extract_metrics,
    # extract_metrics_from_traces,
    # extract_bagged_metrics,
    # extract_bagged_metrics_from_traces,

    # calc_rmse_predicted_vs_ground_truth,

    # calc_tracemetrics,
    # calc_aggregate_metric,
    # calc_aggregate_metrics,
    # calc_mean_cross_validation_metrics,

    # compute_metric_summary_table,

    calc_dataset_likelihood,
    calc_trace_likelihood,

    create_metrics_df,
    calc_likelihood_metrics!,
    calc_trace_metrics!,
    calc_metrics!

const DEFAULT_TRACE_HISTORY = 2*DEFAULT_FRAME_PER_SEC

#########################################################################################################
# EvaluationData

type EvaluationData
    segments::Vector{RunLogSegment}                     # dset.runlog_segments
    runlogs::Vector{RunLog}                             # runlogs_original = load_runlogs(dset)
    streetnets::Dict{AbstractString, StreetNetwork}     # load_streetnets(runlogs_original)
    frame_starts_sim::Vector{Int}                       # new frame_start for the truncated arr_runlogs_for_simulation

    function EvaluationData(dset::ModelTrainingData2)

        segments = dset.runlog_segments
        runlogs = load_runlogs(dset)
        streetnets = load_streetnets(runlogs)

        nsegments = length(segments)
        frame_starts_sim = Array(Int, nsegments) # new frame_start for the truncated arr_runlogs_for_simulation
        for (i, seg) in enumerate(segments)
            where_to_start_copying_from_original_runlog = max(1, seg.frame_start - DEFAULT_TRACE_HISTORY)
            frame_starts_sim[i] = seg.frame_start - where_to_start_copying_from_original_runlog + 1 # where_to_start_simulating_from_runlog_sim
        end

        retval = new()
        retval.segments = segments
        retval.runlogs = runlogs
        retval.streetnets = streetnets
        retval.frame_starts_sim = frame_starts_sim
        retval
    end
end

get_nsegments(evaldata::EvaluationData) = length(evaldata.segments)
function allocate_runlogs_for_simulation(evaldata::EvaluationData, nmodels::Int, nsimulations_per_trace::Int)

    # arr_runlogs_for_simulation::Vector{Matrix{RunLog}}  # nmodels × [ntraces × N_SIMULATIONS_PER_TRACE]

    nsegments = get_nsegments(evaldata)
    arr_runlogs_for_simulation = Array(Matrix{RunLog}, nmodels)
    for k in 1 : nmodels
        arr_runlogs_for_simulation[k] = Array(RunLog, nsegments, nsimulations_per_trace)
    end
    for (i, seg) in enumerate(evaldata.segments)

        where_to_start_copying_from_original_runlog = max(1, seg.frame_start - DEFAULT_TRACE_HISTORY)

        runlog_sim = deepcopy(evaldata.runlogs[seg.runlog_id], where_to_start_copying_from_original_runlog, seg.frame_end)
        for k in 1 : nmodels
            for j in 1 : nsimulations_per_trace
                arr_runlogs_for_simulation[k][i,j] = deepcopy(runlog_sim)
            end
        end
    end

    arr_runlogs_for_simulation
end
function allocate_runlogs_for_simulation(evaldata::EvaluationData)

    # arr_runlogs_for_simulation::Vector{Matrix{RunLog}}  # nmodels × [ntraces × N_SIMULATIONS_PER_TRACE]

    nsegments = get_nsegments(evaldata)
    arr_runlogs_for_simulation = Array(RunLog, nsegments)
    for (i, seg) in enumerate(evaldata.segments)

        where_to_start_copying_from_original_runlog = max(1, seg.frame_start - DEFAULT_TRACE_HISTORY)
        arr_runlogs_for_simulation[i] = deepcopy(evaldata.runlogs[seg.runlog_id], where_to_start_copying_from_original_runlog, seg.frame_end)
    end

    arr_runlogs_for_simulation
end

#########################################################################################################

abstract BehaviorMetric
abstract BehaviorFrameMetric <: BehaviorMetric
abstract BehaviorTraceMetric <: BehaviorMetric

#########################################################################################################
# RootWeightedSquareError

type RootWeightedSquareError <: BehaviorTraceMetric
    F::AbstractFeature
    H::Float64 # horizon [s]
    running_sum::Float64
    n_obs::Int

    RootWeightedSquareError(F::AbstractFeature, horizon::Float64, running_sum::Float64=0.0, n_obs::Int=0) =
        new(F, horizon, running_sum, n_obs)
end
get_name(m::RootWeightedSquareError) = symbol(@sprintf("RWSE_%s_%d_%02d", string(symbol(m.F)), floor(Int, m.H), floor(Int, 100*rem(m.H, 1.0))))
get_score(m::RootWeightedSquareError) = sqrt(m.running_sum / m.n_obs)
function reset!(metric::RootWeightedSquareError)
    metric.running_sum = 0.0
    metric.n_obs = 0
    metric
end
function extract!(
    metric::RootWeightedSquareError,
    seg::RunLogSegment,
    runlog_true::RunLog,
    runlog_sim::RunLog,
    sn::StreetNetwork,
    frame_starts_sim::Int
    )

    frame_skip = round(Int, metric.H/DEFAULT_SEC_PER_FRAME)

    carid = seg.carid
    frame = seg.frame_start + frame_skip
    @assert(frame ≤ seg.frame_end)

    v_true = NaN
    v_montecarlo = NaN
    Δ = NaN

    if symbol(metric.F) == :d_front

        colset = RunLogs.id2colset(runlog_true, carid, seg.frame_start)
        colset_front_orig = get(runlog_true, colset, seg.frame_start, :colset_front)::UInt
        if colset_front_orig != COLSET_NULL

            carid_front = RunLogs.colset2id(runlog_true, colset_front_orig, seg.frame_start)

            frame = seg.frame_start + frame_skip
            colset_front_now = RunLogs.id2colset(runlog_true, carid_front, frame)
            colset_ego_now = RunLogs.id2colset(runlog_true, carid, frame)

            if colset_front_now != COLSET_NULL
                dist_true = Features._get_dist_between(runlog_true, sn, colset_ego_now, colset_front_now, frame)
                if !isnan(dist_true)

                    frame = frame_starts_sim + frame_skip
                    colset_front_now = RunLogs.id2colset(runlog_sim, carid_front, frame)
                    colset_ego_now = RunLogs.id2colset(runlog_sim, carid, frame)

                    if colset_front_now != COLSET_NULL
                        dist_sim = Features._get_dist_between(runlog_sim, sn, colset_ego_now, colset_front_now, frame)

                        if !isnan(dist_sim)
                            v_true = dist_true
                            v_montecarlo = dist_sim

                            Δ = v_true - v_montecarlo
                            metric.running_sum += Δ*Δ
                            metric.n_obs += 1
                        end
                    end
                end
            end
        end

    elseif symbol(metric.F) == :posFt

        colset_orig = RunLogs.id2colset(runlog_true, carid, seg.frame_start)
        lanetag = get(runlog_true, colset_orig, seg.frame_start, :lanetag)::LaneTag

        colset_now = RunLogs.id2colset(runlog_true, carid, frame)
        pos_now = get(runlog_true, colset_now, frame, :inertial)::VecSE2
        extind, lane = project_point_to_lanetag(sn, pos_now.x, pos_now.y, lanetag)
        curvept = curve_at(lane.curve, extind)
        s, d = pt_to_frenet_xy( curvept, pos_now.x, pos_now.y)
        v_true = d

        frame = frame_starts_sim + frame_skip
        colset_now = RunLogs.id2colset(runlog_sim, carid, frame)
        pos_now = get(runlog_sim, colset_now, frame, :inertial)::VecSE2
        extind, lane = project_point_to_lanetag(sn, pos_now.x, pos_now.y, lanetag)
        curvept = curve_at(lane.curve, extind)
        s, d = pt_to_frenet_xy( curvept, pos_now.x, pos_now.y)
        v_montecarlo = d

        Δ = v_true - v_montecarlo
        metric.running_sum += Δ*Δ
        metric.n_obs += 1
    else
        # pull true value
        colset = RunLogs.id2colset(runlog_true, carid, frame)
        v_true = get(metric.F, runlog_true, sn, colset, frame)
        if is_feature_na(v_true)
            v_true = replace_na(metric.F)
        end

        # pull sim value
        frame = frame_starts_sim + frame_skip
        colset = RunLogs.id2colset(runlog_sim, carid, frame)
        v_montecarlo = get(metric.F, runlog_sim, sn, colset, frame)
        if is_feature_na(v_montecarlo)
            v_montecarlo = replace_na(metric.F)
        end

        Δ = v_true - v_montecarlo
        metric.running_sum += Δ*Δ
        metric.n_obs += 1
    end

    # if symbol(metric.F) == :posFt
    #     @printf("%10s  %5.2f:  %10.6f  %10.6f  %10.6f  %10.6f  %10d\n", string(symbol(metric.F)), metric.H, v_true, v_montecarlo, Δ*Δ, metric.running_sum, metric.n_obs)
    # end

    metric
end

#########################################################################################################
# SumSquareJerk

type SumSquareJerk <: BehaviorTraceMetric
    running_sum::Float64
    n_obs::Int

    SumSquareJerk(running_sum::Float64=0.0, n_obs::Int=0) = new(running_sum, n_obs)
end
get_name(::SumSquareJerk) = :sumsquarejerk
get_score(m::SumSquareJerk) = m.running_sum / m.n_obs
function reset!(metric::SumSquareJerk)
    metric.running_sum = 0.0
    metric.n_obs = 0
    metric
end
function extract_sum_square_jerk(
    seg::RunLogSegment,
    runlog::RunLog,
    sn::StreetNetwork,
    frame_start::Int,
    )

    frame_end = frame_start + seg.frame_end - seg.frame_start

    running_sum = 0.0
    for frame in frame_start + N_FRAMES_PER_SIM_FRAME : N_FRAMES_PER_SIM_FRAME : frame_end

        colset = RunLogs.id2colset(runlog, seg.carid, frame)
        jerk = get(JERK, runlog, sn, colset, frame)
        @assert(!isnan(jerk) && !isinf(jerk))

        running_sum += jerk*jerk
    end
    running_sum
end
function extract!(
    metric::SumSquareJerk,
    seg::RunLogSegment,
    runlog_true::RunLog,
    runlog_sim::RunLog,
    sn::StreetNetwork,
    frame_starts_sim::Int
    )

    metric.running_sum += extract_sum_square_jerk(seg, runlog_sim, sn, frame_starts_sim)
    metric.n_obs += 1

    metric
end

#########################################################################################################
# JerkSignInversions

type JerkSignInversions <: BehaviorTraceMetric
    running_sum::Int
    n_obs::Int

    JerkSignInversions(running_sum::Int=0, n_obs::Int=0) = new(running_sum, n_obs)
end
get_name(::JerkSignInversions) = :jerksigninvs
get_score(m::JerkSignInversions) = m.running_sum / m.n_obs
function reset!(metric::JerkSignInversions)
    metric.running_sum = 0
    metric.n_obs = 0
    metric
end
function extract_jerk_sign_inversion_count(
    seg::RunLogSegment,
    runlog::RunLog,
    sn::StreetNetwork,
    frame_start::Int,
    )

    frame_end = frame_start + seg.frame_end - seg.frame_start

    running_sum = 0
    jerk_prev = 0.0
    for frame in frame_start + N_FRAMES_PER_SIM_FRAME : N_FRAMES_PER_SIM_FRAME : frame_end

        colset = RunLogs.id2colset(runlog, seg.carid, frame)

        jerk = get(JERK, runlog, sn, colset, frame)
        is_jerk_inversion = abs(sign(jerk) - sign(jerk_prev)) > 1.5
        running_sum += is_jerk_inversion
        jerk_prev = jerk
    end
    running_sum
end
function extract!(
    metric::JerkSignInversions,
    seg::RunLogSegment,
    runlog_true::RunLog,
    runlog_sim::RunLog,
    sn::StreetNetwork,
    frame_starts_sim::Int
    )

    metric.running_sum += extract_jerk_sign_inversion_count(seg, runlog_sim, sn, frame_starts_sim)
    metric.n_obs += 1

    metric
end

#########################################################################################################
# LagOneAutocorrelation

type LagOneAutocorrelation <: BehaviorTraceMetric
    running_sum::Float64
    n_obs::Int
    LagOneAutocorrelation(running_sum::Float64=0.0, n_obs::Int=0) = new(running_sum, n_obs)
end
_lag_one_autocorrelation(x::Vector{Float64}) = cor(x[2:end], x[1:end-1])
get_name(::LagOneAutocorrelation) = :lagoneautocor
get_score(m::LagOneAutocorrelation) = m.running_sum / m.n_obs
function reset!(metric::LagOneAutocorrelation)
    metric.running_sum = 0.0
    metric.n_obs = 0
    metric
end
function extract_lag_one_autocor(
    seg::RunLogSegment,
    runlog::RunLog,
    sn::StreetNetwork,
    frame_start::Int,
    )

    frame_end = frame_start + seg.frame_end - seg.frame_start

    frame_range = frame_start + N_FRAMES_PER_SIM_FRAME : N_FRAMES_PER_SIM_FRAME : frame_end
    jerk_arr = Array(Float64, length(frame_range))

    running_sum = 0.0
    for (k,frame) in enumerate(frame_range)
        colset = RunLogs.id2colset(runlog, seg.carid, frame)
        jerk_arr[k] = get(JERK, runlog, sn, colset, frame)
        @assert(!isinf(jerk_arr[k]))
        @assert(!isnan(jerk_arr[k]))
    end
    _lag_one_autocorrelation(jerk_arr)
end
function extract!(
    metric::LagOneAutocorrelation,
    seg::RunLogSegment,
    runlog_true::RunLog,
    runlog_sim::RunLog,
    sn::StreetNetwork,
    frame_starts_sim::Int
    )

    metric.running_sum += extract_lag_one_autocor(seg, runlog_sim, sn, frame_starts_sim)
    metric.n_obs += 1

    metric
end

#########################################################################################################
# EmergentKLDivMetric

const KLDIV_METRIC_NBINS = 20
const KLDIV_METRIC_DISC_DICT = Dict(
        symbol(SPEED)                 => LinearDiscretizer(collect(linspace( 0.0, 35.0, KLDIV_METRIC_NBINS+1)), Int),
        symbol(INV_TIMEGAP_FRONT)     => LinearDiscretizer(collect(linspace( 0.0, 10.0, KLDIV_METRIC_NBINS+1)), Int),
        symbol(POSFT)                 => LinearDiscretizer(collect(linspace(-3.0,  3.0, KLDIV_METRIC_NBINS+1)), Int),
        string(SumSquareJerk)         => LinearDiscretizer(collect(linspace( 0.0,  5.0, KLDIV_METRIC_NBINS+1)), Int),
        string(JerkSignInversions)    => LinearDiscretizer(collect(linspace( 0.0, 10.0, KLDIV_METRIC_NBINS+1)), Int),
        string(LagOneAutocorrelation) => LinearDiscretizer(collect(linspace(-1.0,  1.0, KLDIV_METRIC_NBINS+1)), Int),
    )

type EmergentKLDivMetric <: BehaviorTraceMetric
    F::Union{AbstractFeature, BehaviorTraceMetric}
    disc::LinearDiscretizer
    counts_real::Vector{Int}
    counts_sim::Vector{Int}

    function EmergentKLDivMetric(F::Union{AbstractFeature, BehaviorTraceMetric})
        if isa(F, AbstractFeature)
            disc = KLDIV_METRIC_DISC_DICT[symbol(F::AbstractFeature)]
        else
            disc = KLDIV_METRIC_DISC_DICT[string(typeof(F))]
        end
        counts_real = zeros(Int, nlabels(disc))
        counts_sim = fill!(deepcopy(counts_real), 1) # NOTE(tim): uniform Dirichlet prior
        new(F, disc, counts_real, counts_sim)
    end
end
function get_name(metric::EmergentKLDivMetric)
    if isa(metric.F, AbstractFeature)
        symbol("kldiv_" * string(symbol(metric.F::AbstractFeature)))
    else
        symbol("kldiv_" * string(get_name(metric.F::BehaviorTraceMetric)))
    end
end
get_score(m::EmergentKLDivMetric) = calc_kl_div_categorical(m.counts_real, m.counts_sim)
function reset!(metric::EmergentKLDivMetric)
    fill!(metric.counts_real, 0)
    fill!(metric.counts_sim, 1)
    metric
end
function extract!(
    metric::EmergentKLDivMetric,
    seg::RunLogSegment,
    runlog_true::RunLog,
    runlog_sim::RunLog,
    sn::StreetNetwork,
    frame_starts_sim::Int
    )

    carid = seg.carid


    frame_end = frame_starts_sim + seg.frame_end - seg.frame_start
    colset_true = RunLogs.id2colset(runlog_true, carid, seg.frame_end)
    colset_sim = RunLogs.id2colset(runlog_sim, carid, frame_end)

    v_true, v_sim = NaN, NaN

    if isa(metric.F, AbstractFeature)
        F = metric.F::AbstractFeature
        v_true = get(F, runlog_true, sn, colset_true, seg.frame_end)::Float64
        v_sim = get(F, runlog_sim, sn, colset, frame_end)::Float64
    elseif isa(metric.F, SumSquareJerk)
        v_true = extract_sum_square_jerk(seg, runlog_true, sn, seg.frame_start)
        v_sim = extract_sum_square_jerk(seg, runlog_sim, sn, frame_starts_sim)
    elseif isa(metric.F, JerkSignInversions)
        v_true = extract_jerk_sign_inversion_count(seg, runlog_true, sn, seg.frame_start)
        v_sim = extract_jerk_sign_inversion_count(seg, runlog_sim, sn, frame_starts_sim)
    elseif isa(metric.F, LagOneAutocorrelation)
        v_true = extract_lag_one_autocor(seg, runlog_true, sn, seg.frame_start)
        v_sim = extract_lag_one_autocor(seg, runlog_sim, sn, frame_starts_sim)
    else
        @assert("UNKNOWN EMERGENT KLDIV METRIC")
    end

    metric.counts_real[encode(metric.disc, v_true)] += 1
    metric.counts_sim[encode(metric.disc, v_sim)] += 1

    metric
end

#########################################################################################################

calc_kl_div_gaussian(μA::Float64, σA::Float64, μB::Float64, σB::Float64) = log(σB/σA) + (σA*σA + (μA-μB)^2)/(2σB*σB) - 0.5
function calc_kl_div_gaussian(aggmetrics_original::Dict{Symbol,Any}, aggmetrics_target::Dict{Symbol,Any}, sym::Symbol)

    #=
    Compute the kl divergence for the given aggregate metric
    `sym' must refer to an entry in aggmetrics which contains but sym_mean and sym_stdev
     ex: :mean_timegap -> :mean_timegap_mean and :mean_timegap_stdev
    =#

    str = string(sym)
    sym_mean = symbol(str * "_mean")
    sym_stdev = symbol(str * "_stdev")

    μ_orig = aggmetrics_original[sym_mean]
    σ_orig = aggmetrics_original[sym_stdev]
    μ_target = aggmetrics_target[sym_mean]
    σ_target = aggmetrics_target[sym_stdev]

    calc_kl_div_gaussian(μ_orig, σ_orig, μ_target, σ_target)
end

function calc_kl_div_categorical{I<:Real, J<:Real}(counts_p::AbstractVector{I}, counts_q::AbstractVector{J})

    #=
    Calculate the KL-divergence between two categorical distributions
    (also works if is it a piecewise uniform univariate with equally-spaced bins)
    =#

    tot_p = sum(counts_p)
    tot_q = sum(counts_q)

    kldiv = 0.0
    for (P,Q) in zip(counts_p, counts_q)
        if P > 0
            p = P/tot_p # convert to probability
            q = Q/tot_q # convert to probability
            kldiv += p * log(p/q)
        end
    end
    kldiv
end

calc_probability_for_uniform_sample_from_bin(bindiscreteprob::Float64, binwidth::Float64) = bindiscreteprob / binwidth
function calc_probability_for_uniform_sample_from_bin(bindiscreteprob::Float64, disc::LinearDiscretizer, binindex::Int)
    width_of_bin = binwidth(disc, binindex)
    calc_probability_for_uniform_sample_from_bin(bindiscreteprob, width_of_bin)
end

function calc_trace_likelihood(
    runlog::RunLog,
    sn::StreetNetwork,
    seg::RunLogSegment,
    behavior::AbstractVehicleBehavior;
    id::UInt=ID_EGO,
    )

    seg_duration = seg.frame_end - seg.frame_start

    logl = 0.0
    for frame in seg.frame_start : seg.frame_end
        colset = id2colset(runlog, id, frame)

        # TODO - fix specific action
        action_lat = get(FUTUREDESIREDANGLE, runlog, sn, colset, frame)
        action_lon = get(FUTUREACCELERATION, runlog, sn, colset, frame)

        logl += calc_action_loglikelihood(behavior, runlog, sn, colset, frame, action_lat, action_lon)
    end
    logl
end
function calc_dataset_likelihood(dset::ModelTrainingData2, model::AbstractVehicleBehavior, foldset::FoldSet)
    retval = 0.0
    for frame in foldset
        retval += calc_action_loglikelihood(model, dset.dataframe, frame)
    end
    retval
end

function create_metrics_df(nfolds::Int, trace_metric_names::Vector{Symbol}=Symbol[])
    metrics_df = DataFrame()
    metrics_df[:mean_logl_train]   = Array(Float64, nfolds)
    metrics_df[:mean_logl_test]    = Array(Float64, nfolds)
    metrics_df[:median_logl_train] = Array(Float64, nfolds)
    metrics_df[:median_logl_test]  = Array(Float64, nfolds)
    for name in trace_metric_names
        metrics_df[name] = Array(Float64, nfolds)
    end
    metrics_df
end
function calc_likelihood_metrics!(
    metrics_df::DataFrame,
    dset::ModelTrainingData2,
    model::AbstractVehicleBehavior,
    cv_split::FoldAssignment,
    fold::Int,
    metrics_df_index::Int=fold,
    )

    count_logl_train = 0
    count_logl_test = 0
    arr_logl_test  = Array(Float64, length(FoldSet(cv_split, fold, true, :frame)))
    arr_logl_train = Array(Float64, length(FoldSet(cv_split, fold, false, :frame)))

    for frame in 1 : length(cv_split.frame_assignment)
        if cv_split.frame_assignment[frame] == fold
            count_logl_test += 1
            arr_logl_test[count_logl_test] = calc_action_loglikelihood(model, dset.dataframe, frame)
        elseif cv_split.frame_assignment[frame] != 0
            count_logl_train += 1
            arr_logl_train[count_logl_train] = calc_action_loglikelihood(model, dset.dataframe, frame)
        end
    end

    metrics_df[metrics_df_index, :mean_logl_train] = mean(arr_logl_train)
    metrics_df[metrics_df_index, :mean_logl_test] = mean(arr_logl_test)
    metrics_df[metrics_df_index, :median_logl_train] = median(arr_logl_train)
    metrics_df[metrics_df_index, :median_logl_test] = median(arr_logl_test)

    metrics_df
end
function calc_trace_metrics!(
    metrics_df::DataFrame,
    model::AbstractVehicleBehavior,
    trace_metrics::Vector{BehaviorTraceMetric},
    evaldata::EvaluationData,
    arr_runlogs_for_simulation::Vector{RunLog},
    n_simulations_per_trace::Int,
    cv_split::FoldAssignment,
    fold::Int,
    metrics_df_index::Int=fold,
    )

    foldset_seg_test = FoldSet(cv_split, fold, true, :seg)

    # reset metrics
    for metric in trace_metrics
        reset!(metric)
    end

    # simulate traces and perform online metric extraction
    for seg_index in foldset_seg_test

        seg = evaldata.segments[seg_index]
        seg_duration = seg.frame_end - seg.frame_start
        sim_start = evaldata.frame_starts_sim[seg_index]
        sim_end = sim_start + seg_duration
        runlog_true = evaldata.runlogs[seg.runlog_id]
        runlog_sim = arr_runlogs_for_simulation[seg_index]
        sn = evaldata.streetnets[runlog_sim.header.map_name]

        for sim_index in 1 : n_simulations_per_trace

            simulate!(runlog_sim, sn, model, seg.carid, sim_start, sim_end)

            for metric in trace_metrics
                extract!(metric, seg, runlog_true, runlog_sim, sn, sim_start)
            end
        end
    end

    # compute metric scores
    for metric in trace_metrics
        metric_name = get_name(metric)
        metrics_df[metrics_df_index, metric_name] = get_score(metric)
    end

    metrics_df
end
function calc_metrics!(
    metrics_df::DataFrame,
    dset::ModelTrainingData2,
    model::AbstractVehicleBehavior,
    cv_split::FoldAssignment,
    fold::Int,

    trace_metrics::Vector{BehaviorTraceMetric},
    evaldata::EvaluationData,
    arr_runlogs_for_simulation::Vector{RunLog},
    n_simulations_per_trace::Int,

    metrics_df_index::Int=fold,
    )

    calc_likelihood_metrics!(metrics_df, dset, model, cv_split, fold, metrics_df_index)
    calc_trace_metrics!(metrics_df, model, trace_metrics, evaldata, arr_runlogs_for_simulation, n_simulations_per_trace, cv_split, fold, metrics_df_index)

    metrics_df
end