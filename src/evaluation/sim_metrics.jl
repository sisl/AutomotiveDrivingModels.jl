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

    calc_trace_likelihood

const DEFAULT_N_SIMULATIONS_PER_TRACE = 5
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

#########################################################################################################

abstract BehaviorMetric
abstract BehaviorFrameMetric <: BehaviorMetric
abstract BehaviorTraceMetric <: BehaviorMetric

#########################################################################################################
# RootWeightedSquareError

# F isa AbstractFeature
type RootWeightedSquareError <: BehaviorTraceMetric
    F::AbstractFeature
    H::Float64 # horizon
    running_sum::Float64
    n_obs::Int

    RootWeightedSquareError(F::AbstractFeature, horizon::Float64, running_sum::Float64=0.0, n_obs::Int=0) =
        new(F, horizon, running_sum, n_obs)
end
get_name(m::RootWeightedSquareError) = symbol(@sprintf("RWSE_%s_%.2f", string(symbol(m.F)), m.H))
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

    # pull true value
    colset = RunLogs.id2colset(runlog_true, carid, frame)
    v_true = get(metric.F, runlog_true, sn, colset, frame)
    # @assert !is_feature_na(v_true)  # DO I WANT THIS?
    if is_feature_na(v_true)
        v_true = replace_na(metric.F)
    end

    # pull sim value
    frame = frame_starts_sim + frame_skip
    colset = RunLogs.id2colset(runlog_sim, carid, frame)
    v_montecarlo = get(metric.F, runlog_sim, sn, colset, frame)
    # @assert !is_feature_na(v_montecarlo)
    if is_feature_na(v_montecarlo)
        v_montecarlo = replace_na(metric.F)
    end

    Δ = v_true - v_montecarlo
    metric.running_sum += Δ*Δ
    metric.n_obs += 1

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

# ########################################################################################################
# # Bagging

# const DEFAULT_CONFIDENCE_LEVEL = 0.95
# const DEFAULT_N_BAGGING_SAMPLES = 10

# function _bagged_mean_sample(v::AbstractVector{Float64}, m::Int=length(v), bag_range::UnitRange{Int}=1:m)

#     # compute the mean of a bagged subset of v

#     x = 0.0
#     for i in 1 : m
#         x += v[rand(bag_range)]
#     end
#     x / m
# end
# function _bagged_sample(
#     M::DataType,
#     evaldata::EvaluationData,
#     runlogs_for_simulation::Matrix{RunLog},
#     seg_indeces::AbstractVector{Int}, # segment indeces, typically from collect(FoldSet)
#     bagsamples::AbstractVector{Int},  # indeces in foldset that were selected for evaluation; of length seg_indeces
#     m::Int=length(seg_indeces),
#     bag_range::UnitRange{Int}=1:m
#     )

#     @assert(length(bagsamples) == length(seg_indeces))

#     for i in 1 : m
#         bagsamples[i] = rand(bag_range)
#     end

#     get_score(extract(M, evaldata, runlogs_for_simulation, seg_indeces, bagsamples))
# end


# immutable BaggedMetricResult
#     M::DataType # datatype of the source BehaviorMetric
#     μ::Float64 # sample mean
#     σ::Float64 # sample standard deviation
#     min::Float64 # min observed value
#     max::Float64 # max observed value
#     n_bagging_samples::Int
#     confidence_bound::Float64
#     confidence_level::Float64 # ie, 0.95 indicates 95% confidence level

#     function BaggedMetricResult{M<:BehaviorFrameMetric}(
#         ::Type{M},
#         frame_scores::Vector{Float64}, # already extracted, of dimension nvalid_frames
#         n_bagging_samples::Int=DEFAULT_N_BAGGING_SAMPLES,
#         confidence_level::Float64=DEFAULT_CONFIDENCE_LEVEL,
#         )

#         z = 1.41421356237*erfinv(confidence_level)
#         m = length(frame_scores)
#         bag_range = 1 : m

#         μ = _bagged_mean_sample(frame_scores, m, bag_range)
#         ν = 0.0 # running variance sum (variance = Sₖ/(k-1))
#         lo = Inf
#         hi = -Inf

#         # TODO: use OnlineStats.jl
#         for k in 2 : n_bagging_samples

#             # take a bagged sample
#             x = _bagged_mean_sample(frame_scores, m, bag_range)

#             # update running stats
#             μ₂ = μ + (x - μ)/k
#             ν += (x - μ)*(x - μ₂)
#             μ = μ₂

#             lo = min(lo, x)
#             hi = max(hi, x)
#         end

#         σ = sqrt(ν/(n_bagging_samples-1))

#         confidence_bound = z * σ / sqrt(n_bagging_samples)

#         new(M, μ, σ, lo, hi, n_bagging_samples, confidence_bound, confidence_level)
#     end
#     function BaggedMetricResult{M<:BehaviorTraceMetric}(
#         ::Type{M},
#         evaldata::EvaluationData,
#         runlogs_for_simulation::Matrix{RunLog},
#         seg_indeces::AbstractVector{Int}, # segment indeces, typically from collect(FoldSet)
#         bagsamples::AbstractVector{Int},  # indeces in foldset that were selected for evaluation; of length seg_indeces
#         n_bagging_samples::Int=DEFAULT_N_BAGGING_SAMPLES,
#         confidence_level::Float64=DEFAULT_CONFIDENCE_LEVEL,
#         )

#         z = 1.41421356237*erfinv(confidence_level)
#         m = length(seg_indeces)
#         bag_range = 1 : m

#         μ = _bagged_sample(M, evaldata, runlogs_for_simulation, seg_indeces, bagsamples, m, bag_range)
#         ν = 0.0 # running variance sum (variance = Sₖ/(k-1))
#         lo = Inf
#         hi = -Inf

#         for k in 2 : n_bagging_samples

#             # take a bagged sample
#             x = _bagged_sample(M, evaldata, runlogs_for_simulation, seg_indeces, bagsamples, m, bag_range)

#             # update running stats
#             μ₂ = μ + (x - μ)/k
#             ν += (x - μ)*(x - μ₂)
#             μ = μ₂

#             lo = min(lo, x)
#             hi = max(hi, x)
#         end

#         σ = sqrt(ν/(n_bagging_samples-1))

#         confidence_bound = z * σ / sqrt(n_bagging_samples)

#         new(M, μ, σ, lo, hi, n_bagging_samples, confidence_bound, confidence_level)
#     end
# end

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

# function calc_rmse_predicted_vs_ground_truth(
#     pdset_predicted::PrimaryDataset,
#     pdset_truth::PrimaryDataset,
#     seg::PdsetSegment,
#     nframes::Int=get_horizon(seg)
#     )

#     #=
#     RMSE: Root-Mean Square Error

#     the square root of the mean square error
#     =#

#     @assert(nframes > 0)
#     @assert(nframes ≤ get_horizon(seg))

#     total = 0.0
#     validfind = seg.validfind_start - 1
#     for i = 1 : nframes
#         validfind += 1

#         carind_predicted = carid2ind(pdset_predicted, seg.carid, validfind)
#         carind_truth     = carid2ind(pdset_truth,     seg.carid, validfind)

#         pos_x_predicted = get(pdset_predicted, :posGx, carind_predicted, validfind)::Float64
#         pos_y_predicted = get(pdset_predicted, :posGy, carind_predicted, validfind)::Float64
#         pos_x_truth     = get(pdset_truth,     :posGx, carind_truth,     validfind)::Float64
#         pos_y_truth     = get(pdset_truth,     :posGy, carind_truth,     validfind)::Float64

#         Δx = pos_x_predicted - pos_x_truth
#         Δy = pos_y_predicted - pos_y_truth

#         if i == 1
#             if abs(Δx) > 0.01 || abs(Δy) > 0.01
#                 println("validfind:        ", validfind)
#                 println("Δx:               ", Δx)
#                 println("Δy:               ", Δy)
#                 println("pos_x_predicted:  ", pos_x_predicted)
#                 println("pos_y_predicted:  ", pos_y_predicted)
#                 println("pos_x_truth:      ", pos_x_truth)
#                 println("pos_y_truth:      ", pos_y_truth)
#                 println("carind_predicted: ", carind_predicted)
#                 println("carind_truth:     ", carind_truth)
#                 println("seg:              ", seg)
#             end
#             @assert(abs(Δx) < 0.01, "Must have no initial deviation")
#             @assert(abs(Δy) < 0.01, "Must have no initial deviation")
#         end

#         total += Δx*Δx + Δy*Δy
#     end
#     sqrt(total / nframes)
# end

calc_probability_for_uniform_sample_from_bin(bindiscreteprob::Float64, binwidth::Float64) = bindiscreteprob / binwidth
function calc_probability_for_uniform_sample_from_bin(bindiscreteprob::Float64, disc::LinearDiscretizer, binindex::Int)
    width_of_bin = binwidth(disc, binindex)
    calc_probability_for_uniform_sample_from_bin(bindiscreteprob, width_of_bin)
end

# function calc_mean_cross_validation_metrics(aggmetric_set::Vector{Dict{Symbol,Any}})
#     retval = Dict{Symbol, Any}[]

#     n = length(aggmetric_set)
#     temparr = Array(Float64, n)

#     keyset = keys(aggmetric_set[1])
#     for key in keyset
#         for (i,aggmetrics) in enumerate(aggmetric_set)
#             temparr[i] = Float64(get(aggmetrics, key, NaN))
#         end
#         μ = mean(temparr)
#         σ = stdm(temparr, μ)
#         retval[symbol(string(key)*"_mean")] = μ
#         retval[symbol(string(key)*"_stdev")] = σ
#     end

#     retval
# end

########

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