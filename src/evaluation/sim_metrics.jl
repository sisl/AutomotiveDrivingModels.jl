export
    BehaviorMetric,
    BehaviorFrameMetric,
    BehaviorTraceMetric,

    # HistobinMetric,
    # TraceMetricSet,
    # AggregateTraceMetrics,
    EmergentKLDivMetric,
    RootWeightedSquareError,

    LoglikelihoodMetric,
    BaggedMetricResult,

    get_score,
    extract,

    extract_frame_metrics,
    extract_trace_metrics,
    extract_metrics,
    extract_metrics_from_traces,
    extract_bagged_metrics,
    extract_bagged_metrics_from_traces,

    calc_rmse_predicted_vs_ground_truth,

    calc_tracemetrics,
    calc_aggregate_metric,
    calc_aggregate_metrics,
    calc_mean_cross_validation_metrics,

    compute_metric_summary_table

abstract BehaviorMetric
abstract BehaviorFrameMetric <: BehaviorMetric
abstract BehaviorTraceMetric <: BehaviorMetric

#########################################################################################################
# EmergentKLDivMetric

type EmergentKLDivMetric{feature_symbol} <: BehaviorTraceMetric
    kldiv::Float64
end

const KLDIV_METRIC_NBINS = 10
const KLDIV_METRIC_DISC_DICT = Dict(
        symbol(SPEED) => LinearDiscretizer(collect(linspace(0.0,35.0,KLDIV_METRIC_NBINS)), Int),
        symbol(TIMEGAP_X_FRONT) => LinearDiscretizer(collect(linspace(0.0,10.0,KLDIV_METRIC_NBINS)), Int),
        symbol(D_CL) => LinearDiscretizer(collect(linspace(-3.0,3.0,KLDIV_METRIC_NBINS)), Int),
    )


get_score(metric::EmergentKLDivMetric) = metric.kldiv

function extract{Fsym}(::Type{EmergentKLDivMetric{Fsym}},
    pdset_segments::Vector{PdsetSegment},
    pdsets_original::Vector{PrimaryDataset},
    pdsets_for_simulation::Matrix{PrimaryDataset},
    validfind_starts_sim::Vector{Int},
    streetnets::Vector{StreetNetwork},
    foldinds::Vector{Int},
    basics::FeatureExtractBasicsPdSet,
    bagged_selection::Vector{Int},
    kldiv_metric_nbins::Int = KLDIV_METRIC_NBINS,
    )

    F = symbol2feature(Fsym)
    disc = KLDIV_METRIC_DISC_DICT[Fsym]

    counts_orig = zeros(Int, kldiv_metric_nbins) # NOTE(tim): no prior counts
    counts_sim  = ones(Int, kldiv_metric_nbins)  # NOTE(tim): uniform Dirichlet prior

    n_monte_carlo_samples = size(pdsets_for_simulation, 2)

    for i in bagged_selection
        ind = foldinds[i]

        # real-world
        seg = pdset_segments[ind]
        carid = seg.carid
        basics.pdset = pdsets_original[seg.pdset_id]
        basics.sn = streetnets[seg.streetnet_id]

        carind = carid2ind(basics.pdset, carid, seg.validfind_end)
        v = get(F, basics, carind, seg.validfind_end)
        counts_orig[encode(disc, v)] += 1

        # sim

        validfind_end = validfind_starts_sim[i] + seg.validfind_end - seg.validfind_start

        for j = 1 : n_monte_carlo_samples
            basics.pdset = pdsets_for_simulation[i, j]
            carind = carid2ind(basics.pdset, carid, validfind_end)
            basics.runid += 1
            v = get(F, basics, carind, validfind_end)
            counts_sim[encode(disc, v)] += 1
        end
    end

    EmergentKLDivMetric{Fsym}(calc_kl_div_categorical(counts_orig, counts_sim))
end

#########################################################################################################
# RootWeightedSquareError

# F isa AbstractFeature
type RootWeightedSquareError{feature_symbol, horizon} <: BehaviorTraceMetric
    # RWSE for the given horizon
    rwse::Float64
end

get_score(metric::RootWeightedSquareError) = metric.rwse
function extract{Fsym, H}(::Type{RootWeightedSquareError{Fsym, H}},
    pdset_segments::Vector{PdsetSegment},
    pdsets_original::Vector{PrimaryDataset},
    pdsets_for_simulation::Matrix{PrimaryDataset},
    validfind_starts_sim::Vector{Int},
    streetnets::Vector{StreetNetwork},
    foldinds::Vector{Int},
    basics::FeatureExtractBasicsPdSet,
    bagged_selection::Vector{Int},
    )

    F = symbol2feature(Fsym)

    v_true_arr = Array(Float64, 100)
    n_monte_carlo_samples = size(pdsets_for_simulation, 2)

    running_sum = 0.0
    for i in bagged_selection
        ind = foldinds[i]

        seg = pdset_segments[ind]
        carid = seg.carid
        basics.pdset = pdsets_original[seg.pdset_id]
        basics.sn = streetnets[seg.streetnet_id]

        for (j,validfind) in enumerate(seg.validfind_start + N_FRAMES_PER_SIM_FRAME : N_FRAMES_PER_SIM_FRAME : seg.validfind_end)
            carind = carid2ind(basics.pdset, carid, validfind)
            v_true_arr[j] = get(F, basics, carind, validfind)
        end

        validfind_start = validfind_starts_sim[i]
        validfind_end = validfind_start + seg.validfind_end - seg.validfind_start

        for j = 1 : n_monte_carlo_samples
            basics.pdset = pdsets_for_simulation[i, j]

            for (k,validfind) in enumerate(validfind_start + N_FRAMES_PER_SIM_FRAME : N_FRAMES_PER_SIM_FRAME : validfind_end)

                Δt = (validfind - validfind_start)*DEFAULT_SEC_PER_FRAME

                if Δt ≤ H
                    v_true = v_true_arr[k]
                    carind = carid2ind(basics.pdset, carid, validfind)
                    basics.runid += 1
                    v_montecarlo = get(F, basics, carind, validfind)

                    Δ = v_true - v_montecarlo

                    running_sum += Δ*Δ
                end
            end
        end
    end

    NM = n_monte_carlo_samples * length(foldinds)
    RootWeightedSquareError{Fsym, H}(sqrt(running_sum / NM))
end

#########################################################################################################
# LoglikelihoodMetric

type LoglikelihoodMetric <: BehaviorFrameMetric
    # log likelihood of dataset frames on model
    logl::Float64 # (divided by number of frames)
end
function extract(::Type{LoglikelihoodMetric},
    dset::ModelTrainingData,
    behavior::AbstractVehicleBehavior,
    assignment::FoldAssignment,
    fold::Int,
    match_fold::Bool,
    )

    logl = 0.0
    nframes = 0
    for frameind in 1 : nrow(dset.dataframe)
        if is_in_fold(fold, assignment.frame_assignment[frameind], match_fold)
            nframes += 1
            if trains_with_nona(behavior)
                logl += calc_action_loglikelihood(behavior, dset.dataframe_nona, frameind)
            else
                logl += calc_action_loglikelihood(behavior, dset.dataframe, frameind)
            end
        end
    end
    LoglikelihoodMetric(logl/nframes)
end
extract(::Type{LoglikelihoodMetric}, frame_scores::AbstractVector{Float64}) = LoglikelihoodMetric(mean(frame_scores))
function get_frame_score(::Type{LoglikelihoodMetric},
    dset::ModelTrainingData,
    behavior::AbstractVehicleBehavior,
    frameind::Int,
    )

    if trains_with_nona(behavior)
        calc_action_loglikelihood(behavior, dset.dataframe_nona, frameind)
    else
        calc_action_loglikelihood(behavior, dset.dataframe, frameind)
    end
end
get_score(metric::LoglikelihoodMetric) = metric.logl

########################################################################################################
# Bagging

const DEFAULT_CONFIDENCE_LEVEL = 0.95
const DEFAULT_N_BAGGING_SAMPLES = 10

function _bagged_mean_sample(v::AbstractVector{Float64}, m::Int=length(v), bag_range::UnitRange{Int}=1:m)

    x = 0.0
    for i in 1 : m
        x += v[rand(bag_range)]
    end
    x / m
end
function _bagged_sample(
    M::DataType,
    pdset_segments::Vector{PdsetSegment},
    pdsets_original::Vector{PrimaryDataset},
    pdsets_for_simulation::Matrix{PrimaryDataset},
    validfind_starts_sim::Vector{Int},
    streetnets::Vector{StreetNetwork},
    foldinds::Vector{Int},
    basics::FeatureExtractBasicsPdSet,
    bagged_selection::Vector{Int}, # preallocated memory - of same length as foldinds
    m::Int=length(foldinds),
    bag_range::UnitRange{Int}=1:m
    )

    @assert(length(bagged_selection) == length(foldinds))

    for i in 1 : m
        bagged_selection[i] = rand(bag_range)
    end

    get_score(extract(M, pdset_segments, pdsets_original, pdsets_for_simulation,
                      validfind_starts_sim, streetnets, foldinds, basics, bagged_selection))
end

immutable BaggedMetricResult
    M::DataType # datatype of the source BehaviorMetric
    μ::Float64 # sample mean
    σ::Float64 # sample standard deviation
    n_bagging_samples::Int
    confidence_bound::Float64
    confidence_level::Float64 # ie, 0.95 indicates 95% confidence level

    function BaggedMetricResult{B<:BehaviorFrameMetric}(
        ::Type{B},
        frame_scores::Vector{Float64}, # already extracted, of dimension nvalid_frames
        n_bagging_samples::Int=DEFAULT_N_BAGGING_SAMPLES,
        confidence_level::Float64=DEFAULT_CONFIDENCE_LEVEL,
        )

        z = 1.41421356237*erfinv(confidence_level)
        m = length(frame_scores)
        bag_range = 1 : m

        M = _bagged_mean_sample(frame_scores, m, bag_range)
        S = 0.0 # running variance sum (variance = Sₖ/(k-1))

        for k in 2 : n_bagging_samples

            # take a bagged sample
            x = _bagged_mean_sample(frame_scores, m, bag_range)

            # update running stats
            M_new = M + (x - M)/k
            S = S + (x - M)*(x - M_new)
            M = M_new
        end

        μ = M
        σ = sqrt(S/(n_bagging_samples-1))

        confidence_bound = z * σ / sqrt(n_bagging_samples)

        new(B, μ, σ, n_bagging_samples, confidence_bound, confidence_level)
    end
    function BaggedMetricResult{B<:BehaviorTraceMetric}(
        ::Type{B},
        pdset_segments::Vector{PdsetSegment},
        pdsets_original::Vector{PrimaryDataset},
        pdsets_for_simulation::Matrix{PrimaryDataset},
        validfind_starts_sim::Vector{Int},
        streetnets::Vector{StreetNetwork},
        foldinds::Vector{Int},
        basics::FeatureExtractBasicsPdSet,
        bagged_selection::Vector{Int}, # preallocated memory - of same length as foldinds
        n_bagging_samples::Int=DEFAULT_N_BAGGING_SAMPLES,
        confidence_level::Float64=DEFAULT_CONFIDENCE_LEVEL,
        )

        z = 1.41421356237*erfinv(confidence_level)
        m = length(foldinds)
        bag_range = 1 : m

        M = _bagged_sample(B, pdset_segments, pdsets_original, pdsets_for_simulation,
                           validfind_starts_sim, streetnets, foldinds, basics,
                           bagged_selection, m, bag_range)
        S = 0.0 # running variance sum (variance = Sₖ/(k-1))

        for k in 2 : n_bagging_samples

            # take a bagged sample
            x = _bagged_sample(B, pdset_segments, pdsets_original, pdsets_for_simulation,
                               validfind_starts_sim, streetnets, foldinds, basics,
                               bagged_selection, m, bag_range)

            # update running stats
            M_new = M + (x - M)/k
            S = S + (x - M)*(x - M_new)
            M = M_new
        end

        μ = M
        σ = sqrt(S/(n_bagging_samples-1))

        confidence_bound = z * σ / sqrt(n_bagging_samples)

        new(B, μ, σ, n_bagging_samples, confidence_bound, confidence_level)
    end
end

########################################################################################################
# Frame Metric Extraction

function extract_frame_scores!{B<:BehaviorFrameMetric}(
    ::Type{B},
    frame_scores::Vector{Float64}, # preallocated memory, to be filled
    frameinds::Vector{Int}, # lists the indeces which are to be used

    behavior::AbstractVehicleBehavior,
    dset::ModelTrainingData,
    )

    for (i,frameind) in enumerate(frameinds)
        frame_scores[i] = get_frame_score(B, dset, behavior, frameind)
    end

    frame_scores
end
function extract_frame_metrics(
    metric_types_frame::Vector{DataType},
    metric_types_frame_bagged::Vector{DataType},

    behavior::AbstractVehicleBehavior,
    dset::ModelTrainingData,

    frame_scores::Vector{Float64},
    foldinds::Vector{Int},

    n_bagging_samples::Int=DEFAULT_N_BAGGING_SAMPLES, # number of bootstrap aggregated samples to take from data to eval on
    confidence_level::Float64=0.95,
    )

    #=
    Returns
        Vector{BehaviorFrameMetric} over metric_types_frame
        Vector{BaggedMetricResult} over metric_types_frame_bagged
    =#

    retval_straight = Array(BehaviorFrameMetric, length(metric_types_frame))
    retval_bagged = Array(BaggedMetricResult, length(metric_types_frame_bagged))

    done_bagged = falses(length(metric_types_frame_bagged))

    j = 0
    for (i,M) in enumerate(metric_types_frame)
        extract_frame_scores!(M, frame_scores, foldinds, behavior, dset)
        retval_straight[i] = extract(M, frame_scores)
        k = findfirst(metric_types_frame_bagged, M)
        if k != 0
            j += 1
            retval_bagged[j] = BaggedMetricResult(M, frame_scores, n_bagging_samples, confidence_level)
            done_bagged[k] = true
        end
    end

    for (b, M) in zip(done_bagged, metric_types_frame_bagged)
        if !b
            j += 1
            extract_frame_scores!(M, frame_scores, foldinds, behavior, dset)
            retval_bagged[j] = BaggedMetricResult(M, frame_scores, n_bagging_samples, confidence_level)
        end
    end

    (retval_straight, retval_bagged)
end
function extract_frame_metrics{B<:AbstractVehicleBehavior}(
    metric_types_frame::Vector{DataType},
    metric_types_frame_bagged::Vector{DataType},

    behaviors::Vector{B},
    dset::ModelTrainingData,
    assignment::FoldAssignment,
    fold::Int,
    match_fold::Bool,

    n_bagging_samples::Int=DEFAULT_N_BAGGING_SAMPLES, # number of bootstrap aggregated samples to take from data to eval on
    confidence_level::Float64=0.95,
    )

    #=
    Returns
        Vector{Vector{BehaviorFrameMetric}} over behaviors, then metric_types_frame
        Vector{Vector{BaggedMetricResult}} over behaviors, then metric_types_frame_bagged
    =#

    @assert(n_bagging_samples > 1)

    # allocate memory
    n = calc_fold_size(fold, assignment.frame_assignment, match_fold)
    frame_scores = Array(Float64, n) # preallocated memory, to be filled
    foldinds = calc_fold_inds!(Array(Int, n), fold, assignment.frame_assignment, match_fold)

    retval_straight = Array(Vector{BehaviorFrameMetric}, length(behaviors))
    retval_bagged = Array(Vector{BaggedMetricResult}, length(behaviors))

    for (i,b) in enumerate(behaviors)
        (c,d) = extract_frame_metrics(
                        metric_types_frame, metric_types_frame_bagged,
                        b, dset, frame_scores, foldinds,
                        n_bagging_samples, confidence_level,
                        )
        retval_straight[i] = c
        retval_bagged[i] = d
    end

    (retval_straight, retval_bagged)
end

#########################################################################################################
# Trace Metric Extraction

const DEFAULT_N_SIMULATIONS_PER_TRACE = 5
const DEFAULT_TRACE_HISTORY = 2*DEFAULT_FRAME_PER_SEC


function extract_trace_metrics(
    metric_types_trace::Vector{DataType},
    metric_types_trace_bagged::Vector{DataType},

    behavior::AbstractVehicleBehavior,
    dset::ModelTrainingData,

    pdsets_original::Vector{PrimaryDataset}, # [m]
    pdsets_for_simulation::Matrix{PrimaryDataset}, #[n, n_sims]
    validfind_starts_sim::Vector{Int}, # [n]
    streetnets::Vector{StreetNetwork},
    foldinds::Vector{Int},
    bagged_selection::Vector{Int},

    n_bagging_samples::Int=DEFAULT_N_BAGGING_SAMPLES, # number of bootstrap aggregated samples to take from data to eval on
    confidence_level::Float64=0.95,
    )

    #=
    Returns
        Vector{BehaviorFrameMetric} over metric_types_trace
        Vector{BaggedMetricResult} over metric_types_trace_bagged
    =#

    retval_straight = Array(BehaviorTraceMetric, length(metric_types_trace))
    retval_bagged = Array(BaggedMetricResult, length(metric_types_trace_bagged))

    ##################################################################################
    # simulate
    basics = FeatureExtractBasicsPdSet(pdsets_original[1], streetnets[1])
    for (i,ind) in enumerate(foldinds)

        bagged_selection[i] = i # NOTE(tim): used to ensure extract() called later works correctly

        seg = dset.pdset_segments[ind]
        basics.sn = streetnets[seg.streetnet_id]
        validfind_start = validfind_starts_sim[i]
        validfind_end = validfind_start + seg.validfind_end - seg.validfind_start

        for j = 1 : size(pdsets_for_simulation, 2)
            basics.pdset = pdsets_for_simulation[i, j]
            basics.runid += 1

            simulate!(basics, behavior, seg.carid, validfind_start, validfind_end)
        end
    end

    ##################################################################################
    # metric extraction

    done_bagged = falses(length(metric_types_trace_bagged))

    j = 0
    for (i,M) in enumerate(metric_types_trace)
        retval_straight[i] = extract(M, dset.pdset_segments,
                                     pdsets_original, pdsets_for_simulation, validfind_starts_sim,
                                     streetnets, foldinds, basics, bagged_selection)
        k = findfirst(metric_types_trace_bagged, M)
        if k != 0
            j += 1
            retval_bagged[j] = BaggedMetricResult(M, dset.pdset_segments,
                                     pdsets_original, pdsets_for_simulation, validfind_starts_sim,
                                     streetnets, foldinds, basics,
                                     bagged_selection, n_bagging_samples, confidence_level)
            done_bagged[k] = true
        end
    end

    for (b, M) in zip(done_bagged, metric_types_trace_bagged)
        if !b
            j += 1
            extract_trace_scores!(M, frame_scores, foldinds, behavior, dset)
            retval_bagged[j] = BaggedMetricResult(M, dset.pdset_segments,
                                     pdsets_original, pdsets_for_simulation, validfind_starts_sim,
                                     streetnets, foldinds, basics,
                                     bagged_selection, n_bagging_samples, confidence_level)
        end
    end

    (retval_straight, retval_bagged)
end
function extract_trace_metrics{B<:AbstractVehicleBehavior}(
    metric_types_trace::Vector{DataType},
    metric_types_trace_bagged::Vector{DataType},

    behaviors::Vector{B},
    dset::ModelTrainingData,
    assignment::FoldAssignment,
    fold::Int,
    match_fold::Bool,

    pdsets_original::Vector{PrimaryDataset},
    streetnets::Vector{StreetNetwork},

    n_simulations_per_trace::Int=DEFAULT_N_SIMULATIONS_PER_TRACE,
    n_bagging_samples::Int=DEFAULT_N_BAGGING_SAMPLES, # number of bootstrap aggregated samples to take from data to eval on
    confidence_level::Float64=DEFAULT_CONFIDENCE_LEVEL,
    trace_history::Int=DEFAULT_TRACE_HISTORY,
    )

    #=
    Returns
        Vector{Vector{BehaviorTraceMetric}} over behaviors, then metric_types_trace
        Vector{Vector{BaggedMetricResult}} over behaviors, then metric_types_trace_bagged
    =#

    @assert(n_simulations_per_trace > 0)
    @assert(n_bagging_samples > 1)

    # allocate memory
    n = calc_fold_size(fold, assignment.pdsetseg_assignment, match_fold)
    foldinds = calc_fold_inds!(Array(Int, n), fold, assignment.pdsetseg_assignment, match_fold)
    bagged_selection = Array(Int, n)

    # make pdset copies that are only as large as needed (contain history and horizon from pdsets_original)
    pdsets_for_simulation = Array(PrimaryDataset, n, n_simulations_per_trace)
    validfind_starts_sim = Array(Int, n) # new validfind_start for the truncated pdsets_for_simulation

    for (i,ind) in enumerate(foldinds)
        seg = dset.pdset_segments[ind]
        validfind_start = max(1, seg.validfind_start - DEFAULT_TRACE_HISTORY)
        pdset_sim = deepcopy(pdsets_original[seg.pdset_id], validfind_start, seg.validfind_end)
        validfind_starts_sim[i] = clamp(seg.validfind_start-DEFAULT_TRACE_HISTORY, 1, DEFAULT_TRACE_HISTORY+1)

        # print(seg.validfind_start, "  ", validfind_start, "  ", validfind_starts_sim[i], "   ")
        # print(pdsets_original[seg.pdset_id].df_ego[validfind2frameind(pdsets_original[seg.pdset_id], seg.validfind_start),:posGx], "   ")
        # print(pdset_sim.df_ego[validfind2frameind(pdset_sim, validfind_starts_sim[i]), :posGx], "   ")
        # print(pdsets_original[seg.pdset_id].df_ego[validfind2frameind(pdsets_original[seg.pdset_id], seg.validfind_end),:posGx], "   ")
        # println(pdset_sim.df_ego[validfind2frameind(pdset_sim, validfind_starts_sim[i] + seg.validfind_end - seg.validfind_start), :posGx])

        pdsets_for_simulation[i,1] = pdset_sim
        for j in 2 : n_simulations_per_trace
            pdsets_for_simulation[i,j] = deepcopy(pdset_sim)
        end
    end

    retval_straight = Array(Vector{BehaviorTraceMetric}, length(behaviors))
    retval_bagged = Array(Vector{BaggedMetricResult}, length(behaviors))

    for (i,b) in enumerate(behaviors)
        (c,d) = extract_trace_metrics(
                        metric_types_trace, metric_types_trace_bagged,
                        b, dset, pdsets_original,
                        pdsets_for_simulation, validfind_starts_sim,
                        streetnets, foldinds, bagged_selection,
                        n_bagging_samples, confidence_level,
                        )
        retval_straight[i] = c
        retval_bagged[i] = d
    end

    (retval_straight, retval_bagged)
end


#########################################################################################################
# BaggedMetric


function extract_bagged_metrics(
    metric_types_frame_test::Vector{DataType},

    behavior::AbstractVehicleBehavior,
    dset::ModelTrainingData,
    assignment::FoldAssignment,
    fold::Int,
    match_fold::Bool;

    n_bagging_samples::Int=DEFAULT_N_BAGGING_SAMPLES, # number of bootstrap aggregated samples to take from data to eval on
    confidence_level::Float64=0.95,
    )

    @assert(n_bagging_samples > 1)

    nrows = nrow(dset.dataframe)
    n_metrics = length(metric_types_frame_test)
    frame_scores = Array(Float64, nrows, n_metrics)
    nvalid_frames = 0

    for frameind in 1 : nrows
        if is_in_fold(fold, assignment.frame_assignment[frameind], match_fold)
            nvalid_frames += 1
            for (j,metric_type) in enumerate(metric_types_frame_test)
                frame_scores[nvalid_frames, j] = get_frame_score(metric_type, dset, behavior, frameind)
            end
        end
    end

    z = 1.41421356237*erfinv(confidence_level)
    bag_range = 1 : nvalid_frames
    bootstrap_scores = zeros(Float64, n_bagging_samples)
    retval = Array(BaggedMetricResult, n_metrics)
    for (j,metric_type) in enumerate(metric_types_frame_test)

        for i in 1 : n_bagging_samples
            for k in bag_range
                row_index = rand(bag_range)
                bootstrap_scores[i] += frame_scores[row_index, j]
            end
            bootstrap_scores[i] /= nvalid_frames
        end

        μ = mean(bootstrap_scores)
        σ = stdm(bootstrap_scores, μ)

        confidence_bound = z * σ / sqrt(n_bagging_samples)

        retval[j] = BaggedMetricResult(metric_type, μ, σ, confidence_bound, confidence_level, n_bagging_samples)
    end

    retval
end
function extract_bagged_metrics{B<:AbstractVehicleBehavior}(
    metric_types_frame_test::Vector{DataType},

    behaviors::Vector{B},
    dset::ModelTrainingData,
    assignment::FoldAssignment,
    fold::Int,
    match_fold::Bool;

    n_bagging_samples::Int=DEFAULT_N_BAGGING_SAMPLES, # number of bootstrap aggregated samples to take from data to eval on
    confidence_level::Float64=0.95,
    )

    @assert(n_bagging_samples > 1)

    retval = Array(Vector{BaggedMetricResult}, length(behaviors))
    for (i,b) in enumerate(behaviors)
        retval[i] = extract_bagged_metrics(
                            metric_types_frame_test, b, dset, assignment,
                            fold, match_fold, n_bagging_samples=n_bagging_samples,
                            confidence_level=confidence_level
                            )
    end
    retval
end
function extract_bagged_metrics_from_traces{B<:AbstractVehicleBehavior}(
    metric_types_trace_test::Vector{DataType},
    behaviors::Vector{B},

    pdsets_original::Vector{PrimaryDataset},
    pdsets_for_simulation::Vector{PrimaryDataset},
    streetnets::Vector{StreetNetwork},
    pdset_segments::Vector{PdsetSegment},

    assignment::FoldAssignment,
    fold::Int,
    match_fold::Bool; # true for CV where we test on withheld data

    n_bagging_samples::Int=DEFAULT_N_BAGGING_SAMPLES, # number of bootstrap aggregated samples to take from data to eval on
    confidence_level::Float64=0.95,
    )

    @assert(n_bagging_samples > 1)

    ntraces = length(pdset_segments)
    n_metrics = length(metric_types_trace_test)
    n_models = length(behaviors)

    retval = Array(Vector{BaggedMetricResult}, n_models)
    if isempty(metric_types_trace_test)
        for i in 1 : length(retval)
            retval[i] = BaggedMetricResult[]
        end
        return retval
    end

    # select samples
    n_valid_samples = calc_fold_size(fold, assignment.pdsetseg_assignment, match_fold)

    # calc mapping from bagged index to pdset index
    valid_sample_indeces = Array(Int, n_valid_samples)
    fold_count = 0
    for (i,a) in enumerate(assignment.pdsetseg_assignment)
        if is_in_fold(fold, a, match_fold)
            fold_count += 1
            valid_sample_indeces[fold_count] = i
        end
    end

    bagged_assignment = FoldAssignment(Int[], ones(Int, n_valid_samples), 1) # take 'em all
    bagged_pdset_segments = Array(PdsetSegment, n_valid_samples)

    bootstrap_scores = Array(Float64, n_bagging_samples, n_metrics, n_models)
    sample_range = 1:n_valid_samples
    for i = 1 : n_bagging_samples

        # sample a bagged set
        for k = 1 : n_valid_samples
            bagged_pdset_segments[k] = pdset_segments[valid_sample_indeces[rand(sample_range)]]
        end

        # compute metrics for all behaviors ::Vector{Vector{BehaviorTraceMetric}}
        metrics = extract_metrics_from_traces(metric_types_trace_test, behaviors,
                        pdsets_original, pdsets_for_simulation, streetnets, bagged_pdset_segments,
                        bagged_assignment, 1, true)

        # compute scores on those traces
        for k in 1 : n_models
            metric_list = metrics[k+1] # skip the realworld result
            for (j,metric) in enumerate(metric_list)
                bootstrap_scores[i,j,k] = get_score(metric)
            end
        end
    end

    # compute bagged metrics
    z = 1.41421356237*erfinv(confidence_level)
    for k in 1 : n_models
        retval_list = Array(BaggedMetricResult, n_metrics)
        for (j,metric_type) in enumerate(metric_types_trace_test)
            μ = mean(bootstrap_scores[:,j,k])
            σ = stdm(bootstrap_scores[:,j,k], μ)
            confidence_bound = z * σ / sqrt(n_bagging_samples)
            retval_list[j] = BaggedMetricResult(metric_type, μ, σ, confidence_bound,
                                                confidence_level, n_bagging_samples)
        end
        retval[k] = retval_list
    end

    retval
end

#########################################################################################################

function extract_metrics(
    metric_types::Vector{DataType}, # each type should be a BehaviorMetric
    dset::ModelTrainingData,
    behavior::AbstractVehicleBehavior,
    assignment::FoldAssignment,
    fold::Int,
    match_fold::Bool,
    )

    retval = Array(BehaviorFrameMetric, length(metric_types))
    for (i,metric_type) in enumerate(metric_types)
        retval[i] = extract(metric_type, dset, behavior, assignment, fold, match_fold)
    end
    retval
end
function extract_metrics{B<:AbstractVehicleBehavior}(
    metric_types::Vector{DataType},
    models::Vector{B},
    dset::ModelTrainingData,
    assignment::FoldAssignment,
    fold::Int,
    match_fold::Bool,
    )

    retval = Array(Vector{BehaviorFrameMetric}, length(models))

    if isempty(metric_types)
        for i = 1 : length(models)
            retval[i] = BehaviorMetric[]
        end
        return retval
    end


    for (i,model) in enumerate(models)
        retval[i] = extract_metrics(metric_types, dset, model, assignment, fold, match_fold)
    end
    retval
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

function calc_rmse_predicted_vs_ground_truth(
    pdset_predicted::PrimaryDataset,
    pdset_truth::PrimaryDataset,
    seg::PdsetSegment,
    nframes::Int=get_horizon(seg)
    )

    #=
    RMSE: Root-Mean Square Error

    the square root of the mean square error
    =#

    @assert(nframes > 0)
    @assert(nframes ≤ get_horizon(seg))

    total = 0.0
    validfind = seg.validfind_start - 1
    for i = 1 : nframes
        validfind += 1

        carind_predicted = carid2ind(pdset_predicted, seg.carid, validfind)
        carind_truth     = carid2ind(pdset_truth,     seg.carid, validfind)

        pos_x_predicted = get(pdset_predicted, :posGx, carind_predicted, validfind)::Float64
        pos_y_predicted = get(pdset_predicted, :posGy, carind_predicted, validfind)::Float64
        pos_x_truth     = get(pdset_truth,     :posGx, carind_truth,     validfind)::Float64
        pos_y_truth     = get(pdset_truth,     :posGy, carind_truth,     validfind)::Float64

        Δx = pos_x_predicted - pos_x_truth
        Δy = pos_y_predicted - pos_y_truth

        if i == 1
            if abs(Δx) > 0.01 || abs(Δy) > 0.01
                println("validfind:        ", validfind)
                println("Δx:               ", Δx)
                println("Δy:               ", Δy)
                println("pos_x_predicted:  ", pos_x_predicted)
                println("pos_y_predicted:  ", pos_y_predicted)
                println("pos_x_truth:      ", pos_x_truth)
                println("pos_y_truth:      ", pos_y_truth)
                println("carind_predicted: ", carind_predicted)
                println("carind_truth:     ", carind_truth)
                println("seg:              ", seg)
            end
            @assert(abs(Δx) < 0.01, "Must have no initial deviation")
            @assert(abs(Δy) < 0.01, "Must have no initial deviation")
        end

        total += Δx*Δx + Δy*Δy
    end
    sqrt(total / nframes)
end

calc_probability_for_uniform_sample_from_bin(bindiscreteprob::Float64, binwidth::Float64) = bindiscreteprob / binwidth
function calc_probability_for_uniform_sample_from_bin(bindiscreteprob::Float64, disc::LinearDiscretizer, binindex::Int)
    width_of_bin = binwidth(disc, binindex)
    calc_probability_for_uniform_sample_from_bin(bindiscreteprob, width_of_bin)
end

function extract_metrics_from_traces(
    metric_types::Vector{DataType},
    metrics_realworld::Vector{BehaviorTraceMetric},
    behavior::AbstractVehicleBehavior,

    basics::FeatureExtractBasicsPdSet,
    pdsets_original::Vector{PrimaryDataset},
    pdsets_for_simulation::Vector{PrimaryDataset},
    streetnets::Vector{StreetNetwork},
    pdset_segments::Vector{PdsetSegment},

    assignment::FoldAssignment,
    fold::Int,
    match_fold::Bool,
    )

    metrics = Array(BehaviorTraceMetric, length(metric_types))
    for (i,T) in enumerate(metric_types)
        metrics[i] = init(T)
    end

    for (fold_assignment, seg) in zip(assignment.pdsetseg_assignment, pdset_segments)
        if is_in_fold(fold, fold_assignment, match_fold)

            pdset_orig = pdsets_original[seg.pdset_id]
            basics.pdset = pdsets_for_simulation[seg.pdset_id]
            basics.sn = streetnets[seg.streetnet_id]
            basics.runid += 1
            copy_trace!(basics.pdset, pdset_orig, seg.carid, seg.validfind_start, seg.validfind_end)
            simulate!(basics, behavior, seg.carid, seg.validfind_start, seg.validfind_end)

            for metric in metrics
                update_metric!(metric, behavior, pdset_orig, basics, seg)
            end
        end
    end

    for (metric, metric_realworld) in zip(metrics, metrics_realworld)
        complete_metric!(metric, metric_realworld, behavior)
    end

    metrics
end
function extract_metrics_from_traces{B<:AbstractVehicleBehavior}(
    metric_types::Vector{DataType}, # BehaviorTraceMetric
    behaviors::Vector{B},

    pdsets_original::Vector{PrimaryDataset},
    pdsets_for_simulation::Vector{PrimaryDataset},
    streetnets::Vector{StreetNetwork},
    pdset_segments::Vector{PdsetSegment},

    assignment::FoldAssignment,
    fold::Int,
    match_fold::Bool, # true for CV where we test on withheld data
    )

    if isempty(metric_types)
        retval = Array(Vector{BehaviorTraceMetric}, length(behaviors)+1)
        for i = 1 : length(retval)
            retval[i] = BehaviorTraceMetric[]
        end
        return retval
    end

    metrics_realworld = Array(BehaviorTraceMetric, length(metric_types))
    for (i,T) in enumerate(metric_types)
        metrics_realworld[i] = init(T)
    end

    basics = FeatureExtractBasicsPdSet(pdsets_original[1], streetnets[1])
    for (fold_assignment, seg) in zip(assignment.pdsetseg_assignment, pdset_segments)
        if is_in_fold(fold, fold_assignment, match_fold)
            pdset_orig = pdsets_original[seg.pdset_id]
            basics.pdset = pdset_orig
            basics.sn = streetnets[seg.streetnet_id]
            for metric in metrics_realworld
                update_metric!(metric, VEHICLE_BEHAVIOR_NONE, pdset_orig, basics, seg)
            end
        end
    end

    retval = Array(Vector{BehaviorTraceMetric}, length(behaviors)+1)
    retval[1] = metrics_realworld
    for (i,behavior) in enumerate(behaviors)
        retval[i+1] = extract_metrics_from_traces(metric_types, metrics_realworld, behavior,
                                                  basics, pdsets_original, pdsets_for_simulation,
                                                  streetnets, pdset_segments, assignment, fold, match_fold)
    end

    retval
end

function calc_mean_cross_validation_metrics(aggmetric_set::Vector{Dict{Symbol,Any}})
    retval = Dict{Symbol, Any}[]

    n = length(aggmetric_set)
    temparr = Array(Float64, n)

    keyset = keys(aggmetric_set[1])
    for key in keyset
        for (i,aggmetrics) in enumerate(aggmetric_set)
            temparr[i] = Float64(get(aggmetrics, key, NaN))
        end
        μ = mean(temparr)
        σ = stdm(temparr, μ)
        retval[symbol(string(key)*"_mean")] = μ
        retval[symbol(string(key)*"_stdev")] = σ
    end

    retval
end