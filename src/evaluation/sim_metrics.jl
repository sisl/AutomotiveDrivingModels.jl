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

# type EvaluationParams
#     sim_history_in_frames::Int
#     simparams::SimParams
#     histobin_params::ParamsHistobin
# end

# type HistobinMetric <: BehaviorMetric
#     histobin::Matrix{Float64} # histobin image over deviation during run
#     histobin_kldiv::Float64 # kldivergence with respect to original histobin
# end
# type TraceMetricSet <: BehaviorMetric
#     # collection of individually extracted metrics across all traces
#     metrics::Vector{Dict{Symbol,Any}}
# end
# type AggregateTraceMetrics <: BehaviorMetric
#     # metrics derived by aggregated over traces
#     metrics::Dict{Symbol,Any}
# end



# type MetricsSet
#     histobin::Matrix{Float64} # histobin image over deviation during run
#     histobin_kldiv::Float64 # kldivergence with respect to original histobin

#     tracemetrics::Vector{Dict{Symbol, Any}} # metric dictionary computed for every trace
#     aggmetrics::Dict{Symbol,Any} # mean and stdev metrics across traces

#     counts_speed::Vector{Int}
#     counts_timegap::Vector{Int}
#     counts_laneoffset::Vector{Int}
# end

# function extract(::Type{HistobinMetric}, dset::ModelTrainingData)

#         histobin = calc_histobin(pdsets_original, streetnets, pdset_segments, histobin_params,
#                                  fold, pdsetseg_fold_assignment, match_fold)

# end

#########################################################################################################
# EmergentKLDivMetric

type EmergentKLDivMetric{feature_symbol} <: BehaviorTraceMetric
    counts::Vector{Int}
    kldiv::Float64
end

const KLDIV_METRIC_NBINS = 20
const KLDIV_METRIC_DISC_DICT = [
        symbol(SPEED) => LinearDiscretizer(linspace(0.0,35.0,KLDIV_METRIC_NBINS), Int),
        symbol(TIMEGAP_X_FRONT) => LinearDiscretizer(linspace(0.0,10.0,KLDIV_METRIC_NBINS), Int),
        symbol(D_CL) => LinearDiscretizer(linspace(-3.0,3.0,KLDIV_METRIC_NBINS), Int),
    ]


init{Fsym}(::Type{EmergentKLDivMetric{Fsym}}) = EmergentKLDivMetric{Fsym}(ones(Int, KLDIV_METRIC_NBINS), NaN)
function update_metric!{Fsym}(
    metric::EmergentKLDivMetric{Fsym},
    ::AbstractVehicleBehavior,
    pdset_orig::PrimaryDataset,
    basics::FeatureExtractBasicsPdSet, # NOTE(tim): the pdset here is what we use
    seg::PdsetSegment
    )

    validfind_end = seg.validfind_end
    carid = seg.carid
    carind = carid2ind(basics.pdset, carid, validfind_end)
    v = get(symbol2feature(Fsym), basics, carind, validfind_end)
    metric.counts[encode(KLDIV_METRIC_DISC_DICT[Fsym], v)] += 1

    metric
end
function complete_metric!{Fsym}(
    metric::EmergentKLDivMetric{Fsym},
    metric_realworld::EmergentKLDivMetric{Fsym},
    ::AbstractVehicleBehavior,
    )

    metric.kldiv = calc_kl_div_categorical(metric_realworld.counts, metric.counts)
    metric
end
get_score(metric::EmergentKLDivMetric) = metric.kldiv

#########################################################################################################
# RootWeightedSquareError

# F isa AbstractFeature
type RootWeightedSquareError{feature_symbol} <: BehaviorTraceMetric
    # RWSE for the given horizon

    running_sums::Vector{Float64}
    N::Int

    rwse_arr::Vector{Float64}
end
const RWSE_HORIZONS = [1.0,2.0,3.0,4.0] # [s]

init{Fsym}(::Type{RootWeightedSquareError{Fsym}}) = RootWeightedSquareError{Fsym}(zeros(Float64, length(RWSE_HORIZONS)), 0, Array(Float64, length(RWSE_HORIZONS)))
function update_metric!{Fsym}(
    metric::RootWeightedSquareError{Fsym},
    behavior::AbstractVehicleBehavior,
    pdset_orig::PrimaryDataset,
    basics::FeatureExtractBasicsPdSet, # NOTE(tim): the pdset here is what we use
    seg::PdsetSegment,

    n_monte_carlo_samples::Int=10
    )

    @assert((seg.validfind_end - seg.validfind_start)*DEFAULT_SEC_PER_FRAME ≥ RWSE_HORIZONS[end])

    #=
    The Root Weighted Square Error over the target feature F for horizon [sec]

    √(Σ ∫p(f)⋅(f(t+h) - fₑ(t+h))² df ) / N

    The square difference between true and predicted value weighted by probability.
    This function approximates the above value using a Monte Carlo approach:

    √(Σ_traj Σ_mc Σ_t (f(t+h) - fₑ(t+h))²/M ) / N

        N = number of trajectories
        M = n_monte_carlo_samples
        t runs from 0 to horizon (error at 0 should is 0, so is skipped)
    =#

    F = symbol2feature(Fsym)
    carid = seg.carid

    basics_orig = FeatureExtractBasicsPdSet(pdset_orig, basics.sn)
    v_true_arr = Float64[]
    for validfind in seg.validfind_start + N_FRAMES_PER_SIM_FRAME : N_FRAMES_PER_SIM_FRAME : seg.validfind_end
        carind = carid2ind(pdset_orig, carid, validfind)
        push!(v_true_arr, get(F, basics_orig, carind, validfind))
    end

    n_horizons = length(RWSE_HORIZONS)
    rwse_components = zeros(Float64, n_horizons)
    for i in 1 : n_monte_carlo_samples

        basics.runid += 1
        copy_trace!(basics.pdset, pdset_orig, seg.carid, seg.validfind_start, seg.validfind_end)
        simulate!(basics, behavior, seg.carid, seg.validfind_start, seg.validfind_end)

        for (j,validfind) in enumerate(seg.validfind_start + N_FRAMES_PER_SIM_FRAME : N_FRAMES_PER_SIM_FRAME : seg.validfind_end)

            v_true = v_true_arr[j]
            carind = carid2ind(pdset_orig, carid, validfind)
            v_montecarlo = get(F, basics, carind, validfind)

            Δ = v_true - v_montecarlo
            Δ2 = Δ*Δ
            h = n_horizons
            Δt = (validfind - seg.validfind_start)*DEFAULT_SEC_PER_FRAME
            while h > 0 && RWSE_HORIZONS[h] ≥ Δt
                rwse_components[h] += Δ2
                h -= 1
            end
        end
    end
    for h = 1 : n_horizons
        rwse_components[h] /= n_monte_carlo_samples
        metric.running_sums[h] += rwse_components[h]
    end

    metric.N += 1

    metric
end
function update_metric!(
    metric::RootWeightedSquareError,
    behavior::VehicleBehaviorNone,
    pdset_orig::PrimaryDataset,
    basics::FeatureExtractBasicsPdSet, # NOTE(tim): the pdset here is what we use
    seg::PdsetSegment,
    )

    # do nothing for vehicle behavior none (ie, realworld version)

    metric
end
function complete_metric!(
    metric::RootWeightedSquareError,
    metric_realworld::RootWeightedSquareError,
    ::AbstractVehicleBehavior,
    )

    for (i,s) in enumerate(metric.running_sums)
        metric.rwse_arr[i] = sqrt(s / metric.N)
    end

    metric
end
get_score(metric::RootWeightedSquareError) = metric.rwse_arr[end] # NOTE(tim): only extracts final RWSE

#########################################################################################################
# LoglikelihoodMetric

type LoglikelihoodMetric <: BehaviorFrameMetric
    # log likelihood of dataset frames on model
    logl::Float64
end
function extract(::Type{LoglikelihoodMetric},
    dset::ModelTrainingData,
    behavior::AbstractVehicleBehavior,
    assignment::FoldAssignment,
    fold::Int,
    match_fold::Bool,
    )

    logl = 0.0
    for frameind in 1 : nrow(dset.dataframe)
        if is_in_fold(fold, assignment.frame_assignment[frameind], match_fold)
            logl += calc_action_loglikelihood(behavior, dset.dataframe, frameind)
        end
    end
    LoglikelihoodMetric(logl)
end
get_score(metric::LoglikelihoodMetric) = metric.logl

#########################################################################################################
# BaggedMetric

immutable BaggedMetricResult
    M::DataType # datatype of the source BehaviorMetric
    μ::Float64 # sample mean
    σ::Float64 # sample standard deviation
    confidence_bound::Float64
    confidence_level::Float64 # ie, 0.95 indicates 95% confidence level
    n_bagging_samples::Int
end
function extract_bagged_metrics(
    metric_types_frame_test::Vector{DataType},

    behavior::AbstractVehicleBehavior,
    dset::ModelTrainingData,
    assignment::FoldAssignment,
    fold::Int,
    match_fold::Bool;

    n_bagging_samples::Int=10, # number of bootstrap aggregated samples to take from data to eval on
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
                metric = extract(metric_type, dset, behavior, assignment, fold, match_fold)
                frame_scores[nvalid_frames, j] = get_score(metric)
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
                bootstrap_scores[i] += frame_scores[row_index]
            end
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

    n_bagging_samples::Int=10, # number of bootstrap aggregated samples to take from data to eval on
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

    n_bagging_samples::Int=10, # number of bootstrap aggregated samples to take from data to eval on
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

    bootstrap_scores = Array(Float64, n_valid_samples, n_metrics, n_models)
    sample_range = 1:n_valid_samples
    for i = 1 : n_bagging_samples

        # sample a bagged set
        for k = 1 : n_valid_samples
            bagged_pdset_segments[k] = pdset_segments[rand(sample_range)]
        end

        # compute metrics for all behaviors ::Vector{Vector{BehaviorTraceMetric}}
        metrics = extract_metrics_from_traces(metric_types_trace_test, behaviors,
                        pdsets_original, pdsets_for_simulation, streetnets, bagged_pdset_segments,
                        bagged_assignment, 1, true)

        # compute scores on said trace
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

function calc_kl_div_categorical{I<:Integer, J<:Integer}(counts_p::AbstractVector{I}, counts_q::AbstractVector{J})

    #=
    Calculate the KL-divergence between two categorical distributions
    (also works if is it a piecewise uniform univariate with equally-spaced bins)
    =#

    tot_p = sum(counts_p)
    tot_q = sum(counts_q)
    qp_ratio = tot_q / tot_p

    kldiv = 0.0
    for (p,q) in zip(counts_p, counts_q)
        if p > 0 # isapprox(p,0.0), to avoid divide-by-zero errors
            kldiv += p/tot_p * log(p/q*qp_ratio)
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

# function calc_tracemetrics(
#     pdset::PrimaryDataset,
#     sn::StreetNetwork,
#     seg::PdsetSegment;
#     basics::FeatureExtractBasicsPdSet=FeatureExtractBasicsPdSet(pdset, sn),
#     pdset_frames_per_sim_frame::Int=5
#     )

#     has_collision = false # whether any collision occurs
#     has_collision_activecar = false # whether the active car collides with another car
#     n_lanechanges = 0 # number of lanechanges the active car makes
#     elapsed_time = get_elapsed_time(pdset, seg.validfind_start, seg.validfind_end)

#     speed = StreamStats.Var()
#     headway = StreamStats.Var()
#     timegap = StreamStats.Var()
#     centerline_offset = StreamStats.Var()

#     validfind_of_first_offroad = -1
#     n_frames_offroad = 0

#     cur_lanetag = get(pdset, :lanetag, carid2ind(pdset, seg.carid, seg.validfind_start), seg.validfind_start)::LaneTag
#     cur_lane = get_lane(sn, cur_lanetag)

#     for validfind in seg.validfind_start : pdset_frames_per_sim_frame : seg.validfind_end
#         # NOTE(tim): this assumes that the validfinds are continuous

#         carind_active = carid2ind(pdset, seg.carid, validfind)

#         if validfind > seg.validfind_start
#             fut_lanetag = get(pdset, :lanetag, carind_active, validfind)::LaneTag
#             if fut_lanetag != cur_lanetag
#                 if same_tile(cur_lanetag, fut_lanetag) || !has_next_lane(sn, cur_lane)
#                     lane_change_occurred = true
#                 else
#                     cur_lane = next_lane(sn, cur_lane)
#                     cur_lanetag = cur_lane.id
#                     lane_change_occurred = fut_lanetag != cur_lanetag
#                 end

#                 if lane_change_occurred
#                     n_lanechanges += 1
#                 end
#             end
#         end

#         update!(speed, Features._get(SPEED, basics, carind_active, validfind))
#         (best_carind, d_x_front, d_y_front) = Features._get_carind_front_and_dist(basics, carind_active, validfind)
#         update!(headway, d_x_front)

#         if !isinf(d_x_front)
#             timegap_x_front = Features.THRESHOLD_TIMEGAP
#         else
#             v  = get(basics.pdset, :velFx, carind_active, validfind) # our current velocity

#             if v <= 0.0
#                 timegap_x_front = Features.THRESHOLD_TIMEGAP
#             else
#                 timegap_x_front = min(d_x_front / v, Features.THRESHOLD_TIMEGAP)
#             end
#         end
#         update!(timegap, timegap_x_front)


#         offset = get(pdset, :d_cl, carind_active, validfind)
#         update!(centerline_offset, offset)
#         is_on_road = abs(offset) < 2.0 # NOTE(tim): there should be a call for this in StreetNetwork

#         if !is_on_road
#             n_frames_offroad += 1
#             validfind_of_first_offroad = min(validfind, validfind_of_first_offroad)
#         end

#         # check for collision
#         posGx = get(pdset, :posGx, carind_active, validfind)
#         posGy = get(pdset, :posGy, carind_active, validfind)

#         max_carind = get_maxcarind(pdset, validfind)
#         for carind in -1 : max_carind
#             if carind != carind_active

#                 posGx_target = get(pdset, :posGx, carind, validfind)
#                 posGy_target = get(pdset, :posGy, carind, validfind)

#                 Δx = posGx - posGx_target
#                 Δy = posGx - posGy_target

#                 # TODO(tim): these attributes should be in PdSet
#                 # TODO(tim): we should be using a better method for collision checking
#                 if abs(Δx) < DEFAULT_CAR_LENGTH && abs(Δy) < DEFAULT_CAR_WIDTH
#                     has_collision_activecar = true
#                     # NOTE(tim): anything after a collision is invalid - break here
#                     break
#                 end

#                 for carind2 in -1 : max_carind
#                     if carind2 != carind_active &&
#                        carind2 != carind

#                         Δx = posGx_target - get(pdset, :posGx, carind2, validfind)
#                         Δy = posGy_target - get(pdset, :posGy, carind2, validfind)

#                         # TODO(tim): these attributes should be in PdSet
#                         # TODO(tim): we should be using a better method for collision checking
#                         if abs(Δx) < DEFAULT_CAR_LENGTH && abs(Δy) < DEFAULT_CAR_WIDTH
#                             has_collision = true
#                         end
#                     end
#                 end
#             end
#         end
#     end

#     went_offroad = (validfind_of_first_offroad == -1)

#     [
#      :has_collision => has_collision,
#      :has_collision_activecar => has_collision_activecar,
#      :n_lanechanges_activecar => n_lanechanges,
#      :mean_speed_activecar => mean(speed),
#      :mean_centerline_offset_activecar => mean(centerline_offset),
#      :std_speed_activecar => std(speed),
#      :std_centerline_offset_activecar => std(centerline_offset),
#      :n_sec_offroad_ego => went_offroad ? 0.0 : get_elapsed_time(pdset, validfind_of_first_offroad, seg.validfind_end),
#      :elapsed_time => elapsed_time,
#      :time_of_first_offroad => went_offroad ? Inf : gete(pdset, :time, validfind_of_first_offroad)::Float64,
#      :went_offroad => went_offroad,
#      :mean_headway => mean(headway),
#      :mean_timegap => mean(timegap),
#      :logl => NaN, # NOTE(tim): no defined behavior
#     ]::Dict{Symbol, Any}
# end
# function calc_tracemetrics(
#     behavior::AbstractVehicleBehavior,
#     pdset_original::PrimaryDataset,
#     pdset_simulation::PrimaryDataset,
#     sn::StreetNetwork,
#     seg::PdsetSegment;
#     basics::FeatureExtractBasicsPdSet=FeatureExtractBasicsPdSet(pdset_simulation, sn),
#     pdset_frames_per_sim_frame::Int=5
#     )

#     has_collision = false # whether any collision occurs
#     has_collision_activecar = false # whether the active car collides with another car
#     n_lanechanges = 0 # number of lanechanges the active car makes
#     elapsed_time = get_elapsed_time(pdset_simulation, seg.validfind_start, seg.validfind_end)

#     speed = StreamStats.Var()
#     headway = StreamStats.Var()
#     timegap = StreamStats.Var()
#     centerline_offset = StreamStats.Var()

#     total_action_loglikelihood = 0.0

#     validfind_of_first_offroad = -1
#     n_frames_offroad = 0

#     cur_lanetag = get(pdset_simulation, :lanetag, carid2ind(pdset_simulation, seg.carid, seg.validfind_start), seg.validfind_start)::LaneTag
#     cur_lane = get_lane(sn, cur_lanetag)

#     for validfind in seg.validfind_start : pdset_frames_per_sim_frame : seg.validfind_end
#         # NOTE(tim): this assumes that the validfinds are continuous

#         carind_active = carid2ind(pdset_simulation, seg.carid, validfind)

#         if validfind > seg.validfind_start
#             fut_lanetag = get(pdset_simulation, :lanetag, carind_active, validfind)::LaneTag
#             if fut_lanetag != cur_lanetag
#                 if same_tile(cur_lanetag, fut_lanetag) || !has_next_lane(sn, cur_lane)
#                     lane_change_occurred = true
#                 else
#                     cur_lane = next_lane(sn, cur_lane)
#                     cur_lanetag = cur_lane.id
#                     lane_change_occurred = fut_lanetag != cur_lanetag
#                 end

#                 if lane_change_occurred
#                     n_lanechanges += 1
#                 end
#             end
#         end

#         update!(speed, Features._get(SPEED, basics, carind_active, validfind))
#         (best_carind, d_x_front, d_y_front) = Features._get_carind_front_and_dist(basics, carind_active, validfind)
#         update!(headway, d_x_front)

#         if !isinf(d_x_front)
#             timegap_x_front = Features.THRESHOLD_TIMEGAP
#         else
#             v  = get(basics.pdset, :velFx, carind_active, validfind) # our current velocity

#             if v <= 0.0
#                 timegap_x_front = Features.THRESHOLD_TIMEGAP
#             else
#                 timegap_x_front = min(d_x_front / v, Features.THRESHOLD_TIMEGAP)
#             end
#         end
#         update!(timegap, timegap_x_front)

#         if validfind != seg.validfind_end
#             total_action_loglikelihood += calc_action_loglikelihood(basics, behavior, carind_active, validfind)
#             if isinf(total_action_loglikelihood)
#                 println(typeof(behavior))
#                 error("Inf action loglikelihood")
#             end
#         end

#         offset = get(pdset_simulation, :d_cl, carind_active, validfind)
#         update!(centerline_offset, offset)
#         is_on_road = abs(offset) < 2.0 # NOTE(tim): there should be a call for this in StreetNetwork

#         if !is_on_road
#             n_frames_offroad += 1
#             validfind_of_first_offroad = min(validfind, validfind_of_first_offroad)
#         end

#         # check for collision
#         posGx = get(pdset_simulation, :posGx, carind_active, validfind)
#         posGy = get(pdset_simulation, :posGy, carind_active, validfind)

#         max_carind = get_maxcarind(pdset_simulation, validfind)
#         for carind in -1 : max_carind
#             if carind != carind_active

#                 posGx_target = get(pdset_simulation, :posGx, carind, validfind)
#                 posGy_target = get(pdset_simulation, :posGy, carind, validfind)

#                 Δx = posGx - posGx_target
#                 Δy = posGx - posGy_target

#                 # TODO(tim): these attributes should be in PdSet
#                 # TODO(tim): we should be using a better method for collision checking
#                 if abs(Δx) < DEFAULT_CAR_LENGTH && abs(Δy) < DEFAULT_CAR_WIDTH
#                     has_collision_activecar = true
#                     # NOTE(tim): anything after a collision is invalid - break here
#                     break
#                 end

#                 for carind2 in -1 : max_carind
#                     if carind2 != carind_active &&
#                        carind2 != carind

#                         Δx = posGx_target - get(pdset_simulation, :posGx, carind2, validfind)
#                         Δy = posGy_target - get(pdset_simulation, :posGy, carind2, validfind)

#                         # TODO(tim): these attributes should be in PdSet
#                         # TODO(tim): we should be using a better method for collision checking
#                         if abs(Δx) < DEFAULT_CAR_LENGTH && abs(Δy) < DEFAULT_CAR_WIDTH
#                             has_collision = true
#                         end
#                     end
#                 end
#             end
#         end
#     end

#     went_offroad = (validfind_of_first_offroad == -1)

#     [
#      :has_collision => has_collision,
#      :has_collision_activecar => has_collision_activecar,
#      :n_lanechanges_activecar => n_lanechanges,
#      :mean_speed_activecar => mean(speed),
#      :mean_centerline_offset_activecar => mean(centerline_offset),
#      :std_speed_activecar => std(speed),
#      :std_centerline_offset_activecar => std(centerline_offset),
#      :n_sec_offroad_ego => went_offroad ? 0.0 : get_elapsed_time(pdset, validfind_of_first_offroad, seg.validfind_end),
#      :elapsed_time => elapsed_time,
#      :time_of_first_offroad => went_offroad ? Inf : gete(pdset, :time, validfind_of_first_offroad)::Float64,
#      :went_offroad => went_offroad,
#      :mean_headway => mean(headway),
#      :mean_timegap => mean(timegap),
#      :logl => total_action_loglikelihood,
#      :rmse_1000ms => calc_rmse_predicted_vs_ground_truth(pdset_simulation, pdset_original, seg, 4),
#      :rmse_2000ms => calc_rmse_predicted_vs_ground_truth(pdset_simulation, pdset_original, seg, 8),
#      :rmse_3000ms => calc_rmse_predicted_vs_ground_truth(pdset_simulation, pdset_original, seg, 12),
#      :rmse_4000ms => calc_rmse_predicted_vs_ground_truth(pdset_simulation, pdset_original, seg, 16),
#     ]::Dict{Symbol, Any}
# end
# function calc_tracemetrics(
#     pdsets::Vector{PrimaryDataset},
#     streetnets::Vector{StreetNetwork},
#     pdset_segments::Vector{PdsetSegment},
#     fold::Integer,
#     pdsetseg_fold_assignment::Vector{Int},
#     match_fold::Bool
#     )

#     fold_size = calc_fold_size(fold, pdsetseg_fold_assignment, match_fold)

#     @assert(length(pdset_segments) == length(pdsetseg_fold_assignment))

#     metrics = Array(Dict{Symbol, Any}, fold_size)
#     i = 0
#     for (fold_assignment, seg) in zip(pdsetseg_fold_assignment, pdset_segments)
#         if is_in_fold(fold, fold_assignment, match_fold)
#             i += 1
#             metrics[i] = calc_tracemetrics(pdsets[seg.pdset_id], streetnets[seg.streetnet_id], seg)
#         end
#     end

#     metrics
# end

# function calc_aggregate_metric(sym::Symbol, ::Type{Int}, metricset::Vector{Dict{Symbol, Any}})

#     # counts = Dict{Int,Int}()
#     # for i = 1 : length(metricset)
#     #     counts[metricset[i][sym]] = get(counts, metricset[i][sym], 0) + 1
#     # end

#     t_arr = [metricset[i][:elapsed_time] for i in 1 : length(metricset)]
#     tot_time = sum(t_arr)

#     arr = [metricset[i][sym] for i in 1 : length(metricset)]
#     ave = mean(arr)
#     stdev = stdm(arr, ave)
#     weighted_ave = sum([metricset[i][sym] * metricset[i][:elapsed_time] for i in 1 : length(metricset)]) / tot_time

#     (ave, stdev, weighted_ave)
# end
# function calc_aggregate_metric(sym::Symbol, ::Type{Bool}, metricset::Vector{Dict{Symbol, Any}})
#     n_true = sum([metricset[i][sym] for i in 1 : length(metricset)])
#     ave_time_to_true = sum([metricset[i][sym] ? metricset[i][:elapsed_time] : 0.0 for i in 1 : length(metricset)]) / n_true

#     t_arr = [metricset[i][:elapsed_time] for i in 1 : length(metricset)]
#     tot_time = sum(t_arr)

#     odds_true_per_run = n_true / length(metricset)
#     odds_true_per_sec = n_true / tot_time

#     (odds_true_per_run, odds_true_per_sec, ave_time_to_true)
# end
# function calc_aggregate_metric(sym::Symbol, ::Type{Float64}, metricset::Vector{Dict{Symbol, Any}}, use_abs=false)
#     if use_abs
#         arr = convert(Vector{Float64}, [abs(metricset[i][sym]) for i in 1 : length(metricset)])
#     else
#         arr = convert(Vector{Float64}, [metricset[i][sym] for i in 1 : length(metricset)])
#     end

#     inds = find(a->a!=Inf, arr)
#     arr = arr[inds]
#     ave = mean(arr)
#     stdev = stdm(arr, ave)
#     (ave, stdev)
# end

# function _calc_aggregate_metrics(tracemetrics::Vector{Dict{Symbol, Any}})

#     aggmetrics = (Symbol=>Any)[]

#     calc_and_add!(sym::Symbol, ::Type{Float64}, use_abs::Bool=false) = begin
#         res = calc_aggregate_metric(sym, Float64, tracemetrics, use_abs)
#         aggmetrics[symbol(string(sym)*"_mean")] = res[1]
#         aggmetrics[symbol(string(sym)*"_stdev")] = res[2]
#     end
#     add!(sym::Symbol, ::Type{Float64}) = begin
#         aggmetrics[symbol(string(sym)*"_mean")] = NaN
#         aggmetrics[symbol(string(sym)*"_stdev")] = NaN
#     end

#     calc_and_add!(:mean_centerline_offset_activecar, Float64, true)

#     for key in (
#             :mean_speed_activecar, :time_of_first_offroad, :n_lanechanges_activecar,
#             :mean_headway, :mean_timegap, :logl,
#             :rmse_1000ms, :rmse_2000ms, :rmse_3000ms, :rmse_4000ms
#         )
#         if haskey(tracemetrics[1], key)
#             calc_and_add!(key, Float64)
#         else
#             add!(key, Float64)
#         end
#     end

#     res = calc_aggregate_metric(:went_offroad, Bool, tracemetrics)
#     aggmetrics[:went_offroad_odds_true_per_run] = res[1]
#     aggmetrics[:went_offroad_odds_true_per_sec] = res[2]
#     aggmetrics[:went_offroad_ave_time_to_true] = res[3]

#     aggmetrics
# end
# function calc_aggregate_metrics(tracemetrics::Vector{Dict{Symbol, Any}})

#     aggmetrics = _calc_aggregate_metrics(tracemetrics)

#     aggmetrics[:mean_lane_offset_kldiv] = NaN
#     aggmetrics[:mean_speed_ego_kldiv] = NaN
#     aggmetrics[:mean_timegap_kldiv] = NaN
#     aggmetrics[:histobin_kldiv] = NaN

#     aggmetrics
# end
# function calc_aggregate_metrics(
#     tracemetrics::Vector{Dict{Symbol, Any}},
#     metrics_set_orig::MetricsSet,
#     histobin_kldiv::Float64,
#     counts_speed::Vector{Int},
#     counts_timegap::Vector{Int},
#     counts_laneoffset::Vector{Int},
#     )

#     aggmetrics = _calc_aggregate_metrics(tracemetrics)

#     aggmetrics[:mean_speed_ego_kldiv] = calc_kl_div_categorical(metrics_set_orig.counts_speed, counts_speed)
#     aggmetrics[:mean_timegap_kldiv] = calc_kl_div_categorical(metrics_set_orig.counts_timegap, counts_timegap)
#     aggmetrics[:mean_lane_offset_kldiv] = calc_kl_div_categorical(metrics_set_orig.counts_laneoffset, counts_laneoffset)
#     aggmetrics[:histobin_kldiv] = histobin_kldiv

#     # println("model")
#     # println("speed: ", counts_speed)
#     # println("timegap: ", counts_timegap)
#     # println("laneoffset: ", counts_laneoffset)
#     # println("\n")
#     # println("speed: ", aggmetrics[:mean_speed_ego_kldiv])
#     # println("timegap: ", aggmetrics[:mean_timegap_kldiv])
#     # println("laneoffset: ", aggmetrics[:mean_lane_offset_kldiv])

#     aggmetrics
# end


# function create_metrics_set_no_tracemetrics(
#     pdsets_original::Vector{PrimaryDataset},
#     streetnets::Vector{StreetNetwork},
#     pdset_segments::Vector{PdsetSegment},
#     histobin_params::ParamsHistobin,
#     fold::Integer,
#     pdsetseg_fold_assignment::Vector{Int},
#     match_fold::Bool
#     )

#     #=
#     Computes the MetricsSet for the original data

#     Note that this will NOT compute trace log likelihoods or RMSE values
#     as it makes no sense to compare against itself
#     =#

#     histobin = calc_histobin(pdsets_original, streetnets, pdset_segments, histobin_params,
#                              fold, pdsetseg_fold_assignment, match_fold)
#     tracemetrics = calc_tracemetrics(pdsets_original, streetnets, pdset_segments,
#                                      fold, pdsetseg_fold_assignment, match_fold)
#     aggmetrics = calc_aggregate_metrics(tracemetrics)


#     counts_speed = zeros(Int, nlabels(KLDIV_METRIC_DISC_SPEED))
#     counts_timegap = zeros(Int, nlabels(KLDIV_METRIC_DISC_TIMEGAP))
#     counts_laneoffset = zeros(Int, nlabels(KLDIV_METRIC_DISC_LANEOFFSET))

#     for (fold_assignment, seg) in zip(pdsetseg_fold_assignment, pdset_segments)
#         if is_in_fold(fold, fold_assignment, match_fold)

#             pdset = pdsets_original[seg.pdset_id]
#             sn = streetnets[seg.streetnet_id]
#             basics = FeatureExtractBasicsPdSet(pdset, sn)

#             validfind = seg.validfind_end
#             carind = carid2ind(pdset, seg.carid, validfind)

#             speed = get_speed(pdset, carind, validfind)
#             timegap = get(TIMEGAP_X_FRONT, basics, carind, validfind)
#             laneoffset = get(pdset, :d_cl, carind, validfind)

#             counts_speed[encode(KLDIV_METRIC_DISC_SPEED, speed)] += 1
#             counts_timegap[encode(KLDIV_METRIC_DISC_TIMEGAP, timegap)] += 1
#             counts_laneoffset[encode(KLDIV_METRIC_DISC_LANEOFFSET, laneoffset)] += 1
#         end
#     end

#     # println("speed: ", counts_speed)
#     # println("timegap: ", counts_timegap)
#     # println("laneoffset: ", counts_laneoffset)

#     MetricsSet(histobin, 0.0, Dict{Symbol, Any}[], aggmetrics,
#                counts_speed, counts_timegap, counts_laneoffset)
# end

# function create_metrics_sets_no_tracemetrics{B<:AbstractVehicleBehavior}(
#     model_behaviors::Vector{B},
#     pdsets_original::Vector{PrimaryDataset},
#     pdsets_for_simulation::Vector{PrimaryDataset},
#     streetnets::Vector{StreetNetwork},
#     pdset_segments::Vector{PdsetSegment},
#     evalparams::EvaluationParams,
#     fold::Integer,
#     pdsetseg_fold_assignment::Vector{Int},
#     match_fold::Bool, # if true, then we want to only include folds that match target (validation)
#                       # if false, then we want to only include folds that do not (training)
#     )

#     #=
#     Takes trained behavior models and computes MetricsSet for each
#     =#

#     num_models_plus_realworld = 1 + length(model_behaviors)
#     retval = Array(MetricsSet, num_models_plus_realworld)

#     # pull out the parameters
#     simparams = evalparams.simparams
#     histobin_params = evalparams.histobin_params

#     # compute original metrics
#     metrics_set_orig = create_metrics_set_no_tracemetrics(pdsets_original, streetnets, pdset_segments, histobin_params,
#                                                           fold, pdsetseg_fold_assignment, match_fold)
#     histobin_original_with_prior = copy(metrics_set_orig.histobin) .+ 1.0
#     retval[1] = metrics_set_orig

#     # simulate each behavior and compute the behavior metrics
#     basics = FeatureExtractBasicsPdSet(pdsets_for_simulation[1], streetnets[1])
#     behavior_pairs = Array((AbstractVehicleBehavior,Int), 1)
#     fold_size = calc_fold_size(fold, pdsetseg_fold_assignment, match_fold)
#     for (i, behavior) in enumerate(model_behaviors)

#         histobin = allocate_empty_histobin(histobin_params)
#         tracemetrics = Array(Dict{Symbol, Any}, fold_size)

#         counts_speed = zeros(Int, nlabels(KLDIV_METRIC_DISC_SPEED))
#         counts_timegap = zeros(Int, nlabels(KLDIV_METRIC_DISC_TIMEGAP))
#         counts_laneoffset = zeros(Int, nlabels(KLDIV_METRIC_DISC_LANEOFFSET))

#         fold_entry_count = 0
#         for (fold_assignment, seg) in zip(pdsetseg_fold_assignment, pdset_segments)

#             if is_in_fold(fold, fold_assignment, match_fold)

#                 fold_entry_count += 1

#                 pdset_orig = pdsets_original[seg.pdset_id]
#                 basics.pdset = pdsets_for_simulation[seg.pdset_id]
#                 basics.sn = streetnets[seg.streetnet_id]
#                 basics.runid += 1
#                 behavior_pairs[1] = (behavior, seg.carid)
#                 simulate!(basics, behavior_pairs, seg.validfind_start, seg.validfind_end)

#                 validfind = seg.validfind_end
#                 carind = carid2ind(pdset_orig, seg.carid, validfind)

#                 speed = get_speed(pdset_orig, carind, validfind)
#                 timegap = get(TIMEGAP_X_FRONT, basics, carind, validfind)
#                 laneoffset = get(pdset_orig, :d_cl, carind, validfind)

#                 counts_speed[encode(KLDIV_METRIC_DISC_SPEED, speed)] += 1
#                 counts_timegap[encode(KLDIV_METRIC_DISC_TIMEGAP, timegap)] += 1
#                 counts_laneoffset[encode(KLDIV_METRIC_DISC_LANEOFFSET, laneoffset)] += 1

#                 # update the MetricsSet calculation
#                 update_histobin!(histobin, basics.pdset, basics.sn, seg, histobin_params)
#                 tracemetrics[fold_entry_count] = calc_tracemetrics(behavior, pdset_orig, basics.pdset, basics.sn, seg, basics=basics)

#                 # now override the pdset with the original values once more
#                 copy_trace!(basics.pdset, pdset_orig, seg.carid, seg.validfind_start, seg.validfind_end)
#             end
#         end

#         histobin_kldiv = KL_divergence_dirichlet(histobin_original_with_prior, histobin .+ 1.0 )
#         aggmetrics = calc_aggregate_metrics(tracemetrics, metrics_set_orig, histobin_kldiv,
#                                             counts_speed, counts_timegap, counts_laneoffset)

#         retval[i+1] = MetricsSet(histobin, histobin_kldiv, Dict{Symbol, Any}[], aggmetrics,
#                                  counts_speed, counts_timegap, counts_laneoffset)
#     end

#     retval
# end

# function compute_metric_summary_table{S<:String}(
#     behavior_metrics_sets::AbstractVector{MetricsSet},
#     original_metrics_set::MetricsSet,
#     model_names::Vector{S}
#     )


#     df = DataFrame(labels=["mean lane offset", "mean speed", "mean timegap",
#                            "mean lane offset kldiv", "mean speed kldiv", "mean timegap kldiv", "histobin kldiv",
#                            "mean trace log prob", "mean rmse 1s", "mean rmse 2s", "mean rmse 3s", "mean rmse 4s"])

#     aggmetrics_original = original_metrics_set.aggmetrics
#     df[:realworld] = [
#                         @sprintf("%.3f +- %.3f", aggmetrics_original[:mean_centerline_offset_activecar_mean], aggmetrics_original[:mean_centerline_offset_activecar_stdev]),
#                         @sprintf("%.3f +- %.3f", aggmetrics_original[:mean_speed_activecar_mean], aggmetrics_original[:mean_speed_activecar_stdev]),
#                         @sprintf("%.3f +- %.3f", aggmetrics_original[:mean_timegap_mean], aggmetrics_original[:mean_timegap_stdev]),
#                         "", "", "", "", "", "", "", "", ""
#                     ]

#     for (i,behavior_metrics_set) in enumerate(behavior_metrics_sets)

#         behavor_sym = symbol(model_names[i])
#         tracemetrics = behavior_metrics_set.tracemetrics
#         aggmetrics = behavior_metrics_set.aggmetrics

#         mean_trace_log_prob, stdev_trace_log_prob = calc_aggregate_metric(:logl, Float64, tracemetrics)
#         mean_rmse_1s, stdev_rmse_1s = calc_aggregate_metric(:rmse_1000ms, Float64, tracemetrics)
#         mean_rmse_2s, stdev_rmse_2s = calc_aggregate_metric(:rmse_2000ms, Float64, tracemetrics)
#         mean_rmse_3s, stdev_rmse_3s = calc_aggregate_metric(:rmse_3000ms, Float64, tracemetrics)
#         mean_rmse_4s, stdev_rmse_4s = calc_aggregate_metric(:rmse_4000ms, Float64, tracemetrics)

#         df[behavor_sym] = [
#                 @sprintf("%.3f +- %.3f", aggmetrics[:mean_centerline_offset_activecar_mean], aggmetrics[:mean_centerline_offset_activecar_stdev]),
#                 @sprintf("%.3f +- %.3f", aggmetrics[:mean_speed_activecar_mean], aggmetrics[:mean_speed_activecar_stdev]),
#                 @sprintf("%.3f +- %.3f", aggmetrics[:mean_timegap_mean], aggmetrics[:mean_timegap_stdev]),
#                 @sprintf("%.5f", calc_kl_div_gaussian(aggmetrics_original, aggmetrics, :mean_centerline_offset_activecar)),
#                 @sprintf("%.5f", calc_kl_div_gaussian(aggmetrics_original, aggmetrics, :mean_speed_activecar)),
#                 @sprintf("%.5f", calc_kl_div_gaussian(aggmetrics_original, aggmetrics, :mean_timegap)),
#                 @sprintf("%.5f", behavior_metrics_set.histobin_kldiv),
#                 @sprintf("%.4f", mean_trace_log_prob),
#                 @sprintf("%.4f", mean_rmse_1s),
#                 @sprintf("%.4f", mean_rmse_2s),
#                 @sprintf("%.4f", mean_rmse_3s),
#                 @sprintf("%.4f", mean_rmse_4s)
#             ]
#     end

#     df
# end

function calc_mean_cross_validation_metrics(aggmetric_set::Vector{Dict{Symbol,Any}})
    retval = (Symbol=>Any)[]

    n = length(aggmetric_set)
    temparr = Array(Float64, n)

    keyset = keys(aggmetric_set[1])
    for key in keyset
        for (i,aggmetrics) in enumerate(aggmetric_set)
            temparr[i] = float64(get(aggmetrics, key, NaN))
        end
        μ = mean(temparr)
        σ = stdm(temparr, μ)
        retval[symbol(string(key)*"_mean")] = μ
        retval[symbol(string(key)*"_stdev")] = σ
    end

    retval
end
# function calc_mean_cross_validation_metrics(metrics_sets::Vector{MetricsSet})
#     aggmetric_set = map(m->m.aggmetrics, metrics_sets)
#     calc_mean_cross_validation_metrics(aggmetric_set)
# end