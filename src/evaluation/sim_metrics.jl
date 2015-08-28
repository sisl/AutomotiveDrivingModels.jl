export
    MetricsSet,

    create_metrics_set,
    create_metrics_sets,
    create_metrics_set_no_tracemetrics,

    calc_rmse_predicted_vs_ground_truth,

    calc_tracemetrics,
    calc_aggregate_metric,
    calc_aggregate_metrics,
    calc_mean_cross_validation_metrics,

    compute_metric_summary_table

type MetricsSet
    histobin::Matrix{Float64} # histobin image over deviation during run
    histobin_kldiv::Float64 # kldivergence with respect to original histobin
    tracemetrics::Vector{Dict{Symbol, Any}} # metric dictionary computed for every trace
    aggmetrics::Dict{Symbol,Any} # mean and stdev metrics across traces
end

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

function calc_tracemetrics(
    pdset::PrimaryDataset,
    sn::StreetNetwork,
    seg::PdsetSegment;
    basics::FeatureExtractBasicsPdSet=FeatureExtractBasicsPdSet(pdset, sn),
    pdset_frames_per_sim_frame::Int=5
    )

    has_collision = false # whether any collision occurs
    has_collision_activecar = false # whether the active car collides with another car
    n_lanechanges = 0 # number of lanechanges the active car makes
    elapsed_time = get_elapsed_time(pdset, seg.validfind_start, seg.validfind_end)

    speed = StreamStats.Var()
    headway = StreamStats.Var()
    timegap = StreamStats.Var()
    centerline_offset = StreamStats.Var()

    validfind_of_first_offroad = -1
    n_frames_offroad = 0

    cur_lanetag = get(pdset, :lanetag, carid2ind(pdset, seg.carid, seg.validfind_start), seg.validfind_start)::LaneTag
    cur_lane = get_lane(sn, cur_lanetag)

    for validfind in seg.validfind_start : pdset_frames_per_sim_frame : seg.validfind_end
        # NOTE(tim): this assumes that the validfinds are continuous

        carind_active = carid2ind(pdset, seg.carid, validfind)
        
        if validfind > seg.validfind_start
            fut_lanetag = get(pdset, :lanetag, carind_active, validfind)::LaneTag
            if fut_lanetag != cur_lanetag
                if same_tile(cur_lanetag, fut_lanetag) || !has_next_lane(sn, cur_lane)
                    lane_change_occurred = true
                else
                    cur_lane = next_lane(sn, cur_lane)
                    cur_lanetag = LaneTag(sn, cur_lane)
                    lane_change_occurred = fut_lanetag != cur_lanetag
                end

                if lane_change_occurred
                    n_lanechanges += 1
                end
            end
        end

        update!(speed, Features._get(SPEED, basics, carind_active, validfind))
        (best_carind, d_x_front, d_y_front) = Features._get_carind_front_and_dist(basics, carind_active, validfind)
        update!(headway, d_x_front)

        if !isinf(d_x_front)
            timegap_x_front = Features.THRESHOLD_TIMEGAP
        else
            v  = get(basics.pdset, :velFx, carind_active, validfind) # our current velocity

            if v <= 0.0
                return Features.THRESHOLD_TIMEGAP
            end

            timegap_x_front = min(d_x_front / v, Features.THRESHOLD_TIMEGAP)
        end
        update!(timegap, timegap_x_front)
            

        offset = get(pdset, :d_cl, carind_active, validfind)
        update!(centerline_offset, offset)
        is_on_road = abs(offset) < 2.0 # NOTE(tim): there should be a call for this in StreetNetwork

        if !is_on_road
            n_frames_offroad += 1
            validfind_of_first_offroad = min(validfind, validfind_of_first_offroad)
        end

        # check for collision
        posGx = get(pdset, :posGx, carind_active, validfind)
        posGy = get(pdset, :posGy, carind_active, validfind)

        max_carind = get_maxcarind(pdset, validfind)
        for carind in -1 : max_carind
            if carind != carind_active

                posGx_target = get(pdset, :posGx, carind, validfind)
                posGy_target = get(pdset, :posGy, carind, validfind)

                Δx = posGx - posGx_target
                Δy = posGx - posGy_target

                # TODO(tim): these attributes should be in PdSet
                # TODO(tim): we should be using a better method for collision checking
                if abs(Δx) < DEFAULT_CAR_LENGTH && abs(Δy) < DEFAULT_CAR_WIDTH 
                    has_collision_activecar = true
                    # NOTE(tim): anything after a collision is invalid - break here
                    break
                end

                for carind2 in -1 : max_carind
                    if carind2 != carind_active &&
                       carind2 != carind

                        Δx = posGx_target - get(pdset, :posGx, carind2, validfind)
                        Δy = posGy_target - get(pdset, :posGy, carind2, validfind)
            
                        # TODO(tim): these attributes should be in PdSet
                        # TODO(tim): we should be using a better method for collision checking
                        if abs(Δx) < DEFAULT_CAR_LENGTH && abs(Δy) < DEFAULT_CAR_WIDTH 
                            has_collision = true
                        end
                    end                    
                end
            end
        end
    end

    went_offroad = (validfind_of_first_offroad == -1)

    [
     :has_collision => has_collision,
     :has_collision_activecar => has_collision_activecar,
     :n_lanechanges_activecar => n_lanechanges,
     :mean_speed_activecar => mean(speed),
     :mean_centerline_offset_activecar => mean(centerline_offset),
     :std_speed_activecar => std(speed),
     :std_centerline_offset_activecar => std(centerline_offset),
     :n_sec_offroad_ego => went_offroad ? 0.0 : get_elapsed_time(pdset, validfind_of_first_offroad, seg.validfind_end),
     :elapsed_time => elapsed_time,
     :time_of_first_offroad => went_offroad ? Inf : gete(pdset, :time, validfind_of_first_offroad)::Float64,
     :went_offroad => went_offroad,
     :mean_headway => mean(headway),
     :mean_timegap => mean(timegap),
     :logl => NaN, # NOTE(tim): no defined behavior
    ]::Dict{Symbol, Any}
end
function calc_tracemetrics(
    behavior::AbstractVehicleBehavior,
    pdset_original::PrimaryDataset,
    pdset_simulation::PrimaryDataset,
    sn::StreetNetwork,
    seg::PdsetSegment;
    basics::FeatureExtractBasicsPdSet=FeatureExtractBasicsPdSet(pdset_simulation, sn),
    pdset_frames_per_sim_frame::Int=5
    )

    has_collision = false # whether any collision occurs
    has_collision_activecar = false # whether the active car collides with another car
    n_lanechanges = 0 # number of lanechanges the active car makes
    elapsed_time = get_elapsed_time(pdset_simulation, seg.validfind_start, seg.validfind_end)

    speed = StreamStats.Var()
    headway = StreamStats.Var()
    timegap = StreamStats.Var()
    centerline_offset = StreamStats.Var()

    total_action_loglikelihood = 0.0

    validfind_of_first_offroad = -1
    n_frames_offroad = 0

    cur_lanetag = get(pdset_simulation, :lanetag, carid2ind(pdset_simulation, seg.carid, seg.validfind_start), seg.validfind_start)::LaneTag
    cur_lane = get_lane(sn, cur_lanetag)

    for validfind in seg.validfind_start : pdset_frames_per_sim_frame : seg.validfind_end
        # NOTE(tim): this assumes that the validfinds are continuous

        carind_active = carid2ind(pdset_simulation, seg.carid, validfind)
        
        if validfind > seg.validfind_start
            fut_lanetag = get(pdset_simulation, :lanetag, carind_active, validfind)::LaneTag
            if fut_lanetag != cur_lanetag
                if same_tile(cur_lanetag, fut_lanetag) || !has_next_lane(sn, cur_lane)
                    lane_change_occurred = true
                else
                    cur_lane = next_lane(sn, cur_lane)
                    cur_lanetag = LaneTag(sn, cur_lane)
                    lane_change_occurred = fut_lanetag != cur_lanetag
                end

                if lane_change_occurred
                    n_lanechanges += 1
                end
            end
        end

        update!(speed, Features._get(SPEED, basics, carind_active, validfind))
        (best_carind, d_x_front, d_y_front) = Features._get_carind_front_and_dist(basics, carind_active, validfind)
        update!(headway, d_x_front)

        if !isinf(d_x_front)
            timegap_x_front = Features.THRESHOLD_TIMEGAP
        else
            v  = get(basics.pdset, :velFx, carind_active, validfind) # our current velocity

            if v <= 0.0
                return Features.THRESHOLD_TIMEGAP
            end

            timegap_x_front = min(d_x_front / v, Features.THRESHOLD_TIMEGAP)
        end
        update!(timegap, timegap_x_front)

        if validfind != seg.validfind_end
            total_action_loglikelihood += calc_action_loglikelihood(basics, behavior, carind_active, validfind)
            @assert(!isinf(total_action_loglikelihood))
        end

        offset = get(pdset_simulation, :d_cl, carind_active, validfind)
        update!(centerline_offset, offset)
        is_on_road = abs(offset) < 2.0 # NOTE(tim): there should be a call for this in StreetNetwork

        if !is_on_road
            n_frames_offroad += 1
            validfind_of_first_offroad = min(validfind, validfind_of_first_offroad)
        end

        # check for collision
        posGx = get(pdset_simulation, :posGx, carind_active, validfind)
        posGy = get(pdset_simulation, :posGy, carind_active, validfind)

        max_carind = get_maxcarind(pdset_simulation, validfind)
        for carind in -1 : max_carind
            if carind != carind_active

                posGx_target = get(pdset_simulation, :posGx, carind, validfind)
                posGy_target = get(pdset_simulation, :posGy, carind, validfind)

                Δx = posGx - posGx_target
                Δy = posGx - posGy_target

                # TODO(tim): these attributes should be in PdSet
                # TODO(tim): we should be using a better method for collision checking
                if abs(Δx) < DEFAULT_CAR_LENGTH && abs(Δy) < DEFAULT_CAR_WIDTH 
                    has_collision_activecar = true
                    # NOTE(tim): anything after a collision is invalid - break here
                    break
                end

                for carind2 in -1 : max_carind
                    if carind2 != carind_active &&
                       carind2 != carind

                        Δx = posGx_target - get(pdset_simulation, :posGx, carind2, validfind)
                        Δy = posGy_target - get(pdset_simulation, :posGy, carind2, validfind)
            
                        # TODO(tim): these attributes should be in PdSet
                        # TODO(tim): we should be using a better method for collision checking
                        if abs(Δx) < DEFAULT_CAR_LENGTH && abs(Δy) < DEFAULT_CAR_WIDTH 
                            has_collision = true
                        end
                    end                    
                end
            end
        end
    end

    went_offroad = (validfind_of_first_offroad == -1)

    [
     :has_collision => has_collision,
     :has_collision_activecar => has_collision_activecar,
     :n_lanechanges_activecar => n_lanechanges,
     :mean_speed_activecar => mean(speed),
     :mean_centerline_offset_activecar => mean(centerline_offset),
     :std_speed_activecar => std(speed),
     :std_centerline_offset_activecar => std(centerline_offset),
     :n_sec_offroad_ego => went_offroad ? 0.0 : get_elapsed_time(pdset, validfind_of_first_offroad, seg.validfind_end),
     :elapsed_time => elapsed_time,
     :time_of_first_offroad => went_offroad ? Inf : gete(pdset, :time, validfind_of_first_offroad)::Float64,
     :went_offroad => went_offroad,
     :mean_headway => mean(headway),
     :mean_timegap => mean(timegap),
     :logl => total_action_loglikelihood,
     :rmse_1000ms => calc_rmse_predicted_vs_ground_truth(pdset_simulation, pdset_original, seg, 4),
     :rmse_2000ms => calc_rmse_predicted_vs_ground_truth(pdset_simulation, pdset_original, seg, 8),
     :rmse_3000ms => calc_rmse_predicted_vs_ground_truth(pdset_simulation, pdset_original, seg, 12),
     :rmse_4000ms => calc_rmse_predicted_vs_ground_truth(pdset_simulation, pdset_original, seg, 16),
    ]::Dict{Symbol, Any}
end
function calc_tracemetrics(
    pdsets::Vector{PrimaryDataset},
    streetnets::Vector{StreetNetwork},
    pdset_segments::Vector{PdsetSegment},
    fold::Integer,
    pdsetseg_fold_assignment::Vector{Int},
    match_fold::Bool
    )
    
    fold_size = calc_fold_size(fold, pdsetseg_fold_assignment, match_fold)

    metrics = Array(Dict{Symbol, Any}, fold_size)
    i = 0
    for (fold_assignment, seg) in zip(pdsetseg_fold_assignment, pdset_segments)
        if is_in_fold(fold, fold_assignment, match_fold)

            i += 1
            metrics[i] = calc_tracemetrics(pdsets[seg.pdset_id], streetnets[seg.streetnet_id], seg)
        end
    end
    metrics
end

function calc_aggregate_metric(sym::Symbol, ::Type{Int}, metricset::Vector{Dict{Symbol, Any}})

    # counts = Dict{Int,Int}()
    # for i = 1 : length(metricset)
    #     counts[metricset[i][sym]] = get(counts, metricset[i][sym], 0) + 1
    # end

    t_arr = [metricset[i][:elapsed_time] for i in 1 : length(metricset)]
    tot_time = sum(t_arr)

    arr = [metricset[i][sym] for i in 1 : length(metricset)]
    ave = mean(arr)
    stdev = stdm(arr, ave)
    weighted_ave = sum([metricset[i][sym] * metricset[i][:elapsed_time] for i in 1 : length(metricset)]) / tot_time

    (ave, stdev, weighted_ave)
end
function calc_aggregate_metric(sym::Symbol, ::Type{Bool}, metricset::Vector{Dict{Symbol, Any}})
    n_true = sum([metricset[i][sym] for i in 1 : length(metricset)])
    ave_time_to_true = sum([metricset[i][sym] ? metricset[i][:elapsed_time] : 0.0 for i in 1 : length(metricset)]) / n_true

    t_arr = [metricset[i][:elapsed_time] for i in 1 : length(metricset)]
    tot_time = sum(t_arr)

    odds_true_per_run = n_true / length(metricset)
    odds_true_per_sec = n_true / tot_time

    (odds_true_per_run, odds_true_per_sec, ave_time_to_true)
end
function calc_aggregate_metric(sym::Symbol, ::Type{Float64}, metricset::Vector{Dict{Symbol, Any}}, use_abs=false)
    if use_abs
        arr = convert(Vector{Float64}, [abs(metricset[i][sym]) for i in 1 : length(metricset)])
    else
        arr = convert(Vector{Float64}, [metricset[i][sym] for i in 1 : length(metricset)])
    end

    inds = find(a->a!=Inf, arr)
    arr = arr[inds]
    ave = mean(arr)
    stdev = stdm(arr, ave)
    (ave, stdev)
end

function _calc_aggregate_metrics(metricset::Vector{Dict{Symbol, Any}})

    aggmetrics = (Symbol=>Any)[]

    calc_and_add!(sym::Symbol, ::Type{Float64}, use_abs::Bool=false) = begin
        res = calc_aggregate_metric(sym, Float64, metricset, use_abs)
        aggmetrics[symbol(string(sym)*"_mean")] = res[1]
        aggmetrics[symbol(string(sym)*"_stdev")] = res[2]
    end
    add!(sym::Symbol, ::Type{Float64}) = begin
        aggmetrics[symbol(string(sym)*"_mean")] = NaN
        aggmetrics[symbol(string(sym)*"_stdev")] = NaN
    end

    calc_and_add!(:mean_centerline_offset_activecar, Float64, true)

    for key in (
            :mean_speed_activecar, :time_of_first_offroad, :n_lanechanges_activecar, 
            :mean_headway, :mean_timegap, :logl, 
            :rmse_1000ms, :rmse_2000ms, :rmse_3000ms, :rmse_4000ms
        )
        if haskey(metricset[1], key)
            calc_and_add!(key, Float64)
        else
            add!(key, Float64)
        end
    end

    res = calc_aggregate_metric(:went_offroad, Bool, metricset)
    aggmetrics[:went_offroad_odds_true_per_run] = res[1]
    aggmetrics[:went_offroad_odds_true_per_sec] = res[2]
    aggmetrics[:went_offroad_ave_time_to_true] = res[3]

    aggmetrics
end
function calc_aggregate_metrics(metricset::Vector{Dict{Symbol, Any}})

    aggmetrics = _calc_aggregate_metrics(metricset)

    aggmetrics[:mean_lane_offset_kldiv] = NaN
    aggmetrics[:mean_speed_ego_kldiv] = NaN
    aggmetrics[:mean_timegap_kldiv] = NaN
    aggmetrics[:histobin_kldiv] = NaN

    aggmetrics
end
function calc_aggregate_metrics(
    metricset::Vector{Dict{Symbol, Any}},
    aggmetrics_original::Dict{Symbol, Any},
    histobin_kldiv::Float64)

    aggmetrics = _calc_aggregate_metrics(metricset)

    aggmetrics[:mean_lane_offset_kldiv] = calc_kl_div_gaussian(aggmetrics_original, aggmetrics, :mean_centerline_offset_activecar)
    aggmetrics[:mean_speed_ego_kldiv] = calc_kl_div_gaussian(aggmetrics_original, aggmetrics, :mean_speed_activecar)
    aggmetrics[:mean_timegap_kldiv] = calc_kl_div_gaussian(aggmetrics_original, aggmetrics, :mean_timegap)
    aggmetrics[:histobin_kldiv] = histobin_kldiv

    aggmetrics
end

function create_metrics_set(
    pdsets_original::Vector{PrimaryDataset},
    streetnets::Vector{StreetNetwork},
    pdset_segments::Vector{PdsetSegment},
    simparams::SimParams,
    histobin_params::ParamsHistobin,
    fold::Integer,
    pdsetseg_fold_assignment::Vector{Int},
    match_fold::Bool
    )

    #=
    Computes the MetricsSet for the original data

    Note that this will NOT compute trace log likelihoods or RMSE values
    as it makes no sense to compare against itself
    =#

    histobin = calc_histobin(pdsets_original, streetnets, pdset_segments, histobin_params,
                             fold, pdsetseg_fold_assignment, match_fold)
    tracemetrics = calc_tracemetrics(pdsets_original, streetnets, pdset_segments, simparams,
                                     fold, pdsetseg_fold_assignment, match_fold)
    aggmetrics = calc_aggregate_metrics(tracemetrics)

    MetricsSet(histobin, 0.0, tracemetrics, aggmetrics)
end
function create_metrics_set_no_tracemetrics(
    pdsets_original::Vector{PrimaryDataset},
    streetnets::Vector{StreetNetwork},
    pdset_segments::Vector{PdsetSegment},
    histobin_params::ParamsHistobin,
    fold::Integer,
    pdsetseg_fold_assignment::Vector{Int},
    match_fold::Bool
    )

    #=
    Computes the MetricsSet for the original data

    Note that this will NOT compute trace log likelihoods or RMSE values
    as it makes no sense to compare against itself
    =#

    histobin = calc_histobin(pdsets_original, streetnets, pdset_segments, histobin_params,
                             fold, pdsetseg_fold_assignment, match_fold)
    tracemetrics = calc_tracemetrics(pdsets_original, streetnets, pdset_segments,
                                     fold, pdsetseg_fold_assignment, match_fold)
    aggmetrics = calc_aggregate_metrics(tracemetrics)

    MetricsSet(histobin, 0.0, Dict{Symbol, Any}[], aggmetrics)
end

function compute_metric_summary_table{S<:String}(
    behavior_metrics_sets::AbstractVector{MetricsSet}, 
    original_metrics_set::MetricsSet,
    model_names::Vector{S}
    )


    df = DataFrame(labels=["mean lane offset", "mean speed", "mean timegap", 
                           "mean lane offset kldiv", "mean speed kldiv", "mean timegap kldiv", "histobin kldiv",
                           "mean trace log prob", "mean rmse 1s", "mean rmse 2s", "mean rmse 3s", "mean rmse 4s"])

    aggmetrics_original = original_metrics_set.aggmetrics
    df[:realworld] = [
                        @sprintf("%.3f +- %.3f", aggmetrics_original[:mean_centerline_offset_activecar_mean], aggmetrics_original[:mean_centerline_offset_activecar_stdev]),
                        @sprintf("%.3f +- %.3f", aggmetrics_original[:mean_speed_activecar_mean], aggmetrics_original[:mean_speed_activecar_stdev]),
                        @sprintf("%.3f +- %.3f", aggmetrics_original[:mean_timegap_mean], aggmetrics_original[:mean_timegap_stdev]),
                        "", "", "", "", "", "", "", "", ""
                    ]

    for (i,behavior_metrics_set) in enumerate(behavior_metrics_sets)

        behavor_sym = symbol(model_names[i])
        tracemetrics = behavior_metrics_set.tracemetrics
        aggmetrics = behavior_metrics_set.aggmetrics

        mean_trace_log_prob, stdev_trace_log_prob = calc_aggregate_metric(:logl, Float64, tracemetrics)
        mean_rmse_1s, stdev_rmse_1s = calc_aggregate_metric(:rmse_1000ms, Float64, tracemetrics)
        mean_rmse_2s, stdev_rmse_2s = calc_aggregate_metric(:rmse_2000ms, Float64, tracemetrics)
        mean_rmse_3s, stdev_rmse_3s = calc_aggregate_metric(:rmse_3000ms, Float64, tracemetrics)
        mean_rmse_4s, stdev_rmse_4s = calc_aggregate_metric(:rmse_4000ms, Float64, tracemetrics)

        df[behavor_sym] = [
                @sprintf("%.3f +- %.3f", aggmetrics[:mean_centerline_offset_activecar_mean], aggmetrics[:mean_centerline_offset_activecar_stdev]),
                @sprintf("%.3f +- %.3f", aggmetrics[:mean_speed_activecar_mean], aggmetrics[:mean_speed_activecar_stdev]),
                @sprintf("%.3f +- %.3f", aggmetrics[:mean_timegap_mean], aggmetrics[:mean_timegap_stdev]),
                @sprintf("%.5f", calc_kl_div_gaussian(aggmetrics_original, aggmetrics, :mean_centerline_offset_activecar)),
                @sprintf("%.5f", calc_kl_div_gaussian(aggmetrics_original, aggmetrics, :mean_speed_activecar)),
                @sprintf("%.5f", calc_kl_div_gaussian(aggmetrics_original, aggmetrics, :mean_timegap)),
                @sprintf("%.5f", behavior_metrics_set.histobin_kldiv),
                @sprintf("%.4f", mean_trace_log_prob),
                @sprintf("%.4f", mean_rmse_1s),
                @sprintf("%.4f", mean_rmse_2s),
                @sprintf("%.4f", mean_rmse_3s),
                @sprintf("%.4f", mean_rmse_4s)
            ]
    end

    df
end

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
function calc_mean_cross_validation_metrics(metrics_sets::Vector{MetricsSet})
    aggmetric_set = map(m->m.aggmetrics, metrics_sets)
    calc_mean_cross_validation_metrics(aggmetric_set)
end