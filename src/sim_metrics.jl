export
        AggregateMetricSet,

        evaluate_original,
        evaluate_behavior,
        evaluate_behaviors,

        calc_rmse_predicted_vs_ground_truth,
        calc_loglikelihood_of_trace,
        calc_mean_trace_loglikelihood,

        calc_metrics,
        calc_aggregate_metric,
        calc_aggregate_metrics,

        compute_metric_summary_table,

        pull_run_log_likelihoods,
        rank_by_log_likelihood,

        print_aggregate_metrics_human_readable,
        print_aggregate_metrics_csv_readable

type AggregateMetricSet
    histobin::Matrix{Float64} # histobin image over deviation during run
    histobin_kldiv::Float64
    metrics::Vector{Dict{Symbol, Any}} # metric dictionary computed for every trace
end

function evaluate_original(
    simlogs_original::Vector{Matrix{Float64}},
    road::StraightRoadway,
    history::Int,
    simparams::SimParams,
    histobin_params::ParamsHistobin
    )

    #=
    Computes the AggregateMetricSet for the original data

    Note that this will NOT compute trace log likelihoods or RMSE values
    as it makes no sense to compare against itself
    =#

    histobin = calc_histobin(simlogs_original, histobin_params, history)
    metricset = calc_metrics(simlogs_original, road, simparams, history)
    
    AggregateMetricSet(histobin, 0.0, metricset)
end
function evaluate_behavior(
    egobehavior::AbstractVehicleBehavior,
    simlogs_original::Vector{Matrix{Float64}},
    simlogs::Vector{Matrix{Float64}},
    road::StraightRoadway,
    history::Int,
    simparams::SimParams,
    histobin_params::ParamsHistobin,
    histobin_original_with_prior::Matrix{Float64}
    )

    #=
    Evaluates the candidate ego behavior by:
      - running the egobehavior on all of the simlogs
      - computing the validation metrics on the results:
          - histobin
          - set of metrics
    =#

    carind = 1
    logbaseindex = calc_logindexbase(carind)

    histobin = calc_histobin(simlogs, histobin_params, history)
    histobin_kldiv = KL_divergence_dirichlet(histobin_original_with_prior, histobin .+ 1 )
    metricset = calc_metrics(simlogs, road, simparams, history)

    for i in 1 : length(simlogs)

        simlog_model = simlogs[i]
        simlog_original = simlogs_original[i]

        basics_original = FeatureExtractBasics(simlog_original, road, simparams.sec_per_frame, simparams.extracted_feature_cache, i)
        end_frame = get_nframes(simlog_model)

        # log likelihood of the original trajectory
        metricset[i][:logl] = calc_loglikelihood_of_trace(basics_original, egobehavior, carind, history, end_frame)

        for rmse_endframe in [history+2 : 2 : end_frame]
            Δt = (rmse_endframe - history) * simparams.sec_per_frame # [sec]
            sym = symbol(@sprintf("rmse_%dms", Δt*1000))
            metricset[i][sym] = calc_rmse_predicted_vs_ground_truth(simlog_model, simlog_original,
                                                                 logbaseindex, history, rmse_endframe)
        end
    end
    
    AggregateMetricSet(histobin, histobin_kldiv, metricset)
end
function evaluate_behaviors{B<:AbstractVehicleBehavior}(
    behaviors::Vector{B},
    simlogs_original::Vector{Matrix{Float64}},
    behavior_simlogs::Vector{Vector{Matrix{Float64}}},
    road::StraightRoadway,
    history::Int,
    simparams::SimParams,
    histobin_params::ParamsHistobin
    )

    @assert(length(behaviors) == length(behavior_simlogs))

    histobin_original_with_prior = calc_histobin(simlogs_original, histobin_params, history)
    histobin_original_with_prior .+= 1.0

    retval = Array(AggregateMetricSet, length(behaviors))
    for (i, egobehavior) in enumerate(behaviors)
        retval[i] = evaluate_behavior(egobehavior, simlogs_original, behavior_simlogs[i],
                                      road, history, simparams, histobin_params, histobin_original_with_prior)
    end
    retval
end

function calc_kl_div_gaussian(μA::Float64, σA::Float64, μB::Float64, σB::Float64)

    log(σB/σA) + (σA*σA + (μA-μB)^2)/(2σB*σB) - 0.5
end
function calc_kl_div_gaussian(aggmetrics_original::Dict{Symbol,Any}, aggmetrics_target::Dict{Symbol,Any}, sym::Symbol)

    # Compute the kl divergence for the given aggregate metric
    #   `sym' must refer to an entry in aggmetrics which contains but sym_mean and sym_stdev
    #    ex: :mean_timegap -> :mean_timegap_mean and :mean_timegap_stdev

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
    simlog_predicted::Matrix{Float64},
    simlog_truth::Matrix{Float64},
    logindexbase::Int,
    frame_start::Int,
    frame_end::Int
    )

    n = frame_end - frame_start + 1
    @assert(n > 0)

    total = 0.0
    for i = frame_start : frame_end
        pos_x_predicted = simlog_predicted[i, logindexbase+LOG_COL_X]
        pos_x_truth = simlog_truth[i, logindexbase+LOG_COL_X]
        pos_y_predicted = simlog_predicted[i, logindexbase+LOG_COL_Y]
        pos_y_truth = simlog_truth[i, logindexbase+LOG_COL_Y]

        Δx = pos_x_predicted - pos_x_truth
        Δy = pos_y_predicted - pos_y_truth

        total += Δx*Δx + Δy*Δy
    end
    sqrt(total / n)
end

calc_probability_for_uniform_sample_from_bin(bindiscreteprob::Float64, binwidth::Float64) = bindiscreteprob / binwidth
function calc_probability_for_uniform_sample_from_bin(bindiscreteprob::Float64, disc::LinearDiscretizer, binindex::Int)
    width_of_bin = binwidth(disc, binindex)
    calc_probability_for_uniform_sample_from_bin(bindiscreteprob, width_of_bin)
end

function calc_loglikelihood_of_trace(
    basics::FeatureExtractBasics,
    behavior::AbstractVehicleBehavior,
    carind::Int,
    frame_start::Int,
    frame_end::Int
    )

    #=
    Compute the log-likelihood of a trace given a behavior model
    =#

    logl = 0.0
    for frameind = frame_start : frame_end
        logl += calc_action_loglikelihood(basics, behavior, carind, frameind)
    end
    logl
end
function calc_mean_trace_loglikelihood(
    simlogs::Vector{Matrix{Float64}},
    behavior::AbstractVehicleBehavior,
    road::StraightRoadway,
    sec_per_frame::Float64,
    history::Int,
    carind::Int
    )

    mean_trace_logl = 0.0
    extracted_feature_cache = ExtractedFeatureCache()
    basics_seed = 1
    for simlog in simlogs
        basics = FeatureExtractBasics(simlog, road, sec_per_frame, extracted_feature_cache, basics_seed+=1)
        mean_trace_logl += calc_loglikelihood_of_trace(basics, behavior, carind, history, get_nframes(simlog))
    end
    mean_trace_logl / length(simlogs)
end

function calc_metrics(simlog::Matrix{Float64}, road::StraightRoadway, simparams::SimParams, history::Int)

    Δt = simparams.sec_per_frame

    n = size(simlog,1)

    has_collision_ego = false # whether ego car collides with another car
    n_lanechanges_ego = 0 # whether ego car makes a lange change
    elapsed_time = (size(simlog, 1)-history) * Δt
    has_other_cars = size(simlog,2) > LOG_NCOLS_PER_CAR

    mean_speed_ego = mean(simlog[:,LOG_COL_V]) # mean ego speed
    mean_centerline_offset_ego = 0.0
    std_speed_ego  = std(simlog[:,LOG_COL_V])  # stdev of ego speed
    time_of_first_offroad = Inf
    n_frames_offroad_ego = 0

    arr_v_x = (simlog[2:end,LOG_COL_X] - simlog[1:end-1,LOG_COL_X]) ./ Δt
    arr_v_y = (simlog[2:end,LOG_COL_Y] - simlog[1:end-1,LOG_COL_Y]) ./ Δt
    arr_a_x = (arr_v_x[2:end] - arr_v_x[1:end-1]) ./ Δt
    arr_a_y = (arr_v_y[2:end] - arr_v_y[1:end-1]) ./ Δt
    abs_arr_j_x = abs((arr_a_x[2:end] - arr_a_x[1:end-1]) ./ Δt)
    abs_arr_j_y = abs((arr_a_y[2:end] - arr_a_y[1:end-1]) ./ Δt)
    abs_jerk_mean_x = mean(abs_arr_j_x[history:end])
    abs_jerk_std_x = stdm(abs_arr_j_x[history:end], abs_jerk_mean_x)
    abs_jerk_mean_y = mean(abs_arr_j_y[history:end])
    abs_jerk_std_y = stdm(abs_arr_j_y[history:end], abs_jerk_mean_y)

    numcars = get_ncars(simlog)
    if numcars > 1
        mean_headway = mean(simlog[history:end,LOG_NCOLS_PER_CAR+LOG_COL_X] - simlog[history:end,LOG_COL_X])
        mean_timegap = mean((simlog[history:end,LOG_NCOLS_PER_CAR+LOG_COL_X] - simlog[history:end,LOG_COL_X]) ./ simlog[history:end,LOG_COL_V])
    else
        mean_headway = 0.0
        mean_timegap = 0.0
    end

    lane_centers = get_lanecenters(road) # [0,lw,2lw,...]

    for i = history : n
        # check for lange change (by ego car)
        lane_dists = abs(lane_centers .- simlog[i,  LOG_COL_Y])
        cl_cur = indmin(lane_dists)
        mean_centerline_offset_ego += lane_dists[cl_cur]

        if i > 1
            cl_old = indmin(abs(lane_centers .- simlog[i-1,LOG_COL_Y]))
            if cl_old != cl_cur
                n_lanechanges_ego += 1
            end
        end

        if !is_onroad(simlog[i, LOG_COL_Y], road)
            n_frames_offroad_ego += 1
            time_of_first_offroad = min(i*Δt, time_of_first_offroad)
        end

        # check for collision
        if has_other_cars
            # TODO(tim): make work for more than 1 other car
            dx = simlog[i,LOG_COL_X] - simlog[i,LOG_COL_X+LOG_NCOLS_PER_CAR]
            dy = simlog[i,LOG_COL_Y] - simlog[i,LOG_COL_Y+LOG_NCOLS_PER_CAR]
            if abs(dx) < CAR_LENGTH && abs(dy) < CAR_WIDTH
                has_collision_ego = true
                # NOTE(tim): anything after a collision is invalid - break here
                break
            end
        end
    end

    mean_centerline_offset_ego /= (n-history)

    [
     :has_collision_ego=>has_collision_ego,
     :n_lanechanges_ego=>n_lanechanges_ego,
     :mean_speed_ego=>mean_speed_ego,
     :mean_centerline_offset_ego=>mean_centerline_offset_ego,
     :std_speed_ego=>std_speed_ego,
     :n_sec_offroad_ego=>n_frames_offroad_ego * Δt,
     :elapsed_time=>elapsed_time,
     :time_of_first_offroad=>time_of_first_offroad,
     :went_offroad=>time_of_first_offroad!=Inf,
     :jerk_mean_x=>abs_jerk_mean_x,
     :jerk_std_x=>abs_jerk_std_x,
     :jerk_mean_y=>abs_jerk_mean_y,
     :jerk_std_y=>abs_jerk_std_y,
     :final_x=>simlog[end,LOG_COL_X],
     :final_y=>simlog[end,LOG_COL_Y],
     :initial_speed=>simlog[history,LOG_COL_V],
     :mean_headway=>mean_headway,
     :mean_timegap=>mean_timegap,
     :logPA=>sum(simlog[history:end,LOG_COL_logprobweight_A]),
     :logPT=>sum(simlog[history:end,LOG_COL_logprobweight_T]),
     :percent_freeflow=>sum(simlog[history:end,LOG_COL_em] == EM_ID_FREEFLOW) / (n-history),
     :percent_carfollow=>sum(simlog[history:end,LOG_COL_em] == EM_ID_CARFOLLOW) / (n-history),
     :percent_lanechange=>sum(simlog[history:end,LOG_COL_em] == EM_ID_LANECHANGE) / (n-history)
    ]::Dict{Symbol, Any}
end
function calc_metrics(simlogs::Vector{Matrix{Float64}}, road::StraightRoadway, simparams::SimParams, history::Int)

    metrics = Array(Dict{Symbol, Any}, length(simlogs))
    for (i, simlog) in enumerate(simlogs)
        metrics[i] = calc_metrics(simlog, road, simparams, history)
    end
    metrics
end

function calc_aggregate_metric(sym::Symbol, ::Type{Int}, metricset::Vector{Dict{Symbol, Any}})

    counts = Dict{Int,Int}()
    for i = 1 : length(metricset)
        counts[metricset[i][sym]] = get(counts, metricset[i][sym], 0) + 1
    end

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
function calc_aggregate_metrics(metricset::Vector{Dict{Symbol, Any}})

    aggmetrics = (Symbol=>Any)[]

    res = calc_aggregate_metric(:mean_centerline_offset_ego, Float64, metricset, true)
    aggmetrics[:mean_centerline_offset_ego_mean] = res[1]
    aggmetrics[:mean_centerline_offset_ego_stdev] = res[2]

    res = calc_aggregate_metric(:mean_speed_ego, Float64, metricset)
    aggmetrics[:mean_speed_ego_mean] = res[1]
    aggmetrics[:mean_speed_ego_stdev] = res[2]

    res = calc_aggregate_metric(:went_offroad, Bool, metricset)
    aggmetrics[:went_offroad_odds_true_per_run] = res[1]
    aggmetrics[:went_offroad_odds_true_per_sec] = res[2]
    aggmetrics[:went_offroad_ave_time_to_true] = res[3]

    res = calc_aggregate_metric(:time_of_first_offroad, Float64, metricset)
    aggmetrics[:time_of_first_offroad_mean] = res[1]
    aggmetrics[:time_of_first_offroad_stdev] = res[2]

    res = calc_aggregate_metric(:n_lanechanges_ego, Float64, metricset)
    aggmetrics[:n_lanechanges_ego_mean] = res[1]
    aggmetrics[:n_lanechanges_ego_stdev] = res[2]

    res = calc_aggregate_metric(:jerk_mean_x, Float64, metricset)
    aggmetrics[:jerk_mean_x_mean] = res[1]
    aggmetrics[:jerk_mean_x_stdev] = res[2]

    res = calc_aggregate_metric(:jerk_std_x, Float64, metricset)
    aggmetrics[:jerk_std_x_mean] = res[1]
    aggmetrics[:jerk_std_x_stdev] = res[2]

    res = calc_aggregate_metric(:jerk_mean_y, Float64, metricset)
    aggmetrics[:jerk_mean_y_mean] = res[1]
    aggmetrics[:jerk_mean_y_stdev] = res[2]

    res = calc_aggregate_metric(:jerk_std_y, Float64, metricset)
    aggmetrics[:jerk_std_y_mean] = res[1]
    aggmetrics[:jerk_std_y_stdev] = res[2]

    res = calc_aggregate_metric(:mean_headway, Float64, metricset)
    aggmetrics[:mean_headway_mean] = res[1]
    aggmetrics[:mean_headway_stdev] = res[2]

    res = calc_aggregate_metric(:mean_timegap, Float64, metricset)
    aggmetrics[:mean_timegap_mean] = res[1]
    aggmetrics[:mean_timegap_stdev] = res[2]


    aggmetrics[:total_log_prob_lat] = sum([metricset[i][:logPT] for i in 1 : length(metricset)])
    aggmetrics[:total_log_prob_lon] = sum([metricset[i][:logPA] for i in 1 : length(metricset)])
    aggmetrics[:total_log_prob] = aggmetrics[:total_log_prob_lat] + aggmetrics[:total_log_prob_lon]

    aggmetrics
end

function pull_run_log_likelihoods(metricset::Vector{Dict{Symbol, Any}})

    m = length(metricset)
    logls = Array(Float64, m)
    for i = 1 : m
        metrics = metricset[i]
        logls[i] = metrics[:logPA] + metrics[:logPT]
    end
    logls
end
function rank_by_log_likelihood(metricset::Vector{Dict{Symbol, Any}})
    logls = pull_run_log_likelihoods(metricset)
    sortperm(logls)
end

function print_aggregate_metrics_human_readable(metricset::Vector{Dict{Symbol, Any}})
    println("\tmean centerline offset: ", calc_aggregate_metric(:mean_centerline_offset_ego, Float64, metricset, true))
    println("\tego speed:              ", calc_aggregate_metric(:mean_speed_ego, Float64, metricset))
    println("\toffroad rate:           ", calc_aggregate_metric(:went_offroad, Bool, metricset))
    println("\ttime to offroad:        ", calc_aggregate_metric(:time_of_first_offroad, Float64, metricset))
    println("\tlane change rate:       ", calc_aggregate_metric(:n_lanechanges_ego, Float64, metricset))
    println("\tjerk mean x:            ", calc_aggregate_metric(:jerk_mean_x, Float64, metricset))
    println("\tjerk std x:             ", calc_aggregate_metric(:jerk_std_x, Float64, metricset))
    println("\tjerk mean y:            ", calc_aggregate_metric(:jerk_mean_y, Float64, metricset))
    println("\tjerk std y:             ", calc_aggregate_metric(:jerk_std_y, Float64, metricset))
    println("\tmean headway:           ", calc_aggregate_metric(:mean_headway, Float64, metricset))
    println("\tmean timegap:           ", calc_aggregate_metric(:mean_timegap, Float64, metricset))
end
function print_aggregate_metrics_csv_readable(io::IO, simparams::SimParams, metricset::Vector{Dict{Symbol, Any}})
    mean_centerline_offset_ego = calc_aggregate_metric(:mean_centerline_offset_ego, Float64, metricset, true)
    mean_speed_ego = calc_aggregate_metric(:mean_speed_ego, Float64, metricset)
    went_offroad = calc_aggregate_metric(:went_offroad, Bool, metricset)
    time_of_first_offroad = calc_aggregate_metric(:time_of_first_offroad, Float64, metricset)
    n_lanechanges_ego = calc_aggregate_metric(:n_lanechanges_ego, Float64, metricset)
    jerk_mean_x = calc_aggregate_metric(:jerk_mean_x, Float64, metricset)
    jerk_std_x = calc_aggregate_metric(:jerk_std_x, Float64, metricset)
    jerk_mean_y = calc_aggregate_metric(:jerk_mean_y, Float64, metricset)
    jerk_std_y = calc_aggregate_metric(:jerk_std_y, Float64, metricset)
    mean_headway = calc_aggregate_metric(:mean_headway, Float64, metricset)
    mean_timegap = calc_aggregate_metric(:mean_timegap, Float64, metricset)

    str_smoothing_lat = simparams.sampling_lon.smoothing == :none ? "none" :
                        @sprintf("%s {%d}", string(simparams.sampling_lon.smoothing), simparams.sampling_lon.smoothing_counts)
    str_smoothing_lon = simparams.sampling_lat.smoothing == :none ? "none" :
                        @sprintf("%s {%d}", string(simparams.sampling_lat.smoothing), simparams.sampling_lat.smoothing_counts)

    @printf(io, "%.0f, %.3f, ", simparams.n_frames, simparams.sec_per_frame)
    @printf(io, "%s, %s, ", string(typeof(simparams.sampling_lat.sampling_scheme)), str_smoothing_lat)
    @printf(io, "%s, %s, ", string(typeof(simparams.sampling_lon.sampling_scheme)), str_smoothing_lon)
    @printf(io, "%.3f pm %.3f, %.2f pm %.3f, %.3f, %.2f pm %.2f, %.4f pm %.4f, %.3f pm %.2f, %.2f pm %.2f, %.3f pm %.2f, %.2f pm %.2f, %.2f pm %.2f, %.3f pm %.3f",
        mean_centerline_offset_ego[1], mean_centerline_offset_ego[2],
        mean_speed_ego[1], mean_speed_ego[2],
        went_offroad[1],
        time_of_first_offroad[1], time_of_first_offroad[2],
        n_lanechanges_ego[1], n_lanechanges_ego[2],
        jerk_mean_x[1], jerk_mean_x[2],
        jerk_std_x[1], jerk_std_x[2],
        jerk_mean_y[1], jerk_mean_y[2],
        jerk_std_y[1], jerk_std_y[2],
        mean_headway[1], mean_headway[2],
        mean_timegap[1], mean_timegap[2],
        )
end
print_aggregate_metrics_csv_readable(simparams::SimParams, metricset::Vector{Dict{Symbol, Any}}) = print_results_csv_readable(STDOUT, simparams, metricset)

function compute_metric_summary_table{S<:String}(
    behavior_metrics::AbstractVector{AggregateMetricSet}, 
    original_metrics::AggregateMetricSet,
    model_names::Vector{S}
    )


    df = DataFrame(labels=["mean lane offset", "mean speed", "mean timegap", 
                           "mean lane offset kldiv", "mean speed kldiv", "mean timegap kldiv", "histobin kldiv",
                           "mean trace log prob", "mean rmse 1s", "mean rmse 2s", "mean rmse 3s", "mean rmse 4s"])

    aggmetrics_original = calc_aggregate_metrics(original_metrics.metrics)
    df[:realworld] = [
                        @sprintf("%.3f +- %.3f", aggmetrics_original[:mean_centerline_offset_ego_mean], aggmetrics_original[:mean_centerline_offset_ego_stdev]),
                        @sprintf("%.3f +- %.3f", aggmetrics_original[:mean_speed_ego_mean], aggmetrics_original[:mean_speed_ego_stdev]),
                        @sprintf("%.3f +- %.3f", aggmetrics_original[:mean_timegap_mean], aggmetrics_original[:mean_timegap_stdev]),
                        "", "", "", "", "", "", "", "", ""
                    ]

    for (i,behavior_metric) in enumerate(behavior_metrics)

        behavor_sym = symbol(model_names[i])
        aggmetrics = calc_aggregate_metrics(behavior_metric.metrics)

        mean_trace_log_prob, stdev_trace_log_prob = calc_aggregate_metric(:logl, Float64, behavior_metric.metrics)
        mean_rmse_1s, stdev_rmse_1s = calc_aggregate_metric(:rmse_1000ms, Float64, behavior_metric.metrics)
        mean_rmse_2s, stdev_rmse_2s = calc_aggregate_metric(:rmse_2000ms, Float64, behavior_metric.metrics)
        mean_rmse_3s, stdev_rmse_3s = calc_aggregate_metric(:rmse_3000ms, Float64, behavior_metric.metrics)
        mean_rmse_4s, stdev_rmse_4s = calc_aggregate_metric(:rmse_4000ms, Float64, behavior_metric.metrics)

        df[behavor_sym] = [
                @sprintf("%.3f +- %.3f", aggmetrics[:mean_centerline_offset_ego_mean], aggmetrics[:mean_centerline_offset_ego_stdev]),
                @sprintf("%.3f +- %.3f", aggmetrics[:mean_speed_ego_mean], aggmetrics[:mean_speed_ego_stdev]),
                @sprintf("%.3f +- %.3f", aggmetrics[:mean_timegap_mean], aggmetrics[:mean_timegap_stdev]),
                @sprintf("%.5f", calc_kl_div_gaussian(aggmetrics_original, aggmetrics, :mean_centerline_offset_ego)),
                @sprintf("%.5f", calc_kl_div_gaussian(aggmetrics_original, aggmetrics, :mean_speed_ego)),
                @sprintf("%.5f", calc_kl_div_gaussian(aggmetrics_original, aggmetrics, :mean_timegap)),
                @sprintf("%.5f", behavior_metric.histobin_kldiv),
                @sprintf("%.4f", mean_trace_log_prob),
                @sprintf("%.4f", mean_rmse_1s),
                @sprintf("%.4f", mean_rmse_2s),
                @sprintf("%.4f", mean_rmse_3s),
                @sprintf("%.4f", mean_rmse_4s)
            ]
    end

    df
end