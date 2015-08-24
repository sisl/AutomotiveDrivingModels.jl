export 
        ParamsHistobin,

        calc_trace_deviation,
        calc_traceset_deviations,
        
        allocate_empty_histobin,
        update_histobin!,
        calc_histobin,

        KL_divergence_categorical,
        KL_divergence_dirichlet,
        KL_divergence_univariate_gaussian,
        coordinate_descent_histobin_fit!

type ParamsHistobin
    discx :: LinearDiscretizer
    discy :: LinearDiscretizer

    ParamsHistobin(discx::LinearDiscretizer, discy::LinearDiscretizer) = new(discx, discy)
    function ParamsHistobin(binedgesx::Vector{Float64}, binedgesy::Vector{Float64})
        discx = LinearDiscretizer(binedgesx)
        discy = LinearDiscretizer(binedgesx)
        new(discx, discy)
    end
end

function calc_trace_deviation(pdset::PrimaryDataset, sn::StreetNetwork, seg::PdsetSegment)
    
    initial_carind = carid2ind(pdset, seg.carid, seg.validfind_start)
    velFx = get(pdset, :velFx, initial_carind, seg.validfind_start)
    velFy = get(pdset, :velFy, initial_carind, seg.validfind_start)
    initial_speed = hypot(velFx, velFy)

    final_carind = carid2ind(pdset, seg.carid, seg.validfind_end)
    posGx_A = get(pdset, :posGx, initial_carind, seg.validfind_start)
    posGy_A = get(pdset, :posGy, initial_carind, seg.validfind_start)
    posGx_B = get(pdset, :posGx, final_carind, seg.validfind_end)
    posGy_B = get(pdset, :posGy, final_carind, seg.validfind_end)

    Δs, Δd = frenet_distance_between_points(sn, posGx_A, posGy_A, posGx_B, posGy_B)
    if isnan(Δs)
        warn("NaN Δs: ", (Δs, Δd))
        println((posGx_A, posGy_A, posGx_B, posGy_B))
        println(project_point_to_streetmap(posGx_A, posGy_A, sn))
    end
    @assert(!isnan(Δd))
    @assert(!isnan(Δs))
    @assert(!isnan(initial_speed))

    Δt = Δs / initial_speed

    if isnan(Δt)
        println("initial_carind: ", initial_carind)
        println("velFx: ", velFx)
        println("velFy: ", velFy)
        println("seg: ", seg)
        println("Δs: ", Δs)
        println("Δd: ", Δd)
        println("initial_speed: ", initial_speed)
        println(project_point_to_streetmap(posGx_A, posGy_A, sn))
        println(project_point_to_streetmap(posGx_B, posGy_B, sn))
    end
    @assert(!isnan(Δt))
    
    (Δt, Δd)
end
function calc_trace_deviation(simlog::Matrix{Float64}, history::Int)
    
    initial_speed = simlog[history,LOG_COL_V]

    Δx = simlog[end,LOG_COL_X] - simlog[history,LOG_COL_X]
    Δy = simlog[end,LOG_COL_Y] - simlog[history,LOG_COL_Y]
    Δt = Δx / initial_speed
    
    (Δt, Δy)
end
calc_trace_deviation(trace::VehicleTrace) = calc_trace_deviation(trace.log, trace.history)

function calc_traceset_deviations(tracesets::Vector{Vector{VehicleTrace}})

    m = length(tracesets)
    Δt_arr = Array(Float64, m) # x-deviation divided by initial speed
    Δy_arr = Array(Float64, m)

    for (i,traceset) in enumerate(tracesets)

        egotrace = traceset[1]
        Δt, Δy = calc_trace_deviation(egotrace)

        Δt_arr[i] = Δt
        Δy_arr[i] = Δy
    end

    (Δt_arr, Δy_arr)
end
function calc_traceset_deviations(simlogs::Vector{Matrix{Float64}}, history::Int)

    m = length(simlogs)
    Δt_arr = Array(Float64, m) # x-deviation divided by initial speed
    Δy_arr = Array(Float64, m)

    for (i,simlog) in enumerate(simlogs)
        
        Δt, Δy = calc_trace_deviation(simlog, history)

        Δt_arr[i] = Δt
        Δy_arr[i] = Δy
    end

    (Δt_arr, Δy_arr)
end

function allocate_empty_histobin(histobin_params::ParamsHistobin)
    n_bins_x = nlabels(histobin_params.discx)
    n_bins_y = nlabels(histobin_params.discy)
    zeros(Float64, n_bins_x, n_bins_y)
end
function update_histobin!(
    histobin::Matrix{Float64},
    pdset::PrimaryDataset,
    sn::StreetNetwork,
    seg::PdsetSegment,
    histobin_params::ParamsHistobin,
    )

    Δt, Δy = calc_trace_deviation(pdset, sn, seg)

    bin_x = encode(histobin_params.discx, Δt)
    bin_y = encode(histobin_params.discy, Δy)

    histobin[bin_x, bin_y] += 1.0

    histobin
end
function calc_histobin(
    pdsets::Vector{PrimaryDataset},
    streetnets::Vector{StreetNetwork},
    pdset_segments::Vector{PdsetSegment},
    histobin_params::ParamsHistobin,
    fold::Integer,
    pdsetseg_fold_assignment::Vector{Int},
    match_fold::Bool
    )

    histobin = allocate_empty_histobin(histobin_params)

    for (fold_assignment, seg) in zip(pdsetseg_fold_assignment, pdset_segments)

        if is_in_fold(fold, fold_assignment, match_fold)

            update_histobin!(histobin, 
                             pdsets[seg.pdset_id], 
                             streetnets[seg.streetnet_id],
                             seg, histobin_params)
        end
    end

    histobin
end

function calc_histobin(Δt_arr::Vector{Float64}, Δy_arr::Vector{Float64}, histobin_params::ParamsHistobin)

    histobin = allocate_empty_histobin(histobin_params)

    for (Δt, Δy) in zip(Δt_arr, Δy_arr)

        bin_x = encode(histobin_params.discx, Δt)
        bin_y = encode(histobin_params.discy, Δy)

        histobin[bin_x, bin_y] += 1.0
    end

    histobin
end
function calc_histobin(tracesets::Vector{Vector{VehicleTrace}}, histobin_params::ParamsHistobin)

    histobin = allocate_empty_histobin(histobin_params)

    for (i,traceset) in enumerate(tracesets)

        egotrace = traceset[1]
        Δt, Δy = calc_trace_deviation(egotrace)

        bin_x = encode(histobin_params.discx, Δt)
        bin_y = encode(histobin_params.discy, Δy)

        histobin[bin_x, bin_y] += 1.0
    end

    histobin
end
function calc_histobin(simlogs::Vector{Matrix{Float64}}, histobin_params::ParamsHistobin, history::Int)

    histobin = allocate_empty_histobin(histobin_params)

    for (i,simlog) in enumerate(simlogs)

        Δt, Δy = calc_trace_deviation(simlog, history)

        bin_x = encode(histobin_params.discx, Δt)
        bin_y = encode(histobin_params.discy, Δy)

        histobin[bin_x, bin_y] += 1.0
    end

    histobin
end

function KL_divergence_categorical(histobinA::Matrix{Float64}, histobinB::Matrix{Float64})
    
    Ap = histobinA ./ sum(histobinA)
    Bp = histobinB ./ sum(histobinB)

    KL_divergence = 0.0
    for i = 1 : length(Ap)
        KL_divergence += Ap[i]*log(Ap[i]/Bp[i])
    end
    KL_divergence::Float64
end
function KL_divergence_dirichlet(histobinA::Matrix{Float64}, histobinB::Matrix{Float64})
    α0 = sum(histobinA)
    KL_divergence = 0.0
    KL_divergence += lgamma(α0)
    KL_divergence -= lgamma(sum(histobinB))
    KL_divergence -= sum([lgamma(a) for a in histobinA])
    KL_divergence += sum([lgamma(b) for b in histobinB])
    for i = 1 : length(histobinA)
        KL_divergence += (histobinA[i] - histobinB[i])*(digamma(histobinA[i]) - digamma(α0))
    end
    KL_divergence::Float64
end
function KL_divergence_univariate_gaussian(μ1::Float64, μ2::Float64, σ1::Float64, σ2::Float64)
    Δμ = (μ1-μ2)
    log(σ2/σ1) + (σ1*σ1 + Δμ*Δμ)/(2σ2*σ2) - 0.5
end

function coordinate_descent_histobin_fit!{B<:AbstractVehicleBehavior}(
    simlogs         :: Vector{Matrix{Float64}},
    behaviors       :: Vector{B},
    road            :: StraightRoadway,
    history         :: Int,
    target_histobin :: Matrix{Float64},
    histobin_params :: ParamsHistobin,
    simparams       :: SimParams,    
    KLdiv_method    :: Symbol; # ∈ :Dirichlet, :Categorical
    verbosity       :: Int = 0
    )

    if KLdiv_method == :Dirichlet
        KL_div_function = KL_divergence_dirichlet
    elseif KLdiv_method == :Categorical
        KL_div_function = KL_divergence_categorical
    else
        error("unknown KL divergence method $KLdiv_method")
    end

    if verbosity > 0
        println("Coordinte descent $KLdiv_method")
        tic()
    end

    target_histobin_with_prior = target_histobin .+ 1.0

    param_options = (
        [SAMPLE_UNIFORM, SAMPLE_BIN_CENTER, SAMPLE_UNIFORM_ZERO_BIN],
        [(:none, 1), (:SMA, 2), (:SMA, 3), (:SMA, 4), (:SMA, 5),
                     (:WMA, 2), (:WMA, 3), (:WMA, 4), (:WMA, 5)],
        [SAMPLE_UNIFORM, SAMPLE_BIN_CENTER, SAMPLE_UNIFORM_ZERO_BIN],
        [(:none, 1), (:SMA, 2), (:SMA, 3), (:SMA, 4), (:SMA, 5),
                     (:WMA, 2), (:WMA, 3), (:WMA, 4), (:WMA, 5)]
    )

    n_params = length(param_options)
    params_tried = Set{Vector{Int}}() # paraminds
    paraminds = ones(Int, n_params)
    egobehavior = behaviors[1]

    converged = false
    best_KLdiv = Inf
    iter = 0
    while !converged
        iter += 1
        converged = true
        if verbosity > 0
            println("iteration ", iter)
            toc()
            tic()
        end
        
        for coordinate = 1 : n_params
            if verbosity > 1
                println("\tcoordinate ", coordinate)
            end

            for ip in 1 : length(param_options[coordinate])

                newparams = copy(paraminds)
                newparams[coordinate] = ip

                if !in(newparams, params_tried)
                    push!(params_tried, newparams)
                    
                    egobehavior.simparams_lat.sampling_scheme  = param_options[1][newparams[1]]
                    egobehavior.simparams_lon.sampling_scheme  = param_options[3][newparams[3]]
                    egobehavior.simparams_lat.smoothing        = param_options[2][newparams[2]][1]
                    egobehavior.simparams_lat.smoothing_counts = param_options[2][newparams[2]][2]
                    egobehavior.simparams_lon.smoothing        = param_options[4][newparams[4]][1]
                    egobehavior.simparams_lon.smoothing_counts = param_options[4][newparams[4]][2]
                    
                    simulate!(simlogs, behaviors, road, history, simparams)
                    # TODO(tim): compute histobin directly
                    (Δt_arr, Δy_arr) = calc_traceset_deviations(simlogs, history)
                    histobin = calc_histobin(Δt_arr, Δy_arr, histobin_params)
                    histobin .+= 1.0
                    KLdiv = KL_div_function(target_histobin_with_prior,histobin)

                    if KLdiv < best_KLdiv
                        best_KLdiv = KLdiv
                        paraminds[coordinate] = ip
                        converged = false
                        if verbosity > 0
                            println("\tfound better: ", coordinate, " -> ", param_options[coordinate][ip])
                            println("\t\tKL: ", best_KLdiv)
                        end
                    end

                end
            end
        end
    end

    if verbosity > 0
        toc()
        println("optimal params: ", paraminds)
        println("KL-divergence: ", best_KLdiv)
    end


    (ModelSimParams(
        param_options[1][paraminds[1]],
        param_options[2][paraminds[2]][1],
        param_options[2][paraminds[2]][2]),
     ModelSimParams(
        param_options[3][paraminds[3]],
        param_options[4][paraminds[4]][1],
        param_options[4][paraminds[4]][2]),
     best_KLdiv)
end