using AutomotiveDrivingModels

using PGFPlots

function calc_recommended_log_scale_factor(histobins::Vector{Matrix{Float64}})
    max_count = -1
    for histobin in histobins
        max_count = max(max_count, maximum(histobin))
    end
    div = Base.log10(max_count+1)
end
function calc_histobin_image_matrix( histobin::Matrix{Float64}, log_scale_factor::Float64 = NaN)

    imagemat = histobin .+ 1.0
    
    if !isnan(log_scale_factor)
        imagemat = log10(imagemat)
        imagemat ./= log_scale_factor
    else
        imagemat ./= maximum(imagemat)
    end

    imagemat = rotr90(imagemat)
    imagemat = 1.0 - imagemat

    @assert(maximum(imagemat) ≤ 1.0)
    @assert(minimum(imagemat) ≥ 0.0)

    imagemat
end
function plot_histobin_image_matrix(
    imagemat :: Matrix{Float64},
    params   :: ParamsHistobin;
    )

    xrange = (params.discx.binedges[1], params.discx.binedges[end])
    yrange = (params.discy.binedges[1], params.discy.binedges[end])

    p = PGFPlots.Image(imagemat, yrange, xrange, zmin=0.0, zmax=1.0, colorbar=false)
    ax = Axis(p, xlabel="Lateral Deviation (m)", ylabel="Longitudinal Deviation (s)")
end
function plot_histobin_group_plot{S<:String}(
    behavior_metrics_sets::AbstractVector{MetricsSet},
    params::ParamsHistobin,
    names::Vector{S}
    )
    
    nhistobins = length(behavior_metrics_sets)
    histobins = Array(Matrix{Float64}, nhistobins)
    for (i,metrics) in enumerate(behavior_metrics_sets)
        histobins[i] = metrics.histobin
    end

    log_scale_factor = calc_recommended_log_scale_factor(histobins)

    g = GroupPlot(1,1, groupStyle = "
        horizontal sep = 1.5mm, 
        vertical sep = 1.5mm,
        xticklabels at=edge bottom,
        yticklabels at=edge left, ")

    for (i,metrics) in enumerate(behavior_metrics_sets)
        ax = plot_histobin_image_matrix(calc_histobin_image_matrix(metrics.histobin, log_scale_factor), params)
        ax.enlargelimits = false
        ax.axisOnTop = true
        ax.view = "{0}{90}"
        ax.ylabel = nothing
        ax.title = "\\textbf{\\strut "* names[i] * "}"
        push!(g, ax)
    end
    g
end
function plot_histobin_group_plot{S<:String}(
    behavior_metrics_sets::AbstractVector{MetricsSet},
    original_metrics_set::MetricsSet,
    params::ParamsHistobin,
    names::Vector{S}
    )
    
    metrics_sets = Array(MetricsSet, length(behavior_metrics_sets)+1)
    metrics_sets[1] = original_metrics_set
    metrics_sets[2:end] = behavior_metrics_sets

    plot_histobin_group_plot(metrics_sets, params, ["Real World", names...])
end
function save_histobin_image(
    filename         :: String,
    histobin         :: Matrix{Float64},
    params           :: ParamsHistobin;
    log_scale_factor :: Float64 = NaN
    )

    imagemat = calc_histobin_image_matrix(histobin, log_scale_factor)
    ax = plot_histobin_image_matrix(imagemat, params)

    PGFPlots.save(filename, ax)
end