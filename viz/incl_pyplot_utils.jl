using AutomotiveDrivingModels

using PyPlot
using PyCall

function plot_sample_distribution(ax::PyObject, binedges::Vector{Float64}, weights::Vector{Float64};
    x_label :: String = "",
    y_label :: String = "probability",
    color::Any = rand(3)
    )

    left = binedges[1:end-1]
    width = binedges[2:end] - left
    height = weights ./ sum(weights)
    
    ax[:bar](left, height, width, color=color)
    ax[:set_xlim](binedges[1], binedges[end])
    ax[:set_ylim](0.0,1.0)
    ax[:set_ylabel]("probability")
    if !isempty(x_label)
        ax[:set_xlabel](x_label)
    end
end
function plot_sample_distribution(binedges::Vector{Float64}, weights::Vector{Float64};
    x_label :: String = "")

    fig = figure(facecolor="white")
    ax = fig[:add_subplot](111)
    plot_sample_distribution(ax, binedges, weights, x_label=x_label)
end

function plot_sample_counts(ax::PyObject, binedges::Vector{Float64}, counts::Vector{Float64};
    x_label :: String = ""
    )

    left = binedges[1:end-1]
    width = binedges[2:end] - left
    height = counts
    
    ax[:bar](left, height, width, color=rand(3))
    ax[:set_ylabel]("counts")
    if !isempty(x_label)
        ax[:set_xlabel](x_label)
    end
end
function plot_sample_counts(binedges::Vector{Float64}, counts::Vector{Float64};
    x_label :: String = "")

    fig = figure(facecolor="white")
    ax = fig[:add_subplot](111)
    plot_sample_counts(ax, binedges, counts, x_label=x_label)
end

function plot_probability_of_next_action(
    ax::PyObject,
    basics::FeatureExtractBasicsPdSet,
    behavior::AbstractVehicleBehavior,
    carind::Int,
    validfind::Int;
    domain_lat::(Float64, Float64)=(-0.08,0.08),
    domain_lon::(Float64, Float64)=(-3.25,1.8),
    nsamples_lat::Integer=10, # number of points to sample in action_domain
    nsamples_lon::Integer=10 # number of points to sample in action_domain
    )

    #=
    Plot a heatmap of the next action

    This function should return simlog unchanged
    =#

    @assert(domain_lat[1] < domain_lat[2])
    @assert(domain_lon[1] < domain_lon[2])
    @assert(nsamples_lat > 0)
    @assert(nsamples_lon > 0)

    # NOTE(tim): sample points are at the centers of the pcolormesh quadrilaterals
    #   in these arrays, the odd coordinates are the quad vertices and the
    #   even coordinates are the sample points
    all_points_lat = linspace(domain_lat[1], domain_lat[2], 2nsamples_lat+1)
    all_points_lon = linspace(domain_lon[1], domain_lon[2], 2nsamples_lon+1)

    probabilities = Array(Float64, nsamples_lat, nsamples_lon)

    for index_lat = 1 : nsamples_lat

        sample_lat = all_points_lat[2index_lat]

        for index_lon = 1 : nsamples_lon

            sample_lon = all_points_lon[2index_lon]

            logl = calc_action_loglikelihood(basics, behavior, carind, validfind, sample_lat, sample_lon)
            @assert(!isnan(logl))

            prob = exp(logl)
            @assert(prob ≥ 0)

            probabilities[index_lat,index_lon] = prob
        end
    end

    X = repmat(reshape(all_points_lat[1:2:end], 1, nsamples_lat+1), nsamples_lon+1, 1)
    Y = repmat(reshape(all_points_lon[1:2:end], nsamples_lon+1, 1), 1, nsamples_lat+1)

    probabilities ./= maximum(probabilities)
    for i = 1 : length(probabilities)
        probabilities[i] = probabilities[i]^0.25  #NOTE(tim): to make low probs stand out more
    end

    ax[:pcolormesh](X, Y, probabilities', vmin=0.0, vmax=1.0, cmap="gray")
    ax[:set_xlim](domain_lat[1], domain_lat[2])
    ax[:set_ylim](domain_lon[1], domain_lon[2])

    ax
end

function _get_action_loglikelihood_vec(
    basics::FeatureExtractBasicsPdSet,
    behavior::AbstractVehicleBehavior,
    seg::PdsetSegment,
    )

    nframes = get_nframes(seg)

    probabilities = Array(Float64, nframes)
    for (i,validfind) in enumerate(seg.validfind_start : seg.validfind_end)
        carind = carid2ind(basics.pdset, seg.carid, validfind)
        probabilities[i] = calc_action_loglikelihood(basics, behavior, carind, validfind)
        @assert(!isinf(probabilities[i]))
    end

    probabilities
end
function _get_time_vec(pdset::PrimaryDataset, seg::PdsetSegment)
    nframes = get_nframes(seg)
    time_arr = Array(Float64, nframes)
    time_start = gete(pdset, :time, validfind2frameind(pdset, seg.validfind_start))
    for (i,validfind) in enumerate(seg.validfind_start : seg.validfind_end)
        frameind = validfind2frameind(pdset, validfind)
        time_arr[i] = gete(pdset, :time, frameind) - time_start
    end
    time_arr
end

function plot_action_probability(
    ax::PyObject,
    basics::FeatureExtractBasicsPdSet,
    behavior::AbstractVehicleBehavior,
    seg::PdsetSegment;
    color=RGB(rand(), rand(), rand()),
    modelname="???"
    )

    carind = 1
    loglikelihoods = _get_action_loglikelihood_vec(basics, behavior, seg)
    timearr = _get_time_vec(basics.pdset, seg)
    ax[:plot](timearr, loglikelihoods, label=modelname, color=color)
    ax[:set_xlim](timearr[1], timearr[end])
    ax
end
