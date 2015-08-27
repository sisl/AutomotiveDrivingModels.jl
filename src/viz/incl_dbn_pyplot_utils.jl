using PyPlot
using PyCall

function plot_marginal_distribution_over_conditioned_bin(
    fig::Figure,
    basics::FeatureExtractBasicsPdSet,
    behavior::DynamicBayesianNetworkBehavior,
    carind::Int,
    validfind::Int;
    color_lat=[0.7,0.3,0.3], # low probability
    color_lon=[0.3,0.3,0.7], # high probability
    )

    ax_lat = fig[:add_subplot](121)
    ax_lon = fig[:add_subplot](122)
    
    model = behavior.model

    observations = observe(basics, carind, validfind, behavior.indicators)
    assignment = encode(model, observations)
    target_lat = symbol(get_target_lat(model))
    binedges = deepcopy(model.discretizers[indexof(target_lat, model)].binedges)
    weights = Array(Float64, length(binedges)-1)
    calc_probability_distribution_over_assignments(weights, model, assignment, target_lat)

    plot_sample_distribution(ax_lat, binedges, weights, x_label="lateral", color=color_lat)
    ax_lat[:xaxis][:set_ticks](binedges)
    ax_lat[:xaxis][:set_ticklabels](map(e->@sprintf("%.5f", e), binedges), rotation=90)
    ax_lat[:xaxis][:set_label_position]("top")

    observations = observe(basics, carind, validfind, behavior.indicators)
    assignment = encode(model, observations)
    target_lon = symbol(get_target_lon(model))
    binedges = deepcopy(model.discretizers[indexof(target_lon, model)].binedges)
    weights = Array(Float64, length(binedges)-1)
    calc_probability_distribution_over_assignments(weights, model, assignment, target_lon)
    plot_sample_distribution(ax_lon, binedges, weights, x_label="longitudinal", color=color_lon)
    ax_lon[:xaxis][:set_ticks](binedges)
    ax_lon[:xaxis][:set_ticklabels](map(e->@sprintf("%.5f", e), binedges), rotation=90)
    ax_lon[:xaxis][:set_label_position]("top")

    fig
end