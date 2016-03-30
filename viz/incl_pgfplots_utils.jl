using AutomotiveDrivingModels
using PGFPlots

define_color("monokai1", [0xCF/0xFF,0xBF/0xFF,0xAD/0xFF])
define_color("monokai2", [0x27/0xFF,0x28/0xFF,0x22/0xFF])
define_color("monokai3", [0x52/0xFF,0xE3/0xFF,0xF6/0xFF])
define_color("monokai4", [0xA7/0xFF,0xEC/0xFF,0x21/0xFF])
define_color("monokai5", [0xFF/0xFF,0x00/0xFF,0x7F/0xFF])
define_color("monokai6", [0xF9/0xFF,0x97/0xFF,0x1F/0xFF])
define_color("monokai7", [0x79/0xFF,0xAB/0xFF,0xFF/0xFF])


function get_plot_logl_vs_trace(
    behavior::AbstractVehicleBehavior,
    runlog::RunLog,
    sn::StreetNetwork,
    frame_start::Integer,
    frame_end::Integer,
    id::UInt;
    color="blue"
    )

    frames = collect(frame_start:N_FRAMES_PER_SIM_FRAME:frame_end)
    logls = Array(Float64, length(frames))
    targets = get_targets(behavior)

    for (i,frame) in enumerate(frames)

        colset = id2colset(runlog, id, frame)
        action_lat = get(targets.lat, runlog, sn, colset, frame)
        action_lon = get(targets.lon, runlog, sn, colset, frame)
        logls[i] = calc_action_loglikelihood(behavior, runlog, sn, colset, frame, action_lat, action_lon)
    end

    Plots.Linear(frames, logls, style="mark=none, color="*color)
end
function plot_logl_vs_trace(
    behavior::AbstractVehicleBehavior,
    runlog::RunLog,
    sn::StreetNetwork,
    frame_start::Integer,
    frame_end::Integer,
    id::UInt,
    )

    Axis(get_plot_logl_vs_trace(behavior, runlog, sn, frame_start, frame_end, id),
        xlabel="frame", ylabel="action log likelihood")
end
function plot_logl_vs_trace(
    model_names::Vector{AbstractString},
    models::Dict{AbstractString, AbstractVehicleBehavior},
    runlog::RunLog,
    sn::StreetNetwork,
    frame_start::Integer,
    frame_end::Integer,
    id::UInt;

    styles = [
                "mark=none, solid, color=monokai1",
                "mark=none, solid, color=monokai2",
                "mark=none, solid, color=monokai3",
                "mark=none, solid, color=monokai4",
                "mark=none, solid, color=monokai5",
                "mark=none, solid, color=monokai6",
                "mark=none, solid, color=monokai7",
            ],
    include_legend_entry::Bool = false
    )


    plots = Plots.Plot[]
    for (i,name) in enumerate(model_names)
        push!(plots, get_plot_logl_vs_trace(models[name], runlog, sn, frame_start, frame_end, id))
        plots[end].style = styles[i]

        if include_legend_entry
            plots[end].legendentry = name
        end
    end

    Axis(plots, xlabel="frame", ylabel="action log likelihood")
end
function plot_logl_vs_trace(
    model_names::Vector{AbstractString},
    models::Dict{AbstractString, AbstractVehicleBehavior},
    sim_runlogs::Dict{AbstractString, RunLog},
    sn::StreetNetwork,
    frame_start::Integer,
    frame_end::Integer,
    id::UInt;

    styles = [
                "mark=none, solid, color=monokai1",
                "mark=none, solid, color=monokai2",
                "mark=none, solid, color=monokai3",
                "mark=none, solid, color=monokai4",
                "mark=none, solid, color=monokai5",
                "mark=none, solid, color=monokai6",
                "mark=none, solid, color=monokai7",
            ],
    include_legend_entry::Bool = false
    )


    plots = Plots.Plot[]
    for (i,name) in enumerate(model_names)
        push!(plots, get_plot_logl_vs_trace(models[name], sim_runlogs[name], sn, frame_start, frame_end, id))
        plots[end].style = styles[i]

        if include_legend_entry
            plots[end].legendentry = name
        end
    end

    Axis(plots, xlabel="frame", ylabel="action log likelihood")
end

function get_plot_target_lat_vs_trace(
    behavior::AbstractVehicleBehavior,
    runlog::RunLog,
    sn::StreetNetwork,
    frame_start::Integer,
    frame_end::Integer,
    id::UInt;
    color="blue!60"
    )

    frames = collect(frame_start:N_FRAMES_PER_SIM_FRAME:frame_end)
    actions_lat = Array(Float64, length(frames))
    targets = get_targets(behavior)

    for (i,frame) in enumerate(frames)

        colset = id2colset(runlog, id, frame)
        actions_lat[i] = get(targets.lat, runlog, sn, colset, frame)
    end

    Plots.Linear(frames, actions_lat, style="mark=none, thick, color="*color)
end
function plot_target_lat_vs_trace(
    behavior::AbstractVehicleBehavior,
    runlog::RunLog,
    sn::StreetNetwork,
    frame_start::Integer,
    frame_end::Integer,
    id::UInt,
    )

    Axis(get_plot_target_lat_vs_trace(behavior, runlog, sn, frame_start, frame_end, id),
        xlabel="frame", ylabel="target lat")
end
function plot_target_lat_vs_trace(
    model_names::Vector{AbstractString},
    models::Dict{AbstractString, AbstractVehicleBehavior},
    runlog::RunLog,
    sn::StreetNetwork,
    frame_start::Integer,
    frame_end::Integer,
    id::UInt;

    styles = [
                "mark=none, solid, color=monokai1",
                "mark=none, solid, color=monokai2",
                "mark=none, solid, color=monokai3",
                "mark=none, solid, color=monokai4",
                "mark=none, solid, color=monokai5",
                "mark=none, solid, color=monokai6",
                "mark=none, solid, color=monokai7",
            ],
    include_legend_entry::Bool = false,
    )

    plots = Plots.Plot[]
    for (i,name) in enumerate(model_names)
        push!(plots, get_plot_target_lat_vs_trace(models[name], runlog, sn, frame_start, frame_end, id))
        plots[end].style = styles[i]

        if include_legend_entry
            plots[end].legendentry = name
        end
    end

    Axis(plots, xlabel="frame", ylabel="target lat")
end
function plot_target_lat_vs_trace(
    model_names::Vector{AbstractString},
    models::Dict{AbstractString, AbstractVehicleBehavior},
    sim_runlogs::Dict{AbstractString, RunLog},
    sn::StreetNetwork,
    frame_start::Integer,
    frame_end::Integer,
    id::UInt;

    styles = [
                "mark=none, solid, color=monokai1",
                "mark=none, solid, color=monokai2",
                "mark=none, solid, color=monokai3",
                "mark=none, solid, color=monokai4",
                "mark=none, solid, color=monokai5",
                "mark=none, solid, color=monokai6",
                "mark=none, solid, color=monokai7",
            ],
    include_legend_entry::Bool = false,
    )

    plots = Plots.Plot[]
    for (i,name) in enumerate(model_names)
        push!(plots, get_plot_target_lat_vs_trace(models[name], sim_runlogs[name], sn, frame_start, frame_end, id))
        plots[end].style = styles[i]

        if include_legend_entry
            plots[end].legendentry = name
        end
    end

    Axis(plots, xlabel="frame", ylabel="target lat")
end

function get_plot_target_lon_vs_trace(
    behavior::AbstractVehicleBehavior,
    runlog::RunLog,
    sn::StreetNetwork,
    frame_start::Integer,
    frame_end::Integer,
    id::UInt;
    color="blue!80"
    )

    frames = collect(frame_start:N_FRAMES_PER_SIM_FRAME:frame_end)
    actions_lon = Array(Float64, length(frames))
    targets = get_targets(behavior)

    for (i,frame) in enumerate(frames)

        colset = id2colset(runlog, id, frame)
        actions_lon[i] = get(targets.lon, runlog, sn, colset, frame)
    end

    Plots.Linear(frames, actions_lon, style="mark=none, thick, color="*color)
end
function plot_target_lon_vs_trace(
    behavior::AbstractVehicleBehavior,
    runlog::RunLog,
    sn::StreetNetwork,
    frame_start::Integer,
    frame_end::Integer,
    id::UInt,
    )

    Axis(get_plot_target_lon_vs_trace(behavior, runlog, sn, frame_start, frame_end, id),
        xlabel="frame", ylabel="target lon")
end
function plot_target_lon_vs_trace(
    model_names::Vector{AbstractString},
    models::Dict{AbstractString, AbstractVehicleBehavior},
    runlog::RunLog,
    sn::StreetNetwork,
    frame_start::Integer,
    frame_end::Integer,
    id::UInt;

    styles = [
                "mark=none, solid, color=monokai1",
                "mark=none, solid, color=monokai2",
                "mark=none, solid, color=monokai3",
                "mark=none, solid, color=monokai4",
                "mark=none, solid, color=monokai5",
                "mark=none, solid, color=monokai6",
                "mark=none, solid, color=monokai7",
            ],
    include_legend_entry::Bool = false,
    )

    plots = Plots.Plot[]
    for (i,name) in enumerate(model_names)
        push!(plots, get_plot_target_lon_vs_trace(models[name], runlog, sn, frame_start, frame_end, id))
        plots[end].style = styles[i]

        if include_legend_entry
            plots[end].legendentry = name
        end
    end

    Axis(plots, xlabel="frame", ylabel="target lon")
end
function plot_target_lon_vs_trace(
    model_names::Vector{AbstractString},
    models::Dict{AbstractString, AbstractVehicleBehavior},
    sim_runlogs::Dict{AbstractString, RunLog},
    sn::StreetNetwork,
    frame_start::Integer,
    frame_end::Integer,
    id::UInt;

    styles = [
                "mark=none, solid, color=monokai1",
                "mark=none, solid, color=monokai2",
                "mark=none, solid, color=monokai3",
                "mark=none, solid, color=monokai4",
                "mark=none, solid, color=monokai5",
                "mark=none, solid, color=monokai6",
                "mark=none, solid, color=monokai7",
            ],
    include_legend_entry::Bool = false,
    )

    plots = Plots.Plot[]
    for (i,name) in enumerate(model_names)
        push!(plots, get_plot_target_lon_vs_trace(models[name], sim_runlogs[name], sn, frame_start, frame_end, id))
        plots[end].style = styles[i]

        if include_legend_entry
            plots[end].legendentry = name
        end
    end

    Axis(plots, xlabel="frame", ylabel="target lon")
end

function plot_group_logl_vs_trace(
    behavior::AbstractVehicleBehavior,
    runlog::RunLog,
    sn::StreetNetwork,
    frame_start::Integer,
    frame_end::Integer,
    id::UInt,
    )

    g = GroupPlot(3, 1, groupStyle = "horizontal sep = 15mm")
    push!(g, plot_logl_vs_trace(behavior, runlog, sn, frame_start, frame_end, id))
    push!(g, plot_target_lat_vs_trace(behavior, runlog, sn, frame_start, frame_end, id))
    push!(g, plot_target_lon_vs_trace(behavior, runlog, sn, frame_start, frame_end, id))
    g
end
function plot_group_logl_vs_trace(
    model_names::Vector{AbstractString},
    models::Dict{AbstractString, AbstractVehicleBehavior},
    runlog::RunLog,
    sn::StreetNetwork,
    frame_start::Integer,
    frame_end::Integer,
    id::UInt,
    )

    g = GroupPlot(3, 1, groupStyle = "horizontal sep = 15mm")
    push!(g, plot_logl_vs_trace(model_names, models, runlog, sn, frame_start, frame_end, id))
    push!(g, plot_target_lat_vs_trace(model_names, models, runlog, sn, frame_start, frame_end, id))
    push!(g, plot_target_lon_vs_trace(model_names, models, runlog, sn, frame_start, frame_end, id, include_legend_entry=true))

    for axis in g.axes
        axis.width="7cm"
    end

    g.axes[end].legendPos = "outer north east"

    g
end
function plot_group_logl_vs_trace(
    model_names::Vector{AbstractString},
    models::Dict{AbstractString, AbstractVehicleBehavior},
    sim_runlogs::Dict{AbstractString, RunLog},
    sn::StreetNetwork,
    frame_start::Integer,
    frame_end::Integer,
    id::UInt,
    )

    g = GroupPlot(3, 1, groupStyle = "horizontal sep = 15mm")
    push!(g, plot_logl_vs_trace(model_names, models, sim_runlogs, sn, frame_start, frame_end, id))
    push!(g, plot_target_lat_vs_trace(model_names, models, sim_runlogs, sn, frame_start, frame_end, id))
    push!(g, plot_target_lon_vs_trace(model_names, models, sim_runlogs, sn, frame_start, frame_end, id, include_legend_entry=true))

    for axis in g.axes
        axis.width="7cm"
    end

    g.axes[end].legendPos = "outer north east"

    g
end