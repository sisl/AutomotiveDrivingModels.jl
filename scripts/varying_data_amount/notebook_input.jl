using PGFPlots
using Interact
using DataFrames

const RESULTS_DIR = Pkg.dir("AutomotiveDrivingModels", "scripts", "varying_data_amount", "results")
const DASH_TYPES = ["solid", "dashdotted", "dashed", "densely dotted", "loosely dotted", "densely dashdotted", "solid"]

function load_data()

    modelnames = ["Static Gaussian", "Linear Gaussian", "Dynamic Forest", "Random Forest", "Bayesian Network", "Linear Bayesian"] # "Mixture Regression",

    dfs = Dict{AbstractString, DataFrame}()
    for model_name in modelnames

        model_output_name = replace(lowercase(model_name), " ", "_")
        inpath = joinpath(RESULTS_DIR, "data_vs_performance_metrics_" * model_output_name * ".csv")
        dfs[model_name] = readtable(inpath)
    end

    dfs
end

function plot_target_vs_data_amount(dfs::Dict{AbstractString, DataFrame}, target::Symbol)

    plots = PGFPlots.Plots.Plot[]

    for (i, tup) in enumerate(dfs)

        name, df = tup

        percentages = sort(unique(convert(Vector{Float64}, df[:dataset_percentage])))
        color = 30 + 10*i

        targets = zeros(Float64, length(percentages))

        # @printf(io, "\\addplot[%s, %s, thick, mark=none] coordinates{\ncolor=black!", color, DASH_TYPES[i])
        # @printf(io, "\t")

        for (j,p) in enumerate(percentages)
            targetval = 0.0
            nfound = 0
            for k in 1 : nrow(df)
                if df[k, :dataset_percentage] == p &&
                   df[k, :model_name] == name

                   targetval += df[k, target]
                   nfound += 1
                end
            end
            targets[j] = targetval / nfound
        end

        push!(plots, Plots.Linear(percentages, targets, mark="none", style="thick, color=black!" * string(color) * ", " * DASH_TYPES[i]))
    end

    Axis(plots, xmode="log", xlabel="data fraction", ylabel=replace(string(target), "_", " "), width="21cm")
end

