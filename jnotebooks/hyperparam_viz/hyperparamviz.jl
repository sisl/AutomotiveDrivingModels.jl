using DataFrames
using PGFPlots
using Interact

const INPUT_DIR = "/media/tim/DATAPART1/PublicationData/2015_TrafficEvolutionModels/realworld"

function get_hyperparam_logfiles()
    retval = AbstractString[]
    for file in readdir(INPUT_DIR)
        if splitext(file)[2] == ".csv" && ismatch(r"validation_results", file)
            push!(retval, file)
        end
    end
    retval
end

load_hyperparam_log(file::AbstractString) = readtable(joinpath(INPUT_DIR, file))
get_hyperparam_names(df::DataFrame) = map(str->symbol(str), names(df)[7:end])
get_target_names(df::DataFrame) = map(str->symbol(str), names(df)[1:6])

##########

_sym_to_latex_string(sym::Symbol) = replace(string(sym), "_", " ")

function print_log_statistics(df::DataFrame)
    @printf("n entries:             %20d\n", nrow(df))
    @printf("max mean logl train:   %20.6f  %6d\n", maximum(df[:mean_logl_train]), indmax(df[:mean_logl_train]))
    @printf("max mean logl test:    %20.6f  %6d\n", maximum(df[:mean_logl_test]), indmax(df[:mean_logl_test]))
    @printf("max median logl train: %20.6f  %6d\n", maximum(df[:median_logl_train]), indmax(df[:median_logl_train]))
    @printf("max median logl test:  %20.6f  %6d\n", maximum(df[:median_logl_test]), indmax(df[:median_logl_test]))
    @printf("min rwse speed 4sec:   %20.6f  %6d\n", minimum(df[:rwse_speed_4]), indmin(df[:rwse_speed_4]))
    @printf("min rwse posFt 4sec:   %20.6f  %6d\n", minimum(df[:rwse_posft_4]), indmin(df[:rwse_posft_4]))
    println("")

    println("hyperparameters:")
    for λ in get_hyperparam_names(df)
        @printf("\t%20s  %20.6f  %20.6f\n", λ, minimum(df[λ]), maximum(df[λ]))
    end
end

function scatter_performance(df::DataFrame, symA::Symbol, symB::Symbol, target::Symbol)
    Axis(Plots.Scatter(df[symA], df[symB], df[target]), colorbar=true, xlabel=_sym_to_latex_string(symA), ylabel=_sym_to_latex_string(symB), title=_sym_to_latex_string(target))
end
function scatter_performance(df::DataFrame, target::Symbol)

    params = get_hyperparam_names(df)

    g = GroupPlot(length(params), length(params), style=@sprintf("width=%.1fcm", 25.0/length(params)))

    for (j,symB) in enumerate(params)
        for (i,symA) in enumerate(params)
            if symA == symB
                # scatter plot of sym vs. target
                ax = Axis(Plots.Scatter(df[symA],df[target], style="black"))
                if j == length(params)
                    ax.xlabel = _sym_to_latex_string(symA)
                end
                if i == 1
                    ax.ylabel = _sym_to_latex_string(symA)
                end
            else
                # scatter plot of symA vs. symB with target as color
                ax = Axis(Plots.Scatter(df[symA], df[symB], df[target]))
                if j == length(params)
                    ax.xlabel = _sym_to_latex_string(symA)
                end
                if i == 1
                    ax.ylabel = _sym_to_latex_string(symB)
                end
            end

            push!(g, ax)
        end
    end

    g
end
