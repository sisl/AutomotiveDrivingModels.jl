using DataFrames

push!(LOAD_PATH, "/home/tim/Documents/wheelerworkspace/UtilityCode/")
using LaTeXeXport

const TEXFILE = "/home/tim/Documents/wheelerworkspace/Papers/2015_ITS_RiskEstimation/2015_IEEE_ITS_riskest.tex"
const TEXDIR = splitdir(TEXFILE)[1]
const RESULTS_DF_EXPERIMENT_1 = "/home/tim/.julia/v0.3/AutomotiveDrivingModels/scripts/varying_data_amount/results/data_vs_performance_metrics.csv"
const DASH_TYPES = ["solid", "dashdotted", "dashed", "densely dotted", "loosely dotted", "solid"]

function _convert_to_short_name(name::AbstractString)
    retval = ""
    for word in split(name)
        retval *= string(uppercase(word[1]))
    end
    retval
end
function _export_legend{S<:String}(io::IO, modelnames::Vector{S})
    # \legend{GF, SV, RF, DF, BN}

    print(io, "\\legend{")
    for (i,name) in enumerate(modelnames)
        print(io, name)
        if i != length(modelnames)
            print(io, ", ")
        end
    end
    print(io, "}\n")
end
function create_tikzpicture_experiment_1{S<:String}(io::IO, df::DataFrame, modelnames::Vector{S})

    #=
    This outputs, for each model by order of names:

    \addplot[colorE, dotted, thick, mark=none] coordinates{
        (0.0100,3472.2196) (0.0167,3179.6420) (0.0278,6060.3558) (0.0464,6231.8598) (0.0774,7445.1712) (0.1292,8600.2229) (0.2154,8947.1872) (0.3594,9629.5129) (0.5995,9900.6706) (1.0000,9782.7516) };

    And for the legend:


    =#

    percentages = sort(unique(convert(Vector{Float64}, df[:dataset_percentage])))

    for (i,name) in enumerate(modelnames)

        color = "color" * string('A' + i - 1)

        @printf(io, "\\addplot[%s, %s, thick, mark=none] coordinates{\n", color, DASH_TYPES[i])
        @printf(io, "\t")

        for p in percentages
            logl = 0.0
            nframes = 0
            for j in 1 : nrow(df)
                if df[j, :dataset_percentage] == p &&
                   df[j, :model_name] == name

                   logl += df[j, :logl_test] * df[j, :nframes]
                   nframes += df[j, :nframes]
                end
            end
            logl /= nframes

            @printf(io, "(%.4f,%.4f) ", p, logl)
        end
        println(io, "};")
    end

    _export_legend(io, map(_convert_to_short_name, modelnames))
end
# function create_tikzpicture_experiment_4{S<:String}(io::IO, df::DataFrame, model::Vector{S})

#     #=
#     This outputs a csv file for the given model name

#     x y z logl
#     0.0 0.0 1.0 0.0034743774733111514
#     0.0 0.25 0.75 0.015673856116184083
#     0.0 0.5 0.5 0.012098485065977549

#     And for the legend:


#     =#

#     percentages = sort(unique(convert(Vector{Float64}, df[:dataset_percentage])))

#     for (i,name) in enumerate(modelnames)

#         color = "color" * string('A' + i - 1)

#         @printf(io, "\\addplot[%s, %s, thick, mark=none] coordinates{\n", color, DASH_TYPES[i])
#         @printf(io, "\t")

#         for p in percentages
#             logl = 0.0
#             nframes = 0
#             for j in 1 : nrow(df)
#                 if df[j, :dataset_percentage] == p &&
#                    df[j, :model_name] == name

#                    logl += df[j, :logl_test] * df[j, :nframes]
#                    nframes += df[j, :nframes]
#                 end
#             end
#             logl /= nframes

#             @printf(io, "(%.4f,%.4f) ", p, logl)
#         end
#         println(io, "};")
#     end

#     _export_legend(io, map(_convert_to_short_name, modelnames))
# end

df_exp1 = readtable(RESULTS_DF_EXPERIMENT_1)

modelnames = ["Static Gaussian", "Linear Gaussian", "Random Forest", "Dynamic Forest", "Gaussian Mixture Regression", "Bayesian Network"]

fh = STDOUT
# write_to_texthook(TEXFILE, "varydata-experiment-1") do fh
    create_tikzpicture_experiment_1(fh, df_exp1, modelnames)
# end

println("DONE EXPORTING RESULTS")
