using DataFrames
using AutomotiveDrivingModels

push!(LOAD_PATH, "/home/tim/Documents/wheelerworkspace/UtilityCode/")
using LaTeXeXport

const TEXFILE = "/home/tim/Documents/papers/2016_its_car_behaviors_wheeler/its_car_behaviors.tex"
const TEXDIR = splitdir(TEXFILE)[1]

const RESULTS_DIR = Pkg.dir("AutomotiveDrivingModels", "scripts", "varying_data_amount", "results")
const DASH_TYPES = ["solid", "dashdotted", "dashed", "densely dotted", "loosely dotted", "densely dashdotted", "solid"]

function _export_legend{S<:AbstractString}(io::IO, modelnames::Vector{S})
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
# function create_tikzpicture_experiment_1(io::IO, dfs::Dict{AbstractString, DataFrame})

#     #=
#     This outputs, for each model by order of names:

#     \addplot[colorE, dotted, thick, mark=none] coordinates{
#         (0.0100,3472.2196) (0.0167,3179.6420) (0.0278,6060.3558) (0.0464,6231.8598) (0.0774,7445.1712) (0.1292,8600.2229) (0.2154,8947.1872) (0.3594,9629.5129) (0.5995,9900.6706) (1.0000,9782.7516) };
#     =#

#     for (i, tup) in enumerate(dfs)

#         name, df = tup

#         percentages = sort(unique(convert(Vector{Float64}, df[:dataset_percentage])))
#         color = "color" * string('A' + i - 1)

#         @printf(io, "\\addplot[%s, %s, thick, mark=none] coordinates{\n", color, DASH_TYPES[i])
#         @printf(io, "\t")

#         for p in percentages
#             logl = 0.0
#             nfound = 0
#             for j in 1 : nrow(df)
#                 if df[j, :dataset_percentage] == p &&
#                    df[j, :model_name] == name

#                    logl += df[j, :logl_test]
#                    nfound += 1
#                 end
#             end
#             logl /= nfound

#             @printf(io, "(%.4f,%.4f) ", p, logl)
#         end
#         println(io, "};")
#     end
# end
function create_tikzpicture_experiment{S<:AbstractString}(io::IO, dfs::Dict{AbstractString, DataFrame}, target::Symbol, model_names::Vector{S})
    #=
    This outputs, for each model by order of names:

    \addplot[colorE, dotted, thick, mark=none] coordinates{
        (0.0100,3472.2196) (0.0167,3179.6420) (0.0278,6060.3558) (0.0464,6231.8598) (0.0774,7445.1712) (0.1292,8600.2229) (0.2154,8947.1872) (0.3594,9629.5129) (0.5995,9900.6706) (1.0000,9782.7516) };
    =#

    for (i, model_name) in enumerate(model_names)

        df = dfs[model_name]

        percentages = sort(unique(convert(Vector{Float64}, df[:dataset_percentage])))
        color = "color" * string('A' + i - 1)

        @printf(io, "\\addplot[%s, %s, thick, mark=none] coordinates{\n", color, DASH_TYPES[i])
        @printf(io, "\t")

        for p in percentages
            value = 0.0
            nfound = 0
            for j in 1 : nrow(df)
                if df[j, :dataset_percentage] == p &&
                   df[j, :model_name] == model_name

                   value += df[j, target]
                   nfound += 1
                end
            end
            value /= nfound

            @printf(io, "(%.4f,%.4f) ", p, value)
        end
        println(io, "};")
    end
end

modelnames = ["Static Gaussian", "Linear Gaussian", "Random Forest", "Dynamic Forest", "Mixture Regression", "Bayesian Network", "Linear Bayesian"]

dfs = Dict{AbstractString, DataFrame}()
for model_name in modelnames

    model_output_name = replace(lowercase(model_name), " ", "_")
    inpath = joinpath(RESULTS_DIR, "data_vs_performance_metrics_" * model_output_name * ".csv")
    dfs[model_name] = readtable(inpath)
end

fh = STDOUT
write_to_texthook(TEXFILE, "varydata-experiment-logl") do fh
    create_tikzpicture_experiment(fh, dfs, :logl_test, modelnames)
end
write_to_texthook(TEXFILE, "varydata-experiment-rwse") do fh
    create_tikzpicture_experiment(fh, dfs, :rwse_dcl_test, modelnames)
end
write_to_texthook(TEXFILE, "varydata-experiment-smoothness") do fh
    create_tikzpicture_experiment(fh, dfs, :smooth_sumsquare, modelnames)
    _export_legend(fh, map(convert_model_name_to_short_name, modelnames))
end


println("DONE EXPORTING RESULTS")
