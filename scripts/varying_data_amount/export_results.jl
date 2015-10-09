using DataFrames

push!(LOAD_PATH, "/home/tim/Documents/wheelerworkspace/UtilityCode/")
using LaTeXeXport

const TEXFILE = "/home/tim/Documents/wheelerworkspace/Papers/2015_ITS_RiskEstimation/2015_IEEE_ITS_riskest.tex"
const TEXDIR = splitdir(TEXFILE)[1]
const RESULTS_DF_EXPERIMENT_1 = "/home/tim/.julia/v0.3/AutomotiveDrivingModels/scripts/varying_data_amount/results/data_vs_performance_metrics.csv"

function create_tikzpicture_model_logl_vs_param(io::IO, df::DataFrame, active_variable::Symbol)

    #=
    This outputs, for each model by order of names:

    \addplot[black, dotted, mark=none, error bars/.cd,y dir=both,y explicit] coordinates{
          (0,0.0)+=(0,0.0)-=(0,0.0) (1,0.0039)+=(0,0.0000)-=(0,0.0000) (2,0.0169)+=(0,0.0001)-=(0,0.0002) (3,0.0345)+=(0,0.0003)-=(0,0.0006) (4,0.0562)+=(0,0.0006)-=(0,0.0013)};

    And for the legend:

    \legend{Gaussian Filter, Random Forest, Dynamic Forest, Bayesian Network}
    =#

    nmodels = ncol(df)-2
    modelnames = map(i->replace(string(names(df)[i+2]), "_", " "), 1:nmodels)

    dash_types = ["solid", "dashdotted", "dashed", "densely dotted", "dotted"]

    for i in 1 : nmodels

        color = "color" * string('A' + i - 1)

        @printf(io, "\\addplot[%s, %s, thick, mark=none] coordinates{\n", color, dash_types[i])
        @printf(io, "\t")

        for j in 1 : nrow(df)
            active = df[j, active_variable]::Real
            logl = df[j,i+2]::Real
            @printf(io, "(%.4f,%.4f) ", active, logl)
        end
        println(io, "};")
    end

    print(io, "\\legend{")
    for (i,name) in enumerate(modelnames)

        for word in split(name)
            print(io, word[1])
        end
        if i != length(modelnames)
            print(io, ", ")
        end
    end
    print(io, "}\n")
end

df_exp1 = readtable(RESULTS_DF_EXPERIMENT_1)

write_to_texthook(TEXFILE, "varydata-experiment-1") do fh
    create_tikzpicture_model_logl_vs_param(fh, df_exp1, :dataset_amount)
end

println("DONE EXPORTING RESULTS")
