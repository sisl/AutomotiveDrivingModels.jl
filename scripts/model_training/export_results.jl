using AutomotiveDrivingModels

using RandomForestBehaviors
using DynamicBayesianNetworkBehaviors

push!(LOAD_PATH, "/home/tim/Documents/wheelerworkspace/UtilityCode/")
using LaTeXeXport

const TEXFILE = "/home/tim/Documents/wheelerworkspace/Papers/2015_ITS_RiskEstimation/2015_IEEE_ITS_riskest.tex"
const TEXDIR = splitdir(TEXFILE)[1]

const SAVE_FILE_MODIFIER = ""
const EVALUATION_DIR = "/media/tim/DATAPART1/PublicationData/2015_TrafficEvolutionModels/vires_highway_2lane_sixcar/"
const METRICS_OUTPUT_FILE = joinpath(EVALUATION_DIR, "validation_results" * SAVE_FILE_MODIFIER * ".jld")
const MODEL_OUTPUT_JLD_FILE = joinpath(EVALUATION_DIR, "validation_models" * SAVE_FILE_MODIFIER * ".jld")

function create_tikzpicture_model_compare_kldiv{S<:String}(io::IO, metrics_sets::Vector{MetricsSet}, names::Vector{S})
    #=
    The first model is used as the baseline

    This outputs, for each model by order of names:

    \addplot [black,fill=black!20,error bars/.cd,y dir=both,y explicit]
        coordinates{
          (loglikelihood,1.0000)+-(0,0.1774)
          (speed,1.0000)+-(0,0.1986)
          (timegap,1.0000)+-(0,0.1774)
          (laneoffset,1.0000)+-(0,0.1774)
        };
    =#

    nmodels = length(names)
    model_color_percentages = int(linspace(0,100,nmodels+2))[2:end-1]

    arr_logl = Array(Float64, nmodels, 2)
    arr_speed = Array(Float64, nmodels, 2)
    arr_timegap = Array(Float64, nmodels, 2)
    arr_laneoffset = Array(Float64, nmodels, 2)

    for (i,metrics_set) in enumerate(metrics_sets)
        arr_logl[i,1] = metrics_set.aggmetrics[:logl_mean]
        arr_logl[i,2] = metrics_set.aggmetrics[:logl_stdev]
        arr_speed[i,1] = metrics_set.aggmetrics[:mean_speed_ego_kldiv]
        arr_timegap[i,1] = metrics_set.aggmetrics[:mean_timegap_kldiv]
        arr_laneoffset[i,1] = metrics_set.aggmetrics[:mean_lane_offset_kldiv]
    end

    for i in 1 : nmodels
        color = "color" * string('A' + i - 1)
        print(io, "\\addplot [", color, ",fill=", color, "!60,error bars/.cd,y dir=both,y explicit]\n\t\tcoordinates{\n")
        @printf(io, "\t\t\(%s,%.4f)+-(0,%.4f)\n", "loglikelihood", arr_logl[i,1]/arr_logl[1,1], arr_logl[i,2]/arr_logl[1,1])
        @printf(io, "\t\t\(%s,%.4f)\n", "speed",      arr_speed[i,1])
        @printf(io, "\t\t\(%s,%.4f)\n", "timegap",    arr_timegap[i,1])
        @printf(io, "\t\t\(%s,%.4f)\n", "laneoffset", arr_laneoffset[i,1])
        @printf(io, "\t};\n")
    end

    print(io, "\\legend{")
    for (i,name) in enumerate(names)
        print(io, name)
        if i != length(names)
            print(io, ", ")
        end
    end
    print(io, "}\n")
end
function create_tikzpicture_model_compare_rmse{S<:String}(io::IO, metrics_sets::Vector{MetricsSet}, names::Vector{S})

    #=
    The first model is used as the baseline

    This outputs, for each model by order of names:

    \addplot[black, dotted, mark=none, error bars/.cd,y dir=both,y explicit] coordinates{
          (0,0.0)+=(0,0.0)-=(0,0.0) (1,0.0039)+=(0,0.0000)-=(0,0.0000) (2,0.0169)+=(0,0.0001)-=(0,0.0002) (3,0.0345)+=(0,0.0003)-=(0,0.0006) (4,0.0562)+=(0,0.0006)-=(0,0.0013)};

    And for the legend:

    \legend{Gaussian Filter, Random Forest, Dynamic Forest, Bayesian Network}
    =#

    nmodels = length(names)

    dash_types = ["solid", "dashdotted", "dashed", "dotted"]

    for (i,metrics_set) in enumerate(metrics_sets)

        color = "color" * string('A' + i - 1)

        @printf(io, "\\addplot[%s, %s, thick, mark=none, error bars/.cd,y dir=both,y explicit] coordinates{\n", color, dash_types[i])
        @printf(io, "\t(0,0.0)+=(0,0.0)-=(0,0.0) (1,%.4f)+=(0,%.4f)-=(0,%.4f) (2,%.4f)+=(0,%.4f)-=(0,%.4f) (3,%.4f)+=(0,%.4f)-=(0,%.4f) (4,%.4f)+=(0,%.4f)-=(0,%.4f)};\n",
                metrics_set.aggmetrics[:rmse_1000ms_mean], metrics_set.aggmetrics[:rmse_1000ms_stdev], metrics_set.aggmetrics[:rmse_1000ms_stdev],
                metrics_set.aggmetrics[:rmse_2000ms_mean], metrics_set.aggmetrics[:rmse_2000ms_stdev], metrics_set.aggmetrics[:rmse_2000ms_stdev],
                metrics_set.aggmetrics[:rmse_3000ms_mean], metrics_set.aggmetrics[:rmse_3000ms_stdev], metrics_set.aggmetrics[:rmse_3000ms_stdev],
                metrics_set.aggmetrics[:rmse_4000ms_mean], metrics_set.aggmetrics[:rmse_4000ms_stdev], metrics_set.aggmetrics[:rmse_4000ms_stdev]
            )
    end

    print(io, "\\legend{")
    for (i,name) in enumerate(names)
        print(io, name)
        if i != length(names)
            print(io, ", ")
        end
    end
    print(io, "}\n")
end

behaviorset = JLD.load(MODEL_OUTPUT_JLD_FILE, "behaviorset")

metrics_sets_train = JLD.load(METRICS_OUTPUT_FILE, "metrics_sets_train")
metrics_sets_validation = JLD.load(METRICS_OUTPUT_FILE, "metrics_sets_validation")

all_names = String["Real World"]
append!(all_names, behaviorset.names)

write_to_texthook(TEXFILE, "model-compare-kldiv-training") do fh
    create_tikzpicture_model_compare_kldiv(fh, metrics_sets_train[2:end], behaviorset.names)
end
write_to_texthook(TEXFILE, "model-compare-kldiv-test") do fh
    create_tikzpicture_model_compare_kldiv(fh, metrics_sets_validation[2:end], behaviorset.names)
end
write_to_texthook(TEXFILE, "model-compare-rmse") do fh
    create_tikzpicture_model_compare_rmse(fh, metrics_sets_validation[2:end], behaviorset.names)
end

