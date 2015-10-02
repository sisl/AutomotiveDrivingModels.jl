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

function create_tikzpicture_model_compare_logl{S<:String}(io::IO, metrics_sets::Vector{MetricsSet}, names::Vector{S})

    #=
    For each model, add these options

    \addplot+[thick, mark=*, mark options={colorA}, error bars/error bar style={colorA}, error bars/.cd,x dir=both,x explicit]
    coordinates{(1.000,Gaussian Filter)+=(0.664,0)-=(0.664,0)};
    \addplot+[thick, mark=*, mark options={colorB}, error bars/error bar style={colorB}, error bars/.cd,x dir=both,x explicit]
    coordinates{(1.400,Single Variable)+=(0.664,0)-=(0.164,0)};
    \addplot+[thick, mark=*, mark options={colorC}, error bars/error bar style={colorC}, error bars/.cd,x dir=both,x explicit]
    coordinates{(1.400,Random Forest)+=(0.664,0)-=(0.264,0)};
    \addplot+[thick, mark=*, mark options={colorD}, error bars/error bar style={colorD}, error bars/.cd,x dir=both,x explicit]
    coordinates{(1.400,Dynamic Forest)+=(0.664,0)-=(0.364,0)};
    \addplot+[thick, mark=*, mark options={colorE}, error bars/error bar style={colorE}, error bars/.cd,x dir=both,x explicit]
    coordinates{(1.400,Bayesian Network)+=(0.664,0)-=(0.664,0)};
    =#

    for (i,name) in enumerate(names)
        μ = metrics_sets[i].aggmetrics[:logl_mean]
        σ = metrics_sets[i].aggmetrics[:logl_stdev] # TODO(tim): extract correct value!

        color_letter = string('A' + i - 1)

        println(io, "\\addplot+[thick, mark=*, mark options={thick, color", color_letter, "}, error bars/error bar style={color", color_letter, "}, error bars/.cd,x dir=both,x explicit]")
        @printf(io, "\tcoordinates{(%.4f,%s)+=(%.3f,0)-=(%.3f,0)};\n", μ, name, σ, σ)
    end
end
function create_tikzpicture_model_compare_kldiv_barplot{S<:String}(io::IO, metrics_sets::Vector{MetricsSet}, names::Vector{S})
    #=
    The first model is used as the baseline

    This outputs, for each model by order of names:

    \addplot [colorA,fill=colorA!60,error bars/.cd,y dir=both,y explicit]
            coordinates{
            (speed,     0.005)+-(0,0.002)
            (timegap,   0.005)+-(0,0.002)
            (laneoffset,0.010)+-(0,0.002)
        };
    =#

    for i in 1 : length(metrics_sets)
        color = "color" * string('A' + i - 1)
        print(io, "\\addplot [", color, ",fill=", color, "!60,error bars/.cd,y dir=both,y explicit]\n\t\tcoordinates{\n")
        @printf(io, "\t\t\(%-15s%.4f)+-(%.4f)\n", "speed,",      metrics_sets[i].aggmetrics[:mean_speed_ego_kldiv],   0.02) # TODO(tim): fix this
        @printf(io, "\t\t\(%-15s%.4f)+-(%.4f)\n", "timegap,",    metrics_sets[i].aggmetrics[:mean_timegap_kldiv],     0.02) # TODO(tim): fix this
        @printf(io, "\t\t\(%-15s%.4f)+-(%.4f)\n", "laneoffset,", metrics_sets[i].aggmetrics[:mean_lane_offset_kldiv], 0.02) # TODO(tim): fix this
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
function create_tikzpicture_model_compare_rmse_mean{S<:String}(io::IO, metrics_sets::Vector{MetricsSet}, names::Vector{S})

    #=
    The first model is used as the baseline

    This outputs, for each model by order of names:

    \addplot[colorA, solid, thick, mark=none] coordinates{
      (0,0.0) (1,0.0042) (2,0.0180) (3,0.0376) (4,0.0610)};
    \addplot[colorB, dashdotted, thick, mark=none] coordinates{
      (0,0.0) (1,0.0032) (2,0.0133) (3,0.0257) (4,0.0396)};
    \addplot[colorC, dashed, thick, mark=none] coordinates{
      (0,0.0) (1,0.0024) (2,0.0107) (3,0.0235) (4,0.0408)};
    \addplot[colorD, densely dotted, thick, mark=none] coordinates{
      (0,0.0) (1,0.0011) (2,0.0047) (3,0.0105) (4,0.0180)};
    \addplot[colorE, dotted, thick, mark=none] coordinates{
      (0,0.0) (1,0.0015) (2,0.0057) (3,0.0125) (4,0.0380)};

    And for the legend:

    \legend{Gaussian Filter, Random Forest, Dynamic Forest, Bayesian Network}
    =#

    nmodels = length(names)

    dash_types = ["solid", "dashdotted", "dashed", "densely dotted", "dotted"]

    for (i,metrics_set) in enumerate(metrics_sets)

        color = "color" * string('A' + i - 1)

        @printf(io, "\\addplot[%s, %s, thick, mark=none] coordinates{\n", color, dash_types[i])
        @printf(io, "\t(0,0.0) (1,%.4f) (2,%.4f) (3,%.4f) (4,%.4f)};\n",
                metrics_set.aggmetrics[:rmse_1000ms_mean],
                metrics_set.aggmetrics[:rmse_2000ms_mean],
                metrics_set.aggmetrics[:rmse_3000ms_mean],
                metrics_set.aggmetrics[:rmse_4000ms_mean]
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
function create_tikzpicture_model_compare_rmse_stdev{S<:String}(io::IO, metrics_sets::Vector{MetricsSet}, names::Vector{S})

    #=
    The first model is used as the baseline

    This outputs, for each model by order of names:

    \addplot[colorA, solid, thick, mark=none] coordinates{
      (0,0.0) (1,0.0042) (2,0.0180) (3,0.0376) (4,0.0610)};
    \addplot[colorB, dashdotted, thick, mark=none] coordinates{
      (0,0.0) (1,0.0032) (2,0.0133) (3,0.0257) (4,0.0396)};
    \addplot[colorC, dashed, thick, mark=none] coordinates{
      (0,0.0) (1,0.0024) (2,0.0107) (3,0.0235) (4,0.0408)};
    \addplot[colorD, densely dotted, thick, mark=none] coordinates{
      (0,0.0) (1,0.0011) (2,0.0047) (3,0.0105) (4,0.0180)};
    \addplot[colorE, dotted, thick, mark=none] coordinates{
      (0,0.0) (1,0.0015) (2,0.0057) (3,0.0125) (4,0.0380)};

    And for the legend:

    \legend{Gaussian Filter, Random Forest, Dynamic Forest, Bayesian Network}
    =#

    nmodels = length(names)

    dash_types = ["solid", "dashdotted", "dashed", "densely dotted", "dotted"]

    for (i,metrics_set) in enumerate(metrics_sets)

        color = "color" * string('A' + i - 1)

        @printf(io, "\\addplot[%s, %s, thick, mark=none] coordinates{\n", color, dash_types[i])
        @printf(io, "\t(0,0.0) (1,%.4f) (2,%.4f) (3,%.4f) (4,%.4f)};\n",
                metrics_set.aggmetrics[:rmse_1000ms_stdev],
                metrics_set.aggmetrics[:rmse_2000ms_stdev],
                metrics_set.aggmetrics[:rmse_3000ms_stdev],
                metrics_set.aggmetrics[:rmse_4000ms_stdev]
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

println("TRAIN")
create_tikzpicture_model_compare_logl(STDOUT, metrics_sets_train[2:end], behaviorset.names)
println("\nTEST")
create_tikzpicture_model_compare_logl(STDOUT, metrics_sets_validation[2:end], behaviorset.names)

# @assert(behaviorset.names == ["Gaussian Filter", "Single Variable", "Random Forest", "Dynamic Forest", "Bayesian Network"])
# write_to_texthook(TEXFILE, "model-compare-logl-training") do fh
#     create_tikzpicture_model_compare_logl(fh, metrics_sets_train[2:end], behaviorset.names)
# end

# @assert(behaviorset.names == ["Gaussian Filter", "Single Variable", "Random Forest", "Dynamic Forest", "Bayesian Network"])
# write_to_texthook(TEXFILE, "model-compare-logl-testing") do fh
#     create_tikzpicture_model_compare_logl(fh, metrics_sets_test[2:end], behaviorset.names)
# end

create_tikzpicture_model_compare_kldiv_barplot(STDOUT, metrics_sets_validation[2:end], behaviorset.names)
create_tikzpicture_model_compare_rmse_mean(STDOUT, metrics_sets_validation[2:end], behaviorset.names)
create_tikzpicture_model_compare_rmse_stdev(STDOUT, metrics_sets_validation[2:end], behaviorset.names)

# write_to_texthook(TEXFILE, "model-compare-kldiv-test") do fh
#     short_names = ["GF", "SV", "RF", "DF", "BN"]
#     create_tikzpicture_model_compare_kldiv(fh, metrics_sets_validation[2:end], shorten(behaviorset.names))
# end
# write_to_texthook(TEXFILE, "model-compare-rmse-mean") do fh
#     short_names = ["GF", "SV", "RF", "DF", "BN"]
#     create_tikzpicture_model_compare_rmse(fh, metrics_sets_validation[2:end], behaviorset.names)
# end
# write_to_texthook(TEXFILE, "model-compare-rmse-stdev") do fh
#     short_names = ["GF", "SV", "RF", "DF", "BN"]
#     create_tikzpicture_model_compare_rmse(fh, metrics_sets_validation[2:end], behaviorset.names)
# end

