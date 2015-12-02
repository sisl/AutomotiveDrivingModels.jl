using AutomotiveDrivingModels

using RandomForestBehaviors
using DynamicBayesianNetworkBehaviors

push!(LOAD_PATH, "/home/tim/Documents/wheelerworkspace/UtilityCode/")
using LaTeXeXport

const TEXFILE = "/home/tim/Documents/papers/2016_ITS_car_behaviors_wheeler/its_car_behaviors.tex"
const TEXDIR = splitdir(TEXFILE)[1]

const INCLUDE_FILE_BASE = "realworld"

const DASH_TYPES = ["solid", "dashdotted", "dashed", "densely dotted", "loosely dotted", "solid"]
const SAVE_FILE_MODIFIER = "_subset_car_following"
const EVALUATION_DIR = "/media/tim/DATAPART1/PublicationData/2015_TrafficEvolutionModels/" * INCLUDE_FILE_BASE* "/"
const METRICS_OUTPUT_FILE = joinpath(EVALUATION_DIR, "validation_results" * SAVE_FILE_MODIFIER * ".jld")
const MODEL_OUTPUT_JLD_FILE = joinpath(EVALUATION_DIR, "validation_models" * SAVE_FILE_MODIFIER * ".jld")

function _grab_metric{B<:BehaviorMetric}(T::DataType, metrics::Vector{B})
    j = findfirst(m->isa(m, T), metrics)
    metrics[j]
end
function _grab_metric{B<:BaggedMetricResult}(T::DataType, metrics::Vector{B})
    j = findfirst(res->res.M == T, metrics)
    metrics[j]
end
function _grab_score{B<:BehaviorMetric}(T::DataType, metrics::Vector{B})
    j = findfirst(m->isa(m, T), metrics)
    get_score(metrics[j])
end
function _grab_confidence(T::DataType, metrics::Vector{BaggedMetricResult})
    j = findfirst(m-> m.M <: T, metrics)
    confidence = metrics[j].confidence_bound
    @assert(!isnan(confidence))
    if isnan(confidence)
        confidence = 0.0
    end
    confidence
end
function _grab_score_and_confidence{B<:BehaviorMetric}(
    T::DataType,
    metrics::Vector{B},
    metrics_bagged::Vector{BaggedMetricResult},
    )

    μ = _grab_score(T, metrics)
    Δ = _grab_confidence(T, metrics_bagged)

    (μ, Δ)
end
function _convert_to_short_name(name::AbstractString)
    retval = ""
    for word in split(name)
        retval *= string(uppercase(word[1]))
    end
    retval
end

function create_tikzpicture_model_compare_logl{S<:AbstractString, B<:BehaviorFrameMetric}(io::IO,
    metrics_straight::Vector{Vector{B}},
    metrics_bagged::Vector{Vector{BaggedMetricResult}},
    names::Vector{S},
    )

    #=
    For each model, add these options:
    (produces 95% confidence bounds)

    \addplot[thick, mark=*, mark options={thick, colorA}, error bars/error bar style={colorA}, error bars/.cd,x dir=both,x explicit, error bar style={line width=1.5pt}, error mark options={rotate=90, mark size=4pt, line width=1pt}]
    coordinates{(1.000,Gaussian Filter)+=(0.664,0)-=(0.664,0)};
    =#

    for (i,name) in enumerate(names)

        μ, Δ = _grab_score_and_confidence(LoglikelihoodMetric, metrics_straight[i],
                                          metrics_bagged[i])

        color_letter = string('A' + i - 1)

        println(io, "\\addplot[thick, mark=*, mark options={thick, color", color_letter, "}, error bars/error bar style={color", color_letter, "}, error bars/.cd,x dir=both,x explicit, error bar style={line width=1.5pt}, error mark options={rotate=90, mark size=4pt, line width=1pt}]")
        @printf(io, "\tcoordinates{(%.4f,%s)+=(%.3f,0)-=(%.3f,0)};\n", μ, name, Δ, Δ)
    end
end
function create_tikzpicture_model_compare_kldiv_barplot{S<:AbstractString}(io::IO,
    metrics_sets_test_traces::Vector{Vector{BehaviorTraceMetric}},
    metrics_sets_test_traces_bagged::Vector{Vector{BaggedMetricResult}},
    names::Vector{S}
    )
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

    for i in 1 : length(names)

        speed_μ, speed_Δ = _grab_score_and_confidence(EmergentKLDivMetric{symbol(SPEED)}, metrics_sets_test_traces[i],
                                                            metrics_sets_test_traces_bagged[i])
        timegap_μ, timegap_Δ = _grab_score_and_confidence(EmergentKLDivMetric{symbol(TIMEGAP_X_FRONT)}, metrics_sets_test_traces[i],
                                                            metrics_sets_test_traces_bagged[i])
        # offset_μ, offset_Δ = _grab_score_and_confidence(EmergentKLDivMetric{symbol(D_CL)}, metrics_sets_test_traces[i],
        #                                                     metrics_sets_test_traces_bagged[i])

        color = "color" * string('A' + i - 1)
        print(io, "\\addplot [", color, ",fill=", color, "!60,error bars/.cd,y dir=both,y explicit]\n\t\tcoordinates{\n")
        @printf(io, "\t\t\(%-15s%.4f)+=(0,%.4f)-=(0,%.4f)\n", "speed,",      speed_μ,   speed_Δ, speed_Δ)
        @printf(io, "\t\t\(%-15s%.4f)+=(0,%.4f)-=(0,%.4f)\n", "timegap,",    timegap_μ, timegap_Δ, timegap_Δ)
        # @printf(io, "\t\t\(%-15s%.4f)+=(0,%.4f)-=(0,%.4f)\n", "laneoffset,", offset_μ,  offset_Δ, offset_Δ)
        @printf(io, "\t};\n")
    end

    print(io, "\\legend{")
    for (i,name) in enumerate(names)

        print(io, _convert_to_short_name(name))
        if i != length(names)
            print(io, ", ")
        end
    end
    print(io, "}\n")
end
function create_tikzpicture_model_compare_rwse_mean{S<:AbstractString}(io::IO,
    metrics_sets::Vector{Vector{BehaviorTraceMetric}},
    names::Vector{S},
    F::AbstractFeature,
    )

    #=
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

    for (i, metrics_set) in enumerate(metrics_sets[1:end])

        color = "color" * string('A' + i - 1)

        @printf(io, "\\addplot[%s, %s, thick, mark=none] coordinates{\n", color, DASH_TYPES[i])
        @printf(io, "\t(0,0.0) ")
        for horizon in [0.5,1.0,1.5,2.0,2.5,3.0,3.5,4.0]
            rwse = get_score(_grab_metric(RootWeightedSquareError{symbol(F), horizon}, metrics_sets[i]))
            @printf(io, "(%.3f,%.4f) ", horizon, rwse)
        end
        @printf(io, "};\n")
    end
end
function create_tikzpicture_model_compare_rwse_variance{S<:AbstractString}(io::IO, metrics_sets::Vector{Vector{BaggedMetricResult}}, names::Vector{S})

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



    for (i,metrics_set) in enumerate(metrics_sets)

        color = "color" * string('A' + i - 1)

        @printf(io, "\\addplot[%s, %s, thick, mark=none] coordinates{\n", color, DASH_TYPES[i])
        @printf(io, "\t(0,0.0) ")
        for horizon in [0.5,1.0,1.5,2.0,2.5,3.0,3.5,4.0]
            σ = _grab_metric(RootWeightedSquareError{symbol(SPEED), horizon}, metrics_sets[i]).σ
            @printf(io, "(%.3f,%.4f) ", horizon, σ)
        end
        @printf(io, "};\n")
    end
end
function create_tikzpicture_model_compare_rwse_legend{S<:AbstractString}(io::IO, names::Vector{S})

    # \legend{SG, LG, RF, DF, GM, BN}

    print(io, "\\legend{")
    for (i,name) in enumerate(names)
        print(io, _convert_to_short_name(name))
        if i < length(names)
            print(io, ", ")
        end
    end
    println(io, "}")
end

function create_table_validation_across_context_classes{S<:AbstractString, T<:AbstractString}(
    io::IO,
    context_classes::Vector{Dict},
    context_class_names::Vector{S},
    model_names::Vector{T})

    #=
    \begin{tabular}{llSSSSS}
    \toprule
                                   & Context     & {\SG}            & {\LG}            & {\RF}            & {\DF}            & {\BN}            \\
    \midrule
    log-likelihood (test)          & \freeflow   & 2.69+-0.04           & \bfseries 3.90+-0.03 & 2.73+-0.12           & 2.58+-0.07           & 3.89+-0.04           \\
                                   & \following  & 1.51+-0.10           & 2.99+-0.06           & 1.43+-0.30           & 1.77+-0.08           & \bfseries 3.02+-0.07 \\
                                   & \lanechange & -0.43+-0.20          & \bfseries 1.70+-0.19 & -2.28+-0.61          & 0.57+-0.18           & 1.19+-0.18           \\
    KL divergence (speed)          & \freeflow   & 0.25+-0.08           & \bfseries 0.13+-0.02 & 0.13+-0.05           & 0.17+-0.04           & 0.17+-0.08           \\
                                   & \following  & 0.34+-0.13           & 0.47+-0.11           & \bfseries 0.34+-0.12 & \bfseries 0.34+-0.12 & 0.47+-0.13           \\
                                   & \lanechange & 0.45+-0.18           & \bfseries 0.12+-0.09 & 0.45+-0.18           & 0.45+-0.18           & 0.45+-0.18           \\
    KL divergence (timegap)        & \freeflow   & 0.40+-0.08           & \bfseries 0.26+-0.08 & 0.31+-0.08           & 0.26+-0.09           & 0.40+-0.09           \\
                                   & \following  & \bfseries 0.39+-0.15 & 0.92+-0.21           & 0.57+-0.21           & 0.40+-0.21           & 1.32+-0.20           \\
                                   & \lanechange & 1.23+-0.39           & 1.23+-0.39           & \bfseries 0.36+-0.20 & 2.66+-0.32           & 1.23+-0.38           \\
    KL divergence (lane offset)    & \freeflow   & 0.63+-0.11           & 0.85+-0.17           & 0.93+-0.13           & 0.63+-0.15           & \bfseries 0.55+-0.17 \\
                                   & \following  & 0.18+-0.08           & 0.35+-0.06           & 0.15+-0.05           & 0.16+-0.08           & \bfseries 0.06+-0.04 \\
                                   & \lanechange & 2.54+-0.33           & 2.54+-0.40           & \bfseries 1.71+-0.35 & 1.71+-0.39           & 2.66+-0.37           \\
    RWSE (\num{4}\si{s}) [\si{m}]  & \freeflow   & 3.12+-0.60           & \bfseries 2.46+-0.37 & 2.98+-0.49           & 3.21+-0.58           & 3.31+-0.59           \\
                                   & \following  & 5.62+-1.44           & \bfseries 4.18+-1.01 & 5.19+-1.38           & 5.46+-1.37           & 5.69+-1.46           \\
                                   & \lanechange & 4.52+-1.16           & \bfseries 1.73+-0.34 & 4.19+-1.08           & 4.20+-0.98           & 4.48+-1.15           \\
    \bottomrule
    =#

    nmodels = length(model_names)

    print(io, "\\begin{tabular}{ll", "S"^nmodels, "}\n")
    print(io, "\\toprule\n")
    @printf(io, "%30s & %-11s ", "", "Context")
    for name in model_names
        @printf(io, "& {\\%-15s ", _convert_to_short_name(name) * "}")
    end
    print(io, "\\\\\n")
    print(io, "\\midrule\n")

    # Testing Log Likelihood
    for (context_class, context_class_name) in zip(context_classes, context_class_names)

        if context_class_name == context_class_names[1]
            @printf(io, "%-30s &", "log-likelihood (test)")
        else
            @printf(io, "%-30s &", "")
        end

        best_model_index = 0
        best_model_score = -Inf
        for i in 1 : nmodels
            logl_μ, logl_Δ = _grab_score_and_confidence(LoglikelihoodMetric, context_class["metrics_sets_test_frames"][i],
                                                        context_class["metrics_sets_test_frames_bagged"][i])
            if logl_μ ≥ best_model_score
                best_model_score, best_model_index = logl_μ, i
            end
        end

        @printf(io, " %-11s ", "\\"*context_class_name)
        for i in 1 : nmodels
            logl_μ, logl_Δ = _grab_score_and_confidence(LoglikelihoodMetric, context_class["metrics_sets_test_frames"][i],
                                                        context_class["metrics_sets_test_frames_bagged"][i])
            metric_string = @sprintf("%.2f+-%.2f", logl_μ, logl_Δ)
            if i == best_model_index
                metric_string = "\\bfseries " * metric_string
            end
            @printf(io, "& %-20s ", metric_string)
        end
        @printf(io, "\\\\\n")
    end

    # KL divergence, speed
    for (context_class, context_class_name) in zip(context_classes, context_class_names)

        if context_class_name == context_class_names[1]
            @printf(io, "%-30s &", "KL divergence (speed)")
        else
            @printf(io, "%-30s &", "")
        end

        best_model_index = 0
        best_model_score = Inf
        for i in 1 : nmodels
            μ, Δ = _grab_score_and_confidence(EmergentKLDivMetric{symbol(SPEED)}, context_class["metrics_sets_test_traces"][i],
                                                            context_class["metrics_sets_test_traces_bagged"][i])
            if μ ≤ best_model_score
                best_model_score, best_model_index = μ, i
            end
        end

        @printf(io, " %-11s ", "\\"*context_class_name)
        for i in 1 : nmodels
            μ, Δ = _grab_score_and_confidence(EmergentKLDivMetric{symbol(SPEED)}, context_class["metrics_sets_test_traces"][i],
                                                            context_class["metrics_sets_test_traces_bagged"][i])
            metric_string = @sprintf("%.2f+-%.2f", μ, Δ)
            if i == best_model_index
                metric_string = "\\bfseries " * metric_string
            end
            @printf(io, "& %-20s ", metric_string)
        end
        @printf(io, "\\\\\n")
    end

    # KL divergence, timegap
    for (context_class, context_class_name) in zip(context_classes, context_class_names)

        if context_class_name == context_class_names[1]
            @printf(io, "%-30s &", "KL divergence (timegap)")
        else
            @printf(io, "%-30s &", "")
        end

        best_model_index = 0
        best_model_score = Inf
        for i in 1 : nmodels
            μ, Δ = _grab_score_and_confidence(EmergentKLDivMetric{symbol(TIMEGAP_X_FRONT)}, context_class["metrics_sets_test_traces"][i],
                                                            context_class["metrics_sets_test_traces_bagged"][i])
            if μ ≤ best_model_score
                best_model_score, best_model_index = μ, i
            end
        end

        @printf(io, " %-11s ", "\\"*context_class_name)
        for i in 1 : nmodels
            μ, Δ = _grab_score_and_confidence(EmergentKLDivMetric{symbol(TIMEGAP_X_FRONT)}, context_class["metrics_sets_test_traces"][i],
                                                            context_class["metrics_sets_test_traces_bagged"][i])
            metric_string = @sprintf("%.2f+-%.2f", μ, Δ)
            if i == best_model_index
                metric_string = "\\bfseries " * metric_string
            end
            @printf(io, "& %-20s ", metric_string)
        end
        @printf(io, "\\\\\n")
    end

    # KL divergence, lane offset
    for (context_class, context_class_name) in zip(context_classes, context_class_names)

        if context_class_name == context_class_names[1]
            @printf(io, "%-30s &", "KL divergence (lane offset)")
        else
            @printf(io, "%-30s &", "")
        end

        best_model_index = 0
        best_model_score = Inf
        for i in 1 : nmodels
            μ, Δ = _grab_score_and_confidence(EmergentKLDivMetric{symbol(D_CL)}, context_class["metrics_sets_test_traces"][i],
                                                            context_class["metrics_sets_test_traces_bagged"][i])
            if μ ≤ best_model_score
                best_model_score, best_model_index = μ, i
            end
        end

        @printf(io, " %-11s ", "\\"*context_class_name)
        for i in 1 : nmodels
            μ, Δ = _grab_score_and_confidence(EmergentKLDivMetric{symbol(D_CL)}, context_class["metrics_sets_test_traces"][i],
                                                            context_class["metrics_sets_test_traces_bagged"][i])
            metric_string = @sprintf("%.2f+-%.2f", μ, Δ)
            if i == best_model_index
                metric_string = "\\bfseries " * metric_string
            end
            @printf(io, "& %-20s ", metric_string)
        end
        @printf(io, "\\\\\n")
    end

    # RWSE (4s)
    horizon = 4.0
    for (context_class, context_class_name) in zip(context_classes, context_class_names)

        if context_class_name == context_class_names[1]
            @printf(io, "%-30s &", "RWSE (\\num{4}\\si{s}) [\\si{m}]")
        else
            @printf(io, "%-30s &", "")
        end

        best_model_index = 0
        best_model_score = Inf
        for i in 1 : nmodels
            μ = get_score(_grab_metric(RootWeightedSquareError{symbol(SPEED), horizon}, context_class["metrics_sets_test_traces"][i]))
            if μ ≤ best_model_score
                best_model_score, best_model_index = μ, i
            end
        end

        @printf(io, " %-11s ", "\\"*context_class_name)
        for i in 1 : nmodels
            μ = get_score(_grab_metric(RootWeightedSquareError{symbol(SPEED), horizon}, context_class["metrics_sets_test_traces"][i]))
            Δ = _grab_metric(RootWeightedSquareError{symbol(SPEED), horizon}, context_class["metrics_sets_test_traces_bagged"][i]).σ
            metric_string = @sprintf("%.2f+-%.2f", μ, Δ)
            if i == best_model_index
                metric_string = "\\bfseries " * metric_string
            end
            @printf(io, "& %-20s ", metric_string)
        end
        @printf(io, "\\\\\n")
    end

    print(io, "\\bottomrule\n")
    print(io, "\\end{tabular}\n")
end

# behaviorset = JLD.load(MODEL_OUTPUT_JLD_FILE, "behaviorset")

# println(keys(JLD.load(METRICS_OUTPUT_FILE)))

preferred_name_order = ["Static Gaussian", "Linear Gaussian", "Random Forest", "Dynamic Forest", "Mixture Regression", "Bayesian Network"]

names = JLD.load(METRICS_OUTPUT_FILE, "model_names")
metrics_sets_test_frames = JLD.load(METRICS_OUTPUT_FILE, "metrics_sets_test_frames")
metrics_sets_test_frames_bagged = JLD.load(METRICS_OUTPUT_FILE, "metrics_sets_test_frames_bagged")
metrics_sets_train_frames = JLD.load(METRICS_OUTPUT_FILE, "metrics_sets_train_frames")
metrics_sets_train_frames_bagged = JLD.load(METRICS_OUTPUT_FILE, "metrics_sets_train_frames_bagged")
metrics_sets_test_traces = JLD.load(METRICS_OUTPUT_FILE, "metrics_sets_test_traces")
metrics_sets_test_traces_bagged = JLD.load(METRICS_OUTPUT_FILE, "metrics_sets_test_traces_bagged")
# metrics_sets_cv = JLD.load(METRICS_OUTPUT_FILE, "metrics_sets_cv")

perm = Array(Int, length(names))
let
    i = 0
    for (j,name) in enumerate(preferred_name_order)
        ind = findfirst(names, name)
        if ind != 0
            i += 1
            perm[ind] = i
        end
    end
    @assert(i == length(perm))
end

ipermute!(names, perm)
ipermute!(metrics_sets_test_frames, perm)
ipermute!(metrics_sets_test_frames_bagged, perm)
ipermute!(metrics_sets_train_frames, perm)
ipermute!(metrics_sets_train_frames_bagged, perm)
ipermute!(metrics_sets_test_traces, perm)
ipermute!(metrics_sets_test_traces_bagged, perm)

# all_names = String["Real World"]
# append!(all_names, behaviorset.names)

# fh = STDOUT
write_to_texthook(TEXFILE, "model-compare-logl-training") do fh
    create_tikzpicture_model_compare_logl(fh, metrics_sets_train_frames,
                                          metrics_sets_train_frames_bagged, names)
end

write_to_texthook(TEXFILE, "model-compare-logl-testing") do fh
    create_tikzpicture_model_compare_logl(fh, metrics_sets_test_frames,
                                          metrics_sets_test_frames_bagged, names)
end

# # write_to_texthook(TEXFILE, "model-compare-kldiv-testing") do fh
#     create_tikzpicture_model_compare_kldiv_barplot(fh, metrics_sets_test_traces,
#                                                    metrics_sets_test_traces_bagged, names)
# # end
# println("done")
# exit()

write_to_texthook(TEXFILE, "model-compare-rwse-mean-speed") do fh
    create_tikzpicture_model_compare_rwse_mean(fh, metrics_sets_test_traces, names, SPEED)
end
write_to_texthook(TEXFILE, "model-compare-rwse-mean-tau") do fh
    create_tikzpicture_model_compare_rwse_mean(fh, metrics_sets_test_traces, names, TIMEGAP_X_FRONT)
end
write_to_texthook(TEXFILE, "model-compare-rwse-mean-dcl") do fh
    create_tikzpicture_model_compare_rwse_mean(fh, metrics_sets_test_traces, names, D_CL)
end
write_to_texthook(TEXFILE, "model-compare-rwse-legend") do fh
    create_tikzpicture_model_compare_rwse_legend(fh, names)
end


println("done")
exit()

context_class_data = Dict[]
context_class_names = ["freeflow", "following", "lanechange"]
for dset_filepath_modifier in (
    "_subset_free_flow",
    "_subset_car_following",
    "_subset_lane_crossing",
    )

    jld_filepath = joinpath(EVALUATION_DIR, "validation_results" * dset_filepath_modifier * ".jld")
    push!(context_class_data, JLD.load(jld_filepath))
end

write_to_texthook(TEXFILE, "validation-across-context-classes") do fh
    create_table_validation_across_context_classes(fh, context_class_data, context_class_names, names)
end

println("DONE EXPORTING RESULTS TO TEX")