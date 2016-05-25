using AutomotiveDrivingModels

using RandomForestBehaviors
using DynamicBayesianNetworkBehaviors

push!(LOAD_PATH, "/home/tim/Documents/wheelerworkspace/UtilityCode/")
using LaTeXeXport

const TEXFILE = "/home/tim/Documents/papers/2016_its_car_behaviors_wheeler/its_car_behaviors.tex"
const TEXDIR = splitdir(TEXFILE)[1]

const INCLUDE_FILE_BASE = "realworld"

# const NAME_SUFFIX = " Clean"
const NAME_SUFFIX = ""

const DASH_TYPES = ["solid", "dashdotted", "dashed", "densely dotted", "loosely dotted", "densely dashdotted", "solid"]
const PATTERNS = ["horizontal lines", "vertical lines", "north east lines", "north west lines", "grid", "crosshatch", "dots", "crosshatch dots", "fivepointed stars", "sixpointed stars", "bricks"]
# const SAVE_FILE_MODIFIER = "_subset_car_following"
const SAVE_FILE_MODIFIER = "following" # "freeflow"
const EVALUATION_DIR = "/media/tim/DATAPART1/PublicationData/2015_TrafficEvolutionModels/" * INCLUDE_FILE_BASE* "/"
const METRICS_OUTPUT_FILE = joinpath(EVALUATION_DIR, "validation_results" * SAVE_FILE_MODIFIER * ".jld")
const MODEL_OUTPUT_JLD_FILE = joinpath(EVALUATION_DIR, "validation_models" * SAVE_FILE_MODIFIER * ".jld")

function load_model_metrics_data()

    model_metrics = Dict{AbstractString, DataFrame}()

    for dircontent in readdir(EVALUATION_DIR)
        if startswith(dircontent, "validation_results") && endswith(dircontent, ".csv")
            dset_filepath_modifier = match(r"(following|freeflow|lanechange)", dircontent).match
            model_output_name = match(r"((?<=following_)|(?<=freeflow_)|(?<=lanechange_))\S+(?=.csv)", dircontent).match
            model_name = join(map(str->ucfirst(str), split(model_output_name, "_")), " ")
            model_metrics[model_name * dset_filepath_modifier] = readtable(joinpath(EVALUATION_DIR, dircontent))
        end
    end

    model_metrics
end

# function _grab_metric{B<:BehaviorMetric}(T::DataType, metrics::Vector{B})
#     j = findfirst(m->isa(m, T), metrics)
#     metrics[j]
# end
# function _grab_metric{B<:BaggedMetricResult}(T::DataType, metrics::Vector{B})
#     j = findfirst(res->res.M == T, metrics)
#     metrics[j]
# end
# function _grab_score{B<:BehaviorMetric}(T::DataType, metrics::Vector{B})
#     j = findfirst(m->isa(m, T), metrics)
#     get_score(metrics[j])
# end
# function _grab_confidence(T::DataType, metrics::Vector{BaggedMetricResult})
#     j = findfirst(m-> m.M <: T, metrics)
#     confidence = metrics[j].confidence_bound
#     @assert(!isnan(confidence))
#     if isnan(confidence)
#         confidence = 0.0
#     end
#     confidence
# end
# function _grab_extrema(T::DataType, metrics::Vector{BaggedMetricResult})
#     j = findfirst(m-> m.M <: T, metrics)
#     lo = metrics[j].min
#     hi = metrics[j].max
#     (lo,hi)
# end
# function _grab_score_and_confidence{B<:BehaviorMetric}(
#     T::DataType,
#     metrics::Vector{B},
#     metrics_bagged::Vector{BaggedMetricResult},
#     )

#     μ = _grab_score(T, metrics)
#     Δ = _grab_confidence(T, metrics_bagged)

#     (μ, Δ)
# end
# function convert_model_name_to_short_name(name::AbstractString)
#     retval = ""
#     for word in split(name)
#         retval *= string(uppercase(word[1]))
#     end
#     retval
# end

_grab_score(df::DataFrame, sym::Symbol) = mean(df[sym])
function _grab_score_and_extrema(df::DataFrame, sym::Symbol)
    arr = df[sym]
    μ = mean(arr)
    lo, hi = extrema(arr)
    (μ, lo, hi)
end

_get_rwse_colsym(feature_sym::Symbol, H::Float64) = symbol(@sprintf("RWSE_%s_%s", string(feature_sym), replace(@sprintf("%.2f", H), ".", "_")))
_get_kldiv_colsym(feature_sym::Symbol) = symbol(@sprintf("kldiv_%s", string(feature_sym)))


function create_tikzpicture_model_compare_logl{S<:AbstractString}(
    io::IO,
    data::Dict{AbstractString, DataFrame},
    preferred_name_order::Vector{S},
    dset_filepath_modifier::AbstractString,
    test::Bool,
    )

    #=
    For each model, add these options:

    \addplot[thick, mark=*, mark options={thick, colorA}, error bars/error bar style={colorA}, error bars/.cd,x dir=both,x explicit, error bar style={line width=1.5pt}, error mark options={rotate=90, mark size=4pt, line width=1pt}]
    coordinates{(1.000,Gaussian Filter)+=(0.664,0)-=(0.624,0)};
    =#

    for (i,name) in enumerate(preferred_name_order)

        df = data[name*dset_filepath_modifier]
        μ, lo, hi = _grab_score_and_extrema(df, test ? :median_logl_test : :median_logl_train)

        color_letter = string('A' + i - 1)

        println(io, "\\addplot[thick, mark=*, mark options={thick, color", color_letter, "}, error bars/error bar style={color", color_letter, "}, error bars/.cd,x dir=both,x explicit, error bar style={line width=1.5pt}, error mark options={rotate=90, mark size=4pt, line width=1pt}]")
        @printf(io, "\tcoordinates{(%.4f,%s)+=(%.3f,0)-=(%.3f,0)};\n", μ, name, hi-μ, μ-lo)
    end
end
# function create_tikzpicture_model_compare_kldiv_barplot{S<:AbstractString}(io::IO,
#     metrics_sets_test_traces::Vector{Vector{BehaviorTraceMetric}},
#     metrics_sets_test_traces_bagged::Vector{Vector{BaggedMetricResult}},
#     names::Vector{S}
#     )
#     #=
#     The first model is used as the baseline

#     This outputs, for each model by order of names:

#     \addplot [colorA,fill=colorA!60,error bars/.cd,y dir=both,y explicit]
#             coordinates{
#             (speed,     0.005)+-(0,0.002)
#             (timegap,   0.005)+-(0,0.002)
#             (laneoffset,0.010)+-(0,0.002)
#         };
#     =#

#     for i in 1 : length(names)

#         speed_μ, speed_Δ = _grab_score_and_confidence(EmergentKLDivMetric{symbol(SPEED)}, metrics_sets_test_traces[i],
#                                                             metrics_sets_test_traces_bagged[i])
#         timegap_μ, timegap_Δ = _grab_score_and_confidence(EmergentKLDivMetric{symbol(TIMEGAP_X_FRONT)}, metrics_sets_test_traces[i],
#                                                             metrics_sets_test_traces_bagged[i])
#         # offset_μ, offset_Δ = _grab_score_and_confidence(EmergentKLDivMetric{symbol(D_CL)}, metrics_sets_test_traces[i],
#         #                                                     metrics_sets_test_traces_bagged[i])

#         color = "color" * string('A' + i - 1)
#         print(io, "\\addplot [", color, ",fill=", color, "!60,error bars/.cd,y dir=both,y explicit]\n\t\tcoordinates{\n")
#         @printf(io, "\t\t\(%-15s%.4f)+=(0,%.4f)-=(0,%.4f)\n", "speed,",      speed_μ,   speed_Δ, speed_Δ)
#         @printf(io, "\t\t\(%-15s%.4f)+=(0,%.4f)-=(0,%.4f)\n", "timegap,",    timegap_μ, timegap_Δ, timegap_Δ)
#         # @printf(io, "\t\t\(%-15s%.4f)+=(0,%.4f)-=(0,%.4f)\n", "laneoffset,", offset_μ,  offset_Δ, offset_Δ)
#         @printf(io, "\t};\n")
#     end

#     print(io, "\\legend{")
#     for (i,name) in enumerate(names)

#         print(io, convert_model_name_to_short_name(name))
#         if i != length(names)
#             print(io, ", ")
#         end
#     end
#     print(io, "}\n")
# end
function create_tikzpicture_model_compare_smoothness{S<:AbstractString}(
    io::IO,
    data::Dict{AbstractString, DataFrame},
    preferred_name_order::Vector{S},
    dset_filepath_modifier::AbstractString,
    )

    #=
    This outputs, for each model by order of names:

    \addplot [colorA,fill=colorA!60,error bars/.cd,y dir=both,y explicit]
        coordinates{
        (Sum Square,         0.3363)+=(0,0.1299)-=(0,0.1299)
        (Autocorrelation,    0.8469)+=(0,0.2164)-=(0,0.2164)
        (Sign Inversions,    0.2325)+=(0,0.1156)-=(0,0.1156)
      };

    ...

    \legend{SG, LG, RF, DF, MR, BN, LB}
    =#

    for (i,name) in enumerate(preferred_name_order)

        df = data[name*dset_filepath_modifier]
        SSJ_stuff = _grab_score_and_extrema(df, :sumsquarejerk)
        LOA_stuff = _grab_score_and_extrema(df, :lagoneautocor)
        JSI_stuff = _grab_score_and_extrema(df, :jerksigninvs)

        color = "color" * string('A' + i - 1)
        pattern = PATTERNS[mod(i,2)+3] # up left or down left

        print(io, "\\addplot [", color, ",fill=", color, "!60, postaction={pattern=", pattern, ", pattern color=white}, error bars/.cd,y dir=both,y explicit]\n\t\tcoordinates{\n")
        @printf(io, "\t\t(%-15s%.4f)+=(0,%.4f)-=(0,%.4f)\n", "Sum Square,",      SSJ_stuff[1], SSJ_stuff[2], min(SSJ_stuff[1], SSJ_stuff[2]))
        @printf(io, "\t\t(%-15s%.4f)+=(0,%.4f)-=(0,%.4f)\n", "Autocorrelation,", LOA_stuff[1], LOA_stuff[2], min(LOA_stuff[1], LOA_stuff[2]))
        @printf(io, "\t\t(%-15s%.4f)+=(0,%.4f)-=(0,%.4f)\n", "Sign Inversions,", JSI_stuff[1], JSI_stuff[2], min(JSI_stuff[1], JSI_stuff[2]))
        @printf(io, "\t};\n")
    end

    print(io, "\\legend{")
    for (i,name) in enumerate(preferred_name_order)

        print(io, convert_model_name_to_short_name(name))
        if i != length(preferred_name_order)
            print(io, ", ")
        end
    end
    print(io, "}\n")
end
function create_tikzpicture_model_compare_smoothness{T<:AbstractString, S<:AbstractString}(
    io::IO,
    data::Dict{AbstractString, DataFrame},
    context_class_names::Vector{T},
    preferred_name_order::Vector{S},
    colname::Symbol = _get_kldiv_colsym(:sumsquarejerk)
    )



            # for (i,name) in enumerate(preferred_name_order)



            #     if μ ≤ best_model_score
            #         best_model_score, best_model_index = μ, i
            #     end

    for (i,name) in enumerate(preferred_name_order)

        color = "color" * string('A' + i - 1)
        pattern = PATTERNS[mod(i,2)+3] # up left or down left

        print(io, "\\addplot [", color, ",fill=", color, "!60, postaction={pattern=", pattern, ", pattern color=white}, error bars/.cd,y dir=both,y explicit]\n\t\tcoordinates{\n")

        for context_class_name in context_class_names

            df = data[name*context_class_name]
            μ, lo, hi = _grab_score_and_extrema(df, colname)

            @printf(io, "\t\t(%-15s%.4f)+=(0,%.4f)-=(0,%.4f)\n", "\\"*context_class_name*",", μ, hi-μ, μ-lo)
        end

        @printf(io, "\t};\n")
    end

    print(io, "\\legend{")
    for (i,name) in enumerate(preferred_name_order)

        print(io, convert_model_name_to_short_name(name))
        if i != length(preferred_name_order)
            print(io, ", ")
        end
    end
    print(io, "}\n")
end
function create_tikzpicture_model_compare_rwse_mean{S<:AbstractString}(
    io::IO,
    data::Dict{AbstractString, DataFrame},
    preferred_name_order::Vector{S},
    dset_filepath_modifier::AbstractString,
    sym::Symbol,
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

    for (i,name) in enumerate(preferred_name_order)

        df = data[name*dset_filepath_modifier]

        color = "color" * string('A' + i - 1)

        @printf(io, "\\addplot[%s, %s, thick, mark=none] coordinates{\n", color, DASH_TYPES[i])
        print(io, "\t")
        for horizon in [0.0,0.5,1.0,1.5,2.0,2.5,3.0,3.5,4.0]
            rwse = horizon == 0.0 ? 0.0 : _grab_score(df, _get_rwse_colsym(sym, horizon))
            @printf(io, "(%.3f,%.4f) ", horizon, rwse)
        end
        @printf(io, "};\n")
    end
end
function create_tikzpicture_model_compare_rwse_legend{S<:AbstractString}(
    io::IO,
    data::Dict{AbstractString, DataFrame},
    preferred_name_order::Vector{S},
    )

    # \legend{SG, LG, RF, DF, GM, BN}

    print(io, "\\legend{")
    for (i,name) in enumerate(preferred_name_order)
        print(io, convert_model_name_to_short_name(name))
        if i < length(preferred_name_order)
            print(io, ", ")
        end
    end
    println(io, "}")
end

function create_table_validation_across_context_classes{T<:AbstractString, S<:AbstractString}(
    io::IO,
    data::Dict{AbstractString, DataFrame},
    context_class_names::Vector{T},
    preferred_name_order::Vector{S},
    )

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
    KL divergence (headway)        & \freeflow   & 0.40+-0.08           & \bfseries 0.26+-0.08 & 0.31+-0.08           & 0.26+-0.09           & 0.40+-0.09           \\
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

    print(io, "\\begin{tabular}{ll", "S"^length(preferred_name_order), "}\n")
    print(io, "\\toprule\n")
    @printf(io, "%30s & %-11s ", "", "Context")
    for name in preferred_name_order
        @printf(io, "& {\\%-15s ", convert_model_name_to_short_name(name) * "}")
    end
    print(io, "\\\\\n")
    print(io, "\\midrule\n")

    # Testing Log Likelihood
    for context_class_name in context_class_names

        if context_class_name == context_class_names[1]
            @printf(io, "%-30s &", "log-likelihood (test)")
        else
            @printf(io, "%-30s &", "")
        end

        best_model_index = 0
        best_model_score = -Inf
        for (i,name) in enumerate(preferred_name_order)

            df = data[name*context_class_name]
            logl = _grab_score(df, :median_logl_test)

            if logl ≥ best_model_score
                best_model_score, best_model_index = logl, i
            end
        end

        @printf(io, " %-11s ", "\\"*context_class_name)
        for (i,name) in enumerate(preferred_name_order)

            df = data[name*context_class_name]
            logl = _grab_score(df, :median_logl_test)

            metric_string = @sprintf("%.2f", logl)
            if i == best_model_index
                metric_string = "\\bfseries " * metric_string
            end
            @printf(io, "& %-20s ", metric_string)
        end
        @printf(io, "\\\\\n")
    end

    # RWSE
    horizon = 4.0
    for (sym, name, unit) in [(symbol(SPEED), "speed", "m/s"), (symbol(DIST_FRONT), "headway", "m"), (:posFt, "lane offset", "m")]

        counter = 0

        for context_class_name in context_class_names


            if sym == symbol(DIST_FRONT) && context_class_name != "following"
                continue
            end

            colname = _get_rwse_colsym(sym, horizon)

            counter += 1
            if counter == 1
                @printf(io, "%-30s &", "RWSE " * name * " [\\si{" * unit * "}]")
            else
                @printf(io, "%-30s &", "")
            end

            best_model_index = 0
            best_model_score = Inf
            for (i,name) in enumerate(preferred_name_order)

                df = data[name*context_class_name]
                μ = _grab_score(df, colname)

                if μ ≤ best_model_score
                    best_model_score, best_model_index = μ, i
                end
            end

            @printf(io, " %-11s ", "\\"*context_class_name)
            for (i,name) in enumerate(preferred_name_order)

                df = data[name*context_class_name]
                μ = _grab_score(df, colname)

                metric_string = @sprintf("%.2f", μ)
                if i == best_model_index
                    metric_string = "\\bfseries " * metric_string
                end
                @printf(io, "& %-20s ", metric_string)
            end
            @printf(io, "\\\\\n")
        end
    end

    # smoothness
    for (colname, name) in [(_get_kldiv_colsym(:sumsquarejerk), "sum square jerk")] # , (_get_kldiv_colsym(:lagoneautocor), "autocor"), (_get_kldiv_colsym(:jerksigninvs), "sign inv")

        counter = 0

        for context_class_name in context_class_names

            counter += 1
            if counter == 1
                @printf(io, "%-30s &", "KLdiv " * name)
            else
                @printf(io, "%-30s &", "")
            end

            best_model_index = 0
            best_model_score = Inf
            for (i,name) in enumerate(preferred_name_order)

                df = data[name*context_class_name]
                μ = _grab_score(df, colname)

                if μ ≤ best_model_score
                    best_model_score, best_model_index = μ, i
                end
            end

            @printf(io, " %-11s ", "\\"*context_class_name)
            for (i,name) in enumerate(preferred_name_order)

                df = data[name*context_class_name]
                μ = _grab_score(df, colname)

                metric_string = @sprintf("%.2f", μ)
                if i == best_model_index
                    metric_string = "\\bfseries " * metric_string
                end
                @printf(io, "& %-20s ", metric_string)
            end
            @printf(io, "\\\\\n")
        end
    end

    print(io, "\\bottomrule\n")
    print(io, "\\end{tabular}\n")
end
function create_table_feature_ranking{S<:AbstractString}(
    io::IO;
    nfeatures_to_output::Int = 30,
    folds::AbstractVector{Int} = 1:5,
    dset_filepath_modifier::AbstractString = "_following",
    model_names::AbstractVector{S} = ["Linear Gaussian", "Mixture Regression", "Bayesian Network", "Linear Bayesian"], # "Random Forest",
    )

    all_selection_counts = Dict{AbstractString, Dict{AbstractFeature, Int}}() # model_name -> selection_counts

    for model_name in model_names

        model_short_name = convert_model_name_to_short_name(model_name * NAME_SUFFIX)
        selection_counts = Dict{AbstractFeature, Int}()

        for fold in folds
            model_for_fold_path_jld = joinpath(EVALUATION_DIR, model_short_name * dset_filepath_modifier * "_fold" * @sprintf("%02d", fold) * ".jld")
            model = JLD.load(model_for_fold_path_jld, "model")
            for f in get_indicators(model)
                selection_counts[f] = get(selection_counts, f, 0) + 1
            end
        end

        all_selection_counts[model_name] = selection_counts

        # generate ranking
        chosen_features = collect(keys(selection_counts))
        feature_counts = map(f->selection_counts[f], chosen_features)
        p = sortperm(feature_counts, rev=true)

        # println(model_short_name, " top features:")
        # for i in 1:length(p)
        #     @printf("%d %3.1f  %s\n", i, feature_counts[p[i]]/length(folds), string(symbol(chosen_features[p[i]])))
        # end
        # println("")
    end

    # compute aggregate feature ranking
    tot_selection_counts = Dict{AbstractFeature, Int}()
    for selection_counts in values(all_selection_counts)
        for (f,c) in selection_counts
            tot_selection_counts[f] = get(tot_selection_counts, f, 0) + c
        end
    end

    chosen_features = collect(keys(tot_selection_counts))
    feature_counts = map(f->tot_selection_counts[f], chosen_features)
    p = sortperm(feature_counts, rev=true)

    # output the table
    print(io, "\\begin{tabular}{lS", "S"^length(model_names), "}\n")
    print(io, "\\toprule\n")
    print(io, "Feature & Across Models ")
    for model_name in model_names
        print(io, " & ", convert_model_name_to_short_name(model_name))
    end
    print(io, "\\\\\n")
    print(io, "\\midrule\n")
    for i in 1:min(nfeatures_to_output, length(p))
        if i ≤ length(p)
            f = chosen_features[p[i]]
            tot_importance = feature_counts[p[i]] / (length(folds)*length(model_names))

            print(io, lsymbol(f), " & ", @sprintf("%.2f", tot_importance))

            for model_name in model_names
                selection_counts = all_selection_counts[model_name]
                importance = get(selection_counts, f, 0) / length(folds)
                @printf(io, " & %.1f", importance)
            end
            print(io, "\\\\\n")
        end
    end

    print(io, "\\bottomrule\n")
    print(io, "\\end{tabular}\n")
end

fh = STDOUT

write_to_texthook(TEXFILE, "feature-ranking") do fh
    create_table_feature_ranking(fh)
end

println("EXPORTING FOR ", SAVE_FILE_MODIFIER)
preferred_name_order = ["Static Gaussian", "Linear Gaussian", "Random Forest", "Dynamic Forest", "Mixture Regression", "Bayesian Network", "Linear Bayesian"]

data = load_model_metrics_data()

println(keys(data))


write_to_texthook(TEXFILE, "model-compare-smoothness") do fh
    create_tikzpicture_model_compare_smoothness(fh, data, preferred_name_order, SAVE_FILE_MODIFIER)
end

write_to_texthook(TEXFILE, "model-compare-logl-training") do fh
    create_tikzpicture_model_compare_logl(fh, data, preferred_name_order, SAVE_FILE_MODIFIER, false)
end
write_to_texthook(TEXFILE, "model-compare-logl-testing") do fh
    create_tikzpicture_model_compare_logl(fh, data, preferred_name_order, SAVE_FILE_MODIFIER, false)
end

write_to_texthook(TEXFILE, "model-compare-rwse-mean-speed") do fh
    create_tikzpicture_model_compare_rwse_mean(fh, data, preferred_name_order, SAVE_FILE_MODIFIER, symbol(SPEED))
end
if SAVE_FILE_MODIFIER == "following"
    write_to_texthook(TEXFILE, "model-compare-rwse-mean-headway-distance") do fh
        create_tikzpicture_model_compare_rwse_mean(fh, data, preferred_name_order, SAVE_FILE_MODIFIER, symbol(DIST_FRONT))
    end
end
write_to_texthook(TEXFILE, "model-compare-rwse-mean-dcl") do fh
  create_tikzpicture_model_compare_rwse_mean(fh, data, preferred_name_order, SAVE_FILE_MODIFIER, :posFt)
end
write_to_texthook(TEXFILE, "model-compare-rwse-legend") do fh
    create_tikzpicture_model_compare_rwse_legend(fh, data, preferred_name_order)
end

context_class_names = ["freeflow", "following", "lanechange"]

write_to_texthook(TEXFILE, "validation-across-context-classes") do fh
    create_table_validation_across_context_classes(fh, data, context_class_names, preferred_name_order)
end

write_to_texthook(TEXFILE, "model-compare-smoothness") do fh
    create_tikzpicture_model_compare_smoothness(fh, data, context_class_names, preferred_name_order)
end

println("DONE EXPORTING RESULTS TO TEX")