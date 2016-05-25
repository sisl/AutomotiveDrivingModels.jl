
const FLOAT_REGEX = r"[-+]?[0-9]*\.?[0-9]+"

modelnames = ["Static Gaussian", "Linear Gaussian", "Dynamic Forest", "Random Forest", "Mixture Regression", "Bayesian Network", "Linear Bayesian"]

function _convert_to_short_name(name::AbstractString)
    retval = ""
    for word in split(name)
        retval *= string(uppercase(word[1]))
    end
    retval
end

type Acc
    accuracy::Float64
    accuracy_ff::Float64
    accuracy_fo::Float64
    accuracy_lc::Float64
end

d = Dict{AbstractString, Acc}()

for model_name in modelnames

    input_filepath = "context_recognition_" * replace(lowercase(model_name), " ", "_") * ".txt"

    lines = open(readlines, input_filepath)

    acc = Acc(NaN,NaN,NaN,NaN)

    for line in lines
        if ismatch(r"Accuracy:", line)
            acc.accuracy = parse(Float64, match(FLOAT_REGEX, line).match)
        elseif ismatch(r"Accuracy freeflow", line)
            acc.accuracy_ff = parse(Float64, match(FLOAT_REGEX, line).match)
        elseif ismatch(r"Accuracy following", line)
            acc.accuracy_fo = parse(Float64, match(FLOAT_REGEX, line).match)
        elseif ismatch(r"Accuracy lanechange", line)
            acc.accuracy_lc = parse(Float64, match(FLOAT_REGEX, line).match)
        end
    end

    println(model_name)
    @printf("Accuracy Freeflow:   %4.1f\n", acc.accuracy_ff*100.0)
    @printf("Accuracy Following:  %4.1f\n", acc.accuracy_fo*100.0)
    @printf("Accuracy Lanechange: %4.1f\n", acc.accuracy_lc*100.0)
    @printf("Accuracy:            %4.1f\n\n", acc.accuracy*100.0)

    d[model_name] = acc
end

#=
    \begin{tabular}{lccccccc}
      \toprule
      Context Class & SG & LG & DF & RF & BN & LB\\
      \midrule
      \freeflow     & \num{ 0.0} & \num{ 0.0} & \num{ 0.3} & \num{ 0.3} & \num{13.3} & \num{ 0.6}\\
      \following    & \num{ 0.7} & \num{ 0.2} & \num{ 0.2} & \num{ 1.7} & \num{21.7} & \num{ 3.2}\\
      \lanechange   & \num{11.4} & \num{ 4.3} & \num{ 0.7} & \num{12.1} & \num{44.3} & \num{ 0.0}\\
      \midrule
      Accuracy      & \num{ 2.2} & \num{ 0.8} & \num{ 0.4} & \num{ 2.9} & \num{22.4} & \num{ 1.8}\\
      \bottomrule
    \end{tabular}
=#

println("\\begin{tabular}{l" * "c"^length(modelnames) * "}")
println("\t\\toprule")
print("\tContext Class")
for model_name in modelnames
    print(" & ", _convert_to_short_name(model_name))
end
println("\\\\")
println("\t\\midrule")
print("\t\\freeflow    ")
for model_name in modelnames
    @printf(" & \\num{%4.1f}", d[model_name].accuracy_ff*100.0)
end
println("\\\\")
print("\t\\following   ")
for model_name in modelnames
    @printf(" & \\num{%4.1f}", d[model_name].accuracy_fo*100.0)
end
println("\\\\")
print("\t\\lanechange  ")
for model_name in modelnames
    @printf(" & \\num{%4.1f}", d[model_name].accuracy_lc*100.0)
end
println("\\\\")
println("\t\\midrule")
print("\tAccuracy     ")
for model_name in modelnames
    @printf(" & \\num{%4.1f}", d[model_name].accuracy*100.0)
end
println("\\\\")
println("\t\\bottomrule")
println("\\end{tabular}")