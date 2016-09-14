
function _write_array{F<:AbstractFloat}(io::IO, arr::Array{F})
    for v in arr
        @printf(io, "%.16e ", v)
    end
end
function _write_array{I<:Integer}(io::IO, arr::Array{I})
    for v in arr
        @printf(io, "%d ", v)
    end
end

function Base.write(io::IO, gmr::GMR)
    println(io, "Gaussian Mixture Regressor:")
    println(io, "n_targets: ", n_targets(gmr))
    println(io, "n_features: ", n_features(gmr))
    println(io, "n_components: ", n_components(gmr))
    println(io, "vec_A:")
    for (i, arr) in enumerate(gmr.vec_A)
        @printf(io, "\t%d\t", i); _write_array(io, arr); print(io, "\n")
    end
    println(io, "vec_b:")
    for (i, arr) in enumerate(gmr.vec_b)
        @printf(io, "\t%d\t", i); _write_array(io, arr); print(io, "\n")
    end
    println(io, "mixture_Obs:")
    print(io, "\tweights: "); _write_array(io, probs(gmr.mixture_Obs)); print(io, "\n")
    for (i, component) in enumerate(components(gmr.mixture_Obs))
        @printf(io, "\t%d\n", i)
        print(io, "\t\tμ: "); _write_array(io, mean(component)); print(io, "\n")
        print(io, "\t\tΣ: "); _write_array(io, cov(component)); print(io, "\n")
    end
    println(io, "mixture_Act_given_Obs:")
    for (i, component) in enumerate(components(gmr.mixture_Act_given_Obs))
        @printf(io, "\t%d\n", i)
        print(io, "\t\tΣ: "); _write_array(io, cov(component)); print(io, "\n")
    end
end
function Base.read{S<:AbstractString}(io::IO, ::Type{GMR}, lines::Vector{S}=readlines(io), line_index::Int=1)

    function advance!()
        line = strip(lines[line_index])
        line_index += 1
        line
    end
    function advance_until!(str::AbstractString)
        while !contains(lines[line_index], str)
            line_index += 1
        end
        advance!()
    end

    n_targets = parse(Int, split(advance_until!("n_targets"))[2])
    n_features = parse(Int, split(advance_until!("n_features"))[2])
    n_components = parse(Int, split(advance_until!("n_components"))[2])

    vec_A = Array(Matrix{Float64}, n_components)
    vec_b = Array(Vector{Float64}, n_components)
    vec_G = Array(MvNormal, n_components)
    vec_H = Array(MvNormal, n_components)

    advance_until!("vec_A")
    for i in 1 : n_components
        line = advance!()
        tokens = split(line)
        @assert i == parse(Int, tokens[1])
        vec_A[i] = map!(j->parse(Float64, tokens[j+1]), Array(Float64, n_targets, n_features), 1:n_targets*n_features)
    end

    advance_until!("vec_b")
    for i in 1 : n_components
        line = advance!()
        tokens = split(line)
        @assert i == parse(Int, tokens[1])
        vec_b[i] = map!(j->parse(Float64, tokens[j+1]), Array(Float64, n_targets), 1:n_targets)
    end

    advance_until!("mixture_Obs")
    line = advance!()
    tokens = split(line)
    @assert contains(tokens[1], "weights")
    weights = map!(j->parse(Float64, tokens[j+1]), Array(Float64, n_components), 1:n_components)
    components = Array(MvNormal, n_components)
    for i in 1 : n_components
        line = advance!()
        @assert i == parse(Int, line)

        line = advance!()
        tokens = split(line)
        @assert contains(tokens[1], "μ")
        μ = map!(j->parse(Float64, tokens[j+1]), Array(Float64, n_features), 1:n_features)

        line = advance!()
        tokens = split(line)
        @assert contains(tokens[1], "Σ")
        Σ = map!(j->parse(Float64, tokens[j+1]), Array(Float64, n_features, n_features), 1:n_features*n_features)

        components[i] = MvNormal(μ, Σ)
    end
    mixture_Obs = MixtureModel(components, weights)

    advance_until!("mixture_Act_given_Obs")
    components = Array(MvNormal, n_components)
    for i in 1 : n_components
        line = advance!()
        @assert i == parse(Int, line)

        line = advance!()
        tokens = split(line)
        @assert contains(tokens[1], "Σ")
        Σ = map!(j->parse(Float64, tokens[j+1]), Array(Float64, n_targets, n_targets), 1:n_targets*n_targets)

        components[i] = MvNormal(Array(Float64, n_targets), Σ)
    end
    mixture_Act_given_Obs = MixtureModel(components)

    GMR(vec_A, vec_b, mixture_Obs, mixture_Act_given_Obs)
end

function Base.write{A,F}(io::IO, model::GaussianMixtureRegressionDriver{A,F})
    println(io, "Gaussian Mixture Regressor Driver")
    @printf(io, "rec: %d %.3f\n", record_length(model.rec), model.rec.timestep)
    @printf(io, "context: %.3f %d\n", model.context.Δt, model.context.n_integration_steps)

    gmr = model.gmr
    write(io, model.gmr)

    println(io, "action: ", A)
    print(io, "standardization_means: "); _write_array(io, model.extractor.μ); print(io, "\n")
    print(io, "standardization_stdevs: "); _write_array(io, model.extractor.σ); print(io, "\n")
    print(io, "chosen_indeces: "); _write_array(io, model.extractor.extractor.subset); print(io, "\n")
end
function Base.read(io::IO, ::Type{GaussianMixtureRegressionDriver}, extractor::AbstractFeatureExtractor)
    lines = readlines(io)
    line_index = 1

    function advance!()
        line = strip(lines[line_index])
        line_index += 1
        line
    end
    function advance_until!(str::AbstractString)
        while !contains(lines[line_index], str)
            line_index += 1
        end
        advance!()
    end

    line = advance_until!("rec")
    tokens = split(line)
    rec = SceneRecord(parse(Int, tokens[2]), parse(Float64, tokens[3]))

    line = advance_until!("context")
    tokens = split(line)
    context = IntegratedContinuous(parse(Float64, tokens[2]), parse(Int, tokens[3]))

    gmr = read(io, GMR, lines, line_index)

    line = advance_until!("action")
    if contains(line, "AccelTurnrate")
        A = AccelTurnrate
    elseif contains(line, "AccelDesang")
        A = AccelDesang
    elseif contains(line, "LatLonAccel")
        A = LatLonAccel
    else
        warn("unknown action $line")
        warn("defaulting to AccelTurnrate")
        A = AccelTurnrate
    end

    line = advance_until!("standardization_means")
    tokens = split(line)
    μ = map!(token->parse(Float64, token), Array(Float64, length(tokens)-1), tokens[2:end])

    line = advance_until!("standardization_stdevs")
    tokens = split(line)
    σ = map!(token->parse(Float64, token), Array(Float64, length(tokens)-1), tokens[2:end])

    line = advance_until!("chosen_indeces")
    tokens = split(line)
    chosen_indicators = map!(token->parse(Int, token), Array(Int, length(tokens)-1), tokens[2:end])

    subset_extractor = SubsetExtractor(extractor, chosen_indicators)
    standardizing_extractor = StandardizingExtractor(subset_extractor, μ, σ)

    GaussianMixtureRegressionDriver{A, typeof(standardizing_extractor)}(rec, context,
        gmr, standardizing_extractor,
        Array(Float64, length(standardizing_extractor)),
        Array(Float64, length(A)))
end