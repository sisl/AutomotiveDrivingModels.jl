using DataFrames
using StreamStats

export
    FeatureSubsetExtractor,
    DatasetTransformProperties,
    DataPreprocessor,

    DataNaReplacer,
    DataScaler,
    DataClamper,
    DataLinearTransform,
    DataSubset,
    ChainedDataProcessor

##############################

type FeatureSubsetExtractor
    x::Vector{Float64} # output
    indicators::Vector{AbstractFeature}

    FeatureSubsetExtractor(x::Vector{Float64}, indicators::Vector{AbstractFeature}) = new(x, indicators)
    FeatureSubsetExtractor(indicators::Vector{AbstractFeature}) = new(Array(Float64, length(indicators)), indicators)
end

function observe!(extractor::FeatureSubsetExtractor, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)
    x = extractor.x
    for (i,f) in enumerate(extractor.indicators)
        x[i] = get(f, runlog, sn, colset, frame)
    end
    x
end
function observe!(extractor::FeatureSubsetExtractor, dataframe::DataFrame, frame::Integer)
    x = extractor.x
    for (i,f) in enumerate(extractor.indicators)
        sym = symbol(f)
        x[i] = dataframe[frame,sym]
    end
    x
end

##############################

abstract DataPreprocessor

type DataNaReplacer <: DataPreprocessor
    x::Vector{Float64}
    indicators::Vector{AbstractFeature}
end
type DataScaler <: DataPreprocessor
    x::Vector{Float64}
    weights::Vector{Float64}
    offset::Vector{Float64}
end
type DataClamper <: DataPreprocessor
    x::Vector{Float64}
    f_lo::Vector{Float64}
    f_hi::Vector{Float64}
end
type DataLinearTransform <: DataPreprocessor
    x::Vector{Float64} # input
    z::Vector{Float64} # output
    A::Matrix{Float64}
    b::Vector{Float64}
end
type DataSubset <: DataPreprocessor
    x::Vector{Float64} # input
    z::Vector{Float64} # output
    indeces::Vector{Int} # z[i] = x[indeces[i]]
end
type ChainedDataProcessor <: DataPreprocessor
    x::Vector{Float64} # input of first processor
    z::Vector{Float64} # output of last processor
    processors::Vector{DataPreprocessor}

    function ChainedDataProcessor(n::Integer)
        x = Array(Float64, n)
        n_features = length(x)
        new(x, x, DataPreprocessor[])
    end
    function ChainedDataProcessor(extractor::FeatureSubsetExtractor)
        x = extractor.x
        n_features = length(x)
        new(x, x, DataPreprocessor[])
    end
end

function _deepcopy(chain::ChainedDataProcessor, x::Vector{Float64})

    retval = ChainedDataProcessor(length(x))
    for dp in chain.processors
        if isa(dp, DataNaReplacer)
            push!(retval.processors, DataNaReplacer(x, deepcopy(dp.indicators)))
        elseif isa(dp, DataScaler)
            push!(retval.processors, DataScaler(x, deepcopy(dp.weights), deepcopy(dp.offset)))
        elseif isa(dp, DataClamper)
            push!(retval.processors, DataClamper(x, deepcopy(dp.f_lo), deepcopy(dp.f_hi)))
        elseif isa(dp, DataLinearTransform)
            z = deepcopy(dp.z)
            push!(retval.processors, DataLinearTransform(x, z, deepcopy(dp.A), deepcopy(dp.b)))
            x = z
        elseif isa(dp, DataSubset)
            z = deepcopy(dp.z)
            push!(retval.processors, DataSubset(x, z, deepcopy(dp.indeces)))
            x = z
        end
    end
    retval.z = x

    retval
end
function Base.deepcopy(chain::ChainedDataProcessor)

    #=
    Make a copy of the chain, given a newly created extractor_new.
    This will ensure that the io arrays are properly linked in memory and that the
    new one is not linked to the original.
    =#

    _deepcopy(chain, Array(Float64, length(chain.x)))
end
function Base.deepcopy(chain::ChainedDataProcessor, extractor_new::FeatureSubsetExtractor)

    #=
    Make a copy of the chain, given a newly created extractor_new.
    This will ensure that the io arrays are properly linked in memory and that the
    new one is not linked to the original.
    =#

    _deepcopy(chain, extractor_new.x)
end

function process!(dp::DataNaReplacer)

    #=
    NOTE: this requires that dp.x already is filled with input
    =#

    x = dp.x

    for (i,f) in enumerate(dp.indicators)
        val = x[i]
        if isnan(val) || isinf(val)
            val = FeaturesNew.replace_na(f)::Float64
        end
        x[i] = val
    end

    nothing
end
function process!(dp::DataScaler)

    #=
    NOTE: this requires that dp.x already is filled with input
    =#

    x = dp.x

    for (i,v) in enumerate(x)
        w = dp.weights[i]
        b = dp.offset[i]
        x[i] = w*v + b
    end

    nothing
end
function process!(dp::DataClamper)

    #=
    NOTE: this requires that dp.x already is filled with input
    =#

    x = dp.x

    for (i,v) in enumerate(x)
        lo = dp.f_lo[i]
        hi = dp.f_hi[i]
        x[i] = clamp(v, lo, hi)
    end

    nothing
end
function process!(dp::DataLinearTransform)

    #=
    NOTE: this requires that dp.x already is filled with input
    =#

    x = dp.x
    z = dp.z
    A = dp.A
    b = dp.b

    copy!(z, A*x + b)

    nothing
end
function process!(dp::DataSubset)

    #=
    NOTE: this requires that dp.x already is filled with input
    =#

    x = dp.x
    z = dp.z

    for (i,j) in enumerate(dp.indeces)
        z[i] = x[j]
    end

    nothing
end
function process!(dp::ChainedDataProcessor)

    #=
    NOTE: this requires that dp.x already is filled with input
    =#

    for processor in dp.processors
        process!(processor)
    end

    nothing
end

function process!(X::Matrix{Float64}, dp::DataPreprocessor)

    # Processes the data matrix X in place

    x = dp.x
    @assert(!isdefined(dp, :z))

    for i in 1 : size(X, 2)

        for j in 1 : size(X, 1)
            x[j] = X[j,i]
        end

        process!(dp)

        for j in 1 : size(X, 1)
            X[j,i] = x[j]
        end
    end
    X
end
function process!(Z::Matrix{Float64}, X::Matrix{Float64}, dp::DataPreprocessor)
    x = dp.x
    z = dp.z

    # Processes X → Z

    @assert(size(x,2) == size(z,2)) # same number of samples

    for i in 1 : size(X, 2)

        for j in 1 : size(X, 1)
            x[j] = X[j,i]
        end

        process!(dp)

        for j in 1 : size(Z, 1)
            Z[j,i] = z[j]
        end
    end
    Z
end

function Base.reverse(dp::DataScaler)

    #=
    NOTE: this requires that dp.x already is filled with input
    =#

    x = dp.x

    for (i,v) in enumerate(x)
        w = dp.weights[i]
        b = dp.offset[i]
        x[i] = (v - b)/w
    end

    nothing
end

# X = [n_features, n_samples] NOT THE SAME AS A DATAFRAME!
function Base.push!(chain::ChainedDataProcessor, X::Matrix{Float64}, ::Type{DataNaReplacer}, indicators::Vector{AbstractFeature})
    # add a NaReplacer to the chain

    # NOTE - uses the exact same list, no deepcopying
    #      - input is same as output, so chain.z does not change

    dp = DataNaReplacer(chain.z, indicators)
    push!(chain.processors, dp)
    chain
end
function Base.push!(chain::ChainedDataProcessor, X::Matrix{Float64}, ::Type{DataScaler})
    # add a scaler to the chain
    # This will train the scaler to standardize the data
    # - input is same as output, so chain.z does not change

    n_features = size(X, 1)
    n_frames = size(X, 2)

    μ = Array(Float64, n_features)
    σ = Array(Float64, n_features)

    for i in 1 : n_features

        streamstats = StreamStats.Variance()

        for j in 1 : n_frames
            v = X[i,j]
            if !isinf(v)
                update!(streamstats, v)
            end
        end

        μ[i] = mean(streamstats)
        σ[i] = sqrt(streamstats.v_hat)
    end

    # z = (x - μ)/σ
    #   = x/σ - μ/σ
    #   = w*x + b

    w = σ.^-1
    b = -μ./σ

    dp = DataScaler(chain.z, w, b)
    push!(chain.processors, dp)
    chain
end
function Base.push!(chain::ChainedDataProcessor, X::Matrix{Float64}, ::Type{DataClamper})
    # add a clamper to the chain
    # This will train the clamper to clamp to the currently observed max and min
    # - input is same as output, so chain.z does not change

    n_features = size(X, 1)

    lo = Array(Float64, n_features)
    hi = Array(Float64, n_features)

    for i in 1 : n_features
        lo[i] = minimum(X[i,:])
        hi[i] = maximum(X[i,:])
    end

    dp = DataClamper(chain.z, lo, hi)
    push!(chain.processors, dp)
    chain
end
function Base.push!(chain::ChainedDataProcessor, X::Matrix{Float64}, ::Type{DataClamper}, max_n_stdevs_from_mean::Real)
    # add a clamper to the chain
    # This will train the clamper to clamp to the currently observed max and min
    # - if max_n_stdevs_from_mean is set will use that instead
    # - input is same as output, so chain.z does not change

    n_features = size(X, 1)
    @assert(!isnan(max_n_stdevs_from_mean))

    lo = Array(Float64, n_features)
    hi = Array(Float64, n_features)

    for i in 1 : n_features

        arr = X[i,:]
        low, high = extrema(arr)
        μ = mean(arr)
        σ = stdm(arr, μ)

        lo[i] = max(low, μ - max_n_stdevs_from_mean*σ)
        hi[i] = min(high, μ + max_n_stdevs_from_mean*σ)
    end

    dp = DataClamper(chain.z, lo, hi)
    push!(chain.processors, dp)
    chain
end
function Base.push!(chain::ChainedDataProcessor, X::Matrix{Float64}, ::Type{DataLinearTransform}, n_components::Int)
    # add a PCA transform to the chain
    # - has a new output, so reset the chain
    # - X should be de-meaned before this
    # - new data will be z = U*x

    n_features = size(X, 1)
    @assert(n_components ≤ n_features)

    @assert(all(X != NaN))
    @assert(all(X != Inf))

    U, S, V = svd(X)

    A = U[1:n_components,:]
    b = zeros(Float64, n_components)


    z = Array(Float64, n_components)
    push!(chain.processors, DataLinearTransform(chain.z, z, A, b))
    chain.z = z
    chain
end
function Base.push!(chain::ChainedDataProcessor, ::Type{DataSubset}, indeces::Vector{Int})
    # add a PCA transform to the chain
    # - has a new output, so reset the chain

    z = Array(Float64, length(indeces))
    push!(chain.processors, DataSubset(chain.z, z, indeces))
    chain.z = z
    chain
end

#=
tree
NA_REPLACER -> SCALER(standardizer)

tree, gmm, LG
NA_REPLACER -> SCALER(standardizer) -> PCA
=#
