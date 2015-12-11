export
    FeatureSubsetExtractor,
    DataPreprocessor

##############################

type FeatureSubsetExtractor
    x::Vector{Float64} # output
    indicators::Vector{AbstractFeature}
end

function observe!(extractor::FeatureSubsetExtractor, runlog::RunLog, sn::StreetNetwork, colset::UInt, frame::Integer)
    x = extractor.x
    for (i,f) in indicators
        x[i] = get(f, runlog, sn, colset, frame)
    end
    x
end
function observe!(extractor::FeatureSubsetExtractor, dset::ModelTrainingData2, frame::Integer)
    x = extractor.x
    for (i,f) in indicators
        sym = symbol(f)
        x[i] = dset.dataframe[frame,sym]
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

    function ChainedDataProcessor(extractor::FeatureSubsetExtractor)
        x = extractor.x
        n_features = length(x)
        new(x, x, DataPreprocessor[])
    end
end

function process!(dp::DataNaReplacer)

    #=
    NOTE: this requires that dp.x already is filled with input
    =#

    x = dp.x

    for (i,f) in enumerate(dp.indicators)
        val = x[i]
        if isnan(val) || isinf(val)
            val = replace_na(f)::Float64
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

    for (processor) in enumerate(dp.processors)
        process!(processor)
    end

    nothing
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

function _update_X_in_place!(X::Matrix{Float64}, dp::DataPreprocessor)
    x = dp.x
    for i in 1 : size(X, 2)
        copy!(x, X[:,i])
        process!(dp)
        copy!(X[:,i], x)
    end
end

# X = [n_features, n_samples] NOT THE SAME AS A DATAFRAME!
function Base.push!(chain::ChainedDataProcessor, X::Matrix{Float64}, ::Type{DataNaReplacer}, indicators::Vector{AbstractFeature})
    # add a NaReplacer to the chain
    # X will be updated in-place with the post-processed dataset

    # NOTE - uses the exact same list, no deepcopying
    #      - input is same as output, so chain.z does not change

    dp = DataNaReplacer(chain.z, indicators)
    _update_X_in_place!(X, dp)
    push!(chain.processors, dp)
    chain
end
function Base.push!(chain::ChainedDataProcessor, X::Matrix{Float64}, ::Type{DataScaler})
    # add a scaler to the chain
    # This will train the scaler to standardize the data
    # X will be updated in-place with the post-processed dataset
    # - input is same as output, so chain.z does not change

    n_features = size(X, 1)
    μ = vec(mean(X, 2))
    σ = vec(std(X, 2))

    # z = (x - μ)/σ
    #   = x/σ - μ/σ
    #   = w*x + b

    w = σ.^-1
    b = -μ./σ

    dp = DataScaler(chain.z, w, b)
    _update_X_in_place!(X, dp)
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
    _update_X_in_place!(X, dp)
    push!(chain.processors, dp)
    chain
end
function Base.push!(chain::ChainedDataProcessor, X::Matrix{Float64}, ::Type{DataLinearTransform}, n_components::Int)
    # add a PCA transform to the chain
    # - has a new output, so reset the chain
    # - new data will be z = U*x

    n_features = size(X, 1)
    @assert(n_components ≤ n_features)

    z = Array(Float64, n_components)

    U, S, V = svd(X)

    A = U[1:n_components,:]
    b = zeros(Float64, n_components)

    push!(chain.processors, DataLinearTransform(chain.z, z, A, b))
    chain.z = z
    chain
end
function Base.push!(chain::ChainedDataProcessor, X::Matrix{Float64}, ::Type{DataSubset}, indeces::Vector{Int})
    # add a PCA transform to the chain
    # - has a new output, so reset the chain

    z = Array(Float64, length(indeces))
    push!(chain.processors, DataLinearTransform(chain.z, z, indeces))
    chain.z = z
    chain
end

#=
tree
NA_REPLACER -> SCALER(standardizer)

tree, gmm, LG
NA_REPLACER -> SCALER(standardizer) -> PCA
=#