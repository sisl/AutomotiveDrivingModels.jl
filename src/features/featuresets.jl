"""
    FeatureSet
Defines a collection of features to be extracted
"""
abstract FeatureSet

type Features
    ftype::FeatureSet
    data::Matrix{Float64}
    scaling::Vector{Standardization}
    Features(ftype::FeatureSet, data::Matrix{Float64}) = new(ftype, data)
    Features(ftype::FeatureSet, data::Matrix{Float64}, scaling::Vector{Standardization}) = new(ftype, data, scaling)
end

function calc_scaling!(f::Features)
    f.scaling = calc_standardization(f.data)
    f
end