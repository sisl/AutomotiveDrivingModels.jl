export
    AbstractFeatureExtractor,
    FeatureExtractor,
    SubsetExtractor,
    StandardizingExtractor,


    pull_features!,
    rec_length

abstract AbstractFeatureExtractor
rec_length(::AbstractFeatureExtractor) = 1 # length of the SceneRecord for best results
Base.length(::AbstractFeatureExtractor) = error("Not Impemeneted")
pull_features!{F<:AbstractFloat}(::AbstractFeatureExtractor, features::Vector{F}, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0) = error("Not Implemented")

###############################################################
# FeatureExtractor

type FeatureExtractor <: AbstractFeatureExtractor
    features::Vector{AbstractFeature}
    rec_length::Int

    FeatureExtractor(features::Vector{AbstractFeature}, rec_length::Int=1) = new(features, rec_length)
end
rec_length(ext::FeatureExtractor) = ext.rec_length # length of the SceneRecord for best results
Base.length(ext::FeatureExtractor) = length(ext.features)
function pull_features!{F<:AbstractFloat}(ext::FeatureExtractor, features::Vector{F}, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0)

    # NOTE(tim): this is an interation over an abstract vector
    #            as such, this will be slow
    for (i,f) in enumerate(ext.features)
        features[i] = convert(Float64, get(f, rec, roadway, vehicle_index, pastframe))
    end
    features
end

###############################################################
# SubsetExtractor

type SubsetExtractor{F<:AbstractFeatureExtractor, G<:AbstractFloat} <: AbstractFeatureExtractor
    extractor::F
    subset::Vector{Int}
    features::Vector{G}
end
SubsetExtractor{G<:AbstractFloat}(extractor::AbstractFeatureExtractor, subset::Vector{Int}, ::Type{G}=Float64) = SubsetExtractor(extractor, subset, Array(G, length(extractor)))
rec_length(ext::SubsetExtractor) = rec_length(ext.extractor)
Base.length(ext::SubsetExtractor) = length(ext.subset)
function pull_features!{F<:AbstractFloat}(ext::SubsetExtractor, features::Vector{F}, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0)
    pull_features!(ext.extractor, ext.features, rec, roadway, vehicle_index, pastframe)
    for (i,j) in enumerate(ext.subset)
        features[i] = ext.features[j]
    end
    features
end

###############################################################
# StandardizingExtractor

type StandardizingExtractor{F<:AbstractFeatureExtractor} <: AbstractFeatureExtractor
    extractor::F
    μ::Vector{Float64}
    σ::Vector{Float64}
end
rec_length(ext::StandardizingExtractor) = rec_length(ext.extractor)
Base.length(ext::StandardizingExtractor) = length(ext.extractor)
function pull_features!{F<:AbstractFloat}(ext::StandardizingExtractor, features::Vector{F}, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0)
    pull_features!(ext.extractor, features, rec, roadway, vehicle_index, pastframe)
    for i in 1 : length(features)
        features[i] = (features[i] - ext.μ[i]) / ext.σ[i]
    end
    features
end
