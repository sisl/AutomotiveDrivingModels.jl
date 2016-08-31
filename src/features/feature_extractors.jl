export
    AbstractFeatureExtractor,
    FeatureExtractor,

    pull_features!

abstract AbstractFeatureExtractor
Base.length(::AbstractFeatureExtractor) = error("Not Impemeneted")
pull_features!{F<:AbstractFloat}(::AbstractFeatureExtractor, features::Vector{F}, rec::SceneRecord, roadway::Roadway, vehicle_index::Int) = error("Not Implemented")

type FeatureExtractor <: AbstractFeatureExtractor
    features::Vector{AbstractFeature}
end
Base.length(ext::FeatureExtractor) = length(ext.features)
function pull_features!{F<:AbstractFloat}(ext::FeatureExtractor, features::Vector{F}, rec::SceneRecord, roadway::Roadway, vehicle_index::Int)

    # NOTE(tim): this is an interation over an abstract vector
    #            as such, this will be slow
    for (i,f) in enumerate(ext.features)
        features[i] = convert(Float64, get(f, rec, roadway, vehicle_index))
    end
    features
end