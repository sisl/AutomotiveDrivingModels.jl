function Base.get{S<:State1D,D,I,R}(::Feature_PosFt, rec::Vector{EntityFrame{S,D,I}}, roadway::R, vehicle_index::Int, pastframe::Int=length(rec))
    FeatureValue(0.0) # no lateral offset
end
function Base.get{S<:State1D,D,I,R}(::Feature_PosFyaw, rec::Vector{EntityFrame{S,D,I}}, roadway::R, vehicle_index::Int, pastframe::Int=length(rec))
    FeatureValue(0.0) # no lane-relative heading
end
function Base.get{S<:State1D,D,I,R}(::Feature_Speed, rec::Vector{EntityFrame{S,D,I}}, roadway::R, vehicle_index::Int, pastframe::Int=length(rec))
    FeatureValue(rec[pastframe][vehicle_index].state.v)
end
function Base.get{S<:State1D,D,I,R}(::Feature_VelFs, rec::Vector{EntityFrame{S,D,I}}, roadway::R, vehicle_index::Int, pastframe::Int=length(rec))
    FeatureValue(rec[pastframe][vehicle_index].state.v)
end
function Base.get{S<:State1D,D,I,R}(::Feature_VelFt, rec::Vector{EntityFrame{S,D,I}}, roadway::R, vehicle_index::Int, pastframe::Int=length(rec))
    FeatureValue(0.0) # no lateral speed
end
