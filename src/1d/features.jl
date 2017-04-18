function Base.get{S<:State1D,D,I,R}(::Feature_PosFt, rec::EntityQueueRecord{S,D,I}, roadway::R, vehicle_index::Int, pastframe::Int=0)
    FeatureValue(0.0) # no lateral offset
end
function Base.get{S<:State1D,D,I,R}(::Feature_PosFyaw, rec::EntityQueueRecord{S,D,I}, roadway::R, vehicle_index::Int, pastframe::Int=0)
    FeatureValue(0.0) # no lane-relative heading
end
function Base.get{S<:State1D,D,I,R}(::Feature_Speed, rec::EntityQueueRecord{S,D,I}, roadway::R, vehicle_index::Int, pastframe::Int=0)
    FeatureValue(rec[pastframe][vehicle_index].state.v)
end
function Base.get{S<:State1D,D,I,R}(::Feature_VelFs, rec::EntityQueueRecord{S,D,I}, roadway::R, vehicle_index::Int, pastframe::Int=0)
    FeatureValue(rec[pastframe][vehicle_index].state.v)
end
function Base.get{S<:State1D,D,I,R}(::Feature_VelFt, rec::EntityQueueRecord{S,D,I}, roadway::R, vehicle_index::Int, pastframe::Int=0)
    FeatureValue(0.0) # no lateral speed
end