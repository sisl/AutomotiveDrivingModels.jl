immutable FrenetSpeed
    posF::Frenet
    v::Float64 # speed, zero sideslip
end

function Base.get{S<:FrenetSpeed,D,I,R}(::Feature_PosFt, rec::EntityQueueRecord{S,D,I}, roadway::R, vehicle_index::Int, pastframe::Int=0)
    return FeatureValue(rec[pastframe][vehicle_index].state.posF.t)
end
function Base.get{S<:FrenetSpeed,D,I,R}(::Feature_PosFyaw, rec::EntityQueueRecord{S,D,I}, roadway::R, vehicle_index::Int, pastframe::Int=0)
    return FeatureValue(rec[pastframe][vehicle_index].state.posF.ϕ)
end
function Base.get{S<:Frenet,D,I,R}(::Feature_Speed, rec::EntityQueueRecord{S,D,I}, roadway::R, vehicle_index::Int, pastframe::Int=0)
    return FeatureValue(rec[pastframe][vehicle_index].state.v)
end
function Base.get{S<:Frenet,D,I,R}(::Feature_VelFs, rec::EntityQueueRecord{S,D,I}, roadway::R, vehicle_index::Int, pastframe::Int=0)
    v = rec[pastframe][vehicle_index].state.v
    ϕ = rec[pastframe][vehicle_index].state.ϕ
    return FeatureValue(v*cos(ϕ))
end
function Base.get{S<:Frenet,D,I,R}(::Feature_VelFt, rec::EntityQueueRecord{S,D,I}, roadway::R, vehicle_index::Int, pastframe::Int=0)
    v = rec[pastframe][vehicle_index].state.v
    ϕ = rec[pastframe][vehicle_index].state.ϕ
    return FeatureValue(v*sin(ϕ))
end