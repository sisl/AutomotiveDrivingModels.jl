struct PosSpeed1D
    s::Float64 # position
    v::Float64 # speed [m/s]
end
Base.write(io::IO, ::MIME"text/plain", s::PosSpeed1D) = @printf(io, "%.16e %.16e", s.s, s.v)
function Base.read(io::IO, ::MIME"text/plain", ::Type{PosSpeed1D})
    i = 0
    tokens = split(strip(readline(io)), ' ')
    s = parse(Float64, tokens[i+=1])
    v = parse(Float64, tokens[i+=1])
    return PosSpeed1D(s,v)
end

function Base.get{S<:PosSpeed1D,D,I,R}(::Feature_PosFt, rec::EntityQueueRecord{S,D,I}, roadway::R, vehicle_index::Int, pastframe::Int=0)
    FeatureValue(0.0) # no lateral offset
end
function Base.get{S<:PosSpeed1D,D,I,R}(::Feature_PosFyaw, rec::EntityQueueRecord{S,D,I}, roadway::R, vehicle_index::Int, pastframe::Int=0)
    FeatureValue(0.0) # no lane-relative heading
end
function Base.get{S<:PosSpeed1D,D,I,R}(::Feature_Speed, rec::EntityQueueRecord{S,D,I}, roadway::R, vehicle_index::Int, pastframe::Int=0)
    FeatureValue(rec[pastframe][vehicle_index].state.v)
end
function Base.get{S<:PosSpeed1D,D,I,R}(::Feature_VelFs, rec::EntityQueueRecord{S,D,I}, roadway::R, vehicle_index::Int, pastframe::Int=0)
    FeatureValue(rec[pastframe][vehicle_index].state.v)
end
function Base.get{S<:PosSpeed1D,D,I,R}(::Feature_VelFt, rec::EntityQueueRecord{S,D,I}, roadway::R, vehicle_index::Int, pastframe::Int=0)
    FeatureValue(0.0) # no lateral speed
end