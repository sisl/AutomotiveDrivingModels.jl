struct TaggedState{S}
    lanestate::S # state relative to a lane
    tag::LaneTag
end

function Base.show{S}(io::IO, state::TaggedState{S})
    print(io, "TaggedState(")
    show(io, state.lanestate)
    print(io, @sprintf(" at (%d,%d))", state.tag.segment, state.tag.lane))
end
function Base.isapprox{S}(a::TaggedState{S}, b::TaggedState{S};
    rtol::Real=cbrt(eps(Float64)),
    atol::Real=sqrt(eps(Float64))
    )

    return a.tag == b.tag &&
           isapprox(a.lanestate, b.lanestate, atol=atol, rtol=rtol)
end

function Base.write{S}(io::IO, mime::MIME"text/plain", state::TaggedState{S})
    write(io, mime, state.lanestate)
    @printf(io, " %d %d", state.tag.segment, state.tag.lane)
end
# function Base.read{S}(io::IO, ::MIME"text/plain", ::Type{TaggedState{S}})
#     # NOT SURE HOW TO DO THIS...
#     tokens = split(strip(readline(io)), ' ')
#     s = parse(Float64, tokens[1])
#     t = parse(Float64, tokens[2])
#     ϕ = parse(Float64, tokens[3])
#     frenet = Frenet(s,t,ϕ)
#     segment = parse(Int, tokens[4])
#     lane = parse(Int, tokens[5])
#     tag = LaneTag(segment, lane)
#     return TaggedFrenet(frenet, tag)
# end

# function Base.get{S<:TaggedState,D,I,R}(::Feature_PosFt, rec::EntityQueueRecord{S,D,I}, roadway::R, vehicle_index::Int, pastframe::Int=0)
#     FeatureValue(rec[pastframe][vehicle_index].state.frenet.t)
# end
# function Base.get{S<:TaggedState,D,I,R}(::Feature_PosFyaw, rec::EntityQueueRecord{S,D,I}, roadway::R, vehicle_index::Int, pastframe::Int=0)
#     return FeatureValue(rec[pastframe][vehicle_index].state.frenet.ϕ)
# end