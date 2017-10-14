struct RoadwayState
    posG::VecSE2 # global
    posF::Frenet # lane-relative frame
    tag::LaneTag # identifies which curve on roadway
    v::Float64   # speed, zero-sideslip
end

Base.show(io::IO, s::RoadwayState) = print(io, "RoadwayState(", s.posG, ", ", s.posF, ", ",  s.tag, ", ", @sprintf("%.3f", s.v), ")")
function Base.write(io::IO, ::MIME"text/plain", s::RoadwayState)
    @printf(io, "%.16e %.16e %.16e", s.posG.x, s.posG.y, s.posG.θ)
    @printf(io, " %.16e %.16e %.16e", s.posF.s, s.posF.t, s.posF.ϕ)
    @printf(io, " %d %d", s.tag.segment, s.tag.lane)
    @printf(io, " %.16e", s.v)
end
function Base.read(io::IO, ::MIME"text/plain", ::Type{RoadwayState})
    tokens = split(strip(readline(io)), ' ')
    i = 0
    posG = VecSE2(parse(Float64, tokens[i+=1]), parse(Float64, tokens[i+=1]), parse(Float64, tokens[i+=1]))
    posF = Frenet(parse(Float64, tokens[i+=1]), parse(Float64, tokens[i+=1]), parse(Float64, tokens[i+=1]))
    tag = LaneTag(parse(Int, tokens[i+=1]), parse(Int, tokens[i+=1]))
    v = parse(Float64, tokens[i+=1])
    return RoadwayState(posG, posF, tag, v)
end

# are these needed?
get_vel_s(s::RoadwayState) = s.v * cos(s.posF.ϕ) # velocity along the lane
get_vel_t(s::RoadwayState) = s.v * sin(s.posF.ϕ) # velocity perpendicular to lane

# TODO: move these to the correct typewise folder
RoadwayState(posG::VecSE2, roadway::Roadway, v::Float64) = RoadwayState(posG, Frenet(posG, roadway), v)
RoadwayState(posG::VecSE2, lane::Lane, roadway::Roadway, v::Float64) = RoadwayState(posG, Frenet(posG, roadway), v)
RoadwayState(posF::Frenet, roadway::Roadway, v::Float64) = RoadwayState(get_posG(posF, roadway), posF, v)

function _mod2pi2(x::Float64)
    val = mod2pi(x)
    if val > pi
        val -= 2pi
    end
    return val
end
function RoadwayState(roadind::RoadIndex, roadway::Roadway; t::Float64=0.0, ϕ::Float64=0.0)
    s = roadway[roadind].s
    ϕ = _mod2pi2(ϕ)
    Frenet(roadind, s, t, ϕ)
end
function RoadwayState(roadproj::RoadProjection, roadway::Roadway)
    roadind = RoadIndex(roadproj.curveproj.ind, roadproj.tag)
    s = roadway[roadind].s
    t = roadproj.curveproj.t
    ϕ = _mod2pi2(roadproj.curveproj.ϕ)
    Frenet(roadind, s, t, ϕ)
end
Frenet(posG::VecSE2, roadway::Roadway) = Frenet(proj(posG, roadway), roadway)

# function Vec.lerp(a::RoadwayState, b::RoadwayState, t::Float64, roadway::Roadway)
#     posG = lerp(a.posG, b.posG, t)
#     v = lerp(a.v, b.v, t)
#     RoadwayState(posG, roadway, v)
# end

# function move_along(vehstate::RoadwayState, roadway::Roadway, Δs::Float64;
#     ϕ₂::Float64=vehstate.posF.ϕ, t₂::Float64=vehstate.posF.t, v₂::Float64=vehstate.v
#     )

#     roadind = move_along(vehstate.posF.roadind, roadway, Δs)
#     try
#         footpoint = roadway[roadind]
#     catch
#         println(roadind)
#     end
#     footpoint = roadway[roadind]
#     posG = convert(VecE2, footpoint.pos) + polar(t₂, footpoint.pos.θ + π/2)
#     posG = VecSE2(posG.x, posG.y, footpoint.pos.θ + ϕ₂)
#     RoadwayState(posG, roadway, v₂)
# end