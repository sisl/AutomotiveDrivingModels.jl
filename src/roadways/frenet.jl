"""
Frenet
______
roadind: road index
s: distance along lane
t: lane offset, positive is to left. zero point is the centerline of the lane.
ϕ: lane relative heading
"""
struct Frenet
    roadind::RoadIndex{Int64, Float64}
    s::Float64 # distance along lane
    t::Float64 # lane offset, positive is to left. zero point is the centerline of the lane.
    ϕ::Float64 # lane relative heading
end
function Frenet(roadind::RoadIndex, roadway::Roadway; t::Float64=0.0, ϕ::Float64=0.0)
    s = roadway[roadind].s
    ϕ = _mod2pi2(ϕ)
    Frenet(roadind, s, t, ϕ)
end
function Frenet(roadproj::RoadProjection, roadway::Roadway)
    roadind = RoadIndex(roadproj.curveproj.ind, roadproj.tag)
    s = roadway[roadind].s
    t = roadproj.curveproj.t
    ϕ = _mod2pi2(roadproj.curveproj.ϕ)
    Frenet(roadind, s, t, ϕ)
end
function Frenet(lane::Lane, s::Float64, t::Float64=0.0, ϕ::Float64=0.0)
    roadind = RoadIndex(get_curve_index(lane.curve, s), lane.tag)
    return Frenet(roadind, s, t, ϕ)
end
Frenet(posG::VecSE2, roadway::Roadway) = Frenet(proj(posG, roadway), roadway)
Frenet(posG::VecSE2, lane::Lane, roadway::Roadway) = Frenet(proj(posG, lane, roadway), roadway)

# helper
function _mod2pi2(x::Float64)
    val = mod2pi(x)
    if val > pi
        val -= 2pi
    end
    return val
end

"""
    get_posG(frenet::Frenet, roadway::Roadway)
projects the Frenet position into the global frame
"""
function get_posG(frenet::Frenet, roadway::Roadway)
    curvept = roadway[frenet.roadind]
    pos = curvept.pos + polar(frenet.t, curvept.pos.θ + π/2)
    VecSE2(pos.x, pos.y, frenet.ϕ + curvept.pos.θ)
end

const NULL_FRENET = Frenet(NULL_ROADINDEX, NaN, NaN, NaN)

Base.show(io::IO, frenet::Frenet) = print(io, "Frenet(", frenet.roadind, @sprintf(", %.3f, %.3f, %.3f)", frenet.s, frenet.t, frenet.ϕ))
function Base.isapprox(a::Frenet, b::Frenet;
    rtol::Real=cbrt(eps(Float64)),
    atol::Real=sqrt(eps(Float64))
    )

    a.roadind.tag == b.roadind.tag &&
    isapprox(a.roadind.ind.t, b.roadind.ind.t, atol=atol, rtol=rtol) &&
    isapprox(a.s, b.s, atol=atol, rtol=rtol) &&
    isapprox(a.t, b.t, atol=atol, rtol=rtol) &&
    isapprox(a.ϕ, b.ϕ, atol=atol, rtol=rtol)
end
