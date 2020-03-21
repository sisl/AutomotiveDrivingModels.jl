export
    Circ,
    AABB,
    AABB_center_length_width,
    OBB

struct Circ{V<:AbstractVec}
    c::V # center
    r::Float64 # radius
end
Circ(x::Real, y::Real, r::Real) = Circ{VecE2}(VecE2(x, y), r)
Circ(x::Real, y::Real, z::Real, r::Real) = Circ{VecE3}(VecE3(x, y, z), r)

Base.:+(circ::Circ{VecE2}, v::VecE2) = Circ{VecE2}(circ.c + v, circ.r)
Base.:-(circ::Circ{VecE2}, v::VecE2) = Circ{VecE2}(circ.c - v, circ.r)

Base.in(p::V, circ::Circ{V}) where {V} = normsquared(circ.c - p) ≤ circ.r*circ.r
inertial2body(circ::Circ{VecE2}, reference::VecSE2) = Circ{VecE2}(inertial2body(circ.c, reference), circ.r)

# Axis-Aligned Bounding Box
struct AABB
    center::VecE2
    len::Float64 # length along x axis
    wid::Float64 # width along y axis
end
function AABB(bot_left::VecE2, top_right::VecE2)
    center = (bot_left + top_right)/2
    Δ = top_right - bot_left
    return AABB(center, abs(Δ.x), abs(Δ.y))
end

Base.:+(box::AABB, v::VecE2) = AABB(box.center + v, box.len, box.wid)
Base.:-(box::AABB, v::VecE2) = AABB(box.center - v, box.len, box.wid)

function Base.in(P::VecE2, box::AABB)
    -box.len/2 ≤ P.x - box.center.x ≤ box.len/2 &&
    -box.wid/2 ≤ P.y - box.center.y ≤ box.wid/2
end


# Oriented Bounding Box
struct OBB
    aabb::AABB
    θ::Float64
end
function OBB(center::VecE2, len::Float64, wid::Float64, θ::Float64)
    del = VecE2(len/2, wid/2)
    return OBB(AABB(center, len, wid), θ)
end
OBB(center::VecSE2, len::Float64, wid::Float64) = OBB(convert(VecE2, center), len, wid, center.θ)

Base.:+(box::OBB, v::VecE2) = OBB(box.aabb+v, box.θ)
Base.:-(box::OBB, v::VecE2) = OBB(box.aabb-v, box.θ)

function Base.in(P::VecE2, box::OBB)
    C = box.aabb.center
    in(rot(P-C, -box.θ)+C, box.aabb)
end

function inertial2body(box::OBB, reference::VecSE2)
    c = VecSE2(box.aabb.center, box.θ)
    c′ = inertial2body(c, reference)
    OBB(c′, box.aabb.len, box.aabb.wid)
end
