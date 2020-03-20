export
    Ray,
    intersects      # true if A and B intersect


const Ray = VecSE2 # has an origin and an orientation
function intersects(A::Ray, B::Ray)
    as = convert(VecE2, A)
    bs = convert(VecE2, B)
    ad = polar(1.0, A.θ)
    bd = polar(1.0, B.θ)

    dx = bs.x - as.x
    dy = bs.y - as.y
    det = bd.x * ad.y - bd.y * ad.x

    if det == 0.0
        return false
    end

    u = (dy * bd.x - dx * bd.y) / det
    v = (dy * ad.x - dx * ad.y) / det
    return u > 0.0 && v > 0.0
end
function intersects(ray::Ray, line::Line;
    ε::Float64 = 1e-10,
    )

    v₁ = VecE2(ray) - line.C
    v₂ = polar(1.0, line.θ)
    v₃ = polar(1.0, ray.θ + π/2)

    denom = v₂⋅v₃

    if !(denom ≈ 0.0)
        t₁ = (v₂×v₁) / denom # time for ray (0 ≤ t₁)
        return -ε ≤ t₁
    else
        # denom is zero if the line and the ray are parallel
        # only collide if they are perfectly aligned
        return isapprox(angledist(ray, line), 0.0, atol=ε)
    end
end
function intersects(ray::Ray, seg::LineSegment)
    R = VecE2(ray)
    v₁ = R - seg.A
    v₂ = seg.B - seg.A
    v₃ = polar(1.0, ray.θ + π/2)

    denom = v₂⋅v₃

    if !isapprox(denom, 0.0, atol=1e-10)
        t₁ = (v₂×v₁) / denom # time for ray (0 ≤ t₁)
        t₂ = (v₁⋅v₃) / denom # time for segment (0 ≤ t₂ ≤ 1)
        return 0 ≤ t₁ && 0 ≤ t₂ ≤ 1
    else
        # denom is zero if the segment and the ray are parallel
        # only collide if they are perfectly aligned
        # must ensure that at least one point is in the positive ray direction
        r = polar(1.0, ray.θ)
        return are_collinear(R, seg.A, seg.B) &&
               (r⋅(seg.A - R) ≥ 0 || r⋅(seg.B - R) ≥ 0)
    end
end

"""
returns VecE2 of where intersection occurs, and VecE2(NaN,NaN) otherwise
"""
function Base.intersect(A::VecSE2, B::VecSE2)

    as = convert(VecE2, A)
    bs = convert(VecE2, B)
    ad = polar(1.0, A.θ)
    bd = polar(1.0, B.θ)

    dx = bs.x - as.x
    dy = bs.y - as.y
    det = bd.x * ad.y - bd.y * ad.x
    if !(det ≈ 0.0) # no intersection
        u = (dy * bd.x - dx * bd.y) / det
        v = (dy * ad.x - dx * ad.y) / det
        if u > 0.0 && v > 0.0
            return as + u*ad
        end
    end

    # TODO - if det == 0 could be the case that they are colinear, and the first point of intersection should be taken

    return VecE2(NaN,NaN) # no intersection
end
function Base.intersect(ray::Ray, seg::LineSegment)
    R = VecE2(ray)
    v₁ = VecE2(R) - seg.A
    v₂ = seg.B - seg.A
    v₃ = polar(1.0, ray.θ + π/2)

    denom = v₂⋅v₃

    if !isapprox(denom, 0.0, atol=1e-10)
        t₁ = (v₂×v₁) / denom # time for ray (0 ≤ t₁)
        t₂ = (v₁⋅v₃) / denom # time for segment (0 ≤ t₂ ≤ 1)
        if 0 ≤ t₁ && 0 ≤ t₂ ≤ 1
            return R + polar(t₁, ray.θ)
        end
    else
        # denom is zero if the segment and the ray are parallel
        # only collide if they are perfectly aligned
        # must ensure that at least one point is in the positive ray direction
        r = polar(1.0, ray.θ)
        if are_collinear(R, seg.A, seg.B) &&
               (r⋅(seg.A - R) ≥ 0 || r⋅(seg.B - R) ≥ 0)
            return R
        end
    end

    return VecE2(NaN,NaN) # no intersection
end

"""
What side of the ray you are on
Is -1 if on the left, 1 if on the right, and 0 if on the ray
"""
function get_side(ray::Ray, p::VecE2)
    ab = polar(1.0, ray.θ)
    return sign(ab.x*(p.y-ray.y) - ab.y*(p.x-ray.x))
end