export
    Projectile,
    propagate,
    closest_time_of_approach,
    closest_approach_distance,
    closest_time_of_approach_and_distance,
    get_intersection_time

struct Projectile
    pos::VecSE2 # position (x,y,θ)
    v::Float64  # speed
end

propagate(P::Projectile, Δt::Float64) = Projectile(P.pos + polar(P.v*Δt, P.pos.θ), P.v)
"""
The time at which two projectiles are closest to one another
A negative value indicates that this occurred in the past.
see: http://geomalgorithms.com/a07-_distance.html
"""
function closest_time_of_approach(A::Projectile, B::Projectile)

    W = convert(VecE2, A.pos - B.pos)
    Δ = polar(A.v, A.pos.θ) - polar(B.v, B.pos.θ)
    aΔ = normsquared(Δ)

    if aΔ ≈ 0.0
        return 0.0
    else
        return -(W⋅Δ) / aΔ
    end
end
function closest_approach_distance(A::Projectile, B::Projectile, t_CPA::Float64=closest_time_of_approach(A, B))
    Aₜ = convert(VecE2, propagate(A, t_CPA).pos)
    Bₜ = convert(VecE2, propagate(B, t_CPA).pos)
    return LinearAlgebra.norm(Aₜ - Bₜ)
end
function closest_time_of_approach_and_distance(A::Projectile, B::Projectile)
    t_CPA = closest_time_of_approach(A, B)
    d_CPA = closest_approach_distance(A, B, t_CPA)
    return (t_CPA, d_CPA)
end

function get_intersection_time(A::Projectile, seg::LineSegment)

    o = VecE2(A.pos)
    v₁ = o - seg.A
    v₂ = seg.B - seg.A
    v₃ = polar(1.0, A.pos.θ + π/2)

    denom = (v₂⋅v₃)

    if !isapprox(denom, 0.0, atol=1e-10)
        d₁ = (v₂×v₁) / denom # time for projectile (0 ≤ t₁)
        t₂ = (v₁⋅v₃) / denom # time for segment (0 ≤ t₂ ≤ 1)
        if 0.0 ≤ d₁ && 0.0 ≤ t₂ ≤ 1.0
            return d₁/A.v
        end
    else
        # denom is zero if the segment and the projectile are parallel
        # only collide if they are perfectly aligned
        if are_collinear(A.pos, seg.A, seg.B)
            dist_a = normsquared(seg.A - o)
            dist_b = normsquared(seg.B - o)
            return sqrt(min(dist_a, dist_b)) / A.v
        end
    end

    return Inf
end
function closest_time_of_approach_and_distance(P::Projectile, seg::LineSegment, if_no_col_skip_eval::Bool=false)

    o = VecE2(P.pos)
    v₁ = o - seg.A
    v₂ = seg.B - seg.A
    v₃ = polar(1.0, P.pos.θ + π/2)

    denom = (v₂⋅v₃)

    if !isapprox(denom, 0.0, atol=1e-10)

        d₁ = (v₂×v₁) / denom # time for projectile (0 ≤ d₁)
        t₂ = (v₁⋅v₃) / denom # time for segment (0 ≤ t₂ ≤ 1)

        if 0.0 ≤ d₁ && 0.0 ≤ t₂ ≤ 1.0
            t = d₁/P.v
            return (t, 0.0)
        else
            # no collision, get time of closest approach to each endpoint
            if !if_no_col_skip_eval
                r = polar(1.0, P.pos.θ)
                projA = proj(seg.A - o, r, VecE2)
                projB = proj(seg.B - o, r, VecE2)
                tA = max(LinearAlgebra.norm(projA)/P.v * sign(r⋅projA), 0.0)
                tB = max(LinearAlgebra.norm(projB)/P.v * sign(r⋅projB), 0.0)
                pA = VecE2(propagate(P, tA).pos)
                pB = VecE2(propagate(P, tB).pos)
                distA = normsquared(seg.A - pA)
                distB = normsquared(seg.B - pB)
                if distA < distB
                    return (tA, sqrt(distA))
                else
                    return (tB, sqrt(distB))
                end
            else
                return (Inf, Inf)
            end
        end
    else
        # denom is zero if the segment and the projectile are parallel
        # only collide if they are perfectly aligned
        if are_collinear(P.pos, seg.A, seg.B)
            # either closest now, will be to A, or will be to B
            dist_a = normsquared(seg.A - o)
            dist_b = normsquared(seg.B - o)
            t = sqrt(min(dist_a, dist_b)) / P.v
            return (t, 0.0)
        else
            # not colinear
            if !if_no_col_skip_eval
                # either closest now, will be to A, or will be to B
                r = polar(1.0, P.pos.θ)
                projA = proj(seg.A - o, r, VecE2)
                projB = proj(seg.B - o, r, VecE2)
                tA = max(LinearAlgebra.norm(projA)/P.v * sign(r⋅projA), 0.0)
                tB = max(LinearAlgebra.norm(projB)/P.v * sign(r⋅projB), 0.0)
                pA = VecE2(propagate(P, tA).pos)
                pB = VecE2(propagate(P, tB).pos)
                distA = normsquared(seg.A - pA)
                distB = normsquared(seg.B - pB)
                if distA < distB
                    return (tA, distA)
                else
                    return (tB, distB)
                end
            else
                return (Inf,Inf)
            end
        end
    end
end
