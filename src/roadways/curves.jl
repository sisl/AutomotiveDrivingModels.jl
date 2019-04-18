"""
    CurvePt{T}
describes a point on a curve, associated with a curvature and the derivative of the curvature

- `pos::VecSE2{T}` # global position and orientation
- `s::T`  # distance along the curve
- `k::T` # curvature
- `kd::T` # derivative of curvature
"""
struct CurvePt{T}
    pos::VecSE2{T} # global position and orientation
    s::T  # distance along the curve
    k::T # curvature
    kd::T# derivative of curvature
end

function CurvePt(pos::VecSE2{T}, s::T, k::T = convert(T, NaN)) where T 
    return CurvePt{T}(pos, s, k, convert(T, NaN))
end

Base.show(io::IO, pt::CurvePt) = @printf(io, "CurvePt({%.3f, %.3f, %.3f}, %.3f, %.3f, %.3f)", pt.pos.x, pt.pos.y, pt.pos.θ, pt.s, pt.k, pt.kd)

Vec.lerp(a::CurvePt, b::CurvePt, t::T) where T <: Real = CurvePt(lerp(a.pos, b.pos, t), a.s + (b.s - a.s)*t, a.k + (b.k - a.k)*t, a.kd + (b.kd - a.kd)*t)

############

"""
    Curve{T} 
is a vector of curve points
"""
const Curve{T} = Vector{CurvePt{T}} where T

"""
    get_lerp_time_unclamped(A::VecE2, B::VecE2, Q::VecE2)
Get the interpolation scalar t for the point on the line AB closest to Q
This point is P = A + (B-A)*t
"""
function get_lerp_time_unclamped(A::VecE2, B::VecE2, Q::VecE2)

    a = Q - A
    b = B - A
    c = proj(a, b, VecE2)

    if b.x != 0.0
        t = c.x / b.x
    elseif b.y != 0.0
        t = c.y / b.y
    else
        t = 0.0 # no lerping to be done
    end

    t
end
get_lerp_time_unclamped(A::VecSE2, B::VecSE2, Q::VecSE2) = get_lerp_time_unclamped(convert(VecE2, A), convert(VecE2, B), convert(VecE2, Q))
get_lerp_time_unclamped(A::CurvePt, B::CurvePt, Q::VecSE2) = get_lerp_time_unclamped(convert(VecE2, A.pos), convert(VecE2, B.pos), convert(VecE2, Q))

"""
    get_lerp_time(A::VecE2, B::VecE2, Q::VecE2)
Get lerp time t∈[0,1] such that lerp(A, B) is as close as possible to Q
"""
get_lerp_time(A::VecE2, B::VecE2, Q::VecE2) = clamp(get_lerp_time_unclamped(A, B, Q), 0.0, 1.0)
get_lerp_time(A::CurvePt, B::CurvePt, Q::VecSE2) = get_lerp_time(convert(VecE2, A.pos), convert(VecE2, B.pos), convert(VecE2, Q))

"""
    CurveIndex{I <: Integer, T <: Real}
Given a `Curve` object `curve` one can call `curve[ind]`
where `ind` is a `CurveIndex`. The field `t` can be used to interpolate between two 
points in the curve. 

# Fields
- `i`::I` index in the curve , ∈ [1:length(curve)-1]
- `t::T` ∈ [0,1] for linear interpolation
"""
struct CurveIndex{I <: Integer, T <: Real}
    i::I     # index in curve, ∈ [1:length(curve)-1]
    t::T # ∈ [0,1] for linear interpolation
end
const CURVEINDEX_START = CurveIndex(1,0.0)
Base.show(io::IO, ind::CurveIndex) = @printf(io, "CurveIndex(%d, %.3f)", ind.i, ind.t)

curveindex_end(curve::Curve) = CurveIndex(length(curve)-1,1.0)

Base.getindex(curve::Curve, ind::CurveIndex) = lerp(curve[ind.i], curve[ind.i+1], ind.t)

"""
    is_at_curve_end(ind::CurveIndex, curve::Curve)
returns true if the curve index is at the end of the curve
"""
function is_at_curve_end(ind::CurveIndex, curve::Curve)
    (ind.i == 1 && ind.t == 0.0) ||
    (ind.i == length(curve)-1 && ind.t == 1.0)
end

"""
    index_closest_to_point(curve::Curve, target::AbstractVec)
returns the curve index closest to the point described by `target`.
`target` must be [x, y].
"""
function index_closest_to_point(curve::Curve, target::AbstractVec)

    a = 1
    b = length(curve)
    c = div(a+b, 2)

    @assert(length(curve) ≥ b)

    sqdist_a = normsquared(VecE2(curve[a].pos - target))
    sqdist_b = normsquared(VecE2(curve[b].pos - target))
    sqdist_c = normsquared(VecE2(curve[c].pos - target))

    while true
        if b == a
            return a
        elseif b == a + 1
            return sqdist_b < sqdist_a ? b : a
        elseif c == a + 1 && c == b - 1
            if sqdist_a < sqdist_b && sqdist_a < sqdist_c
                return a
            elseif sqdist_b < sqdist_a && sqdist_b < sqdist_c
                return b
            else
                return c
            end
        end

        left = div(a+c, 2)
        sqdist_l = normsquared(VecE2(curve[left].pos - target))

        right = div(c+b, 2)
        sqdist_r = normsquared(VecE2(curve[right].pos - target))

        if sqdist_l < sqdist_r
            b = c
            sqdist_b = sqdist_c
            c = left
            sqdist_c = sqdist_l
        else
            a = c
            sqdist_a = sqdist_c
            c = right
            sqdist_c = sqdist_r
        end
    end

    error("index_closest_to_point reached unreachable statement")
end

"""
    get_curve_index(curve::Curve{T}, s::T) where T <: Real
Return the CurveIndex for the closest s-location on the curve
"""
function get_curve_index(curve::Curve{T}, s::T) where T <: Real

    if s ≤ 0.0
        return CURVEINDEX_START
    elseif s ≥ curve[end].s
        return curveindex_end(curve)
    end

    a = 1
    b = length(curve)

    fa = curve[a].s - s
    fb = curve[b].s - s

    n = 1
    while true
        if b == a+1
            extind = a + -fa/(fb-fa)
            ind = floor(Int, extind)
            t = rem(extind, 1.0)
            return CurveIndex(ind, t)
        end

        c = div(a+b, 2)
        fc = curve[c].s - s
        n += 1

        if sign(fc) == sign(fa)
            a, fa = c, fc
        else
            b, fb = c, fc
        end
    end

    error("get_curve_index failed for s=$s")
end

"""
    get_curve_index(ind::CurveIndex, curve::Curve, Δs::T) where T <: Real
Return the CurveIndex at ind's s position + Δs
"""
function get_curve_index(ind::CurveIndex, curve::Curve, Δs::T) where T <: Real

    L = length(curve)
    ind_lo, ind_hi = ind.i, ind.i+1

    s_lo = curve[ind_lo].s
    s_hi = curve[ind_hi].s
    s = lerp(s_lo, s_hi, ind.t)

    if Δs ≥ 0.0

        if s + Δs ≥ s_hi && ind_hi < L
            while s + Δs ≥ s_hi && ind_hi < L
                Δs -= (s_hi - s)
                s = s_hi
                ind_lo += 1
                ind_hi += 1
                s_lo = curve[ind_lo].s
                s_hi = curve[ind_hi].s
            end
        else
            Δs = s + Δs - s_lo
        end

        t = Δs/(s_hi - s_lo)
        CurveIndex(ind_lo, t)
    else
        while s + Δs < s_lo  && ind_lo > 1
            Δs += (s - s_lo)
            s = s_lo
            ind_lo -= 1
            ind_hi -= 1
            s_lo = curve[ind_lo].s
            s_hi = curve[ind_hi].s
        end

        Δs = s + Δs - s_lo
        t = Δs/(s_hi - s_lo)
        CurveIndex(ind_lo, t)
    end
end

"""
    CurveProjection{I <: Integer, T <: Real}
The result of a point projected to a Curve

# Fields
- `ind::CurveIndex{I, T}`
- `t::T` lane offset 
- `ϕ::T` lane-relative heading [rad]
"""
struct CurveProjection{I <: Integer, T <: Real}
    ind::CurveIndex{I, T}
    t::T # lane offset
    ϕ::T # lane-relative heading [rad]
end
Base.show(io::IO, curveproj::CurveProjection) = @printf(io, "CurveProjection({%d, %.3f}, %.3f, %.3f)", curveproj.ind.i, curveproj.ind.t, curveproj.t, curveproj.ϕ)
function get_curve_projection(posG::VecSE2, footpoint::VecSE2, ind::CurveIndex)
    F = inertial2body(posG, footpoint)
    CurveProjection(ind, F.y, F.θ)
end

"""
    Vec.proj(posG::VecSE2, curve::Curve) 
Return a CurveProjection obtained by projecting posG onto the curve
"""
function Vec.proj(posG::VecSE2{T}, curve::Curve{T}) where T

    ind = index_closest_to_point(curve, posG)

    curveind = CurveIndex{Int64, T}(0,NaN)
    footpoint = VecSE2{T}(NaN, NaN, NaN)

    if ind > 1 && ind < length(curve)
        t_lo = get_lerp_time(curve[ind-1], curve[ind],   posG)
        t_hi = get_lerp_time(curve[ind],   curve[ind+1], posG)

        p_lo = lerp(curve[ind-1].pos, curve[ind].pos,   t_lo)
        p_hi = lerp(curve[ind].pos,   curve[ind+1].pos, t_hi)

        d_lo = norm(VecE2(p_lo - posG))
        d_hi = norm(VecE2(p_hi - posG))

        if d_lo < d_hi
            footpoint = p_lo
            curveind = CurveIndex(ind-1, t_lo)
        else
            footpoint = p_hi
            curveind = CurveIndex(ind, t_hi)
        end
    elseif ind == 1
        t = get_lerp_time( curve[1], curve[2], posG )
        footpoint = lerp( curve[1].pos, curve[2].pos, t)
        curveind = CurveIndex(ind, t)
    else # ind == length(curve)
        t = get_lerp_time( curve[end-1], curve[end], posG )
        footpoint = lerp( curve[end-1].pos, curve[end].pos, t)
        curveind = CurveIndex(ind-1, t)
    end

    get_curve_projection(posG, footpoint, curveind)
end
