export
    LineSegment,
    parallel

struct LineSegment
    A::VecE2
    B::VecE2
end
Base.:+(seg::LineSegment, V::VecE2) = LineSegment(seg.A + V, seg.B + V)
Base.:-(seg::LineSegment, V::VecE2) = LineSegment(seg.A - V, seg.B - V)
Base.convert(::Type{Line}, seg::LineSegment) = Line(seg.A, seg.B)
Base.convert(::Type{LineSegment}, line::Line) = LineSegment(line.A, line.B)

get_polar_angle(seg::LineSegment) = mod2pi(atan(seg.B.y - seg.A.y, seg.B.x - seg.A.x))

"""
The distance between the line segment and the point P
"""
function get_distance(seg::LineSegment, P::VecE2)

    ab = seg.B - seg.A
    pb = P - seg.A

    denom = normsquared(ab)
    if denom == 0.0
        return 0.0
    end

    r = (ab⋅pb)/denom

    if r ≤ 0.0
        LinearAlgebra.norm(P - seg.A)
    elseif r ≥ 1.0
        LinearAlgebra.norm(P - seg.B)
    else
        LinearAlgebra.norm(P - (seg.A + r*ab))
    end
end

"""
What side of the line you are on, based on A → B
"""
get_side(seg::LineSegment, p::VecE2) = sign((seg.B.x-seg.A.x) * (p.y-seg.A.y) - (seg.B.y-seg.A.y) * (p.x-seg.A.x))


"""
The angular distance between the two line segments
"""
function angledist(segA::LineSegment, segB::LineSegment)
    u = segA.B - segA.A
    v = segB.B - segB.A
    sqdenom = (u⋅u)*(v⋅v)
    if isapprox(sqdenom, 0.0, atol=1e-10)
        return NaN
    end
    return acos((u⋅v) / sqrt(sqdenom))
end

"""
True if the two segments are parallel
"""
function parallel(segA::LineSegment, segB::LineSegment, ε::Float64=1e-10)
    θ = angledist(segA, segB)
    return isapprox(θ, 0.0, atol=ε)
end

"""
Given P colinear with seg, the function checks if
point P lies on the line segment.
"""
function on_segment(P::VecE2, seg::LineSegment)
    return P.x ≤ max(seg.A.x, seg.B.x) && P.x ≥ min(seg.A.x, seg.B.x) &&
           P.y ≤ max(seg.A.y, seg.B.y) && P.y ≥ min(seg.A.y, seg.B.y)
end
on_segment(A::VecE2, P::VecE2, B::VecE2) = on_segment(P, LineSegment(A,B))

"""
returns true if line segments segP and segQ intersect
"""
function intersects(segP::LineSegment, segQ::LineSegment)

    p1 = segP.A
    q1 = segP.B
    p2 = segQ.A
    q2 = segQ.B

    # Find the four orientations needed for general and
    # special cases
    o1 = orientation(p1, q1, p2)
    o2 = orientation(p1, q1, q2)
    o3 = orientation(p2, q2, p1)
    o4 = orientation(p2, q2, q1)

    # Special Cases
    if (o1 != o2 && o3 != o4) || # General case
       (o1 == 0 && on_segment(p1, p2, q1)) || # p1, q1 and p2 are colinear and p2 lies on segment p1q1
       (o2 == 0 && on_segment(p1, q2, q1)) || # p1, q1 and p2 are colinear and q2 lies on segment p1q1
       (o3 == 0 && on_segment(p2, p1, q2)) || # p2, q2 and p1 are colinear and p1 lies on segment p2q2
       (o4 == 0 && on_segment(p2, q1, q2))    # p2, q2 and q1 are colinear and q1 lies on segment p2q2

        return true
    end
    return false # Doesn't fall in any of the above cases
end
