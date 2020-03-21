export
    LineSegment1D

"""
A line segment in one dimension with a ≤ b
"""
struct LineSegment1D
    a::Float64
    b::Float64
    LineSegment1D(a::Real, b::Real) = new(convert(Float64, min(a,b)), convert(Float64, max(a,b)))
end

Base.:+(seg::LineSegment1D, v::Real) = LineSegment1D(seg.a + v, seg.b + v)
Base.:-(seg::LineSegment1D, v::Real) = LineSegment1D(seg.a - v, seg.b - v)

"""
The distance between the line segment and the point P
"""
function get_distance(seg::LineSegment1D, P::Real)::Float64
    if P < seg.a
        return seg.a - P
    elseif P > seg.b
        return P - seg.b
    else
        return 0.0
    end
end

Base.in(v::Float64, P::LineSegment1D) = P.a ≤ v ≤ P.b
Base.in(Q::LineSegment1D, P::LineSegment1D) = Q.a ≥ P.a && Q.b ≤ P.b

intersects(P::LineSegment1D, Q::LineSegment1D) = P.b ≥ Q.a && Q.b ≥ P.a
