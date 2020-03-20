export
    Line,
    get_polar_angle,
    get_distance,
    get_side

struct Line
    C::VecE2
    θ::Float64
end
Line(A::VecE2, B::VecE2) = Line((A+B)/2, atan(B - A))

Base.:-(line::Line, V::VecE2) = Line(line.C - V, line.θ)
Base.:+(line::Line, V::VecE2) = Line(line.C + V, line.θ)

get_polar_angle(line::Line) = line.θ
rot(line::Line, Δθ::Float64) = Line(line.C, line.θ+Δθ)
Base.rot180(line::Line) = rot(line, 1π)
Base.rotl90(line::Line) = rot(line,  π/2)
Base.rotr90(line::Line) = rot(line, -π/2)

"""
The distance between the line and the point P
"""
function get_distance(line::Line, P::VecE2)

    ab = polar(1.0, line.θ)
    pb = P - line.C

    denom = normsquared(ab)
    if denom ≈ 0.0
        return 0.0
    end

    r = (ab⋅pb)/denom
    return LinearAlgebra.norm(P - (line.C + r*ab))
end


"""
What side of the line you are on
Is 1 if on the left, -1 if on the right, and 0 if on the line
"""
function get_side(line::Line, p::VecE2, ε::Float64=1e-10)
    ab = polar(1.0, line.θ)
    signed_dist = ab.x*(p.y-line.C.y) - ab.y*(p.x-line.C.x)
    if abs(signed_dist) < ε
        return 0
    else
        return convert(Int, sign(signed_dist))
    end
end
