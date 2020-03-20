#=
Geometry in 2d euclidean space
=#

"""
deltaangle(a::Real, b::Real)

Return the minimum δ such that
    a + δ = mod(b, 2π)
"""
deltaangle(a::Real, b::Real) = atan(sin(b-a), cos(b-a))

"""
Distance between two angles
"""
angledist( a::Real, b::Real ) = abs(deltaangle(a,b))

"""
Linear interpolation between angles
"""
lerp_angle(a::Real, b::Real, t::AbstractFloat) = a + deltaangle(a, b)*t

"""
True if the values are collinear within a tolerance
"""
function are_collinear(a::AbstractVec, b::AbstractVec, c::AbstractVec, tol::Float64=1e-8)
    # http://mathworld.wolfram.com/Collinear.html
    # if val = 0 then they are collinear
    val = a.x*(b.y-c.y) + b.x*(c.y-a.y)+c.x*(a.y-b.y)
    abs(val) < tol
end

"""
To find orientation of ordered triplet (p, q, r).
The function returns following values
0 --> p, q and r are colinear
1 --> Clockwise
2 --> Counterclockwise
"""
function orientation(P::VecE2, Q::VecE2, R::VecE2)
    val = (Q.y - P.y)*(R.x - Q.x) - (Q.x - P.x)*(R.y - Q.y)
    if val ≈ 0
        return 0  # colinear
    end
    return (val > 0) ? 1 : 2
end

function inertial2body(point::VecE2, reference::VecSE2)

    #=
    Convert a point in an inertial cartesian coordinate frame
    to be relative to a body's coordinate frame

    The body's position is given relative to the same inertial coordinate frame
    =#

    s, c = sin(reference.θ), cos(reference.θ)
    Δx = point.x - reference.x
    Δy = point.y - reference.y
    VecE2(c*Δx + s*Δy, c*Δy - s*Δx)
end
function inertial2body(point::VecSE2, reference::VecSE2)

    #=
    Convert a point in an inertial cartesian coordinate frame
    to be relative to a body's coordinate frame

    The body's position is given relative to the same inertial coordinate frame
    =#

    s, c = sin(reference.θ), cos(reference.θ)
    Δx = point.x - reference.x
    Δy = point.y - reference.y
    VecSE2(c*Δx + s*Δy, c*Δy - s*Δx, point.θ - reference.θ)
end
function body2inertial(point::VecE2, reference::VecSE2)

    #=
    Convert a point in a body-relative cartesian coordinate frame
    to be relative to a the inertial coordinate frame the body is described by
    =#

    c, s = cos(reference.θ), sin(reference.θ)
    VecE2(c*point.x -s*point.y + reference.x, s*point.x +c*point.y + reference.y)
end
function body2inertial(point::VecSE2, reference::VecSE2)

    #=
    Convert a point in a body-relative cartesian coordinate frame
    to be relative to a the inertial coordinate frame the body is described by
    =#

    c, s = cos(reference.θ), sin(reference.θ)
    VecSE2(c*point.x -s*point.y + reference.x, s*point.x +c*point.y + reference.y, reference.θ + point.θ)
end

include("1d.jl")
include("lines.jl")
include("line_segments.jl")
include("rays.jl")
include("projectiles.jl")
include("solids.jl")
include("hyperplanes.jl")