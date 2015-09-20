#=
Geometry in 2d euclidean space
=#

# signed delta angle
deltaangle( a::Real, b::Real ) = atan2(sin(a-b), cos(a-b))
# distance between two angles
angledist( a::Real, b::Real ) = abs(tan2(sin(a-b), cos(a-b)))

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