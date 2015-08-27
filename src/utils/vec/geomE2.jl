#=
Geometry in 2d euclidean space
=#

# signed delta angle
deltaangle( a::Real, b::Real ) = atan2(sin(a-b), cos(a-b))
# distance between two angles
angledist( a::Real, b::Real ) = abs(tan2(sin(a-b), cos(a-b)))

function inertial2body(inertial::VecE2, reference::VecSE2)

    #=
    Convert a point in an inertial cartesian coordinate frame
    to be relative to a body's coordinate frame

    The body's position is given relative to the same inertial coordinate frame
    =#

    s, c = sin(reference.θ), cos(reference.θ)
    Δx = inertial.x - reference.x
    Δy = inertial.y - reference.y
    VecE2(c*Δx + s*Δy, c*Δy - s*Δx)
end
function inertial2body(inertial::VecSE2, reference::VecSE2)

    #=
    Convert a point in an inertial cartesian coordinate frame
    to be relative to a body's coordinate frame

    The body's position is given relative to the same inertial coordinate frame
    =#

    s, c = sin(reference.θ), cos(reference.θ)
    Δx = inertial.x - reference.x
    Δy = inertial.y - reference.y
    VecSE2(c*Δx + s*Δy, c*Δy - s*Δx, inertial.θ - reference.θ)
end
function body2inertial(inertial::VecE2, reference::VecSE2)

    #=
    Convert a point in a body-relative cartesian coordinate frame
    to be relative to a the inertial coordinate frame the body is described by
    =#

    c, s = cos(reference.θ), sin(reference.θ)
    VecE2(c*inertial.x -s*inertial.y + reference.x, s*inertial.x +c*inertial.y + reference.y)
end
function body2inertial(inertial::VecSE2, reference::VecSE2)

    #=
    Convert a point in a body-relative cartesian coordinate frame
    to be relative to a the inertial coordinate frame the body is described by
    =#

    c, s = cos(reference.θ), sin(reference.θ)
    VecSE2(c*inertial.x -s*inertial.y + reference.x, s*inertial.x +c*inertial.y + reference.y, reference.θ + inertial.θ)
end