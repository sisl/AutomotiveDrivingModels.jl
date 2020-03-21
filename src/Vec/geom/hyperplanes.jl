export
    Plane3,
    get_signed_distance

"""
Plane3

A hyperplane is an affine subspace of dimension n-1 in a space of dimension n.
For example, a hyperplane in a plane is a line; a hyperplane in 3-space is a plane.

This class represents an hyperplane in 3D space. It is the zero set of
n⋅x + d = 0 where n is a unit normal vector of the plane (linear part)
and d is the distance (offset) to the origin.
"""
struct Plane3
    normal::VecE3 # should always be normalized
    offset::Float64

    Plane3() = new(VecE3(1.0,0.0,0.0), 0.0)

    """
    Constructs a plane from its normal n and distance to the origin d
    such that the algebraic equation of the plane is n⋅x + d = 0.
    """
    Plane3(normal::VecE3, offset::Real) = new(LinearAlgebra.normalize(normal), convert(Float64, offset))


    """
    Construct a plane from its normal and a point on the plane.
    """
    function Plane3(normal::VecE3, P::VecE3)
        n = LinearAlgebra.normalize(normal)
        offset = -n⋅P
        return new(n, offset)
    end
end

"""
Constructs a plane passing through the three points.
"""
function Plane3(p0::VecE3, p1::VecE3, p2::VecE3)
    v0 = p2 - p0
    v1 = p1 - p0

    normal = v0×v1
    nnorm = LinearAlgebra.norm(normal)
    if nnorm <= LinearAlgebra.norm(v0)*LinearAlgebra.norm(v1)*eps()
        M = hcat(convert(Vector{Float64}, v0),
                 convert(Vector{Float64}, v1))'
        s = LinearAlgebra.svdfact(M, thin=false)
        normal = convert(VecE3, s[:V][:,2])
    else
        normal = LinearAlgebra.normalize(normal)
    end

    offset = -p0⋅normal

    return Plane3(normal, offset)
end

"""
Returns the signed distance between the plane and a point
"""
get_signed_distance(plane::Plane3, A::VecE3) = plane.normal⋅A + plane.offset

"""
Returns the absolute distance between the plane and a point
"""
get_distance(plane::Plane3, A::VecE3) = abs(get_signed_distance(plane, A))

"""
Returns the projection of a point onto the plane
"""
proj(A::VecE3, plane::Plane3) = A - get_signed_distance(plane, A) * plane.normal

"""
What side of the plane you are on
Is -1 if on the negative side, 1 if on the positive side, and 0 if on the plane
"""
function get_side(plane::Plane3, A::VecE3, ε::Float64=DUMMY_PRECISION)
    signed_dist = get_signed_distance(plane, A)
    if abs(signed_dist) < ε
        return 0
    else
        return sign(signed_dist)
    end
end