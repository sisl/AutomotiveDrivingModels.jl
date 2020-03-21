export
    Quat,
    quat_for_a2b,
    get_axis,
    get_rotation_angle,

    RPY

struct Quat <: FieldVector{4, Float64}
    x::Float64
    y::Float64
    z::Float64
    w::Float64
end
Quat(v::VecE3, w::Float64) = Quat(v.x, v.y, v.z, w)
Quat(w::Float64, v::VecE3) = Quat(v.x, v.y, v.z, w)

Base.show(io::IO, q::Quat) = @printf(io, "Quat({%6.3f, %6.3f, %6.3f}, %6.3f)", q.x, q.y, q.z, q.w)

"""
Conjugate of a quaternion
"""
Base.conj(q::Quat) = Quat(-q.x,-q.y,-q.z, q.w)

"""
A VecE3 of the imaginary part (x,y,z)
"""
Base.imag(q::Quat) = VecE3(q.x, q.y, q.z)

"""
Return the multiplicative inverse of q.
Note that in most cases, i.e., if you simply want the opposite rotation,
and/or the quaternion is normalized, then it is enough to use the conjugate.
Based on Eigen's inverse.

Returns the zero Quat if q is zero.
"""
function Base.inv(q::Quat)
  n2 = q.x^2 + q.y^2 + q.z^2 + q.w^2 # squared norm
  if n2 > 0
    return Quat(conj(q) / n2)
  else
    # return an invalid result to flag the error
    return Quat(0.0,0.0,0.0,0.0)
  end
end

"""
Rotate a vector by a quaternion.
If you are going to rotate a bunch of vectors, covert q to a rotation matrix first.

Based on the Eigen implementation for _transformVector
"""
function rot(q::Quat, v::VecE3)
    uv = imag(q)×v
    uv += uv
    return v + q.w*uv + imag(q)×uv;
end

"""
Quaternion multiplication
"""
function Base.:*(a::Quat, b::Quat)
    r1, v1 = a.w, imag(a)
    r2, v2 = b.w, imag(b)
    return Quat(r1*v2 + r2*v1 + v1×v2, r1*r2 - v1⋅v2)
end
Base.:*(v::VecE3, b::Quat) = Quat(0.0, v) * b
Base.:*(a::Quat, v::VecE3) = a * Quat(0.0, v)

"""
The angle (in radians) between two rotations
"""
function angledist(a::Quat, b::Quat)
    d = a * conj(b)
    return 2*atan(LinearAlgebra.norm(imag(d)), abs(d.w))
end

"""
The quaternion that transforms a into b through a rotation
Based on Eigen's implementation for setFromTwoVectors
"""
function quat_for_a2b(a::VecE3, b::VecE3)
    v0 = LinearAlgebra.normalize(a)
    v1 = LinearAlgebra.normalize(b)
    c = v1⋅v0

    # if dot == -1, vectors are nearly opposites
    # => accurately compute the rotation axis by computing the
    #    intersection of the two planes. This is done by solving:
    #       x^T v0 = 0
    #       x^T v1 = 0
    #    under the constraint:
    #       ||x|| = 1
    #    which yields a singular value problem
    if c < -1.0 + DUMMY_PRECISION
        c = max(c, -1.0)
        M = hcat(convert(Vector{Float64}, v0),
                 convert(Vector{Float64}, v1))'
        s = LinearAlgebra.svdfact(M, thin=false)
        axis = convert(VecE3, s[:V][:,2])
        w2 = (1.0 + c)*0.5
        return Quat(axis * sqrt(1.0 - w2), sqrt(w2))
    else
        axis = v0×v1
        s = sqrt((1.0+c)*2.0)
        invs = 1.0/s
        return Quat(axis * invs, s * 0.5)
    end
end

"""
Returns the spherical linear interpolation between the two quaternions
a and b for t ∈ [0,1].

Based on the Eigen implementation for slerp.
"""
function lerp(a::Quat, b::Quat, t::Float64)

    d = a⋅b
    absD = abs(d)

    if(absD ≥ 1.0 - eps())
        scale0, scale1 = 1.0 - t, t
    else
        # θ is the angle between the 2 quaternions
        θ = acos(absD)
        sinθ = sin(θ)

        scale0, scale1 = sin( ( 1.0 - t ) * θ) / sinθ,
                         sin( ( t * θ) ) / sinθ
    end

    if d < 0
        scale1 = -scale1
    end

    return Quat(scale0*a + scale1*b)
end

"""
The rotation axis for a quaternion.
"""
function get_axis(q::Quat)

    θ = 2*atan(sqrt(q.x*q.x + q.y*q.y + q.z*q.z), q.w)
    h = sin(θ/2)

    x = q.x / h
    y = q.y / h
    z = q.z / h

    VecE3(x, y, z)
end

"""
The rotation angle for a quaternion.
"""
get_rotation_angle(q::Quat) = 2.0*acos(q.w)

"""
Draw a random unit quaternion following a uniform distribution law on SO(3).

Based on the Eigen implementation for UnitRandom.
"""
function Base.rand(::Type{Quat})
    u1 = rand()
    u2 = 2π*rand()
    u3 = 2π*rand()
    a = sqrt(1 - u1)
    b = sqrt(u1)
    return Quat(a*sin(u2), a*cos(u2), b*sin(u3), b*cos(u3))
end

"""
Roll Pitch Yaw
"""
struct RPY <: FieldVector{3, Float64}
    r::Float64
    p::Float64
    y::Float64
end
function Base.convert(::Type{RPY}, q::Quat)

    q2 = LinearAlgebra.normalize(q)
    x = q2[1]
    y = q2[2]
    z = q2[3]
    w = q2[4]

    # roll (x-axis rotation)
    sinr = 2(w * x + y * z)
    cosr = 1.0 - 2(x * x + y * y)
    roll = atan(sinr, cosr)

    # pitch (y-axis rotation)
    sinp = 2(w * y - z * x)
    if (abs(sinp) >= 1)
        pitch = π/2 * sign(sinp) # use 90 degrees if out of range
    else
        pitch = asin(sinp)
    end

    # yaw (z-axis rotation)
    siny = 2(w * z + x * y)
    cosy = 1.0 - 2(y * y + z * z)
    yaw = atan(siny, cosy)

    RPY(roll, pitch, yaw)
end
function Base.convert(::Type{Matrix{Float64}}, quat::Quat)

    # convert it to a rotation matrix

    w = quat.w
    x = quat.x
    y = quat.y
    z = quat.z

    n = w * w + x * x + y * y + z * z
    s = (n == 0.0) ? 0.0 : 2.0 / n
    wx = s * w * x; wy = s * w * y; wz = s * w * z
    xx = s * x * x; xy = s * x * y; xz = s * x * z
    yy = s * y * y; yz = s * y * z; zz = s * z * z

    [ 1 - (yy + zz)         xy - wz          xz + wy;
           xy + wz     1 - (xx + zz)         yz - wx;
           xz - wy          yz + wx     1 - (xx + yy) ]
end
function Base.convert(::Type{Quat}, rpy::RPY)
    t0 = cos(rpy.y/2)
    t1 = sin(rpy.y/2)
    t2 = cos(rpy.r/2)
    t3 = sin(rpy.r/2)
    t4 = cos(rpy.p/2)
    t5 = sin(rpy.p/2)

    qw = t0 * t2 * t4 + t1 * t3 * t5
    qx = t0 * t3 * t4 - t1 * t2 * t5
    qy = t0 * t2 * t5 + t1 * t3 * t4
    qz = t1 * t2 * t4 - t0 * t3 * t5

    return Quat(qx, qy, qz, qw)
end