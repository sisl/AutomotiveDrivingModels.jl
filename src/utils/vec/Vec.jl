module Vec

export
    AbstractVec,
    VecE,  # an abstract euclidean-group vector
    VecSE, # an abstract special euclidean-group element

    VecE2, # two-element Float64 vector, {x, y}
    VecE3, # three-element Float64 vector, {x, y, z}
    VecSE2, # point in special euclidean group of order 2, {x, y, θ}

    polar, # construct a polar vector with (r,θ)

    proj,  # vector projection
           # proj(a::vec, b::vec, ::Type{Float64}) will do scalar projection of a onto b
           # proj(a::vec, b::vec, ::Type{Vec}) will do vector projection of a onto b

    lerp,  # linear interpolation between two vec's

    dist,  # scalar distance between two vec's
    dist2, # squared scalar distance between two vec's

    rot,   # rotate the vector, always using the Right Hand Rule
    rot_normalized, # like rot, but assumes axis is normalized

    deltaangle, # signed delta angle
    angledist,  # distance between two angles

    inertial2body,
    body2inertial

abstract AbstractVec
abstract VecE <: AbstractVec
abstract VecSE <: AbstractVec

include("vecE2.jl")
include("vecE3.jl")
include("vecSE2.jl")

function Base.isapprox(x::VecE, y::VecE;
    _absx::Float64 = abs(x),
    _absy::Float64 = abs(y),
    _maxeps::Float64 = max(eps(_absx), eps(_absy)),
    rtol::Real=cbrt(_maxeps),
    atol::Real=sqrt(_maxeps)
    )

    dist2(x, y) <= atol + rtol*max(_absx, _absy)
end

include("geomE2.jl")
include("coordinate_transforms.jl")
include("quat.jl")

end # module
