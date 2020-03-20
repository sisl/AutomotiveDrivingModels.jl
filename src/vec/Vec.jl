__precompile__()

module Vec

using StaticArrays
using LinearAlgebra
using Printf
import LinearAlgebra: ⋅, ×

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
    invlerp,
    lerp_angle,

    normsquared,         # see docstrings below
    scale_euclidean,
    clamp_euclidean,
    normalize_euclidian,

    dist,  # scalar distance between two vec's
    dist2, # squared scalar distance between two vec's

    rot,   # rotate the vector, always using the Right Hand Rule
    rot_normalized, # like rot, but assumes axis is normalized

    deltaangle, # signed delta angle
    angledist,  # distance between two angles

    inertial2body,
    body2inertial,

    orientation,
    are_collinear,
    get_intersection,

    Circ,
    AABB,
    OBB

abstract type AbstractVec{N, R} <: FieldVector{N, R} end
abstract type VecE{N, R} <: AbstractVec{N, R} end
abstract type VecSE{N, R} <: AbstractVec{N, R} end

lerp(a::Real, b::Real, t::Real) = a + (b-a)*t
invlerp(a::Real, b::Real, c::Real) = (c - a)/(b-a)

"The L2 norm squared."
normsquared(a::AbstractVec) = sum(x^2 for x in a)

"Scale the euclidean part of the vector by a factor b while leaving the orientation part unchanged."
scale_euclidean(a::VecE, b::Real) = b.*a

"Clamp each element of the euclidean part of the vector while leaving the orientation part unchanged."
clamp_euclidean(a::VecE, lo::Real, hi::Real) = clamp.(a, lo, hi)

"Normalize the euclidean part of the vector while leaving the orientation part unchanged."
normalize_euclidian(a::VecE, p::Real=2) = normalize(a, p)

include("common.jl")
include("vecE2.jl")
include("vecE3.jl")
include("vecSE2.jl")

include("geom/geom.jl")
include("coordinate_transforms.jl")
include("quat.jl")

Base.vec(v::Union{AbstractVec, Quat, RPY}) = convert(Vector{Float64}, v)

end # module
