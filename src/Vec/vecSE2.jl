#=
VecSE2: a 2d euclidean vector with an orientation
=#

struct VecSE2{R<:Real} <: VecSE{3,R}
    x::R
    y::R
    θ::R
end

VecSE2() = VecSE2(0.0,0.0,0.0)
VecSE2(x::Real, y::Real) = VecSE2(x,y,0.0)
VecSE2(a::VecE2, θ::Real=0.0) = VecSE2(a.x, a.y, θ)
VecSE2(t::Tuple) = VecSE2(promote(t...)...)

Base.convert(::Type{VecE2}, a::VecSE2) = VecE2(a.x, a.y)
VecE2(a::VecSE2) = VecE2(a.x, a.y)

polar(r::Real, ϕ::Real, θ::Real) = VecSE2(r*cos(ϕ), r*sin(ϕ), θ)

Base.show(io::IO, a::VecSE2) = @printf(io, "VecSE2({%.3f, %.3f}, %.3f)", a.x, a.y, a.θ)


Base.:+(a::VecSE2, b::VecE2) = VecSE2(a.x+b.x, a.y+b.y, a.θ)
Base.:+(a::VecE2, b::VecSE2) = VecSE2(a.x+b.x, a.y+b.y, b.θ)

Base.:-(a::VecSE2, b::VecE2) = VecSE2(a.x-b.x, a.y-b.y, a.θ)
Base.:-(a::VecE2,  b::VecSE2) = VecSE2(a.x-b.x, a.y-b.y, -b.θ)

op_overload_error = """
    Operator overloading for Real and VecSE2 has been removed.

    To add or subtract, use VecSE2(x,y,θ) + b*VecE2(1.0, 1.0).
    To multiply or divide, use scale_euclidean(VecSE2(x,y,θ), b)
"""
Base.:-(b::Real, a::VecSE2) = error(op_overload_error)
Base.:-(a::VecSE2, b::Real) = error(op_overload_error)
Base.:+(b::Real, a::VecSE2) = error(op_overload_error)
Base.:+(a::VecSE2, b::Real) = error(op_overload_error)
Base.:*(b::Real, a::VecSE2) = error(op_overload_error)
Base.:*(a::VecSE2, b::Real) = error(op_overload_error)
Base.:/(a::VecSE2, b::Real) = error(op_overload_error)
Base.:^(a::VecSE2, b::Integer) = error(op_overload_error)
Base.:^(a::VecSE2, b::AbstractFloat) = error(op_overload_error)

Base.:%(a::VecSE2, b::Real) = error(op_overload_error)

scale_euclidean(a::VecSE2, b::Real) = VecSE2(b*a.x, b*a.y, a.θ)
clamp_euclidean(a::VecSE2, lo::Real, hi::Real) = VecSE2(clamp(a.x, lo, hi), clamp(a.y, lo, hi), a.θ)

norm(a::VecSE2, p::Real=2) = error("norm is not defined for VecSE2 - use norm(VecE2(v)) to get the norm of the Euclidean part")
normsquared(a::VecSE2) = norm(a)^2 # this will correctly throw an error
normalize(a::VecSE2, p::Real=2) = error("normalize is not defined for VecSE2. Use normalize_euclidean(v) to normalize the euclidean part of the vector")

function normalize_euclidian(a::VecSE2, p::Real=2)
    n = norm(VecE2(a))
    return VecSE2(a.x/n, a.y/n, a.θ)
end

Base.atan(a::VecSE2) = atan(a.y, a.x)

function lerp(a::VecSE2, b::VecSE2, t::Real)
    x = a.x + (b.x-a.x)*t
    y = a.y + (b.y-a.y)*t
    θ = lerp_angle(a.θ, b.θ, t)
    VecSE2(x, y, θ)
end

Base.rot180(a::VecSE2) = VecSE2(a.x, a.y, a.θ+π)
Base.rotl90(a::VecSE2) = VecSE2(a.x, a.y, a.θ+0.5π)
Base.rotr90(a::VecSE2) = VecSE2(a.x, a.y, a.θ-0.5π)
rot(a::VecSE2, Δθ::Float64) = VecSE2(a.x, a.y, a.θ+Δθ)

Base.mod2pi(a::VecSE2) = VecSE2(a.x, a.y, mod2pi(a.θ))
