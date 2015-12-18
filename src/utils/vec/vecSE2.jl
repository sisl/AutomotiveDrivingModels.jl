#=
VecSE2: a 2d euclidean vector with an orientation
=#

immutable VecSE2 <: VecSE
    x :: Float64
    y :: Float64
    θ :: Float64

    VecSE2(x::Real=0.0, y::Real=0.0, θ::Real=0.0) = new(x, y, θ)
    VecSE2(a::VecE2, θ::Real=0.0) = new(a.x, a.y, θ)
end

polar(r::Real, ϕ::Real, θ::Real) = VecSE2(r*cos(ϕ), r*sin(ϕ), θ)

Base.length(::VecSE2) = 3
Base.copy(a::VecSE2) = VecSE2(a.x, a.y, a.θ)
Base.convert(::Type{Vector{Float64}}, a::VecSE2) = [a.x, a.y, a.θ]
Base.convert(::Type{VecE3}, a::VecSE2) = VecE3(a.x, a.y, a.θ)
Base.convert(::Type{VecE2}, a::VecSE2) = VecE2(a.x, a.y)
function Base.convert{R<:Real}(::Type{VecSE2}, a::AbstractArray{R})
    @assert(length(a) == 3)
    VecSE2(a[1], a[2], a[3])
end
Base.show(io::IO, a::VecSE2) = @printf(io, "VecSE2({%.3f, %.3f}, %.3f)", a.x, a.y, a.θ)

# function Base.isapprox(x::VecE, y::VecE;
#     _absx::Float64 = abs(x),
#     _absy::Float64 = abs(y),
#     _maxeps::Float64 = max(eps(_absx), eps(_absy)),
#     rtol::Real=cbrt(_maxeps),
#     atol::Real=sqrt(_maxeps)
#     )

#     dist2(x, y) <= atol + rtol*max(_absx, _absy)
# end

Base.(:(+))(a::VecSE2, b::VecE2) = VecSE2(a.x+b.x, a.y+b.y, a.θ)
Base.(:(+))(a::VecSE2, b::VecSE2) = VecSE2(a.x+b.x, a.y+b.y, a.θ+b.θ)

Base.(:(-))(a::VecSE2, b::VecE2) = VecSE2(a.x-b.x, a.y-b.y, a.θ)
Base.(:(-))(a::VecSE2, b::VecSE2) = VecSE2(a.x-b.x, a.y-b.y, a.θ-b.θ)

Base.(:(==))(a::VecSE2, b::VecSE2) = isequal(a.x, b.x) && isequal(a.y, b.y) && isequal(a.θ, b.θ)
Base.isequal(a::VecSE2, b::VecSE2) = isequal(a.x, b.x) && isequal(a.y, b.y) && isequal(a.θ, b.θ)

Base.abs(a::VecSE2) = hypot(a.x, a.y)
Base.hypot(a::VecSE2) = hypot(a.x, a.y)
Base.abs2(a::VecSE2) = a.x*a.x + a.y*a.y
function Base.norm(a::VecSE2)
    m = abs(a)
    VecSE2(a.x/m, a.y/m)
end

lerp(a::VecSE2, b::VecSE2, t::Real) = VecSE2(a.x + (b.x-a.x)*t, a.y + (b.y-a.y)*t, a.θ + (b.θ-a.θ)*t)

Base.rot180(a::VecSE2) = VecSE2(a.x, a.y, a.θ+π)
Base.rotl90(a::VecSE2) = VecSE2(a.x, a.y, a.θ+0.5π)
Base.rotr90(a::VecSE2) = VecSE2(a.x, a.y, a.θ-0.5π)
rot(a::VecSE2, Δθ::Float64) = VecSE2(a.x, a.y, a.θ+Δθ)

Base.mod2pi(a::VecSE2) = VecSE2(a.x, a.y, mod2pi(a.θ))