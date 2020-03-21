#=
VecE2: a 2d euclidean vector
=#

struct VecE2{R<:Real} <: VecE{2,R}
    x::R
    y::R
end
VecE2() = VecE2(0.0,0.0)
VecE2(x::Integer, y::Integer) = VecE2(float(x), float(y))
VecE2(t::Tuple) = VecE2(promote(t...)...)

polar(r::Real, θ::Real) = VecE2(r*cos(θ), r*sin(θ))

Base.show(io::IO, a::VecE2) = @printf(io, "VecE2(%.3f, %.3f)", a.x, a.y)

Base.atan(a::VecE2) = atan(a.y, a.x)

dist(a::VecE2, b::VecE2) = hypot(a.x-b.x, a.y-b.y)
function dist2(a::VecE2, b::VecE2)
    Δx = a.x-b.x
    Δy = a.y-b.y
    Δx*Δx + Δy*Δy
end

proj(a::VecE2, b::VecE2, ::Type{Float64}) = (a.x*b.x + a.y*b.y) / hypot(b.x, b.y) # dot(a,b) / |b|
function proj(a::VecE2, b::VecE2, ::Type{VecE2})
    # dot(a,b) / dot(b,b) ⋅ b
    s = (a.x*b.x + a.y*b.y) / (b.x*b.x + b.y*b.y)
    VecE2(s*b.x, s*b.y)
end

lerp(a::VecE2, b::VecE2, t::Real) = VecE2(a.x + (b.x-a.x)*t, a.y + (b.y-a.y)*t)

Base.rot180(a::VecE2) = VecE2(-a.x, -a.y)
Base.rotl90(a::VecE2) = VecE2(-a.y,  a.x)
Base.rotr90(a::VecE2) = VecE2( a.y, -a.x)
function rot(a::VecE2, θ::Float64)

    #=
    Rotates counter-clockwise about origin
    =#

    c = cos(θ)
    s = sin(θ)

    VecE2(a.x*c - a.y*s, a.x*s+a.y*c)
end
