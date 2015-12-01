#=
VecE2: a 2d euclidean vector
=#

immutable VecE2 <: VecE
    x :: Float64
    y :: Float64

    VecE2() = new(0.0,0.0)
    VecE2(x::Real, y::Real) = new(x,y)
end

polar(r::Real, θ::Real) = VecE2(r*cos(θ), r*sin(θ))

Base.length(::VecE2) = 2
Base.copy(a::VecE2) = VecE2(a.x, a.y)
Base.convert(::Type{Vector{Float64}}, a::VecE2) = [a.x, a.y]
function Base.convert{R<:Real}(::Type{VecE2}, a::AbstractArray{R})
    @assert(length(a) == 2)
    VecE2(a[1], a[2])
end

Base.(:(+))(a::VecE2, b::Real)  = VecE2(a.x+b, a.y+b)
Base.(:(+))(b::Real,  a::VecE2) = VecE2(a.x+b, a.y+b)
Base.(:(+))(a::VecE2, b::VecE2) = VecE2(a.x+b.x, a.y+b.y)

Base.(:(-))(a::VecE2, b::Real)  = VecE2(a.x-b, a.y-b)
Base.(:(-))(a::VecE2, b::VecE2) = VecE2(a.x-b.x, a.y-b.y)

Base.(:(*))(a::VecE2, b::Real) = VecE2(a.x*b, a.y*b)
Base.(:(*))(b::Real, a::VecE2) = VecE2(a.x*b, a.y*b)

Base.(:(/))(a::VecE2, b::Real) = VecE2(a.x/b, a.y/b)

Base.(:(^))(a::VecE2, b::Integer) = VecE2(a.x^b, a.y^b)
Base.(:(^))(a::VecE2, b::AbstractFloat) = VecE2(a.x^b, a.y^b)

Base.(:(%))(a::VecE2, b::Real) = VecE2(a.x%b, a.y%b)

Base.(:(==))(a::VecE2, b::VecE2) = isequal(a.x, b.x) && isequal(a.y, b.y)
Base.isequal(a::VecE2, b::VecE2) = isequal(a.x, b.x) && isequal(a.y, b.y)

Base.isfinite(a::VecE2) = isfinite(a.x) && isfinite(a.y)
Base.isinf(a::VecE2) = isinf(a.x) || isinf(a.y)
Base.isnan(a::VecE2) = isnan(a.x) || isnan(a.y)

Base.round(a::VecE2) = VecE2(round(a.x), round(a.y))
Base.floor(a::VecE2) = VecE2(floor(a.x), floor(a.y))
Base.ceil(a::VecE2) = VecE2(ceil(a.x), ceil(a.y))
Base.trunc(a::VecE2) = VecE2(trunc(a.x), trunc(a.y))

Base.abs(a::VecE2) = hypot(a.x, a.y)
Base.hypot(a::VecE2) = hypot(a.x, a.y)
Base.abs2(a::VecE2) = a.x*a.x + a.y*a.y
function Base.norm(a::VecE2)
    m = abs(a)
    VecE2(a.x/m, a.y/m)
end

Base.atan2(a::VecE2) = atan2(a.y, a.x)

dist(a::VecE2, b::VecE2) = hypot(a.x-b.x, a.y-b.y)
function dist2(a::VecE2, b::VecE2)
    Δx = a.x-b.x
    Δy = a.y-b.y
    Δx*Δx + Δy*Δy
end

Base.dot(a::VecE2, b::VecE2) = a.x*b.x + a.y*b.y
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