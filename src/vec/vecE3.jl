#=
VecE3: a 3d euclidean vector
=#


struct VecE3{R<:Real} <: VecE{3,R}
    x::R
    y::R
    z::R
end
VecE3() = VecE3(0.0,0.0,0.0)
VecE3(x::Integer, y::Integer, z::Integer) = VecE3(float(x), float(y), float(z))
VecE3(t::Tuple) = VecE3(promote(t...)...)

Base.show(io::IO, a::VecE3) = @printf(io, "VecE3(%.3f, %.3f, %.3f)", a.x, a.y, a.z)

function dist(a::VecE3, b::VecE3)
    Δx = a.x-b.x
    Δy = a.y-b.y
    Δz = a.z-b.z
    sqrt(Δx*Δx + Δy*Δy + Δz*Δz)
end
function dist2(a::VecE3, b::VecE3)
    Δx = a.x-b.x
    Δy = a.y-b.y
    Δz = a.z-b.z
    Δx*Δx + Δy*Δy + Δz*Δz
end

proj(a::VecE3, b::VecE3, ::Type{Float64}) = (a.x*b.x + a.y*b.y + a.z*b.z) / sqrt(b.x*b.x + b.y*b.y + b.z*b.z) # dot(a,b) / |b|
function proj(a::VecE3, b::VecE3, ::Type{VecE3})
    # dot(a,b) / dot(b,b) ⋅ b
    s = (a.x*b.x + a.y*b.y + a.z*b.z) / (b.x*b.x + b.y*b.y + b.z*b.z)
    VecE3(s*b.x, s*b.y, s*b.z)
end

lerp(a::VecE3, b::VecE3, t::Real) = VecE3(a.x + (b.x-a.x)*t, a.y + (b.y-a.y)*t, a.z + (b.z-a.z)*t)

function rot(a::VecE3, axis::VecE3, θ::Real)

    #=
    rotate the point a: (x,y,z) about the axis: (u,v,w) by the angle θ following the
    right hand rule
    =#

    x,y,z = a.x, a.y, a.z
    u,v,w = axis.x, axis.y, axis.z

    u² = u*u
    v² = v*v
    w² = w*w

    m = u² + v² + w²

    c = cos(θ)
    s = sin(θ)
    mc = 1.0 - c

    rtms = sqrt(m)*s

    magc = m*c
    ux_vy_wz_mc = (u*x + v*y + w*z)*mc

    new_x = (u*ux_vy_wz_mc + x*magc + rtms*(-w*y + v*z)) / m
    new_y = (v*ux_vy_wz_mc + y*magc + rtms*( w*x - u*z)) / m
    new_z = (w*ux_vy_wz_mc + z*magc + rtms*(-v*x + u*y)) / m

    VecE3(new_x, new_y, new_z)
end
function rot_normalized(a::VecE3, axis::VecE3, θ::Real)

    #=
    rotate the point a: (x,y,z) about the axis: (u,v,w) by the angle θ following the
    right hand rule

    Here we assume axis is normalized
    =#

    x,y,z = a.x, a.y, a.z
    u,v,w = axis.x, axis.y, axis.z

    u² = u*u
    v² = v*v
    w² = w*w

    m = u² + v² + w²

    c = cos(θ)
    s = sin(θ)
    mc = 1.0 - c

    ux_vy_wz_mc = (u*x + v*y + w*z)*mc

    new_x = u*ux_vy_wz_mc + x*c + (-w*y + v*z)*s
    new_y = v*ux_vy_wz_mc + y*c + ( w*x - u*z)*s
    new_z = w*ux_vy_wz_mc + z*c + (-v*x + u*y)*s

    VecE3(new_x, new_y, new_z)
end
