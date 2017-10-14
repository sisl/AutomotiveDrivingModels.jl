struct AccelDesang
    a::Float64
    ϕdes::Float64
end
Base.show(io::IO, a::AccelDesang) = @printf(io, "AccelDesang(%6.3f,%6.3f)", a.a, a.ϕdes)
Base.length(::Type{AccelDesang}) = 2
Base.convert(::Type{AccelDesang}, v::Vector{Float64}) = AccelDesang(v[1], v[2])
function Base.copy!(v::Vector{Float64}, a::AccelDesang)
    v[1] = a.a
    v[2] = a.ϕdes
    v
end
