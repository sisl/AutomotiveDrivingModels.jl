immutable Frenet
    s::Float64 # distance along lane
    t::Float64 # lane offset, positive is to left
    ϕ::Float64 # lane relative heading, zero sideslip (sometimes called point mass model)
end

Base.show(io::IO, frenet::Frenet) = print(io, @sprintf("Frenet(%.3f, %.3f, %.3f)", frenet.s, frenet.t, frenet.ϕ))
function Base.isapprox(a::Frenet, b::Frenet;
    rtol::Real=cbrt(eps(Float64)),
    atol::Real=sqrt(eps(Float64))
    )

    return isapprox(a.s, b.s, atol=atol, rtol=rtol) &&
           isapprox(a.t, b.t, atol=atol, rtol=rtol) &&
           isapprox(a.ϕ, b.ϕ, atol=atol, rtol=rtol)
end

Base.write(io::IO, ::MIME"text/plain", frenet::Frenet) = @printf(io, "%.16e %.16e %.16e", frenet.s, frenet.t, frenet.ϕ)
function Base.read(io::IO, ::MIME"text/plain", ::Type{Frenet})
    tokens = split(strip(readline(io)), ' ')
    s = parse(Float64, tokens[1])
    t = parse(Float64, tokens[2])
    ϕ = parse(Float64, tokens[3])
    return Frenet(s,t,ϕ)
end