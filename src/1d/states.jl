immutable State1D
    s::Float64 # position
    v::Float64 # speed [m/s]
end
Base.write(io::IO, ::MIME"text/plain", s::State1D) = @printf(io, "%.16e %.16e", s.s, s.v)
function Base.read(io::IO, ::MIME"text/plain", ::Type{State1D})
    i = 0
    tokens = split(strip(readline(io)), ' ')
    s = parse(Float64, tokens[i+=1])
    v = parse(Float64, tokens[i+=1])
    return State1D(s,v)
end