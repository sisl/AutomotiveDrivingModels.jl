Base.write(io::IO, ::MIME"text/plain", ::Nothing) = nothing
function Base.read(io::IO, ::MIME"text/plain", ::Nothing)
    readline(io)
    return nothing
end

Base.write(io::IO, ::MIME"text/plain", i::Integer) = print(io, i)
Base.read(io::IO, ::MIME"text/plain", ::Type{I}) where {I<:Integer} = parse(I, readline(io))

Base.write(io::IO, ::MIME"text/plain", r::Float64) = print(io, r)
Base.read(io::IO, ::MIME"text/plain", ::Type{F}) where {F<:AbstractFloat} = parse(F, readline(io))