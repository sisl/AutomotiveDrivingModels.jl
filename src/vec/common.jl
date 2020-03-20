const DUMMY_PRECISION = 1e-12

Base.isfinite(a::AbstractVec) = all(isfinite, a)
Base.isinf(a::AbstractVec) = any(isinf, a)
Base.isnan(a::AbstractVec) = any(isnan, a)

function StaticArrays.similar_type(::Type{V}, ::Type{F}, size::Size{N}) where {V<:AbstractVec, F<:AbstractFloat, N}
    if size == Size(V) && eltype(V) == F
        return V
    else # convert back to SArray
        return SArray{Tuple{N...},F,length(size),prod(size)}
    end
end

Base.abs(a::AbstractVec) = error("abs(v::AbstractVec) has been removed. Use norm(v) to get the norm; abs.(v) to get the element-wise absolute value.")
