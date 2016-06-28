function _draw_action_from_MvNormal!(a::Vector{Float64}, P::MvNormal)
    # this assumes it is 2x2

    μ::Vector{Float64} = P.μ
    Σ::Matrix{Float64} = P.Σ.mat

    a[1] = randn()*sqrt(Σ[1,1]) + μ[1]

    # compute conditional values
    μ₂ = μ[2] + Σ[1,2]*(a[1] - μ[1])/Σ[1,1]
    Σ₂ = Σ[2,2] - Σ[1,2]*Σ[1,2]/Σ[1,1]
    a[2] = randn()*sqrt(Σ₂) + μ₂

    a
end
function _get_e_and_denom(P::MvNormal, a::Vector{Float64})

    μ::Vector{Float64} = P.μ
    Σ::Matrix{Float64} = P.Σ.mat

    Δx = a[1] - μ[1]
    Δy = a[2] - μ[2]
    det = Σ[1,1]*Σ[2,2] - Σ[1,2]*Σ[1,2]

    e = exp(-0.5*((Σ[2,2]*Δx - Σ[1,2]*Δy)*Δx + (Σ[1,1]*Δy - Σ[1,2]*Δx)*Δy) / det)
    denom = sqrt(4*π*π*det)

    (e,denom)
end
function _pdf_of_MvNormal(P::MvNormal{PDMats.PDMat{Float64,Matrix{Float64}},Vector{Float64}}, a::Vector{Float64})
    # this assumes it is 2x2

    e, denom = _get_e_and_denom(P, a)
    e / denom
end
function _logpdf_of_MvNormal(P::MvNormal{PDMats.PDMat{Float64,Matrix{Float64}},Vector{Float64}}, a::Vector{Float64})
    # this assumes it is 2x2

    e, denom = _get_e_and_denom(P, a)
    log(e) - log(denom)
end