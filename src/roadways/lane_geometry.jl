const DEFAULT_OFFSET_TOLERANCE = 1e-8

# function _get_based_on_s(sequential::Vector, s::Real)
#     # NOTE: only works on things with an 's' field
#     N = length(sequential)
#     @assert N ≥ 1
#     i = 1
#     while i < N && sequential[i+1].s ≤ s
#         i += 1
#     end
#     return sequential[i]
# end

abstract type LaneGeometry end
struct GeomLine <: LaneGeometry
end
struct GeomArc <: LaneGeometry
    κ::Float64 # [1/m], constant curvature throughout the element
               # positive indicates a left-hand turn
end
# struct GeomClothoid <: LaneGeometry
# end
const LaneGeometries = Union{GeomLine,GeomArc}

struct LaneGeometryRecord{G<:LaneGeometries}
    posG::VecSE2 # start position in global coordinates (x,y,θ)
    len::Float64 # length of the element's reference line
    geo::G
    s::Float64 # start position (s-coordinate)

    function LaneGeometryRecord{G}(posG::VecSE2, len::Float64, geo, s::Float64) where {G<:LaneGeometries}
        isfinite(posG) || throw(ArgumentError("LaneGeometryRecord posG must be finite"))
        isfinite(len) || throw(ArgumentError("LaneGeometryRecord length must be finite"))
        len > 0  || throw(ArgumentError("LaneGeometryRecord length must be positive"))
        isfinite(s) || throw(ArgumentError("LaneGeometryRecord s must be finite"))
        s ≥ 0 || throw(ArgumentError("LaneGeometryRecord starting s must be nonnegative"))
        new(posG, len, geo, s)
    end
end
LaneGeometryRecord{G<:LaneGeometries}(posG::VecSE2, len::Real, geo::G) = LaneGeometryRecord{G}(posG, convert(Float64, len), geo, 0.0)
LaneGeometryRecord{G<:LaneGeometries}(posG::VecSE2, len::Real, geo::G, s::Real) = LaneGeometryRecord{G}(posG, convert(Float64, len), geo, convert(Float64, s))

Base.length(lanegeo::LaneGeometryRecord) = lanegeo.len

"""
    get(lanegeo, s)

Returns the VecSE2 in the global frame corresponding to the given s.
Note: s is relative to the start of the overall reference line
Throws an error if s is out of bounds
"""
function Base.get(lanegeo::LaneGeometryRecord{GeomLine}, s::Float64; offset_tol::Float64 = DEFAULT_OFFSET_TOLERANCE)
    Δs = s - lanegeo.s
    -offset_tol ≤ Δs ≤ lanegeo.len + offset_tol || throw(DomainError())
    θ = lanegeo.posG.θ
    return lanegeo.posG + polar(Δs, θ)
end
function Base.get(lanegeo::LaneGeometryRecord{GeomArc}, s::Float64; offset_tol::Float64 = DEFAULT_OFFSET_TOLERANCE)
    Δs = s - lanegeo.s
    -offset_tol ≤ Δs ≤ lanegeo.len + offset_tol || throw(DomainError())
    κ = lanegeo.geo.\kappa
    P = lanegeo.posG
    θ = P.θ
    if isapprox(κ, 0.0)
        # use a straight segment
        return P + polar(Δs, θ)
    else
        r = 1 / κ
        C = P + polar(r, θ + π/2) # center of rotation
        ϕ = Δs / r # rotation about C
        return C + polar(r, θ+ϕ-π/2, ϕ)
    end
end

"""
    proj(posG, lanegeo, Float64)

Returns a tuple (s,⟂) where
s is the s-value of the closest footpoint of posG onto the lanegeo.
    It is the absolute s value, ie not necessarily 0 at the start of the lanegeo

⟂ is true if the projection posG is perpendicular to the lane geo (it is false when off either end).
    If posG is exactly on the line it is considered perpendicular.
"""
function Vec.proj(posG::Union{VecE2, VecSE2}, lanegeo::LaneGeometryRecord{GeomLine}, ::Type{Float64};
    # perp_tolerance::Float64=1e-8,
    )

    isfinite(posG) || throw(ArgumentError("posG must be finite"))

    C = VecE2(lanegeo.posG) # origin
    P = VecE2(posG)
    D = P - C # 2D position wrt to start of lanegeo
    Q = polar(1.0, lanegeo.posG.θ) # line unit ray vector
    Δs = proj(D, Q, Float64) # get distance along

    if Δs < 0.0
        return (lanegeo.s, false)
    elseif Δs > lanegeo.len
        return (lanegeo.s + lanegeo.len, false)
    else
        # F = C+Δs*Q # footpoint
        # ⟂ = isapprox(dot(Q, P-F), 0.0, atol=perp_tolerance) # whether it is perpendicular
        return (lanegeo.s + Δs, true)
    end
end
function Vec.proj(posG::Union{VecE2, VecSE2}, lanegeo::LaneGeometryRecord{GeomArc}, ::Type{Float64};
    # perp_tolerance::Float64=1e-8,
    critical_tolerance::Float64=1e-8,
    warn_on_critical::Bool=false,
    )

    isfinite(posG) || throw(ArgumentError("posG must be finite"))

    κ = lanegeo.geo.\kappa
    if isapprox(κ, 0.0) # treat it as a line
        return proj(posG, LaneGeometryRecord(lanegeo.posG, lanegeo.len, GeomLine(), lanegeo.s))
    end

    r = 1/κ

    O = VecE2(lanegeo.posG) # origin
    P = VecE2(posG)
    C = O + polar(r, lanegeo.posG.θ + π/2)
    CP = P-C

    if isapprox(abs2(CP), 0.0, atol=critical_tolerance) # on the C point
        # This is a critical location, and any ψ is valid
        # We will default to s = lanegeo.s
        return (lanegeo.s, true)
    end

    ψ₀ = atan2(O-C)
    ψ = atan2(CP) - ψ₀ # directed angle from O to P about C
    Δs = r*ψ

    if Δs < 0.0
        return (lanegeo.s, false)
    elseif Δs > lanegeo.len
        return (lanegeo.s + lanegeo.len, false)
    else
        # F = C + polar(r, ψ + ψ₀) # footpoint
        # ⟂ = isapprox(dot(Q, P-F), 0.0, atol=perp_tolerance) # whether it is perpendicular
        return (lanegeo.s + Δs, true)
    end


    Δs = proj(D, Q, Float64) # get distance along
end