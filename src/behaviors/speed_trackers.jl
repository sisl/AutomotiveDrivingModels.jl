"""
    ProportionalSpeedTracker <: LaneFollowingDriver
Longitudinal proportional speed control.

# Fields
- `a::Float64 = NaN`  predicted acceleration
- `σ::Float64 = NaN` optional stdev on top of the model, set to zero or NaN for deterministic behavior
- `k::Float64 = 1.0` proportional constant for speed tracking [s⁻¹]
- `v_des::Float64 = 29.0`  desired speed [m/s]
"""
@with_kw mutable struct ProportionalSpeedTracker <: LaneFollowingDriver
    a::Float64 = NaN # predicted acceleration
    σ::Float64 = NaN# optional stdev on top of the model, set to zero or NaN for deterministic behavior
    k::Float64 = 1.0# proportional constant for speed tracking [s⁻¹]
    v_des::Float64 = 29.0 # desired speed [m/s]
end
get_name(::ProportionalSpeedTracker) = "ProportionalSpeedTracker"
function set_desired_speed!(model::ProportionalSpeedTracker, v_des::Float64)
    model.v_des = v_des
    model
end
function track_longitudinal!(model::ProportionalSpeedTracker, v_ego::Float64, v_oth::Float64, headway::Float64)
    Δv = model.v_des - v_ego
    model.a = Δv*model.k # predicted accel to match target speed
    return model
end
function Base.rand(model::ProportionalSpeedTracker)
    if isnan(model.σ) || model.σ ≤ 0.0
        LaneFollowingAccel(model.a)
    else
        LaneFollowingAccel(rand(Normal(model.a, model.σ)))
    end
end
function Distributions.pdf(model::ProportionalSpeedTracker, a::LaneFollowingAccel)
    if isnan(model.σ) || model.σ ≤ 0.0
        Inf
    else
        pdf(Normal(model.a, model.σ), a.a)
    end
end
function Distributions.logpdf(model::ProportionalSpeedTracker, a::LaneFollowingAccel)
    if isnan(model.σ) || model.σ ≤ 0.0
        Inf
    else
        logpdf(Normal(model.a, model.σ), a.a)
    end
end