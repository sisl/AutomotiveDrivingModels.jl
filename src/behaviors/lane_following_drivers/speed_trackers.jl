@with_kw mutable struct ProportionalSpeedTracker <: LaneFollowingDriver
    a::Float64 = NaN # predicted acceleration
    k::Float64 = 1.0 # proportional constant for speed tracking [s⁻¹]
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
Base.rand(model::ProportionalSpeedTracker) = Accel(model.a)
Distributions.pdf(model::ProportionalSpeedTracker, a::Accel) = isapprox(a.a, model.a) ? Inf : 0.0
Distributions.logpdf(model::ProportionalSpeedTracker, a::Accel) = isapprox(a.a, model.a) ? Inf : -Inf