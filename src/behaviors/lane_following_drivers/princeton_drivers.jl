@with_kw mutable struct PrincetonDriver <: LaneFollowingDriver
    a::Float64 = NaN # predicted acceleration
    k::Float64 = 1.0 # proportional constant for speed tracking [s⁻¹]
    v_des::Float64 = 29.0 # desired speed [m/s]
end
get_name(::PrincetonDriver) = "PrincetonDriver"
function set_desired_speed!(model::PrincetonDriver, v_des::Float64)
    model.v_des = v_des
    model
end
function track_longitudinal!(model::PrincetonDriver, v_ego::Float64, v_oth::Float64, headway::Float64)

    v_des = model.v_des
    k = model.k

    if !isnan(v_oth)
        v_des = min(v_oth*(1-exp(-k*headway/v_oth - 1)), v_des)
    end

    Δv = v_des - v_ego
    model.a = Δv*k # predicted accel to match target speed

    model
end
Base.rand(model::PrincetonDriver) = Accel(model.a)
Distributions.pdf(model::PrincetonDriver, a::Accel) = isapprox(a.a, model.a) ? Inf : 0.0
Distributions.logpdf(model::PrincetonDriver, a::Accel) = isapprox(a.a, model.a) ? Inf : -Inf
