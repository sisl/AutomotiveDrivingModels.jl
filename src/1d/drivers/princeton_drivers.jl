type PrincetonDriver <: LaneFollowingDriver
    a::Float64 # predicted acceleration
    σ::Float64 # optional stdev on top of the model, set to zero or NaN for deterministic behavior
    k::Float64 # proportional constant for speed tracking [s⁻¹]
    v_des::Float64 # desired speed [m/s]

    function PrincetonDriver(;
        σ::Float64 = NaN,
        k::Float64 = 1.0,
        v_des::Float64 = 29.0,
        )

        retval = new()
        retval.a = NaN
        retval.σ = σ
        retval.k = k
        retval.v_des = v_des
        retval
    end
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
function Base.rand(model::PrincetonDriver)
    if isnan(model.σ) || model.σ ≤ 0.0
        LaneFollowingAccel(model.a)
    else
        LaneFollowingAccel(rand(Normal(model.a, model.σ)))
    end
end
function Distributions.pdf(model::PrincetonDriver, a::LaneFollowingAccel)
    if isnan(model.σ) || model.σ ≤ 0.0
        Inf
    else
        pdf(Normal(model.a, model.σ), a.a)
    end
end
function Distributions.logpdf(model::PrincetonDriver, a::LaneFollowingAccel)
    if isnan(model.σ) || model.σ ≤ 0.0
        Inf
    else
        logpdf(Normal(model.a, model.σ), a.a)
    end
end
