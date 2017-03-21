type PrincetonLongitudinalDriver <: LongitudinalDriverModel
    a::Float64 # predicted acceleration
    σ::Float64 # optional stdev on top of the model, set to zero or NaN for deterministic behavior
    k::Float64 # proportional constant for speed tracking [s⁻¹]
    v_des::Float64 # desired speed [m/s]

    function PrincetonLongitudinalDriver(;
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
get_name(::PrincetonLongitudinalDriver) = "PrincetonLongitudinalDriver"
function set_desired_speed!(model::PrincetonLongitudinalDriver, v_des::Float64)
    model.v_des = v_des
    model
end
function track_longitudinal!(model::PrincetonLongitudinalDriver, v_ego::Float64, v_oth::Float64, headway::Float64)

    v_des = model.v_des
    k = model.k

    if !isnan(v_oth)
        v_des = min(v_oth*(1-exp(-k*headway/v_oth - 1)), v_des)
    end

    Δv = v_des - v_ego
    model.a = Δv*k # predicted accel to match target speed

    model
end
function observe!(model::PrincetonLongitudinalDriver, scene::Scene, roadway::Roadway, egoid::Int)
    ego_index = findfirst(scene, egoid)
    track_longitudinal!(model, scene, roadway, ego_index, 0)
    model
end
function Base.rand(model::PrincetonLongitudinalDriver)
    if isnan(model.σ) || model.σ ≤ 0.0
        model.a
    else
        rand(Normal(model.a, model.σ))
    end
end
function Distributions.pdf(model::PrincetonLongitudinalDriver, a_lon::Float64)
    if isnan(model.σ) || model.σ ≤ 0.0
        Inf
    else
        pdf(Normal(model.a, model.σ), a_lon)
    end
end
function Distributions.logpdf(model::PrincetonLongitudinalDriver, a_lon::Float64)
    if isnan(model.σ) || model.σ ≤ 0.0
        Inf
    else
        logpdf(Normal(model.a, model.σ), a_lon)
    end
end
