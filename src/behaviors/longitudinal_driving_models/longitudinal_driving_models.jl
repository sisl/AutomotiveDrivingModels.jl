export
        LongitudinalDriverModel,
        ProportionalSpeedTracker,
        IntelligentDriverModel

abstract LongitudinalDriverModel
get_name(::LongitudinalDriverModel) = "???"
reset_hidden_state!(model::LongitudinalDriverModel) = model # do nothing by default
observe!(model::LongitudinalDriverModel, scene::Scene, roadway::Roadway, egoid::Int) = model  # do nothing by default
Base.rand(model::LongitudinalDriverModel) = error("rand not implemented for model $model")
Distributions.pdf(model::LongitudinalDriverModel, a_lon::Float64) = error("pdf not implemented for model $model")
Distributions.logpdf(model::LongitudinalDriverModel, a_lon::Float64) = error("logpdf not implemented for model $model")

type ProportionalSpeedTracker <: LongitudinalDriverModel
    a::Float64 # predicted acceleration
    σ::Float64 # optional stdev on top of the model, set to zero or NaN for deterministic behavior
    k::Float64 # proportional constant for speed tracking [s⁻¹]
    v_des::Float64 # desired speed [m/s]

    function ProportionalSpeedTracker(;
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
get_name(::ProportionalSpeedTracker) = "ProportionalSpeedTracker"
function observe!(model::ProportionalSpeedTracker, scene::Scene, roadway::Roadway, egoid::Int)

    ego_index = get_index_of_first_vehicle_with_id(scene, egoid)
    veh_ego = scene[ego_index]
    v = veh_ego.state.v

    Δv = model.v_des - v
    model.a = Δv*model.k # predicted accel to match target speed

    model
end
function Base.rand(model::ProportionalSpeedTracker)
    if isnan(model.σ) || model.σ ≤ 0.0
        model.a
    else
        rand(Normal(model.a, model.σ))
    end
end
function Distributions.pdf(model::ProportionalSpeedTracker, a_lon::Float64)
    if isnan(model.σ) || model.σ ≤ 0.0
        Inf
    else
        pdf(Normal(model.a, model.σ), a_lon)
    end
end
function Distributions.logpdf(model::ProportionalSpeedTracker, a_lon::Float64)
    if isnan(model.σ) || model.σ ≤ 0.0
        Inf
    else
        logpdf(Normal(model.a, model.σ), a_lon)
    end
end



"""
Commonly referred to as IDM
"""
type IntelligentDriverModel <: LongitudinalDriverModel
    a::Float64 # predicted acceleration
    σ::Float64 # optional stdev on top of the model, set to zero or NaN for deterministic behavior

    k_spd::Float64 # proportional constant for speed tracking when in freeflow [s⁻¹]

    δ::Float64 # acceleration exponent [-]
    T::Float64 # desired time headway [s]
    v_des::Float64 # desired speed [m/s]
    s_min::Float64 # minimum acceptable gap [m]
    a_max::Float64 # maximum acceleration ability [m/s²]
    d_cmf::Float64 # comfortable deceleration [m/s²] (positive)

    function IntelligentDriverModel(;
        σ::Float64     =     NaN,
        k_spd::Float64 =   1.0,
        δ::Float64     =   4.0,
        T::Float64     =   1.5,
        v_des::Float64 =  29.0, # typically overwritten
        s_min::Float64 =   2.0,
        a_max::Float64 =   3.0,
        d_cmf::Float64 =   2.0,
        )

        retval = new()
        retval.a = NaN
        retval.k_spd = k_spd
        retval.k_lat = k_lat
        retval.δ     = δ
        retval.T     = T
        retval.v_des = v_des
        retval.s_min = s_min
        retval.a_max = a_max
        retval.d_cmf = d_cmf
        retval
    end
end
get_name(::IntelligentDriverModel) = "IDM"
function observe!(model::IntelligentDriverModel, scene::Scene, roadway::Roadway, egoid::Int)

    # update the predicted accel

    # ego_index = get_index_of_first_vehicle_with_id(scene, carid)
    # veh_ego = scene[ego_index]
    # v = veh_ego.state.v

    # ind_fore = get_neighbor_index_fore(scene, ego_index)
    # if ind_fore > 0
    #     veh_fore = scene[ind_fore]
    #     s_gap = get_headway_dist_between(veh_ego, veh_fore)

    #     Δv = veh_fore.state.v - v
    #     s_des = model.s_min + v*model.T - v*Δv / (2*sqrt(model.a_max*model.d_cmf))
    #     a_idm = model.a_max * (1.0 - (v/model.v_des)^model.δ - (s_des/s_gap)^2)
    #     @assert(!isnan(a_idm))

    #     model.P.μ[1] = a_idm
    # else
    #     # no lead vehicle, just drive to match desired speed
    #     Δv = model.v_des - v
    #     model.a = Δv*model.k_spd # predicted accel to match target speed
    # end

    # model.a = clamp(model.a, -model.d_cmf, model.a_max)
    model.a = 0.0 # TODO: implement this
    model
end
function Base.rand(model::IntelligentDriverModel)
    if isnan(model.σ) || model.σ ≤ 0.0
        model.a
    else
        rand(Normal(model.a, model.σ))
    end
end
function Distributions.pdf(model::IntelligentDriverModel, a_lon::Float64)
    if isnan(model.σ) || model.σ ≤ 0.0
        Inf
    else
        pdf(Normal(model.a, model.σ), a_lon)
    end
end
function Distributions.logpdf(model::IntelligentDriverModel, a_lon::Float64)
    if isnan(model.σ) || model.σ ≤ 0.0
        Inf
    else
        logpdf(Normal(model.a, model.σ), a_lon)
    end
end
