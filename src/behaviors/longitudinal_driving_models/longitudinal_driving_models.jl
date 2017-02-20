export
        LongitudinalDriverModel,
        ProportionalSpeedTracker,
        IntelligentDriverModel,
        StaticLongitudinalDriver,
        PrincetonLongitudinalDriver,
        track_longitudinal!

abstract LongitudinalDriverModel
get_name(::LongitudinalDriverModel) = "???"
set_desired_speed!(::LongitudinalDriverModel, v_des::Float64) = model # # do nothing by default
reset_hidden_state!(model::LongitudinalDriverModel) = model # do nothing by default
track_longitudinal!(model::LongitudinalDriverModel, scene::Scene, roadway::Roadway, ego_index::Int, target_index::Int) = model # do nothing by default
observe!(model::LongitudinalDriverModel, scene::Scene, roadway::Roadway, egoid::Int) = model  # do nothing by default
Base.rand(model::LongitudinalDriverModel) = error("rand not implemented for model $model")
Distributions.pdf(model::LongitudinalDriverModel, a_lon::Float64) = error("pdf not implemented for model $model")
Distributions.logpdf(model::LongitudinalDriverModel, a_lon::Float64) = error("logpdf not implemented for model $model")

################################

type StaticLongitudinalDriver <: LongitudinalDriverModel
    a::Float64
    StaticLongitudinalDriver(a::Float64=0.0) = new(a)
end
get_name(::StaticLongitudinalDriver) = "ProportionalSpeedTracker"
Base.rand(model::StaticLongitudinalDriver) = model.a
Distributions.pdf(model::StaticLongitudinalDriver, a_lon::Float64) = a_lon == model.a ? Inf : 0.0
Distributions.logpdf(model::StaticLongitudinalDriver, a_lon::Float64) = a_lon == model.a ? Inf : -Inf

################################

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
function set_desired_speed!(model::ProportionalSpeedTracker, v_des::Float64)
    model.v_des = v_des
    model
end
function track_longitudinal!(model::ProportionalSpeedTracker, scene::Scene, roadway::Roadway, ego_index::Int, target_index::Int)
    veh_ego = scene[ego_index]
    v = veh_ego.state.v

    Δv = model.v_des - v
    model.a = Δv*model.k # predicted accel to match target speed

    model
end
function observe!(model::ProportionalSpeedTracker, scene::Scene, roadway::Roadway, egoid::Int)
    ego_index = get_index_of_first_vehicle_with_id(scene, egoid)
    track_longitudinal!(model, scene, roadway, ego_index, 0)
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

################################

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
function track_longitudinal!(model::PrincetonLongitudinalDriver, scene::Scene, roadway::Roadway, ego_index::Int, target_index::Int)

    veh_ego = scene[ego_index]
    v_des = model.v_des

    if target_index > 0
        veh_target = scene[target_index]
        speed_M = veh_target.state.v
        dist_M = abs(veh_target.state.posG - veh_ego.state.posG) - veh_target.def.length/2 - veh_ego.def.length/2
        v_des = min(speed_M*(1-exp(-model.k*dist_M/speed_M - 1)), model.v_des)
    end

    Δv = v_des - veh_ego.state.v
    model.a = Δv*model.k # predicted accel to match target speed

    model
end
function observe!(model::PrincetonLongitudinalDriver, scene::Scene, roadway::Roadway, egoid::Int)
    ego_index = get_index_of_first_vehicle_with_id(scene, egoid)
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

################################

"""
Commonly referred to as IDM
"""
@with_kw type IntelligentDriverModel <: LongitudinalDriverModel
    a::Float64 = NaN # predicted acceleration
    σ::Float64 = NaN # optional stdev on top of the model, set to zero or NaN for deterministic behavior

    k_spd::Float64 = 1.0 # proportional constant for speed tracking when in freeflow [s⁻¹]

    δ::Float64 = 4.0 # acceleration exponent [-]
    T::Float64  = 1.5 # desired time headway [s]
    v_des::Float64 = 29.0 # desired speed [m/s]
    s_min::Float64 = 5.0 # minimum acceptable gap [m]
    a_max::Float64 = 3.0 # maximum acceleration ability [m/s²]
    d_cmf::Float64 = 2.0 # comfortable deceleration [m/s²] (positive)
    d_max::Float64 = 9.0 # maximum decelleration [m/s²] (positive)
end
get_name(::IntelligentDriverModel) = "IDM"
function set_desired_speed!(model::IntelligentDriverModel, v_des::Float64)
    model.v_des = v_des
    model
end
function track_longitudinal!(model::IntelligentDriverModel, scene::Scene, roadway::Roadway, ego_index::Int, target_index::Int)
    veh_ego = scene[ego_index]
    v = veh_ego.state.v

    if target_index > 0
        veh_target = scene[target_index]

        s_gap = get_frenet_relative_position(get_rear_center(veh_target),
                                             veh_ego.state.posF.roadind, roadway).Δs

        if s_gap > 0.0
            Δv = veh_target.state.v - v
            s_des = model.s_min + v*model.T - v*Δv / (2*sqrt(model.a_max*model.d_cmf))
            v_ratio = model.v_des > 0.0 ? (v/model.v_des) : 1.0
            model.a = model.a_max * (1.0 - v_ratio^model.δ - (s_des/s_gap)^2)
        elseif s_gap > -veh_ego.def.length
            model.a = -model.d_max
        else
            Δv = model.v_des - v
            model.a = Δv*model.k_spd
        end

        if isnan(model.a)

            warn("IDM acceleration was NaN!")
            if s_gap > 0.0
                Δv = veh_target.state.v - v
                s_des = model.s_min + v*model.T - v*Δv / (2*sqrt(model.a_max*model.d_cmf))
                println("\tΔv: ", Δv)
                println("\ts_des: ", s_des)
                println("\tv_des: ", model.v_des)
                println("\tδ: ", model.δ)
                println("\ts_gap: ", s_gap)
            elseif s_gap > -veh_ego.def.length
                println("\td_max: ", model.d_max)
            end

            model.a = 0.0
        end
    else
        # no lead vehicle, just drive to match desired speed
        Δv = model.v_des - v
        model.a = Δv*model.k_spd # predicted accel to match target speed
    end

    model.a = clamp(model.a, -model.d_max, model.a_max)

    model
end
function observe!(model::IntelligentDriverModel, scene::Scene, roadway::Roadway, egoid::Int)

    # update the predicted accel

    vehicle_index = get_index_of_first_vehicle_with_id(scene, egoid)
    veh_ego = scene[vehicle_index]
    fore_res = get_neighbor_fore_along_lane(scene, vehicle_index, roadway, VehicleTargetPointFront(), VehicleTargetPointRear(), VehicleTargetPointFront())

    track_longitudinal!(model, scene, roadway, vehicle_index, fore_res.ind)
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
