export
        LateralDriverModel

abstract LateralDriverModel
get_name(::LateralDriverModel) = "???"
reset_hidden_state!(model::LateralDriverModel) = model # do nothing by default
observe!(model::LateralDriverModel, scene::Scene, roadway::Roadway, egoid::Int) = model  # do nothing by default
Base.rand(model::LateralDriverModel) = error("rand not implemented for model $model")
Distributions.pdf(model::LateralDriverModel, a_lon::Float64) = error("pdf not implemented for model $model")
Distributions.logpdf(model::LateralDriverModel, a_lon::Float64) = error("logpdf not implemented for model $model")

type ProportionalLaneTracker <: LateralDriverModel
    a::Float64 # predicted acceleration
    σ::Float64 # optional stdev on top of the model, set to zero or NaN for deterministic behavior
    k::Float64 # proportional constant for lane tracking

    function ProportionalLaneTracker(;
        σ::Float64     = NaN,
        k_spd::Float64 = 1.0,
        )

        retval = new()
        retval.a = NaN
        retval.σ = σ
        retval.k_spd = k_spd
        retval
    end
end
get_name(::ProportionalLaneTracker) = "ProportionalLaneTracker"
function observe!(model::ProportionalLaneTracker, scene::Scene, roadway::Roadway, egoid::Int)

    ego_index = get_index_of_first_vehicle_with_id(scene, carid)
    veh_ego = scene[ego_index]
    t = veh_ego.state.posF.t
    model.a = -t*model.k

    model
end
function Base.rand(model::ProportionalLaneTracker)
    if isnan(model.σ) || model.σ ≤ 0.0
        model.a
    else
        rand(Normal(model.a, model.σ))
    end
end
function Distributions.pdf(model::ProportionalLaneTracker, a_lat::Float64)
    if isnan(model.σ) || model.σ ≤ 0.0
        Inf
    else
        pdf(Normal(model.a, model.σ), a_lon)
    end
end
function Distributions.logpdf(model::ProportionalLaneTracker, a_lat::Float64)
    if isnan(model.σ) || model.σ ≤ 0.0
        Inf
    else
        logpdf(Normal(model.a, model.σ), a_lon)
    end
end

