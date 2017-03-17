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
track_longitudinal!(model::LongitudinalDriverModel, v_ego::Float64, v_oth::Float64, headway::Float64) = model # do nothing by default
Base.rand(model::LongitudinalDriverModel) = error("rand not implemented for model $model")
Distributions.pdf(model::LongitudinalDriverModel, a_lon::Float64) = error("pdf not implemented for model $model")
Distributions.logpdf(model::LongitudinalDriverModel, a_lon::Float64) = error("logpdf not implemented for model $model")


function track_longitudinal!(model::LongitudinalDriverModel, scene::Scene, roadway::Roadway, ego_index::Int, target_index::Int)
    veh_ego = scene[ego_index]
    v_ego = veh_ego.state.v
    v_oth = NaN
    headway = NaN

    if target_index > 0 && target_index != ego_index
        veh_target = scene[target_index]
        s_gap = get_frenet_relative_position(get_rear_center(veh_target), veh_ego.state.posF.roadind, roadway).Δs
        if s_gap > 0.0
            headway = s_gap
            v_oth = scene[target_index].state.v
        end
    end

    return track_longitudinal!(model, v_ego, v_oth, headway)
end
function observe!(model::LongitudinalDriverModel, scene::Scene, roadway::Roadway, egoid::Int)

    vehicle_index = get_index_of_first_vehicle_with_id(scene, egoid)
    veh_ego = scene[vehicle_index]
    fore_res = get_neighbor_fore_along_lane(scene, vehicle_index, roadway, VehicleTargetPointFront(), VehicleTargetPointRear(), VehicleTargetPointFront())
    track_longitudinal!(model, scene, roadway, vehicle_index, fore_res.ind)

    return model
end

include("static_longitudinal_drivers.jl")
include("proportional_speed_trackers.jl")
include("princeton_longitudinal_drivers.jl")
include("intelligent_driver_models.jl")