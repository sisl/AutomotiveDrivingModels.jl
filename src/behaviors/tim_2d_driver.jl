"""
	Tim2DDriver
Driver that combines longitudinal driver and lateral driver into one model.

# Constructors
	Tim2DDriver(timestep::Float64;mlon::LaneFollowingDriver=IntelligentDriverModel(), mlat::LateralDriverModel=ProportionalLaneTracker(), mlane::LaneChangeModel=TimLaneChanger(timestep))

# Fields
- `mlon::LaneFollowingDriver = IntelligentDriverModel()` Longitudinal driving model
- `mlat::LateralDriverModel = ProportionalLaneTracker()` Lateral driving model
- `mlane::LaneChangeModel =TimLaneChanger` Lane change model
"""
mutable struct Tim2DDriver <: DriverModel{LatLonAccel}
    mlon::LaneFollowingDriver
    mlat::LateralDriverModel
    mlane::LaneChangeModel
end
function Tim2DDriver(
        timestep::Float64;
        mlon::LaneFollowingDriver=IntelligentDriverModel(),
        mlat::LateralDriverModel=ProportionalLaneTracker(),
        mlane::LaneChangeModel=TimLaneChanger(timestep),
        )
    return Tim2DDriver(mlon, mlat, mlane)
end

get_name(::Tim2DDriver) = "Tim2DDriver"
function set_desired_speed!(model::Tim2DDriver, v_des::Float64)
    set_desired_speed!(model.mlon, v_des)
    set_desired_speed!(model.mlane, v_des)
    model
end
function track_longitudinal!(driver::LaneFollowingDriver, scene::Frame{Entity{VehicleState, D, I}}, roadway::Roadway, vehicle_index::Int64, fore::NeighborLongitudinalResult) where {D, I}
    v_ego = vel(scene[vehicle_index])
    if fore.ind != nothing
        headway, v_oth = fore.Δs, vel(scene[fore.ind].state)
    else
        headway, v_oth = NaN, NaN
    end
    return track_longitudinal!(driver, v_ego, v_oth, headway)
end
function observe!(driver::Tim2DDriver, scene::Frame{Entity{S, D, I}}, roadway::Roadway, egoid::I) where {S, D, I}

    observe!(driver.mlane, scene, roadway, egoid)

    lane_change_action = rand(driver.mlane)
    ego = get_by_id(scene, egoid)
    vehicle_index = findfirst(egoid, scene)

    if lane_change_action.dir == DIR_MIDDLE
        fore = get_neighbor_fore_along_lane(scene, vehicle_index, roadway, VehicleTargetPointFront(), VehicleTargetPointRear(), VehicleTargetPointFront())
    elseif lane_change_action.dir == DIR_LEFT
        fore = get_neighbor_fore_along_left_lane(scene, vehicle_index, roadway, VehicleTargetPointFront(), VehicleTargetPointRear(), VehicleTargetPointFront())
    else
        @assert(lane_change_action.dir == DIR_RIGHT)
        fore = get_neighbor_fore_along_right_lane(scene, vehicle_index, roadway, VehicleTargetPointFront(), VehicleTargetPointRear(), VehicleTargetPointFront())
    end

    track_lateral!(driver.mlat, posf(ego).t, velf(ego).t)
    track_longitudinal!(driver.mlon, scene, roadway, vehicle_index, fore)

    driver
end
Base.rand(rng::AbstractRNG, driver::Tim2DDriver) = LatLonAccel(rand(rng, driver.mlat), rand(rng, driver.mlon).a)
Distributions.pdf(driver::Tim2DDriver, a::LatLonAccel) = pdf(driver.mlat, a.a_lat) * pdf(driver.mlon, a.a_lon)
Distributions.logpdf(driver::Tim2DDriver, a::LatLonAccel) = logpdf(driver.mlat, a.a_lat) * logpdf(driver.mlon, a.a_lon)
