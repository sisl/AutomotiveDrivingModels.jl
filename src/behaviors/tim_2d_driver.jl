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
@with_kw mutable struct Tim2DDriver <: DriverModel{LatLonAccel}
    mlon::LaneFollowingDriver = IntelligentDriverModel()
    mlat::LateralDriverModel = ProportionalLaneTracker()
    mlane::LaneChangeModel = TimLaneChanger()
end

function set_desired_speed!(model::Tim2DDriver, v_des::Float64)
    set_desired_speed!(model.mlon, v_des)
    set_desired_speed!(model.mlane, v_des)
    model
end

function track_longitudinal!(driver::LaneFollowingDriver, scene::Scene{Entity{VehicleState, D, I}}, roadway::Roadway, vehicle_index::Int64, fore::NeighborLongitudinalResult) where {D, I}
    v_ego = vel(scene[vehicle_index].state)
    if fore.ind != nothing
        headway, v_oth = fore.Δs, vel(scene[fore.ind].state)
    else
        headway, v_oth = NaN, NaN
    end
    return track_longitudinal!(driver, v_ego, v_oth, headway)
end

function observe!(driver::Tim2DDriver, scene::Scene{Entity{S, D, I}}, roadway::Roadway, egoid::I) where {S, D, I}

    observe!(driver.mlane, scene, roadway, egoid)

    vehicle_index = findfirst(egoid, scene)
    ego = scene[vehicle_index]
    lane_change_action = rand(driver.mlane)

    laneoffset = get_lane_offset(lane_change_action, scene, roadway, vehicle_index)
    lateral_speed = velf(scene[vehicle_index]).t

    if lane_change_action.dir == DIR_MIDDLE
        target_lane = get_lane(roadway, ego)
    elseif lane_change_action.dir == DIR_LEFT
        target_lane = leftlane(roadway, ego)
    else
        @assert(lane_change_action.dir == DIR_RIGHT)
        target_lane = rightlane(roadway, ego)
    end
    fore = find_neighbor(scene, roadway, ego, 
                        lane=target_lane, 
                        targetpoint_ego=VehicleTargetPointFront(), 
                        targetpoint_neighbor=VehicleTargetPointRear())

    track_lateral!(driver.mlat, laneoffset, lateral_speed)
    track_longitudinal!(driver.mlon, scene, roadway, vehicle_index, fore)

    driver
end

Base.rand(rng::AbstractRNG, driver::Tim2DDriver) = LatLonAccel(rand(rng, driver.mlat), rand(rng, driver.mlon).a)

Distributions.pdf(driver::Tim2DDriver, a::LatLonAccel) = pdf(driver.mlat, a.a_lat) * pdf(driver.mlon, a.a_lon)

Distributions.logpdf(driver::Tim2DDriver, a::LatLonAccel) = logpdf(driver.mlat, a.a_lat) * logpdf(driver.mlon, a.a_lon)
