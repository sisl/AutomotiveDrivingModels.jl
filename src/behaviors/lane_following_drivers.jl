abstract type LaneFollowingDriver <: DriverModel{LaneFollowingAccel} end
track_longitudinal!(model::LaneFollowingDriver, v_ego::Float64, v_oth::Float64, headway::Float64) = model # do nothing by default

function observe!(model::LaneFollowingDriver, scene::Frame{Entity{S, D, I}}, roadway::Roadway, egoid::I) where {S, D, I}

    ego = get_by_id(scene, egoid)

    fore = findneighbor(scene, roadway, ego, targetpoint_ego=VehicleTargetPointFront(), targetpoint_neighbor=VehicleTargetPointRear())

    v_ego = vel(ego.state)
    v_oth = NaN
    headway = NaN

    if fore.ind != nothing
        v_oth = vel(scene[fore.ind].state)
        headway = fore.Δs
    end

    track_longitudinal!(model, v_ego, v_oth, headway)

    return model
end

mutable struct StaticLaneFollowingDriver <: LaneFollowingDriver
    a::LaneFollowingAccel
end
StaticLaneFollowingDriver() = StaticLaneFollowingDriver(LaneFollowingAccel(0.0))
StaticLaneFollowingDriver(a::Float64) = StaticLaneFollowingDriver(LaneFollowingAccel(a))
get_name(::StaticLaneFollowingDriver) = "ProportionalSpeedTracker"
Base.rand(rng::AbstractRNG, model::StaticLaneFollowingDriver) = model.a
Distributions.pdf(model::StaticLaneFollowingDriver, a::LaneFollowingAccel) = isapprox(a.a, model.a.a) ? Inf : 0.0
Distributions.logpdf(model::StaticLaneFollowingDriver, a::LaneFollowingAccel) = isapprox(a.a, model.a.a) ? Inf : -Inf
