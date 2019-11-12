struct LatLonSeparableDriver{A} <: DriverModel{A}
    mlat::LateralDriverModel
    mlon::LaneFollowingDriver
end

LatLonSeparableDriver(mlat::LateralDriverModel, mlon::LaneFollowingDriver) = LatLonSeparableDriver{LatLonAccel}(mlat, mlon)

function observe!(model::LaneFollowingDriver, scene::Frame{Entity{S, D, I}}, roadway::Roadway, egoid::I) where {S, D, I}

    vehicle_index = findfirst(egoid, scene)

    fore = get_neighbor_fore_along_lane(scene, vehicle_index, roadway, VehicleTargetPointFront(), VehicleTargetPointRear(), VehicleTargetPointFront())

    v_ego = vel(scene[vehicle_index].state)
    v_oth = NaN
    headway = NaN

    if fore.ind != nothing
        v_oth = vel(scene[fore.ind].state)
        headway = fore.Δs
    end

    track_longitudinal!(model, v_ego, v_oth, headway)

    return model
end

get_name(model::LatLonSeparableDriver) = @sprintf("%s + %s", get_name(model.mlat), get_name(model.mlon))
function set_desired_speed!(model::LatLonSeparableDriver, v_des::Float64)
    set_desired_speed!(model.mlon, v_des)
    model
end
function observe!(model::LatLonSeparableDriver, scene::Frame{Entity{S, D, I}}, roadway::Roadway, egoid::I) where {S, D, I}
    observe!(model.mlat, scene, roadway, egoid)
    observe!(model.mlon, scene, roadway, egoid)
    model
end
function Base.rand(rng::AbstractRNG, model::LatLonSeparableDriver{A}) where A
    action_lat = rand(rng, model.mlat)
    action_lon = rand(rng, model.mlon)
    A(action_lat, action_lon)
end
Distributions.pdf(model::LatLonSeparableDriver{A}, a::A) where A = pdf(model.mlat, a.a_lat) * pdf(model.mlon, a.a_lon)
Distributions.logpdf(model::LatLonSeparableDriver{A}, a::A) where A = logpdf(model.mlat, a.a_lat) + logpdf(model.mlon, a.a_lon)
