type LatLonSeparableDriver <: DriverModel{LatLonAccel}
    mlat::LateralDriverModel
    mlon::LaneFollowingDriver
end

function observe!(model::LaneFollowingDriver, scene::Scene, roadway::Roadway, egoid::Int)

    vehicle_index = findfirst(scene, egoid)

    fore = get_neighbor_fore_along_lane(scene, vehicle_index, roadway, VehicleTargetPointFront(), VehicleTargetPointRear(), VehicleTargetPointFront())

    v_ego = scene[vehicle_index].state.v
    v_oth = NaN
    headway = NaN

    if fore.ind != 0
        v_oth = scene[fore.ind].state.v
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
function observe!(model::LatLonSeparableDriver, scene::Scene, roadway::Roadway, egoid::Int)
    observe!(model.mlat, scene, roadway, egoid)
    observe!(model.mlon, scene, roadway, egoid)
    model
end
function Base.rand(model::LatLonSeparableDriver)
    alat = rand(model.mlat)
    alon = rand(model.mlon).a
    LatLonAccel(alat, alon)
end
Distributions.pdf(model::LatLonSeparableDriver, a::LatLonAccel) = pdf(model.mlat, a.a_lat) * pdf(model.mlon, a.a_lon)
Distributions.logpdf(model::LatLonSeparableDriver, a::LatLonAccel) = logpdf(model.mlat, a.a_lat) + logpdf(model.mlon, a.a_lon)