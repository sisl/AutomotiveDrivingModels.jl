# TODO: Not sure if this driver model is being used at all
struct LatLonSeparableDriver <: DriverModel{LatLonAccel}
    mlat::LateralDriverModel
    mlon::LaneFollowingDriver
end

function observe!(model::LaneFollowingDriver, scene::Frame{Entity{S, D, I}}, roadway::Roadway, egoid::I) where {S, D, I}

    vehicle_index = findfirst(egoid, scene)

    fore = get_neighbor_fore_along_lane(scene, vehicle_index, roadway, VehicleTargetPointFront(), VehicleTargetPointRear(), VehicleTargetPointFront())

    v_ego = scene[vehicle_index].state.v
    v_oth = NaN
    headway = NaN

    if fore.ind != nothing
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
function observe!(model::LatLonSeparableDriver, scene::Frame{Entity{S, D, I}}, roadway::Roadway, egoid::I) where {S, D, I}
    observe!(model.mlat, scene, roadway, egoid)
    observe!(model.mlon, scene, roadway, egoid)
    model
end
function Base.rand(rng::AbstractRNG, model::LatLonSeparableDriver)
    alat = rand(rng, model.mlat)
    alon = rand(rng, model.mlon).a
    LatLonAccel(alat, alon)
end
Distributions.pdf(model::LatLonSeparableDriver, a::LatLonAccel) = pdf(model.mlat, a.a_lat) * pdf(model.mlon, a.a_lon)
Distributions.logpdf(model::LatLonSeparableDriver, a::LatLonAccel) = logpdf(model.mlat, a.a_lat) + logpdf(model.mlon, a.a_lon)
