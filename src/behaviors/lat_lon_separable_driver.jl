struct LatLonSeparableDriver{A} <: DriverModel{A}
    mlat::LateralDriverModel
    mlon::LaneFollowingDriver
end

LatLonSeparableDriver(mlat::LateralDriverModel, mlon::LaneFollowingDriver) = LatLonSeparableDriver{LatLonAccel}(mlat, mlon)

function set_desired_speed!(model::LatLonSeparableDriver, v_des::Float64)
    set_desired_speed!(model.mlon, v_des)
    model
end
function observe!(model::LatLonSeparableDriver, scene::Scene{Entity{S, D, I}}, roadway::Roadway, egoid::I) where {S, D, I}
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
