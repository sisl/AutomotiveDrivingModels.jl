type LatLonSeparableDriver <: DriverModel{LatLonAccel}
    mlat::LateralDriverModel
    mlon::LaneFollowingDriver
end

get_name(model::LatLonSeparableDriver) = @sprintf("%s + %s", get_name(model.mlat), get_name(model.mlon))
function set_desired_speed!(model::LatLonSeparableDriver, v_des::Float64)
    set_desired_speed!(model.mlon, v_des)
    model
end

function Base.rand(model::LatLonSeparableDriver)
    alat = rand(model.mlat)
    alon = rand(model.mlon).a
    LatLonAccel(alat, alon)
end
Distributions.pdf(model::LatLonSeparableDriver, a::LatLonAccel) = pdf(model.mlat, a.a_lat) * pdf(model.mlon, a.a_lon)
Distributions.logpdf(model::LatLonSeparableDriver, a::LatLonAccel) = logpdf(model.mlat, a.a_lat) + logpdf(model.mlon, a.a_lon)