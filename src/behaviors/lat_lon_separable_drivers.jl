export LatLonSeparableDriver

type LatLonSeparableDriver <: DriverModel{LatLonAccel, IntegratedContinuous}
    C::IntegratedContinuous
    mlat::LateralDriverModel
    mlon::LongitudinalDriverModel
end

get_name(model::LatLonSeparableDriver) = @sprintf("%s + %s", get_name(model.mlat), get_name(model.mlon))
action_context(model::LatLonSeparableDriver) = model.C
function observe!(model::LatLonSeparableDriver, scene::Scene, roadway::Roadway, egoid::Int)
    observe!(model.mlat, scene, roadway, egoid)
    observe!(model.mlon, scene, roadway, egoid)
    model
end
function Base.rand(model::LatLonSeparableDriver)
    # alat = rand(model.mlat)
    # alon = rand(model.mlon)
    alat = 0.0
    alon = 0.0
    LatLonAccel(alat, alon)
end
Distributions.pdf(model::LatLonSeparableDriver, a::LatLonAccel) = pdf(model.mlat, a.a_lat) * pdf(model.mlon, a.a_lon)
Distributions.logpdf(model::LatLonSeparableDriver, a::LatLonAccel) = logpdf(model.mlat, a.a_lat) + logpdf(model.mlon, a.a_lon)