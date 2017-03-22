export LaneFollowingDriver

type LaneFollowingDriver <: DriverModel{LaneFollowingAccel}
    mlon::LongitudinalDriverModel
end

get_name(model::LaneFollowingDriver) = @sprintf("%s", get_name(model.mlon))
function observe!(model::LaneFollowingDriver, scene::Scene, roadway::Roadway, egoid::Int)
    observe!(model.mlon, scene, roadway, egoid)
    model
end
Base.rand(model::LaneFollowingDriver) = LaneFollowingAccel(rand(model.mlon))
Distributions.pdf(model::LaneFollowingDriver, a::LaneFollowingAccel) = pdf(model.mlon, a.a)
Distributions.logpdf(model::LaneFollowingDriver, a::LaneFollowingAccel) = logpdf(model.mlon, a.a)