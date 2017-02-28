export LaneFollowingDriver

type LaneFollowingDriver{C<:ActionContext} <: DriverModel{LaneFollowingAccel, C}
    context::C
    mlon::LongitudinalDriverModel
end

get_name(model::LaneFollowingDriver) = @sprintf("%s", get_name(model.mlon))
action_context(model::LaneFollowingDriver) = model.context
function observe!(model::LaneFollowingDriver, scene::Scene, roadway::Roadway, egoid::Int)
    observe!(model.mlon, scene, roadway, egoid)
    model
end
Base.rand(model::LaneFollowingDriver) = LaneFollowingAccel(rand(model.mlon))
Distributions.pdf(model::LaneFollowingDriver, a::LaneFollowingAccel) = pdf(model.mlon, a.a)
Distributions.logpdf(model::LaneFollowingDriver, a::LaneFollowingAccel) = logpdf(model.mlon, a.a)