type StaticLaneFollowingDriver <: LaneFollowingDriver
    a::LaneFollowingAccel
    StaticLaneFollowingDriver(a::LaneFollowingAccel=LaneFollowingAccel(0.0)) = new(a)
    StaticLaneFollowingDriver(a::Float64) = new(LaneFollowingAccel(a))
end
get_name(::StaticLaneFollowingDriver) = "ProportionalSpeedTracker"
Base.rand(model::StaticLaneFollowingDriver) = model.a
Distributions.pdf(model::StaticLaneFollowingDriver, a::LaneFollowingAccel) = isapprox(a.a, model.a.a) ? Inf : 0.0
Distributions.logpdf(model::StaticLaneFollowingDriver, a::LaneFollowingAccel) = isapprox(a.a, model.a.a) ? Inf : -Inf