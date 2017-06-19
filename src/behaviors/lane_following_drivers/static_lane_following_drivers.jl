type StaticLaneFollowingDriver <: LaneFollowingDriver
    a::Accel
    StaticLaneFollowingDriver(a::Accel=Accel(0.0)) = new(a)
    StaticLaneFollowingDriver(a::Float64) = new(Accel(a))
end
get_name(::StaticLaneFollowingDriver) = "StaticLaneFollowingDriver"
Base.rand(model::StaticLaneFollowingDriver) = model.a
Distributions.pdf(model::StaticLaneFollowingDriver, a::Accel) = isapprox(a.a, model.a.a) ? Inf : 0.0
Distributions.logpdf(model::StaticLaneFollowingDriver, a::Accel) = isapprox(a.a, model.a.a) ? Inf : -Inf