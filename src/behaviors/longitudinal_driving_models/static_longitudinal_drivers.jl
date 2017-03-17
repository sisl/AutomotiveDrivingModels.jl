type StaticLongitudinalDriver <: LongitudinalDriverModel
    a::Float64
    StaticLongitudinalDriver(a::Float64=0.0) = new(a)
end
get_name(::StaticLongitudinalDriver) = "ProportionalSpeedTracker"
Base.rand(model::StaticLongitudinalDriver) = model.a
Distributions.pdf(model::StaticLongitudinalDriver, a_lon::Float64) = a_lon == model.a ? Inf : 0.0
Distributions.logpdf(model::StaticLongitudinalDriver, a_lon::Float64) = a_lon == model.a ? Inf : -Inf