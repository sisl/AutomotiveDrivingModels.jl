type StochasticLaneFollowingDriver{D<:LaneFollowingDriver, P<:ContinuousUnivariateDistribution} <: LaneFollowingDriver
    submodel::D
    distr::P
end
get_name(::StochasticLaneFollowingDriver) = "StochasticLaneFollowingDriver"
Base.rand(model::StochasticLaneFollowingDriver) = Accel(rand(model.submodel).a + rand(model.distr))
function Distributions.pdf(model::StochasticLaneFollowingDriver, a::Accel)
    subpdf = pdf(model.submodel, a)
    distrpdf = pdf(model.distr, a.a)
    if isinf(subpdf)
        return distrpdf
    else
        return subpdf * distrpdf
    end
end
function Distributions.logpdf(model::StochasticLaneFollowingDriver, a::Accel)
    subpdf = pdf(model.submodel, a)
    distrpdf = pdf(model.distr, a.a)
    if isinf(subpdf)
        return distrpdf
    else
        return subpdf + distrpdf
    end
end