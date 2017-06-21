type StochasticLaneFollowingDriver{D<:LaneFollowingDriver, P<:ContinuousUnivariateDistribution} <: LaneFollowingDriver
    submodel::D
    distr::P
end
get_name(::StochasticLaneFollowingDriver) = "StochasticLaneFollowingDriver"

function track_longitudinal!(model::StochasticLaneFollowingDriver, v_ego::Float64, v_oth::Float64, headway::Float64)
    track_longitudinal!(model.submodel, v_ego, v_oth, headway)
    return model
end

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