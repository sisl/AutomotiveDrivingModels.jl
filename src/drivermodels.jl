"""
DriverModel{A}

A driver model is a probability distribution over actions,
where A is the action type.
"""
abstract type DriverModel{A} end

get_name(model::DriverModel) = string(typeof(model))
action_type{A}(::DriverModel{A}) = A
reset_hidden_state!(model::DriverModel) = model # do nothing by default

Base.rand(model::DriverModel) = error("rand not implemented for model $model")
Distributions.pdf{A}(model::DriverModel{A}, a::A) = error("pdf not implemented for model $model")
Distributions.logpdf{A}(model::DriverModel{A}, a::A) = error("logpdf not implemented for model $model")

function prime_with_history!{S,D,I,R}(
    model::DriverModel,
    trajdata::ListRecord{Entity{S,D,I}},
    roadway::R,
    frame_start::Int,
    frame_end::Int,
    egoid::I,
    scene::EntityFrame{S,D,I} = allocate_frame(trajdata),
    )

    reset_hidden_state!(model)

    for frame in frame_start : frame_end
        get!(scene, trajdata, frame)
        observe!(model, scene, roadway, egoid)
    end

    return model
end
function prime_with_history!{S,D,I,R}(model::DriverModel, rec::EntityQueueRecord{S,D,I}, roadway::R, egoid::I;
    pastframe_start::Int=1-nframes(rec),
    pastframe_end::Int=0,
    )

    reset_hidden_state!(model)

    for pastframe in pastframe_start : pastframe_end
        scene = rec[pastframe]
        observe!(model, scene, roadway, egoid)
    end

    model
end

####

mutable struct StaticDriver{A,P<:Distribution} <: DriverModel{A}
    distribution::P
end

get_name(::StaticDriver) = "StaticDriver"
observe!{S,D,I,R}(model::StaticDriver, scene::EntityFrame{S,D,I}, roadway::R, egoid::I) = model
function Base.rand{A,P}(model::StaticDriver{A,P})
    a = rand(model.distribution)
    return convert(A, a)
end
function Distributions.pdf{A, P<:ContinuousUnivariateDistribution}(model::StaticDriver{A,P}, a::A)
    return pdf(model.distribution, convert(Float64, a))
end
function Distributions.pdf{A, P<:ContinuousMultivariateDistribution}(model::StaticDriver{A,P}, a::A)
    return pdf(model.distribution, convert(Vector{Float64}, a))
end
function Distributions.logpdf{A, P<:ContinuousUnivariateDistribution}(model::StaticDriver{A,P}, a::A)
    return logpdf(model.distribution, convert(Float64, a))
end
function Distributions.logpdf{A, P<:ContinuousMultivariateDistribution}(model::StaticDriver{A,P}, a::A)
    return logpdf(model.distribution, convert(Vector{Float64}, a))
end