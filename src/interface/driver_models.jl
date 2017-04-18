export
    DriverModel,

    StaticDriver,

    get_name,
    action_type,
    set_desired_speed!,
    observe!,
    reset_hidden_state!,
    prime_with_history!

abstract DriverModel{DriveAction}

get_name(::DriverModel) = "???"
action_type{A}(::DriverModel{A}) = A
set_desired_speed!(model::DriverModel, v_des::Float64) = model # do nothing by default
reset_hidden_state!(model::DriverModel) = model # do nothing by default
observe!{S,D,I,R}(model::DriverModel, scene::EntityFrame{S,D,I}, roadway::R, egoid::Int) = model  # do nothing by default
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


type StaticDriver{A,P<:ContinuousMultivariateDistribution} <: DriverModel{A}
    distribution::P
end

get_name(::StaticDriver) = "StaticDriver"
function Base.rand{A,P}(model::StaticDriver{A,P})
    a = rand(model.distribution)
    return convert(A, a)
end
Distributions.pdf{A}(model::StaticDriver{A}, a::A) = pdf(model.distribution, convert(Vector{Float64}, a))
Distributions.logpdf{A}(model::StaticDriver{A}, a::A) = logpdf(model.distribution, convert(Vector{Float64}, a))