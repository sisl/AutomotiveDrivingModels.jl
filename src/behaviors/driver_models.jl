export
    DriverModel,

    get_name,
    action_type,
    action_context,
    observe!,
    reset_hidden_state!,
    prime_with_history!

abstract DriverModel{DriveAction, ActionContext}

get_name(::DriverModel) = "???"
action_type{A,C}(::DriverModel{A, C}) = A
action_context{A,C}(model::DriverModel{A, C}) = error("action_context not implemented for model $model")
reset_hidden_state!(model::DriverModel) = model # do nothing by default
observe!(model::DriverModel, scene::Scene, roadway::Roadway, egoid::Int) = model  # do nothing by default
Base.rand{A}(model::DriverModel{A}) = error("rand not implemented for model $model")
Distributions.pdf{A}(model::DriverModel{A}, a::A) = error("pdf not implemented for model $model")
Distributions.logpdf{A}(model::DriverModel{A}, a::A) = error("logpdf not implemented for model $model")

function prime_with_history!(model::DriverModel, trajdata::Trajdata, roadway::Roadway, frame_start::Int, frame_end::Int, egoid::Int, scene::Scene=Scene())

    reset_hidden_state!(model)

    for frame in frame_start : frame_end
        get!(scene, trajdata, frame)
        observe!(model, scene, roadway, egoid)
    end

    model
end