"""
    DriverModel{DriveAction}
A DriverModel represents a specific driving behavior. 
It specifies the action taken by the agent at a given scene.
The ation will be of type `DriveAction`.
It can be interpreted as a distribution, the likelihood of taking a certain action 
in a given scene. 
The DriverModel type is an abstract type! Custom driver models should inherit from it.
"""
abstract type DriverModel{DriveAction} end

"""
    get_name(::DriverModel)
returns the name of the driver model
"""
function get_name end

"""
    action_type(::DriverModel{A}) where {A}
returns the type of the actions that are sampled from the model
"""
action_type(::DriverModel{A}) where {A} = A

"""
    set_desired_speed!(model::DriverModel, v_des::Float64)
Sets a desired speed.
This method is relevant for models like IDM where the vehicle tracks a nominal speed.
"""
function set_desired_speed! end

"""
    reset_hidden_state!(model::DriverModel)
Resets the hidden states of the model. 
"""
function reset_hidden_state! end

"""
    observe!(model::DriverModel, scene, roadway, egoid)
Observes the scene and updates the model states accordingly. 
"""
function observe! end 

"""
    rand(model::DriverModel)
Samples an action from the model.
"""
Base.rand(model::DriverModel) = rand(Random.GLOBAL_RNG, model)
Base.rand(rng::AbstractRNG, model::DriverModel) = error("AutomotiveDrivingModelsError: Base.rand(::AbstractRNG, ::$(typeof(model))) not implemented")

function prime_with_history!(
    model::DriverModel,
    trajdata::ListRecord{S,D,I},
    roadway::R,
    frame_start::Int,
    frame_end::Int,
    egoid::I,
    scene::EntityFrame{S,D,I} = allocate_frame(trajdata),
    ) where {S,D,I,R}

    reset_hidden_state!(model)

    for frame in frame_start : frame_end
        get!(scene, trajdata, frame)
        observe!(model, scene, roadway, egoid)
    end

    return model
end
function prime_with_history!(model::DriverModel, rec::EntityQueueRecord{S,D,I}, roadway::R, egoid::I;
    pastframe_start::Int=1-nframes(rec),
    pastframe_end::Int=0,
    ) where {S,D,I,R}

    reset_hidden_state!(model)

    for pastframe in pastframe_start : pastframe_end
        scene = rec[pastframe]
        observe!(model, scene, roadway, egoid)
    end

    model
end

####

"""
    StaticDriver{A,P<:ContinuousMultivariateDistribution} <: DriverModel{A}

A driver model where actions are always sampled by the same distribution specified 
by the field `distribution`.

# Fields
- `distribution::P`
"""
struct StaticDriver{A,P<:ContinuousMultivariateDistribution} <: DriverModel{A}
    distribution::P
end

get_name(::StaticDriver) = "StaticDriver"
function Base.rand(rng::AbstractRNG, model::StaticDriver{A,P}) where {A,P}
    a = rand(rng, model.distribution)
    return convert(A, a)
end

Distributions.pdf(model::StaticDriver{A}, a::A) where {A} = pdf(model.distribution, convert(Vector{Float64}, a))
Distributions.logpdf(model::StaticDriver{A}, a::A) where {A} = logpdf(model.distribution, convert(Vector{Float64}, a))
set_desired_speed!(model::StaticDriver, v::Float64) = model
observe!(model::StaticDriver, scene::S, roadway::Roadway, egoid::I) where {S, I} = model
