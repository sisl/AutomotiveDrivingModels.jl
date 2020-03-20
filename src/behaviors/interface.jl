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
    reset_hidden_states!(models::Dict{I,M}) where {M<:DriverModel}
reset hidden states of all driver models in `models`
"""
function reset_hidden_states!(models::Dict{I,M}) where {I, M<:DriverModel}
    for model in values(models)
        reset_hidden_state!(model)
    end
    return models
end

"""
    observe!(model::DriverModel, scene, roadway, egoid)
Observes the scene and updates the model states accordingly. 
"""
function observe! end 

"""
    rand(model::DriverModel)
    rand(rng::AbstractRNG, model::DriverModel)
Samples an action from the model.
"""
Base.rand(model::DriverModel) = rand(Random.GLOBAL_RNG, model)
Base.rand(rng::AbstractRNG, model::DriverModel) = error("AutomotiveDrivingModelsError: Base.rand(::AbstractRNG, ::$(typeof(model))) not implemented")

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

function Base.rand(rng::AbstractRNG, model::StaticDriver{A,P}) where {A,P}
    a = rand(rng, model.distribution)
    return convert(A, a)
end

Distributions.pdf(model::StaticDriver{A}, a::A) where {A} = pdf(model.distribution, convert(Vector{Float64}, a))
Distributions.logpdf(model::StaticDriver{A}, a::A) where {A} = logpdf(model.distribution, convert(Vector{Float64}, a))
set_desired_speed!(model::StaticDriver, v::Float64) = model
observe!(model::StaticDriver, scene::S, roadway::Roadway, egoid::I) where {S, I} = model
