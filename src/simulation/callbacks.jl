"""
Internals, run all callbacks
"""
function _run_callbacks(callbacks::C, scenes::Vector{Frame{Entity{S,D,I}}}, actions::Union{Nothing, Vector{Frame{A}}}, roadway::R, models::Dict{I,M}, tick::Int) where {S,D,I,A<:EntityAction,R,M<:DriverModel,C<:Tuple{Vararg{Any}}}
    isdone = false
    for callback in callbacks
        isdone |= run_callback(callback, scenes, actions, roadway, models, tick)
    end
    return isdone
end

"""
    run_callback(callback, scenes::Vector{EntityFrame}, actions::Union{Nothing, Vector{A}}, roadway::Roadway, models::Dict{I, DriverModel}, tick::Int64)

Given a callback type, `run_callback` will be run at every step of a simulation run using `simulate`. 
By overloading the `run_callback` method for a custom callback type one can log information or interrupt a simulation. 
The `run_callback` function is expected to return a boolean. If `true` the simulation is stopped. 

# Inputs:
- `callback` the custom callback type used for dispatch 
- `scenes` where the simulation data is stored, note that it is only filled up to the current time step (`scenes[1:tick+1]`)
- `actions` where the actions are stored, it is only filled up to `actions[tick]`
- `roadway` the roadway where entities are moving
- `models` a dictionary mapping entity IDs to driver models
- `tick` the index of the current time step
"""
function run_callback end

## Implementations of useful callbacks

"""
    CollisionCallback

Terminates the simulation once a collision occurs
"""
@with_kw struct CollisionCallback
    mem::CPAMemory=CPAMemory()
end

function run_callback(
    callback::CollisionCallback,
    scenes::Vector{Frame{E}},
    actions::Union{Nothing, Vector{Frame{A}}},
    roadway::R,
    models::Dict{I,M},
    tick::Int
) where {E<:Entity,A<:EntityAction,R,I,M<:DriverModel}
    return !is_collision_free(scenes[tick], callback.mem)
end
