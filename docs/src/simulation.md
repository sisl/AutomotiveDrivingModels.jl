# Simulation

Simulations can be run using the `simulate!` function. 
A simulation updates the initial scene forward in time. Each simulation step consists of the following operations:
- call the `observe!` function for each vehicle to update their driver model given the current scene 
- sample an action from the driver model by calling `rand` on the driver model.
- update the state of each vehicle using the sampled action and the `propagate` method.
- repeat for the desired number of steps

There are two main ways to call the `simulate!` function:
- `simulate!(scene::Frame{E}, roadway::R, models::Dict{I,M}, nticks::Int64, timestep::Float64; rng::AbstractRNG = Random.GLOBAL_RNG, scenes::Vector{Frame{E}} = [Frame(E, length(scene)) for i=1:nticks+1], callbacks = nothing)` which simply returns a vector of scenes. This vector can be pre-allocated and passed as a keyword argument. The randomness of the simulation can be controlled by passing a random number generator. 
-  `simulate!(rec::EntityQueueRecord{S,D,I}, scene::EntityFrame{S,D,I}, roadway::R, models::Dict{I,M}, nticks::Int, callbacks::C)` which fills in a given `QueueRecord` object. The `QueueRecord` data structure is defined in Records.jl. This methods is slower and suffers from type stability issues. It will be deprecated in future releases.

See the tutorials for examples.

```@docs 
    simulate!
    get_actions!
    tick!
```

## Callbacks 

One can define callback function that will be run at each simulation step. The callback function can interrupt the simulation if it return `true`. It is also useful to log simulation information. 

To implement a custom callback function you must implement a type and the associated `run_callback` method. Here is an example of a callback that checks if a vehicle longitudinal position has reached some goal position and stops the simulation if it is the case.
```julia
struct ReachGoalCallback # a callback that checks if vehicle veh_id has reach a certain position 
    goal_pos::Float64
    veh_id::Int64
end 

function AutomotiveDrivingModels.run_callback(cb::ReachGoalCallback,
    scenes::Vector{Scene},
    roadway::R,
    models::Dict{I,M},
    tick::Int,
    ) where {S,D,I,R,M<:DriverModel}
    veh = get_by_id(last(scenes), cb.veh_id)
    return veh.state.posF.s > cb.goal_pos 
end
```

A callback for collision is already implemented: `CollisionCallback`.

```@docs
    run_callback
    CollisionCallback
```

## Others 

```@docs 
    reset_hidden_states!
```
