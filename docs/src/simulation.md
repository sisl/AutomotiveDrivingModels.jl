# Simulation

Simulations can be run using the `simulate` function. 
A simulation updates the initial scene forward in time. Each simulation step consists of the following operations:
- call the `observe!` function for each vehicle to update their driver model given the current scene 
- sample an action from the driver model by calling `rand` on the driver model.
- update the state of each vehicle using the sampled action and the `propagate` method.
- repeat for the desired number of steps

```@docs 
    simulate
    simulate!
```

See the tutorials for more examples.

## Callbacks 

One can define callback functions that will be run at each simulation step. The callback function can terminate the simulation by returning `true`. The default return value of a callback function should be `false`. Callback functions are also useful to log simulation information. 

To run a custom callback function in the simulation loop, you must implement a custom callback type and an associated `run_callback` method for that type with the following signature

```julia
function AutomotiveDrivingModels.run_callback(
    cb::ReachGoalCallback,
    scenes::Vector{Scene{E}},
    actions::Vector{Scene{A}},
    roadway::R,
    models::Dict{I,M},
    tick::Int,
    ) where {E<:Entity,A<:EntityAction,R,I,M<:DriverModel}
end
```
The `scenes` object holds a snapshot of a scene at each timestep in the range `1:tick`, and the `actions` object holds a `scene` of `EntityAction`s which record the action of each vehicle for the time steps `1:(tick-1)`.

Here is an example of a callback that checks if a vehicle's longitudinal position has reached some goal position and stops the simulation if it is the case.
```julia
struct ReachGoalCallback # a callback that checks if vehicle veh_id has reach a certain position 
    goal_pos::Float64
    veh_id::Int64
end 

function AutomotiveDrivingModels.run_callback(
    cb::ReachGoalCallback,
    scenes::Vector{Scene{E}},
    actions::Vector{Scene{A}},
    roadway::R,
    models::Dict{I,M},
    tick::Int,
    ) where {E<:Entity,A<:EntityAction,R,I,M<:DriverModel}
    veh = get_by_id(last(scenes), cb.veh_id)
    return veh.state.posF.s > cb.goal_pos 
end
```

A callback for collision is already implemented: `CollisionCallback`.

```@docs
    CollisionCallback
```

```@docs 
    run_callback
```

## Woking with datasets

When working with datasets or pre-recorded datasets, one can replay the simulation using `simulate_from_history`. It allows to update the state of an ego vehicle while other vehicles follow the trajectory given in the dataset.

```@docs 
    simulate_from_history
    simulate_from_history!
    observe_from_history!
    maximum_entities
```
