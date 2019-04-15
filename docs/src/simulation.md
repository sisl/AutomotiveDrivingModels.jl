# Simulation

Simulation can be done using the `simulate!` function. 
A simulation updates the initial scene forward in time. Each simulation step consists of the following operation:
- get the actions of each agent in the scene by calling `observe!` on their respective behavior model (see the behavior section for more information)
- update the scene forward using `tick!`. It consists in applying the individual actions of each agent and updates their states individually. The actions can be of different type for each vehicle. The state is updated by calling the `propagate` method associated to the corresponding action type. (This done automatically thanks to Julia's mutliple dispatch). See the action section for more information on the `propagate` method.
- repeat for the desired number of steps

See the tutorials for examples.

```@docs 
    get_actions!
    tick!
    simulate!
```

## Callbacks 

One can define callback function that will be run at each simulation step. The callback function can interrupt the simulation if it return `false`. It is also useful to log simulation information. 

To implement a custom callback function you must implement a type and the associated `run_callback` method. Here is an example of a callback that checks if a vehicle longitudinal position has reached some goal position and stops the simulation if it is the case.
```julia
struct ReachGoalCallback # a callback that checks if vehicle veh_id has reach a certain position 
    goal_pos::Float64
    veh_id::Int64
end 

function AutomotiveDrivingModels.run_callback(cb::ReachGoalCallback, rec::EntityQueueRecord{S,D,I},
    roadway::R,
    models::Dict{I,M},
    tick::Int,
    ) where {S,D,I,R,M<:DriverModel}
    veh = get_by_id(rec[0], cb.veh_id)
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