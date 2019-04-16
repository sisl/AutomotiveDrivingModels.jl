# Behaviors
These stands one level above the `actions`. They provide a higher level decision that
the `actions` then implement in order to propagate the simulation forward.

A behavior model can be interpreted as a control law. Given the current scene, representing all 
the vehicles present in the environment, a behavior model returns an action to execute. 

## Interface

We provide an interface to interact with behavior model or implement your own. To implement your own driver model you can create a type that inherits from the abstract type `DriverModel`. Then you can implement the following methods:

```@docs
    DriverModel{DriveAction}
    get_name(::DriverModel)
    action_type(::DriverModel{A}) where A
    set_desired_speed!(model::DriverModel, v_des::Float64)
    reset_hidden_state!(model::DriverModel)
    observe!(model::DriverModel, scene::EntityFrame{S,D,I}, roadway::R, egoid::Integer) where {S,D,I,R}
    Base.rand(model::DriverModel)
```

`observe!` and `rand` are usually the most important methods to implement. `observe!` sets the model state in a given situation and `rand` allows to sample an action from the model.


## Available Behaviors
```@docs
    IntelligentDriverModel
```

```@docs
    Tim2DDriver
```

```@docs
    PrincetonDriver
```

```@docs
    SidewalkPedestrianModel
```

```@docs 
    StaticDriver
```

## Lane change helper functions
These are not standalone driver models but are used by the driver models to do
lane changing and lateral control.

```@docs
    MOBIL
```

```@docs
    TimLaneChanger
```

```@docs
    ProportionalLaneTracker
```

## Longitudinal helper functions

These are not standalone driver models but are used to do longitudinal control by the
driver models.

```@docs
    ProportionalSpeedTracker
```
