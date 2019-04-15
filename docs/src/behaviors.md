# Behaviors
These stands one level above the `actions`. They provide a higher level decision that
the `actions` then implement in order to propagate the simulation forward.


## Interface

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
