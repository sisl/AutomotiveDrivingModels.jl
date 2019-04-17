# Driving Actions
In the driving stack, the `actions` lie one level below
the `behaviors`. While the `behaviors` provide the high level
decision making, the actions enable the execution of these decisions in
the simulation.


## Interface

```@docs
    propagate
```

## Action types available 

```@docs
    AccelDesang
```
```@docs
    AccelSteeringAngle
```
```@docs
    AccelTurnrate
```
```@docs
    LaneFollowingAccel
```
```@docs
    LatLonAccel
```
```@docs
    PedestrianLatLonAccel
```