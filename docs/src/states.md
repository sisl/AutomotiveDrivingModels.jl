# States 

In this section of the documentation we explain the default vehicle state type provided by `AutomotiveDrivingModels`
as well as the data types used to represent driving scene. Most of the underlying structures are defined in `Records.jl`. 
The data structure provided in ADM.jl are concrete instances of parametric types defined in Records. It is possible in principle to define your custom state definition and use the interface defined in ADM.jl.

## Agent States

Agents are represented by  the entity data type provided by `Records.jl`.
The entity data type has three field: a state, a definition, and an id. 

The state of an entity usually describes physical quantity such as position and velocity. 

Two states data structure are provided.

### 1D states and vehicles

```@docs 
    State1D
    Vehicle1D
```

### 2D states and vehicles


```@docs 
    VehicleState
    Vec.lerp(a::VehicleState, b::VehicleState, t::Float64, roadway::Roadway)
    get_vel_s
    get_vel_t
    move_along(vehstate::VehicleState, roadway::Roadway, Δs::Float64; ϕ₂::Float64=vehstate.posF.ϕ, ::Float64=vehstate.posF.t, v₂::Float64=vehstate.v)
    Vehicle
    get_front
    get_rear
    get_center
    get_footpoint
    Base.convert(::Type{Vehicle}, veh::Entity{VehicleState, D, Int64}) where D<:AbstractAgentDefinition
```


## Scenes

A Scene represents a collection of vehicles at a given time. 

```@docs 
    Scene
    SceneRecord
    Trajdata
```
