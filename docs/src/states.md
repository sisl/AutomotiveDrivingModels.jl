# States 

In this section of the documentation we explain the default vehicle state type provided by `AutomotiveDrivingModels`
as well as the data types used to represent a driving scene. Most of the underlying structures are defined in `Records.jl`. 
The data structures provided in ADM.jl are concrete instances of parametric types defined in Records. It is possible in principle to define your custom state definition and use the interface defined in ADM.jl.

## Entity state

Entities are represented by the `Entity` data type provided by `Records.jl` (https://github.com/sisl/Records.jl/blob/master/src/entities.jl).
The `Entity` data type has three fields: a state, a definition and an id. 

The state of an entity usually describes physical quantity such as position and velocity. 

Two state data structures are provided.

## Defining your own state type

You can define your own state type if the provided `VehicleState` does not contain the right information.
There are a of couple functions that need to be defined such that other functions in AutomotiveDrivingModels can work smoothly with your custom state type.

```@docs
    posg
    posf
    vel
    velf
    velg
```

**Example of a custom state type containing acceleration:**

```julia

# you can use composition to define your custom state type based on existing ones
struct MyVehicleState
    veh::VehicleState
    acc::Float64
end

# define the functions from the interface 
posg(s::MyVehicleState) = posg(s.veh) # those functions are implemented for the `VehicleState` type
posf(s::MyVehicleState) = posf(s.veh)
velg(s::MyVehicleState) = velg(s.veh)
velf(s::MyVehicleState) = velf(s.veh)
vel(s::MyVehicleState) = vel(s.veh)
```

### 1D states and vehicles

```@docs 
    State1D
    Vehicle1D
```

### 2D states and vehicles

Here we list useful functions to interact with vehicle states and retrieve interesting information like the position of the front of the vehicle or the lane to which the vehicle belongs.

```@docs 
    VehicleState
    Vec.lerp(a::VehicleState, b::VehicleState, t::Float64, roadway::Roadway)
    move_along(vehstate::VehicleState, roadway::Roadway, Δs::Float64; ϕ₂::Float64=vehstate.posF.ϕ, ::Float64=vehstate.posF.t, v₂::Float64=vehstate.v)
    Vehicle
    get_front
    get_rear
    get_center
    get_footpoint
    get_lane
    leftlane(::Roadway, ::Entity)
    rightlane(::Roadway, ::Entity)
    Base.convert(::Type{Vehicle}, veh::Entity{VehicleState, D, Int64}) where D<:AbstractAgentDefinition
```


## Scenes

A Scene represents a collection of vehicles at a given time. 

```@docs 
    Scene
    SceneRecord
    Trajdata
```
