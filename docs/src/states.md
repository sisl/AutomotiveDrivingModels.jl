# States 

In this section of the documentation we explain the default vehicle state type provided by `AutomotiveDrivingModels`
as well as the data types used to represent driving scene. Most of the underlying structures are defined in `Records.jl`. 
The data structure provided in ADM.jl are concrete instances of parametric types defined in Records. It is possible in principle to define your custom state definition and use the interface defined in ADM.jl.

## Agent States

Agents are represented by  the entity data type provided by `Records.jl`.
The entity data type has three field: a state, a definition, and an id. 

The state of an entity usually describes physical quantity such as position and velocity. 


## Scenes

