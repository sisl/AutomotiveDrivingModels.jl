# Agent Definition

Agent Definitions describe the static properties of a traffic participants such as the length and width of the vehicle.
It also contains information on the type of agent (car, pedestrian, motorcycle...).

## Interface

You can implement your own agent definition by creating a new type inheriting from the `AbstractAgentDefinition` type.
The following three functions must be implemented for your custom type:

```@docs
    AbstractAgentDefinition
    length
    width
    class
```
Agent classes such as car, truck, and pedestrian are defined by integer constant in a submodule AgentClass.

```@docs
    AgentClass
```

## Available Agent Definitions

```@docs
    VehicleDef
    BicycleModel
```