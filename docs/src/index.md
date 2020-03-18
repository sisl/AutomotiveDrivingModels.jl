# About

This is the documentation for AutomotiveDrivingModels.jl. 

## Concepts

This section defines a few terms that are used across the package. 
See the specific section of the documentation for a more thorough explanation.

- **Entity**: An entity is a traffic participant that navigates in the environment, it is defined by a physical state (position, velocity, ...), an agent definition (whether it is a car or pedestrian, how large it is, ...), and an ID.
- **Scene**: A scene represents a snapshot in time of a driving situation, it essentially consists of a list of entities at a given time.
- **Driver Model**: A driver model is a distribution over actions. Given a scene, each entity can update its model, we call this process observation (the corresponding method is `observe!`). After observing the scene, an action can be sampled from the driver model (using `rand`).
- **Actions**: An action consists of a command applied to move the entity (e.g. longitudinal acceleration, steering). The state of the entity is updated using the `propagate` method which encodes the dynamics model.

This package provides a default structure for representing entity states, entities, scenes, driver models and actions.
However it has been designed to support custom types. 

`AutomotiveDrivingModels` is templated to efficiently run simulations with different types of entities.
An entity represents an agent in the simulation, and it is parameterized by

- `S`: state of the entity, may change over time
- `D`: definition of the entity, does not change over time
- `I`: unique identifier for the entity, typically an `Int64` or `Symbol`

In addition to the state, definition and identifier for each simulation agent,
one can also customize the actions, environment and the driver models used by
the agents.

Each section of the documentation contains an interface, which is a list of functions that a user must implement to use its own types.

## Tutorials

The following examples will showcase some of the simulation functionality of `AutomotiveDrivingModels`

- [Driving on a Straight Roadway](@ref)
- [Driving in a Stadium](@ref)
- [Intersection](@ref)
- [Crosswalk](@ref)
- [Sidewalk](@ref)

# !!! note
#     All AutomotiveDrivingModels tutorials are available as
#     [Jupyter notebooks](https://nbviewer.jupyter.org/)
#     by clicking on the badge at the beginning of the tutorial!
