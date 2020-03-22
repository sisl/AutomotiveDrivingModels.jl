# AutomotiveDrivingModels

This is the documentation for AutomotiveDrivingModels.jl. 

## Concepts

This section defines a few terms that are used across the package. 

### Entity

An entity (or sometimes called agent) is a traffic participant that navigates in the environment, it is defined by a physical state (position, velocity, ...), an agent definition (whether it is a car or pedestrian, how large it is, ...), and an ID.

`AutomotiveDrivingModels` is templated to efficiently run simulations with different types of entities.
An entity represents an agent in the simulation, and it is parameterized by

- `S`: state of the entity, may change over time
- `D`: definition of the entity, does not change over time
- `I`: unique identifier for the entity, typically an `Int64` or `Symbol`

The interface for implementing your own state type is described in [States](@ref). 
Similarly, an interface for implementing your own entity definition is described in [Agent Definition](@ref)

In addition to the state, definition and identifier for each simulation agent,
one can also customize the actions, environment and the driver models used by
the agents.

### Actions

An action consists of a command applied to move the entity (e.g. longitudinal acceleration, steering). The state of the entity is updated using the `propagate` method which encodes the dynamics model.

### Scene

A scene represents a snapshot in time of a driving situation, it essentially consists of a list of entities at a given time.

It is implemented using the `Frame` object. `Frame` support most of the operation that one can do on a collection (`iterate`, `in`, `push!`, ...). 
In addition it supports `get_by_id` to retrieve an entity by its ID.

### Driver Model 

A driver model is a distribution over actions. Given a scene, each entity can update its model, we call this process observation (the corresponding method is `observe!`). After observing the scene, an action can be sampled from the driver model (using `rand`).

## Tutorials

The following examples will showcase some of the simulation functionality of `AutomotiveDrivingModels`

- [Driving on a Straight Roadway](@ref)
- [Driving in a Stadium](@ref)
- [Intersection](@ref)
- [Crosswalk](@ref)
- [Sidewalk](@ref)
- [Feature Extraction](@ref)

!!! note
    All AutomotiveDrivingModels tutorials are available as
    [Jupyter notebooks](https://nbviewer.jupyter.org/)
    by clicking on the badge at the beginning of the tutorial!
