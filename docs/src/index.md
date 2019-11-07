# About

This is the documentation for AutomotiveDrivingModels.jl. 

## Concepts

This section defines a few term that are used accross the package. 
See the specific section of the documentation for a more thorough explanation.

- **Entity**: An entity is a traffic participants that navigates in the environment, it is defined by a physical state (position, velocity, ...), an agent definition (whether it is a car or pedestrian, how large it is, ...), and an ID.
- **Scene**: A scene represents a snapshot in time of a driving situation, it essentially consists of a list of entities at a given time.
- **Driver Model**: A driver model is a distribution over action. Given a scene, each entity can updates its model, we call this process observation (the corresponding method is `observe!`). After observing the scene, an action can be sampled from the driver model (using `rand`).
- **Actions**: An action consists of a command applied to move the entity (e.g. longitudinal acceleration, steering). The state of the entity is updated using the `propagate` method which encodes the dynamics model.

This package provides default structure for representing entity states, entities, scenes, driver model and actions.
However it has been design to support custom types. 
Each section of the documentation contains an interface, which is a list of function that a user must implement to use its own types.

```@contents
```
