# Examples

`AutomotiveDrivingModels` is templated to efficiently run simulations with different types of entities.
An entity represents an agent in the simulation, and it is parameterized by

- `S`: state of the entity, may change over time
- `D`: definition of the entity, does not change over time
- `I`: unique identifier for the entity, typically an `Int64` or `Symbol`

In addition to the state, definition and identifier for each simulation agent,
one can also customize the actions, environment and the driver models used by
the agents.

The following examples will showcase some of the simulation functionality of `AutomotiveDrivingModels`

```@contents
Pages = [
    "examples/straight_roadway.md",
    "examples/stadium.md",
    "examples/intersection.md",
    "examples/crosswalk.md",
    "examples/sidewalk.md",
]

```
