# Feature Extraction

AutomotiveDrivingModels.jl implements a feature extraction pipeline to extract information from pre-recorded simulation 
in a `QueueRecord` or `ListRecord`. (see Records.jl for info on the data structure)

!!! warning
Work in progress, the feature extraction functionalities are being redesigned 
Follow [#72](https://github.com/sisl/AutomotiveDrivingModels.jl/pull/72) for more information
PRs for documenting feature extraction are more than welcome

## Finding neighbors

Finding neighbors of an entity is a common query and can be done using the `find_neighbor` function. 
This function allows to find the neighbor of an entity within a scene using a forward search method. 
The search algorithm moves longitudinally along a given lane and its successors (according to the given road topology). 
Once it finds a car it stops searching and return the results as a `NeighborLongitudinalResult`

```@docs 
    NeighborLongitudinalResult
```

The `find_neighbor` function accepts different keyword argument to search front or rear neighbors, or neighbors on different lanes.

```@docs 
    find_neighbor
```

When computing the distance to a neighbor, one might want to choose different reference points on the vehicle (center to center, bumper to bumper, etc...). AutomotiveDrivingModels provides the `VehicleTargetPoint` types to do so. 
One can choose among three possible instances to use the front, center, or rear point of a vehicle to compute the distance to the neighbor.

```@docs 
    VehicleTargetPoint,
    VehicleTargetPointCenter,
    VehicleTargetPointFront,
    VehicleTargetPointRear
```

To get the relative position of two vehicles in the Frenet frame, the `get_frenet_relative_position` can be used. 
It stores the result in a `FrenetRelativePosition` object.

```@docs
    get_frenet_relative_position
    FrenetRelativePosition
```

## Lane Features

```@docs
    get_lane_width
    get_markerdist_left
    get_markerdist_right
```
