# Feature Extraction [WIP]

AutomotiveDrivingModels.jl implements a feature extraction pipeline to extract information from pre-recorded simulation 
in a `QueueRecord` or `ListRecord`. (see Records.jl for info on the data structure)

PRs for documenting feature extraction are more than welcome

## Neighbors Features

Here is a list of function to retrieve information about the neighbors of a given vehicle. 

```@docs
    NeighborLongitudinalResult
    get_neighbor_fore_along_lane
    get_neighbor_fore_along_left_lane
    get_neighbor_fore_along_right_lane
    get_neighbor_rear_along_lane
    get_neighbor_rear_along_left_lane
    get_neighbor_rear_along_right_lane
    FrenetRelativePosition
    get_frenet_relative_position
```

## Lane Features

```@docs
    get_lane_width
    get_markerdist_left
    get_markerdist_right
```