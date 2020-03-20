# Feature Extraction

AutomotiveDrivingModels.jl provides useful functions to extract information from a scene.

## Feature Extraction Pipeline

The function `extract_features` can be used to extract information from a list of scenes. It takes as input a vector of frame (which could be the output of `simulate`), as well as a list of feature functions to use. The output is a dictionary of DataFrame from [DataFrames.jl](https://github.com/JuliaData/DataFrames.jl).

```@docs
    extract_features
```


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
    VehicleTargetPoint
    VehicleTargetPointCenter
    VehicleTargetPointFront
    VehicleTargetPointRear
```

To get the relative position of two vehicles in the Frenet frame, the `get_frenet_relative_position` can be used. 
It stores the result in a `FrenetRelativePosition` object.

```@docs
    get_frenet_relative_position
    FrenetRelativePosition
```

## Implementing Your Own Feature Function

In AutomotiveDrivingModels, features are functions. The current interface supports three methods: 
- `myfeature(::Roadway, ::Entity)` 
- `myfeature(::Roadway, ::Frame, ::Entity)`
- `myfeature(::Roadway, ::Vector{<:Frame}, ::Entity)` 
For each of those methods, the last argument correspond to the entity with one of the ID given to the top level `extract_features` function. 
Creating a new feature consists of implementing **one** of those methods for your feature function.

As an example, let's define a feature function that returns the distance to the rear neighbor. Such feature will use the second method since it needs information about the whole frame to find the neighbor. If there are not rear neighbor then the function will return `missing`. `DataFrame` are designed to handled missing values so it should not be an issue.

```julia
function distance_to_rear_neighbor(roadway::Roadway, scene::Frame, ego::Entity)
    neighbor = find_neighbor(scene, roadway, veh, rear=true)
    if neighbor.ind === nothing 
        return missing 
    else 
        return neighbor.Δs
    end
end
```

Now you can use your feature function in extract features, the name of the function is used to name the column of the dataframe: 

```julia
    dfs = extract_features((distance_to_rear_neighbor,), roadway, scenes, [1])
    dfs[1].distance_to_rear_neighbor # contains history of distances to rear neighor
```

!!!note 
    If the supported methods are limiting for your use case please open an issue or submit a PR. 
    It should be straightforward to extend the `extract_features` function to support other methods, as well as adding new feature trait.

## List of Available Features 

```@docs
  posgx
  posgy
  posgθ
  posfs
  posft
  posfϕ
  vel(roadway::Roadway, veh::Entity)
  velfs
  velft
  velgx
  velgy
  time_to_crossing_right
  time_to_crossing_left
  estimated_time_to_lane_crossing
  iswaiting
  iscolliding
  distance_to
  acc
  accfs
  accft
  jerk
  jerkft
  turn_rate_g
  turn_rate_f
  isbraking
  isaccelerating
  lane_width
  markerdist_left
  markerdist_right
  road_edge_dist_left
  road_edge_dist_right
  lane_offset_left
  lane_offset_right
  n_lanes_left(roadway::Roadway, veh::Entity)
  n_lanes_right(roadway::Roadway, veh::Entity)
  has_lane_left
  has_lane_right
  lane_curvature
  dist_to_front_neighbor
  front_neighbor_speed
  time_to_collision
```
