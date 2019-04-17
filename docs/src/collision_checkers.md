# Collision Checker

There are two collisions checkers currently available in AutomotiveDrivingModels.jl. 
The first collision checker is accessible through the function `is_colliding` and relies on Minkowski sum. 
The second one is accessible through `collision_checker` and uses the parallel axis theorem. The latter is a bit faster. 
A benchmark script is available in `test/collision_checkers_benchmark.jl` and relies on static arrays. 

## Parallel Axis Theorem

This collision checker relies on the parallel axis theorem. It checks that two convex polygon overlap

```@docs
    collision_checker
```

Vehicles can be converted to polygon (static matrices containing four vertices).

```@docs
    polygon
```


## Minkowski Sum

Here are the methods available using Minkowski sum.

```@docs
    is_colliding
    ConvexPolygon
    CPAMemory
    CollisionCheckResult
    to_oriented_bounding_box!
    get_oriented_bounding_box
    is_potentially_colliding
    get_collision_time
    get_first_collision
    is_collision_free
    get_distance
    get_edge
```
