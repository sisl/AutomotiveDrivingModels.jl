#=
 TODO find an interface to keep both collision checkers around: Minkowski and Parallel Axis
abstract type CollisionChecker end 
struct MinkowskiChecker <: CollisionChecker end
struct ParallelAxisChecker <: CollisionChecker end 
is_colliding(vehA, vehB, collision_checker::AbstractCollisionChecker = ParallelAxisChecker())

=#