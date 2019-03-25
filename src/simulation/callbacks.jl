"""
    CollisionCallback

Terminates the simulation once a collision occurs
"""
@with_kw struct CollisionCallback
    mem::CPAMemory=CPAMemory()
end
function run_callback(
    callback::CollisionCallback,
    rec::EntityQueueRecord{S,D,I},
    roadway::R,
    models::Dict{I,M},
    tick::Int,
    ) where {S,D,I,R,M<:DriverModel}

    return !is_collision_free(rec[0], callback.mem)
end
