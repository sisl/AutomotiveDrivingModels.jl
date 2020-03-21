"""
Run all callbacks
"""
function _run_callbacks(callbacks::C, scenes::Vector{Frame{Entity{S,D,I}}}, actions::Union{Nothing, Vector{Frame{A}}}, roadway::R, models::Dict{I,M}, tick::Int) where {S,D,I,A<:EntityAction,R,M<:DriverModel,C<:Tuple{Vararg{Any}}}
    isdone = false
    for callback in callbacks
        isdone |= run_callback(callback, scenes, actions, roadway, models, tick)
    end
    return isdone
end

## Implementations of useful callbacks

"""
    CollisionCallback

Terminates the simulation once a collision occurs
"""
@with_kw struct CollisionCallback
    mem::CPAMemory=CPAMemory()
end

function run_callback(
    callback::CollisionCallback,
    scenes::Vector{Frame{E}},
    actions::Union{Nothing, Vector{Frame{A}}},
    roadway::R,
    models::Dict{I,M},
    tick::Int
) where {E<:Entity,A<:EntityAction,R,I,M<:DriverModel}
    return !is_collision_free(scenes[tick], callback.mem)
end
