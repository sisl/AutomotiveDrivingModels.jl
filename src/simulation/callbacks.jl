"""
Run all callbacks
"""
function _run_callbacks(callbacks::C, scenes::Union{EntityQueueRecord{S,D,I}, Vector{Frame{Entity{S,D,I}}}}, actions::Union{Nothing, Vector{Frame{A}}}, roadway::R, models::Dict{I,M}, tick::Int) where {S,D,I,A<:ActionMapping,R,M<:DriverModel,C<:Tuple{Vararg{Any}}}
    isdone = false
    for callback in callbacks
        isdone |= run_callback(callback, scenes, actions, roadway, models, tick)
    end
    return isdone
end

function simulate!(
    ::Type{A},
    rec::EntityQueueRecord{S,D,I},
    scene::EntityFrame{S,D,I},
    roadway::R,
    models::Dict{I,M},
    nticks::Int,
    callbacks::C,
    ) where {S,D,I,A,R,M<:DriverModel,C<:Tuple{Vararg{Any}}}

    empty!(rec)
    update!(rec, scene)

    # potential early out right off the bat
    if _run_callbacks(callbacks, rec, nothing, roadway, models, 0)
        return rec
    end

    actions = Array{A}(undef, length(scene))
    for tick in 1 : nticks
        get_actions!(actions, scene, roadway, models)
        tick!(scene, roadway, actions, get_timestep(rec))
        update!(rec, scene)
        if _run_callbacks(callbacks, rec, nothing, roadway, models, tick)
            break
        end
    end

    return rec
end
function simulate!(
    rec::EntityQueueRecord{S,D,I},
    scene::EntityFrame{S,D,I},
    roadway::R,
    models::Dict{I,M},
    nticks::Int,
    callbacks::C,
    ) where {S,D,I,R,M<:DriverModel,C<:Tuple{Vararg{Any}}}

    return simulate!(Any, rec, scene, roadway, models, nticks, callbacks)
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
    rec::EntityQueueRecord{S,D,I},
    roadway::R,
    models::Dict{I,M},
    tick::Int
    ) where {S,D,I,R,M<:DriverModel}

    return !is_collision_free(rec[0], callback.mem)
end

function run_callback(
    callback::CollisionCallback,
    scenes::Vector{Frame{E}},
    actions::Union{Nothing, Vector{Frame{A}}},
    roadway::R,
    models::Dict{I,M},
    tick::Int
) where {E<:Entity,A<:ActionMapping,R,I,M<:DriverModel}
    return !is_collision_free(scenes[tick], callback.mem)
end
