export
    run_callback,
    CollisionCallback

# run callback and return whether simlation should terminate
run_callback{S,Def,I,D<:DriverModel}(callback::Any, rec::QueueRecord{Entity{S,Def,I}}, roadway::Any, models::Dict{I,D}, tick::Int) = error("run_callback not implemented for callback $(typeof(callback))")

"""
    CollisionCallback

Terminates the simulation once a collision occurs
"""
@with_kw type CollisionCallback
    mem::CPAMemory=CPAMemory()
end
function run_callback{S,Def,I,D<:DriverModel}(
    callback::CollisionCallback,
    rec::QueueRecord{Entity{S,Def,I}},
    roadway::Any,
    models::Dict{I,D},
    tick::Int,
    )

    return !is_collision_free(rec[0], callback.mem)
end

function _run_callbacks{S,Def,I, D<:DriverModel, C<:Tuple{Vararg{Any}}}(callbacks::C, rec::QueueRecord{Entity{S,Def,I}}, roadway::Any, models::Dict{I,D}, tick::Int)
    isdone = false
    for callback in callbacks
        isdone |= run_callback(callback, rec, roadway, models, tick)
    end
    return isdone
end
function simulate!{S,Def,I, D<:DriverModel, C<:Tuple{Vararg{Any}}}(
    rec::QueueRecord{Entity{S,Def,I}},
    scene::Frame{Entity{S,Def,I}},
    roadway::Any,
    models::Dict{I,D},
    nticks::Int,
    callbacks::C,
    )

    empty!(rec)
    update!(rec, scene)

    # potential early out right off the bat
    if _run_callbacks(callbacks, rec, roadway, models, 0)
        return rec
    end

    actions = Array(DriveAction, length(scene))
    for tick in 1 : nticks
        get_actions!(actions, scene, roadway, models)
        tick!(scene, roadway, actions, rec.timestep)
        update!(rec, scene)
        if _run_callbacks(callbacks, rec, roadway, models, tick)
            break
        end
    end

    return rec
end