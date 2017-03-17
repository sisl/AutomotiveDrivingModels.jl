export
    run_callback,
    CollisionCallback

# run callback and return whether simlation should terminate
run_callback{D<:DriverModel}(callback::Any, rec::SceneRecord, roadway::Roadway, models::Dict{Int,D}, tick::Int) = error("run_callback not implemented for callback $(typeof(callback))")

"""
    CollisionCallback

Terminates the simulation once a collision occurs
"""
@with_kw type CollisionCallback
    mem::CPAMemory=CPAMemory()
end
function run_callback{D<:DriverModel}(
    callback::CollisionCallback,
    rec::SceneRecord,
    roadway::Roadway,
    models::Dict{Int,D},
    tick::Int,
    )

    scene = get_scene(rec, 0)
    return !is_collision_free(scene, callback.mem)
end

function _run_callbacks{D<:DriverModel, C<:Tuple{Vararg{Any}}}(callbacks::C, rec::SceneRecord, roadway::Roadway, models::Dict{Int,D}, tick::Int)
    isdone = false
    for callback in callbacks
        isdone |= run_callback(callback, rec, roadway, models, tick)
    end
    return isdone
end
function simulate!{D<:DriverModel, C<:Tuple{Vararg{Any}}}(
    rec::SceneRecord,
    scene::Scene,
    roadway::Roadway,
    models::Dict{Int,D},
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
        tick!(scene, roadway, actions, models)
        update!(rec, scene)
        if _run_callbacks(callbacks, rec, roadway, models, tick)
            break
        end
    end

    return rec
end