export
        get_actions!,
        tick!,
        simulate!

function get_actions!{A<:DriveAction, D<:DriverModel}(
    actions::Vector{A},
    scene::Scene,
    roadway::Roadway,
    models::Dict{Int, D}, # id → model
    )


    for (i,veh) in enumerate(scene)
        model = models[veh.def.id]
        observe!(model, scene, roadway, veh.def.id)
        actions[i] = rand(model)
    end

    actions
end

function tick!{A<:DriveAction, D<:DriverModel}(
    scene::Scene,
    roadway::Roadway,
    actions::Vector{A},
    models::Dict{Int, D}, # id → model
    )

    for (veh, action) in zip(scene, actions)
        model = models[veh.def.id]
        context = action_context(model)
        veh.state = propagate(veh, action, context, roadway)
    end

    scene
end
function tick!{A<:DriveAction, C<:ActionContext}(
    scene::Scene,
    roadway::Roadway,
    actions::Vector{A},
    contexts::Vector{C}
    )

    for (veh, action, context) in zip(scene, actions, contexts)
        veh.state = propagate(veh, action, context, roadway)
    end

    scene
end
function tick!{A<:DriveAction}(
    scene::Scene,
    roadway::Roadway,
    actions::Vector{A},
    context::ActionContext,
    )

    for (veh, action) in zip(scene, actions)
        veh.state = propagate(veh, action, context, roadway)
    end

    scene
end

"""
Run nticks of simulation and place all nticks+1 scenes into the SceneRecord
"""
function simulate!{D<:DriverModel}(
    rec::SceneRecord,
    scene::Scene,
    roadway::Roadway,
    models::Dict{Int,D},
    nticks::Int
    )

    empty!(rec)
    update!(rec, scene)
    actions = Array(DriveAction, length(scene))

    for tick in 1 : nticks
        get_actions!(actions, scene, roadway, models)
        tick!(scene, roadway, actions, models)
        update!(rec, scene)
    end

    return rec
end

function _run_callbacks(rec, roadway, models, callbacks)
    isdone = false
    for callback in callbacks
        isdone |= callback(rec, roadway, models)
    end
    return isdone
end
function simulate!{D<:DriverModel}(
    rec::SceneRecord,
    scene::Scene,
    roadway::Roadway,
    models::Dict{Int,D},
    nticks::Int,
    callbacks::Vector, # run after any scene is added to the SceneRecord
                       # a callback need only implement eval(rec, roadway, models)
                       # and return whether to terminate early
    )

    empty!(rec)
    update!(rec, scene)

    # potential early out right off the bat
    if _run_callbacks(rec, roadway, models, callbacks)
        return rec
    end

    actions = Array(DriveAction, length(scene))
    for tick in 1 : nticks
        get_actions!(actions, scene, roadway, models)
        tick!(scene, roadway, actions, models)
        update!(rec, scene)
        if _run_callbacks(rec, roadway, models, callbacks)
            break
        end
    end

    return rec
end


"""
    Run a simulation and store the resulting scenes in the provided SceneRecord.
Only the ego vehicle is simulated; the other vehicles are as they were in the provided trajdata
Other vehicle states will be interpolated
"""
function simulate!(
    rec::SceneRecord,
    models::DriverModel,
    egoid::Int,
    trajdata::Trajdata,
    time_start::Float64,
    time_end::Float64;
    prime_history::Int=0, # no prime-ing
    scene::Scene = Scene(),
    )

    Δt = rec.timestep
    roadway = trajdata.roadway

    # prime with history
    empty!(rec)
    reset_hidden_state!(model)
    for h in 1:prime_history
        t = time_start - Δt * (prime_history - h + 1)
        update!(rec, get!(scene, trajdata, t))
        observe!(model, scene, roadway, egoid)
    end

    # add current frame
    update!(rec, get!(scene, trajdata, time_start))
    observe!(model, scene, roadway, egoid)

    # run simulation
    t = time_start
    context = action_context(model)
    prev_ego_state = get_by_id(scene, egoid).state
    while t < time_end

        # pull orig scene
        get!(scene, trajdata, t)

        # propagate ego vehicle and set
        ego_action = rand(model)
        ego_veh = get_by_id(scene, egoid)
        ego_veh.state = prev_ego_state
        prev_ego_state = ego_veh.state = propagate(ego_veh, ego_action, context, roadway)

        # update record
        update!(rec, scene)

        # observe
        observe!(model, scene, roadway, ego_veh.def.id)

        # update time
        t += Δt
    end

    rec
end