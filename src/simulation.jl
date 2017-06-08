"""
    observe!(models, scene, roadway)

Call observe on each model.
"""
function observe!{S,D,I,R,M<:DriverModel}(
    models::Dict{I,M}, # id → model
    scene::EntityFrame{S,D,I},
    roadway::R,
    )

    for veh in scene
        model = models[veh.id]
        observe!(model, scene, roadway, veh.id)
    end

    return models
end

"""
    get_actions!(actions, scene, roadway, models)

Get the actions for each model.
"""
function get_actions!{S,D,I,A,R,M<:DriverModel}(
    actions::Vector{A},
    scene::EntityFrame{S,D,I},
    roadway::R,
    models::Dict{I, M}, # id → model
    )

    for (i,veh) in enumerate(scene)
        model = models[veh.id]
        actions[i] = rand(model)
    end

    return actions
end

"""
    tick!(scene, roadway, actions, Δt)

Propagate a scene forward by one timestep based on the given actions.
"""
function tick!{S,D,I,A,R}(
    scene::EntityFrame{S,D,I},
    roadway::R,
    actions::Vector{A},
    Δt::Float64,
    )

    for (i,veh) in enumerate(scene)
        state′ = propagate(veh, actions[i], roadway, Δt)
        scene[i] = Entity(state′, veh.def, veh.id)
    end

    return scene
end

"""
    reset_hidden_states!(models)

Reset the hidden states in all models in the dict.
"""
function reset_hidden_states!{M<:DriverModel}(models::Dict{Int,M})
    for model in values(models)
        reset_hidden_state!(model)
    end
    return models
end

"""
Run nticks of simulation and place all nticks+1 scenes into the QueueRecord
"""
function simulate!{S,D,I,A,R,M<:DriverModel}(
    ::Type{A},
    rec::EntityQueueRecord{S,D,I},
    scene::EntityFrame{S,D,I},
    roadway::R,
    models::Dict{I,M},
    nticks::Int,
    )

    empty!(rec)
    update!(rec, scene)
    actions = Array(A, length(scene))

    for tick in 1 : nticks
        observe!(models, scene, roadway)
        get_actions!(actions, scene, roadway, models)
        tick!(scene, roadway, actions, rec.timestep)
        update!(rec, scene)
    end

    return rec
end
function simulate!{S,D,I,R,M<:DriverModel}(
    rec::EntityQueueRecord{S,D,I},
    scene::EntityFrame{S,D,I},
    roadway::R,
    models::Dict{I,M},
    nticks::Int,
    )

    return simulate!(Any, rec, scene, roadway, models, nticks)
end


"""
    Run a simulation and store the resulting scenes in the provided QueueRecord.
Only the ego vehicle is simulated; the other vehicles are as they were in the provided trajdata
Other vehicle states will be interpolated
"""
function simulate!{S,D,I}(
    rec::EntityQueueRecord{S,D,I},
    model::DriverModel,
    egoid::I,
    trajdata::ListRecord{Entity{S,D,I}},
    frame_start::Int,
    frame_end::Int;
    prime_history::Int=0, # no prime-ing
    scene::EntityFrame{S,D,I} =  allocate_frame(trajdata),
    )

    @assert(isapprox(get_timestep(rec), get_timestep(trajdata)))

    roadway = trajdata.roadway

    # prime with history
    prime_with_history!(model, trajdata, roadway, frame_start, frame_end, egoid, scene)

    # add current frame
    update!(rec, get!(scene, trajdata, time_start))
    observe!(model, scene, roadway, egoid)

    # run simulation
    frame_index = frame_start
    ego_veh = get_by_id(scene, egoid)
    while t < time_end

        # pull original scene
        get!(scene, trajdata, frame_index)

        # propagate ego vehicle and set
        ego_action = rand(model)
        ego_state = propagate(ego_veh, ego_action, roadway, get_timestep(rec))
        ego_veh = Entity(ego_veh, ego_state)
        scene[findfirst(scene, ego_veh.id)] = ego_veh

        # update record
        update!(rec, scene)

        # observe
        observe!(model, scene, roadway, ego_veh.id)

        # update time
        frame_index += 1
    end

    return rec
end
