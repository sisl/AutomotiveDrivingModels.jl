"""
    get_actions!(actions::Vector{A}, scene::EntityFrame{S,D,I}, roadway::R, models::Dict{I, M},) where {S,D,I,A,R,M<:DriverModel}
Fill in `actions` with the actions of each agent present in the scene. It calls `observe!`
and `rand` for each driver models. 
`actions` will contain the actions to apply to update the state of each vehicle.
"""
function get_actions!(
    actions::Vector{A},
    scene::EntityFrame{S,D,I},
    roadway::R,
    models::Dict{I, M}, # id → model
    ) where {S,D,I,A,R,M<:DriverModel}


    for (i,veh) in enumerate(scene)
        model = models[veh.id]
        observe!(model, scene, roadway, veh.id)
        actions[i] = rand(model)
    end

    actions
end

"""
    tick!(scene::EntityFrame{S,D,I}, roadway::R, actions::Vector{A}, Δt::Float64) where {S,D,I,A,R}
update `scene` in place by updating the state of each vehicle given their current action in `actions`. 
It calls the `propagate` method for each vehicle in the scene.
"""
function tick!(
    scene::EntityFrame{S,D,I},
    roadway::R,
    actions::Vector{A},
    Δt::Float64,
    ) where {S,D,I,A,R}

    for i in 1 : length(scene)
        veh = scene[i]
        state′ = propagate(veh, actions[i], roadway, Δt)
        scene[i] = Entity(state′, veh.def, veh.id)
    end

    return scene
end

"""
    reset_hidden_states!(models::Dict{Int,M}) where {M<:DriverModel}
reset hidden states of all driver models in `models`
"""
function reset_hidden_states!(models::Dict{Int,M}) where {M<:DriverModel}
    for model in values(models)
        reset_hidden_state!(model)
    end
    return models
end

"""
    simulate!(::Type{A}, rec::EntityQueueRecord{S,D,I}, scene::EntityFrame{S,D,I}, roadway::R, models::Dict{I,M}, nticks::Int) where {S,D,I,A,R,M<:DriverModel}
    simulate!(rec::EntityQueueRecord{S,D,I}, scene::EntityFrame{S,D,I}, roadway::R, models::Dict{I,M}, nticks::Int) where {S,D,I,R,M<:DriverModel}
Run nticks of simulation and place all nticks+1 scenes into the QueueRecord

    simulate!(::Type{A},rec::EntityQueueRecord{S,D,I}, scene::EntityFrame{S,D,I}, roadway::R, models::Dict{I,M}, nticks::Int, callbacks::C) where {S,D,I,A,R,M<:DriverModel,C<:Tuple{Vararg{Any}}}
    simulate!(rec::EntityQueueRecord{S,D,I}, scene::EntityFrame{S,D,I}, roadway::R, models::Dict{I,M}, nticks::Int, callbacks::C) where {S,D,I,A,R,M<:DriverModel,C<:Tuple{Vararg{Any}}}
Callback objects can also be passed in the simulate! function.

"""
function simulate!(
    ::Type{A},
    rec::EntityQueueRecord{S,D,I},
    scene::EntityFrame{S,D,I},
    roadway::R,
    models::Dict{I,M},
    nticks::Int,
    ) where {S,D,I,A,R,M<:DriverModel}

    empty!(rec)
    update!(rec, scene)
    actions = Array{A}(undef, length(scene))

    for tick in 1 : nticks
        get_actions!(actions, scene, roadway, models)
        tick!(scene, roadway, actions, rec.timestep)
        update!(rec, scene)
    end

    return rec
end
function simulate!(
    rec::EntityQueueRecord{S,D,I},
    scene::EntityFrame{S,D,I},
    roadway::R,
    models::Dict{I,M},
    nticks::Int,
    ) where {S,D,I,R,M<:DriverModel}

    return simulate!(Any, rec, scene, roadway, models, nticks)
end

"""
    Run a simulation and store the resulting scenes in the provided QueueRecord.
Only the ego vehicle is simulated; the other vehicles are as they were in the provided trajdata
Other vehicle states will be interpolated
"""
function simulate!(
    rec::EntityQueueRecord{S,D,I},
    model::DriverModel,
    egoid::I,
    trajdata::ListRecord{S,D,I},
    roadway::R,
    frame_start::Int,
    frame_end::Int;
    prime_history::Int=0, # no prime-ing
    scene::EntityFrame{S,D,I} =  allocate_frame(trajdata),
    ) where {S,D,I,R}

    @assert(isapprox(get_timestep(rec), get_timestep(trajdata)))

    # prime with history
    prime_with_history!(model, trajdata, roadway, frame_start, frame_end, egoid, scene)

    # add current frame
    update!(rec, get!(scene, trajdata, frame_start))
    observe!(model, scene, roadway, egoid)

    # run simulation
    frame_index = frame_start
    ego_veh = get_by_id(scene, egoid)
    while frame_index < frame_end

        # pull original scene
        get!(scene, trajdata, frame_index)

        # propagate ego vehicle and set
        ego_action = rand(model)
        ego_state = propagate(ego_veh, ego_action, roadway, get_timestep(rec))
        ego_veh = Entity(ego_veh, ego_state)
        scene[findfirst(ego_veh.id, scene)] = ego_veh

        # update record
        update!(rec, scene)

        # observe
        observe!(model, scene, roadway, ego_veh.id)

        # update time
        frame_index += 1
    end

    return rec
end