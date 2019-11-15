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
    DEPRECATION WARNING: this version of `simulate!` is now deprecated.

    simulate!(scene::Frame{E}, roadway::R, models::Dict{I,M<:DriverModel}, nticks::Int64, timestep::Float64; rng::AbstractRNG = Random.GLOBAL_RNG, scenes::Vector{Frame{E}} = [Frame(E, length(scene)) for i=1:nticks+1], callbacks=nothing)

Run `nticks` steps of simulation with time step `dt` and return a vector of scenes from time step 0 to nticks.

    simulate!(::Type{A}, rec::EntityQueueRecord{S,D,I}, scene::EntityFrame{S,D,I}, roadway::R, models::Dict{I,M<:DriverModel}, nticks::Int)
    simulate!(rec::EntityQueueRecord{S,D,I}, scene::EntityFrame{S,D,I}, roadway::R, models::Dict{I,M<:DriverModel}, nticks::Int)

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
    Base.depwarn(
"`simulate!` using `EntityQueueRecord`s is deprecated since v0.7.10 and may be removed in future versions.
 You should pass a pre-allocated vector of entitites `scenes::Vector{Frame{Entity{S,D,I}}}` to `simulate!`
 or use the convenience function `simulate` without pre-allocation instead.",
        :simulate_rec
    )

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
    nticks::Int
    ) where {S,D,I,R,M<:DriverModel}

    return simulate!(Any, rec, scene, roadway, models, nticks)
end


"""
Simulate a scene. For detailed information, consult the documentation of `simulate!`.
By default, returns a vector containing one scene per time step.
"""
function simulate(
    scene::Frame{E},
    roadway::R,
    models::Dict{I,M},
    nticks::Int64,
    timestep::Float64;
    rng::AbstractRNG = Random.GLOBAL_RNG,    
    callbacks = nothing,
) where {E<:Entity,A,R,I,M<:DriverModel}
    scenes = [Frame(E, length(scene)) for i=1:nticks+1]
    n = simulate!(
        scene, roadway, models, nticks, timestep, scenes, nothing,
        rng=rng, callbacks=callbacks
    )
    return scenes[1:(n+1)]
end


"""

    simulate!(
        scene::Frame{E}, roadway::R, models::Dict{I,M},
        nticks::Int64, timestep::Float64,
        scenes::Vector{Frame{E}}, actions::Union{Nothing, Vector{Frame{A}}} = nothing;
        rng::AbstractRNG = Random.GLOBAL_RNG, callbacks = nothing
    ) where {E<:Entity,A<:EntityAction,R,I,M<:DriverModel}

Simulate the entities in `scene` along a `roadway` for a maximum of
`nticks` time steps of size `timestep`.
Returns the number of successfully performed timesteps.

At each time step, `models` is used to determine the action for each agent.
`scenes` and `actions` are pre-allocated vectors of `Frame`s containing either
`Entity`s (for scenes) or `EntityAction`s (for actions).
If `actions` is equal to `nothing` (default), the action history is not tracked.
`scenes` must always be provided.

`callbacks` is an array of callback functions which are invoked before
the simulation starts and after every simulation step.
Any callback function can cause an early termination by returning `true`
(the default return value for callback functions should be `false`).
The random number generator for the simulation can be provided using the `rng`
keyword argument, it defaults to `Random.GLOBAL_RNG`.
"""
function simulate!(
    scene::Frame{E},
    roadway::R,
    models::Dict{I,M},
    nticks::Int64,
    timestep::Float64,
    scenes::Vector{Frame{E}},
    actions::Union{Nothing, Vector{Frame{A}}} = nothing;
    rng::AbstractRNG = Random.GLOBAL_RNG,    
    callbacks = nothing
    ) where {E<:Entity,A<:EntityAction,R,I,M<:DriverModel}

    copyto!(scenes[1], scene)

    # potential early out right off the bat
    if (callbacks !== nothing) && _run_callbacks(callbacks, scenes, actions, roadway, models, 1)
        return 0
    end

    for tick in 1:nticks

        empty!(scenes[tick + 1])
        if (actions !== nothing) empty!(actions[tick]) end
        
        for (i, veh) in enumerate(scenes[tick])

            observe!(models[veh.id], scenes[tick], roadway, veh.id)
            a = rand(rng, models[veh.id])
        
            veh_state_p  = propagate(veh, a, roadway, timestep)

            push!(scenes[tick + 1], Entity(veh_state_p, veh.def, veh.id))
            if (actions !== nothing) push!(actions[tick], EntityAction(a, veh.id)) end
            
        end

        if !(callbacks === nothing) && _run_callbacks(callbacks, scenes, actions, roadway, models, tick+1)
            return tick
        end
    end
    return nticks
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
