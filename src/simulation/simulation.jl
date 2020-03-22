"""
    simulate(
        scene::Scene{E}, roadway::R, models::Dict{I,M}, nticks::Int64, timestep::Float64;
        rng::AbstractRNG = Random.GLOBAL_RNG, callbacks = nothing
    ) where {E<:Entity,A,R,I,M<:DriverModel}

Simulate a `scene`. For detailed information, consult the documentation of `simulate!`.
By default, returns a vector containing one scene per time step.
"""
function simulate(
    scene::Scene{E},
    roadway::R,
    models::Dict{I,M},
    nticks::Int64,
    timestep::Float64;
    rng::AbstractRNG = Random.GLOBAL_RNG,    
    callbacks = nothing,
) where {E<:Entity,A,R,I,M<:DriverModel}
    scenes = [Scene(E, length(scene)) for i=1:nticks+1]
    n = simulate!(
        scene, roadway, models, nticks, timestep, scenes, nothing,
        rng=rng, callbacks=callbacks
    )
    return scenes[1:(n+1)]
end

"""

    simulate!(
        scene::Scene{E}, roadway::R, models::Dict{I,M},
        nticks::Int64, timestep::Float64,
        scenes::Vector{Scene{E}}, actions::Union{Nothing, Vector{Scene{A}}} = nothing;
        rng::AbstractRNG = Random.GLOBAL_RNG, callbacks = nothing
    ) where {E<:Entity,A<:EntityAction,R,I,M<:DriverModel}

Simulate the entities in `scene` along a `roadway` for a maximum of
`nticks` time steps of size `timestep`.
Returns the number of successfully performed timesteps.

At each time step, `models` is used to determine the action for each agent.
`scenes` and `actions` are pre-allocated vectors of `Scene`s containing either
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
    scene::Scene{E},
    roadway::R,
    models::Dict{I,M},
    nticks::Int64,
    timestep::Float64,
    scenes::Vector{Scene{E}},
    actions::Union{Nothing, Vector{Scene{A}}} = nothing;
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
