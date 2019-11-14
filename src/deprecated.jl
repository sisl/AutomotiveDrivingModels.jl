@deprecate get_vel_s velf(state).s
@deprecate get_vel_t velf(state).t

@deprecate propagate(veh, state, roadway, Δt) propagate(veh, state, a, roadway, Δt)

function simulate!(
    scene::Frame{E},
    roadway::R,
    models::Dict{I,M},
    nticks::Int64,
    timestep::Float64;
    rng::AbstractRNG = Random.GLOBAL_RNG,
    callbacks = nothing
    ) where {E,A,R,I,M<:DriverModel}
    Base.depwarn(
"`simulate!` without specifying a pre-allocated data structure for `scenes` is now deprecated.\n
You probably want to use the `simulate` function instead.\n
Alternatively, you can provide a pre-allocated data structure via the `scenes=` keyword",
        :simulate_no_prealloc
    )
    return simulate(scene, roadway, models, nticks, timestep, rng=rng, callbacks=callbacks)
end

function run_callback(callback, scenes, actions::Union{Nothing, Vector{Frame{ActionMapping}}}, roadway, models, tick)
    Base.depwarn(
"Using a deprecated version of `run_callback`. Since v0.7.10, user-defined callback functions should also take an `actions` argument.\n
Check the section `Simulation > Callbacks` in the package documentation for more information
 on how to define callback functions.\n
Your function call is being forwarded to `run_callback` without actions argument.",
        :run_callback_no_actions
    )
    return run_callback(callback, scenes, roadway, models, tick)
end
