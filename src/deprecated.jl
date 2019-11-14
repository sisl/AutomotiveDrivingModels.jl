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
) where {E<:Entity,A,R,I,M<:DriverModel}
    Base.depwarn(
"`simulate!` without specifying a pre-allocated data structure for `scenes` is now deprecated.\n
You probably want to use the `simulate` function instead.\n
Alternatively, you can provide a pre-allocated data structure via the `scenes=` keyword",
        :simulate_no_prealloc
    )
    return simulate(scene, roadway, models, nticks, timestep, rng=rng, callbacks=callbacks)
end

"""
    # TODO: this should be removed, but where to document the function then?
    run_callback(callback::Any, scenes::Vector{F}, roadway::R, models::Dict{I,M}, tick::Int) where {F,I,R,M<:DriverModel}    
    run_callback(callback::Any, rec::EntityQueueRecord{S,D,I}, roadway::R, models::Dict{I,M}, tick::Int) where {S,D,I,R,M<:DriverModel}
run callback and return whether simlation should terminate
A new method should be implemented when defining a new callback object.
"""
function run_callback(callback, scenes, actions::Union{Nothing, Vector{Frame{A}}}, roadway, models, tick) where {A<:EntityAction}
    Base.depwarn(
"Using a deprecated version of `run_callback`. Since v0.7.10, user-defined callback functions should also take an `actions` argument.
 If you have implemented `run_callback` with an actions argument, make sure the method signature is more specific than this one.\n
Check the section `Simulation > Callbacks` in the package documentation for more information
 on how to define callback functions.\n
Your function call is being forwarded to `run_callback` without actions argument.",
        :run_callback_no_actions
    )
    return run_callback(callback, scenes, roadway, models, tick)
end
