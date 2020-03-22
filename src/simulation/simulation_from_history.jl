"""
    observe_from_history!(model::DriverModel, roadway::Roadway, trajdata::Vector{<:EntityFrame}, egoid, start::Int, stop::Int)

Given a prerecorded trajectory `trajdata`, run the observe function of a driver model for the scenes between `start` and `stop` for the vehicle of id `egoid`.
The ego vehicle does not take any actions, it just observe the scenes,
"""
function observe_from_history!(
        model::DriverModel,
        roadway::Roadway,
        trajdata::Vector{<:EntityFrame},
        egoid,
        start::Int = 1, 
        stop::Int = length(trajdata))
    reset_hidden_state!(model)
    
    for i=start:stop
        observe!(model, trajdata[i], roadway, egoid)
    end

    return model
end

"""
    maximum_entities(trajdata::Vector{<:EntityFrame})
return the maximum number of entities present in a given frame of the trajectory.
"""
function maximum_entities(trajdata::Vector{<:EntityFrame})
    return maximum(capacity, trajdata)
end

"""
    simulate_from_history(model::DriverModel, roadway::Roadway, trajdata::Vector{Frame{E}}, egoid, timestep::Float64, 
                          start::Int = 1, stop::Int = length(trajdata);
                          rng::AbstractRNG = Random.GLOBAL_RNG) where {E<:Entity}

Replay the given trajectory except for the entity `egoid` which follows the given driver model. 
See `simulate_from_history!` if you want to provide a container for the results or log actions.
"""
function simulate_from_history(
        model::DriverModel,
        roadway::Roadway, 
        trajdata::Vector{Frame{E}},
        egoid,
        timestep::Float64,
        start::Int = 1,
        stop::Int = length(trajdata);
        rng::AbstractRNG = Random.GLOBAL_RNG
        ) where {E<:Entity}
    scenes = [Frame(E, maximum_entities(trajdata)) for i=1:(stop - start + 1)]
    n = simulate_from_history!(model, roadway, trajdata, egoid, timestep,
                               start, stop, scenes, 
                               rng=rng)
    return scenes[1:(n+1)]
end

"""
    simulate_from_history!(model::DriverModel, roadway::Roadway, trajdata::Vector{Frame{E}}, egoid, timestep::Float64, 
                          start::Int, stop::Int, scenes::Vector{Frame{E}};
                          actions::Union{Nothing, Vector{Frame{A}}} = nothing, rng::AbstractRNG = Random.GLOBAL_RNG) where {E<:Entity}

Replay the given trajectory except for the entity `egoid` which follows the given driver model. 
The resulting trajectory is stored in `scenes`, the actions of the ego vehicle are stored in `actions`.

"""
function simulate_from_history!(
    model::DriverModel, 
    roadway::Roadway,
    trajdata::Vector{Frame{E}},
    egoid,
    timestep::Float64,
    start::Int,
    stop::Int,
    scenes::Vector{Frame{E}};
    actions::Union{Nothing, Vector{Frame{A}}} = nothing,
    rng::AbstractRNG = Random.GLOBAL_RNG
    ) where {E<:Entity, A<:EntityAction}

    # run model (unsure why it is needed, it was in the old code )
    observe_from_history!(model, roadway, trajdata, egoid, start, stop)

    copyto!(scenes[1], trajdata[start])
    for tick=1:(stop - start)
        
        empty!(scenes[tick + 1])
        if (actions !== nothing) empty!(actions[tick]) end

        ego = get_by_id(scenes[tick], egoid)
        observe!(model, scenes[tick], roadway, egoid)
        a = rand(rng, model)

        ego_state_p = propagate(ego, a, roadway, timestep)

        copyto!(scenes[tick+1], trajdata[start+tick])
        egoind = findfirst(egoid, scenes[tick+1])
        scenes[tick+1][egoind] = Entity(ego_state_p, ego.def, egoid)

        if (actions !== nothing) push!(actions[tick], EntityAction(a, egoid)) end

    end
    return (stop - start)
end
