export
    run_callback


# run callback and return whether simlation should terminate
run_callback{S,D,I,R,M<:DriverModel}(callback::Any, rec::Vector{EntityFrame{S,D,I}}, roadway::R, models::Dict{I,M}, tick::Int) = error("run_callback not implemented for callback $(typeof(callback))")

function _run_callbacks{S,D,I,R,M<:DriverModel,C<:Tuple{Vararg{Any}}}(callbacks::C, rec::Vector{EntityFrame{S,D,I}}, roadway::R, models::Dict{I,M}, tick::Int)
    isdone = false
    for callback in callbacks
        isdone |= run_callback(callback, rec, roadway, models, tick)
    end
    return isdone
end
function simulate!{S,D,I,A,R,M<:DriverModel,C<:Tuple{Vararg{Any}}}(
    ::Type{A},
    scene::EntityFrame{S,D,I},
    roadway::R,
    models::Dict{I,M},
    nticks::Int,
    timestep::Float64,
    callbacks::C,
    rec::Vector{EntityFrame{S,D,I}} = Vector{EntityFrame{S,D,I}}()
    )

    push!(rec, copy(scene))

    # potential early out right off the bat
    if _run_callbacks(callbacks, rec, roadway, models, 0)
        return rec
    end

    actions = Array{A}(length(scene))
    for tick in 1 : nticks
        get_actions!(actions, scene, roadway, models)
        tick!(scene, roadway, actions, timestep)
        push!(rec, copy(scene))
        if _run_callbacks(callbacks, rec, roadway, models, tick)
            break
        end
    end

    return rec
end
function simulate!{S,D,I,R,M<:DriverModel,C<:Tuple{Vararg{Any}}}(
    scene::EntityFrame{S,D,I},
    roadway::R,
    models::Dict{I,M},
    nticks::Int,
    callbacks::C,
    rec::Vector{EntityFrame{S,D,I}} = Vector{EntityFrame{S,D,I}}()
    )

    return simulate!(Any, scene, roadway, models, nticks, timestep, callbacks, rec)
end
