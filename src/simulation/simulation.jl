export
        get_actions!,
        tick!

function get_actions!{A<:DriveAction, D<:DriverModel}(
    actions::Vector{A},
    scene::Scene,
    roadway::Roadway,
    models::Dict{Int, D}, # id → model
    )


    i = 0
    for veh in scene
        if haskey(models, veh.def.id)
            model = models[veh.def.id]
            observe!(model, scene, roadway, veh.def.id)
            actions[i+=1] = rand(model)
        end
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