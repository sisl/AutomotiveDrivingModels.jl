export
        get_actions!,
        tick!

function get_actions!{A<:DriveAction}(
    actions::Vector{A},
    scene::Scene,
    roadway::Roadway,
    models::Dict{Int, DriverModel}, # id → model
    )


    i = 0
    for veh in scene
        if haskey(models, veh.id)
            model = models[veh.id]
            observe!(model, scene, roadway, veh.id)
            actions[i+=1] = rand(model)
        end
    end

    actions
end

function tick!(
    scene::Scene,
    roadway::Roadway,
    models::Dict{Int, DriverModel}, # id → model
    frame::Int,
    actions::Vector{DriveAction},
    )

    roadway = get_roadway(scene)

    j = 0
    for veh in scene
        if haskey(models, veh.id)
            veh.state = propagate(veh, actions[j+=1], roadway)
        else
            if iscarinframe(trajdata, veh.id, frame)
                veh.state = get_vehiclestate(trajdata, veh.id, frame)
            else
                veh.state = VehicleState(VecSE2(NaN,NaN,NaN), NaN) # car disappears
            end
        end
    end

    scene
end
function tick!(
    scene::Scene,
    actions::Vector{DriveAction},
    )

    roadway = get_roadway(scene)

    for (veh_index, veh) in enumerate(scene)
        veh.state = propagate(veh, actions[veh_index], roadway)
    end

    scene
end