type Scene
    trajdataid::Int
    vehicles::Vector{Vehicle} # this is a pre-allocated array that is at least as large as the maximum number of vehicles in a Trajdata frame
    n_vehicles::Int

    function Scene(n_vehicles::Int=500; trajdataid::Int=-1)
        vehicles = Array(Vehicle, n_vehicles)
        for i in 1 : length(vehicles)
            vehicles[i] = Vehicle()
        end
        new(trajdataid, vehicles, 0)
    end
    function Scene(
        vehicles::Vector{Vehicle},
        n_vehicles::Int=length(vehicles);
        trajdataid::Int=-1,
        )

        new(trajdataid, vehicles, n_vehicles)
    end
end

# iteration
Base.start(scene::Scene) = 1
Base.done(scene::Scene, i::Int) = i > length(scene)
Base.next(scene::Scene, i::Int) = (scene.vehicles[i], i+1)

# copying
function Base.copy!(dest::Scene, src::Scene)
    for i in 1 : src.n_vehicles
        copy!(dest.vehicles[i], src.vehicles[i])
    end
    dest.n_vehicles = src.n_vehicles
    dest.trajdataid = src.trajdataid
    dest
end

function Base.push!(scene::Scene, veh::Vehicle)
    scene.n_vehicles += 1
    scene.vehicles[scene.n_vehicles] = veh
    scene
end

Base.length(scene::Scene) = scene.n_vehicles
Base.getindex(scene::Scene, i::Int) = scene.vehicles[i]
function Base.setindex!(scene::Scene, veh::Vehicle, i::Int)
    scene.vehicles[i] = veh
    scene
end
function Base.empty!(scene::Scene)
    scene.n_vehicles = 0
    scene
end
function Base.get!(scene::Scene, trajdata::Trajdata, frame::Int)

    scene.trajdataid = trajdata.id
    scene.n_vehicles = 0

    if frame_inbounds(trajdata, frame)
        frame = trajdata.frames[frame]
        for i in frame.lo : frame.hi
            scene.n_vehicles += 1
            veh = scene.vehicles[scene.n_vehicles]
            s = trajdata.states[i]
            veh.state = s.state
            veh.def = get_vehicledef(trajdata, s.id)
        end
    end

    scene
end
function Base.deleteat!(scene::Scene, vehicle_index::Int)
    for i in vehicle_index : scene.n_vehicles - 1
        copy!(scene.vehicles[i], scene.vehicles[i+1])
    end
    scene.n_vehicles -= 1
    scene
end

function get_index_of_first_vehicle_with_id(scene::Scene, id::Int)
    retval = 0
    for i in 1 : scene.n_vehicles
        if scene.vehicles[i].def.id == id
            retval = i
            break
        end
    end
    retval
end
iscarinframe(scene::Scene, id::Int) = get_index_of_first_vehicle_with_id(scene, id) != 0

function get_vehiclestate(scene::Scene, id::Int)
    for i in 1 : scene.n_vehicles
        if scene.vehicles[i].def.id == id
            return scene.vehicles[i].state
        end
    end
    error("get_vehiclestate: vehicle with id $id not found")
end
function get_vehicle!(veh::Vehicle, scene::Scene, id::Int)
    for i in 1 : scene.n_vehicles
        if scene.vehicles[i].def.id == id
            copy!(veh, scene.vehicles[i])
            return veh
        end
    end
    error("get_vehicle!: vehicle with id $id not found")
end
get_vehicle(scene::Scene, id::Int) = get_vehicle!(Vehicle(), scene, id)