type SceneRecord
    scenes::Vector{Scene}
    timestep::Float64
    nscenes::Int # number of active scenes

    function SceneRecord(max_n_scenes::Int, timestep::Float64, max_n_vehicles::Int=500)
        scenes = Array(Scene, max_n_scenes)
        for i in 1 : length(scenes)
            scenes[i] = Scene(max_n_vehicles)
        end
        new(scenes, timestep, 0)
    end
end

Base.show(io::IO, rec::SceneRecord) = print(io, "SceneRecord(nscenes=", rec.nscenes, ")")

Base.length(rec::SceneRecord) = rec.nscenes
function Base.deepcopy(rec::SceneRecord)
    retval = SceneRecord(length(rec.scenes), rec.timestep, length(rec.scenes[1].vehicles))
    for i in 1 : rec.nscenes
        copy!(retval.scenes[i], rec.scenes[i])
    end
    retval
end

record_length(rec::SceneRecord) = length(rec.scenes)
pastframe_inbounds(rec::SceneRecord, pastframe::Int) = 1 ≤ 1-pastframe ≤ rec.nscenes
get_scene(rec::SceneRecord, pastframe::Int) = rec.scenes[1 - pastframe]
get_elapsed_time(rec::SceneRecord, pastframe::Int) = (1-pastframe)*rec.timestep
function get_elapsed_time(
    rec::SceneRecord,
    pastframe_farthest_back::Int,
    pastframe_most_recent::Int,
    )

    (pastframe_most_recent - pastframe_farthest_back)*rec.timestep
end

function Base.empty!(rec::SceneRecord)
    rec.nscenes = 0
    rec
end

Base.getindex(rec::SceneRecord, vehicle_index::Int, pastframe::Int) = get_scene(rec, pastframe)[vehicle_index]
Base.getindex(rec::SceneRecord, vehicle_index::Int) = Base.getindex(rec, vehicle_index, 0)

function Base.setindex!(rec::SceneRecord, veh::Vehicle, vehicle_index::Int, pastframe::Int)

    veh_internal = rec[vehicle_index, pastframe]
    copy!(veh_internal, veh)

    rec
end
Base.setindex!(rec::SceneRecord, veh::Vehicle, vehicle_index::Int) = Base.setindex!(rec, veh, vehicle_index, 0)

function get_vehiclestate(rec::SceneRecord, id::Int, pastframe::Int=0)
    scene = get_scene(rec, pastframe)
    get_vehiclestate(scene, id)
end
function get_vehicle!(veh::Vehicle, rec::SceneRecord, id::Int, pastframe::Int=0)
    scene = get_scene(rec, pastframe)
    get_vehicle!(veh, scene, id)
end
get_vehicle(rec::SceneRecord, id::Int, pastframe::Int=0) = get_vehicle!(rec, id, pastframe)

function iscarinframe(rec::SceneRecord, id::Int, pastframe::Int=0)
    scene = get_scene(rec, pastframe)
    iscarinframe(scene, id)
end
function get_index_of_first_vehicle_with_id(rec::SceneRecord, id::Int, pastframe::Int=0)
    scene = get_scene(rec, pastframe)
    get_index_of_first_vehicle_with_id(scene, id)
end

function push_back_records!(rec::SceneRecord)
    for i in min(rec.nscenes+1,length(rec.scenes)) : -1 : 2
        copy!(rec.scenes[i], rec.scenes[i-1])
    end
    rec
end
function Base.insert!(rec::SceneRecord, scene::Scene, pastframe::Int=0)
    scene_internal = get_scene(rec, pastframe)
    copy!(scene_internal, scene)
    rec
end
function update!(rec::SceneRecord, scene::Scene)
    push_back_records!(rec)
    insert!(rec, scene, 0)
    rec.nscenes = min(rec.nscenes+1, record_length(rec))
    rec
end

function Base.get!(scene::Scene, rec::SceneRecord, pastframe::Int=0)
    scene_internal = get_scene(rec, pastframe)
    copy!(scene, scene_internal)
    scene
end

function Base.convert(::Type{Trajdata}, rec::SceneRecord)

    frames = Array(TrajdataFrame, length(rec))
    states = Array(TrajdataState, nstates(rec))
    defs = Dict{Int, VehicleDef}()

    lo = 1
    time = 0.0
    for (i,pastframe) in enumerate(1-length(rec) : 0)
        scene = get_scene(rec, pastframe)

        hi = lo
        for veh in scene
            defs[veh.id] = veh.def
            states[hi] = TrajdataState(veh.id, veh.state)
            hi += 1
        end

        frames[i] = TrajdataFrame(lo, hi, time)
        lo = hi
        time += rec.timestep
    end

    return Trajdata(Roadway(), frames, states, defs)
end