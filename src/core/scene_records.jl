type SceneRecord
    scenes::Vector{Scene}
    nscenes::Int # number of active scenes

    function SceneRecord(max_n_scenes::Int, max_n_vehicles::Int=500)
        scenes = Array(Scene, max_n_scenes)
        for i in 1 : length(scenes)
            scenes[i] = Scene(max_n_vehicles)
        end
        new(scenes, 0)
    end
end

Base.length(rec::SceneRecord) = rec.nscenes

record_length(rec::SceneRecord) = length(rec.scenes)
get_scene(rec::SceneRecord, pastframe::Int) = rec.scenes[1 - pastframe]

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
    rec.nscenes += 1
    rec
end

function Base.get!(scene::Scene, rec::SceneRecord, pastframe::Int=0)
    scene_internal = get_scene(rec, pastframe)
    copy!(scene, scene_internal)
    scene
end