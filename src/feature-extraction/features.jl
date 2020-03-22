## define features traits

# most general feature(roadway, scenes, actions, vehid)
abstract type AbstractFeature end

# feature(roadway, veh)
struct EntityFeature <: AbstractFeature end
# feature(roadway, scene, veh)
struct SceneFeature <: AbstractFeature end
# feature(roadway, scenes, veh)
struct TemporalFeature <: AbstractFeature end
# feature(actions, veh)
# struct TemporalActionFeature <: AbstractFeature end 
# feature(action, veh)
# struct ActionFeature <: AbstractFeature end

"""
    featuretype(f::Function)
function used for dispatching feature types depending on the method of the feature function 
"""
function featuretype(f::Function)
    if static_hasmethod(f, Tuple{Roadway, Entity})
        return EntityFeature()
    elseif static_hasmethod(f, Tuple{Roadway, Scene, Entity})
        return SceneFeature()
    elseif static_hasmethod(f, Tuple{Roadway, Vector{<:Scene}, Entity})
        return TemporalFeature()
    else
        error(""""
              unsupported feature type
              try adding a new feature trait or implement one of the existing feature type supported
              """)
    end
    return 
end

## Feature extraction functions 

# Top level extract several features
"""
    extract_features(features, roadway::Roadway, scenes::Vector{<:Scene}, ids::Vector{I}) where I 

Extract information from a list of scenes. It returns a dictionary of DataFrame objects. 

# Inputs

- `features`: a tuple of feature functions. The feature types supported are `EntityFeature`, `SceneFeature` and `TemporalFeature`. Check the documentation for the list of available feature functions, and how to implement you own feature function.
- `roadway::Roadway` : the roadway associated to the scenes.
- `scenes::Vector{<:Scene}` : the simulation data from which we wish to extract information from. Each scene in the vector corresponds to one time step.
- `ids::Vector{I}`: a list of entity IDs for which we want to extract the information. 

# Output

A dictionary mapping IDs to `DataFrame` objects. For a given ID, the DataFrame's columns correspond to the name of the input feature functions. 
The row correspond to the feature value for each scene (time history).

# Example:

```julia
roadway = gen_straight_roadway(4, 100.0) 

scene_0 = Scene([
            Entity(VehicleState(VecSE2( 0.0,0.0,0.0), roadway, 10.0), VehicleDef(AgentClass.CAR, 5.0, 2.0), 1),
            Entity(VehicleState(VecSE2(10.0,0.0,0.0), roadway, 10.0), VehicleDef(AgentClass.CAR, 5.0, 2.0), 2),
        ])

scene_1 = Scene([
            Entity(VehicleState(VecSE2( 10.0,0.0,0.0), roadway, 10.0), VehicleDef(AgentClass.CAR, 5.0, 2.0), 1),
            Entity(VehicleState(VecSE2(20.0,0.0,0.0), roadway, 10.0), VehicleDef(AgentClass.CAR, 5.0, 2.0), 2),
        ])

dfs = extract_features((posgx, iscolliding), roadway, [scene_0, scene_1], [1,2])

dfs[1].posgx # history of global x position for vehicle of ID 1
```
""" 
function extract_features(features, roadway::Roadway, scenes::Vector{<:Scene}, ids::Vector{I}) where I 
    dfs_list = broadcast(f -> extract_feature(featuretype(f), f, roadway, scenes, ids), features)
    # join all the dataframes 
    dfs = Dict{I, DataFrame}()
    for id in ids 
        dfs[id] = DataFrame([Symbol(features[i]) => dfs_list[i][id] for i=1:length(features)])
    end
    return dfs
end


# feature is a general (temporal) feature, broadcast on id only
function extract_feature(ft::AbstractFeature, feature, roadway::Roadway, scenes::Vector{<:Scene}, ids)
    values = feature.(Ref(roadway), Ref(scenes), ids)  # vector of vectors
    return Dict(zip(ids, values))
end

# feature is a scene feature, broadcast on ids and scenes
function extract_feature(ft::SceneFeature, feature, roadway::Roadway, scenes::Vector{<:Scene}, ids)
    values = broadcast(ids) do id 
        veh_hist = get_by_id.(scenes, Ref(id))
        feature.(Ref(roadway), scenes, veh_hist)
    end
    return Dict(zip(ids, values))
end

# feature is an entity feature, broadcast on ids and scenes 
function extract_feature(ft::EntityFeature, feature, roadway::Roadway, scenes::Vector{<:Scene}, ids)
    values = broadcast(ids) do id
        veh_hist= get_by_id.(scenes, Ref(id))
        feature.(Ref(roadway), veh_hist)
    end
    return Dict(zip(ids, values))
end

## feature functions 

# vehicle only features
"""
    posgx(roadway::Roadway, veh::Entity)
returns x position in the global frame
""" 
posgx(roadway::Roadway, veh::Entity) = posg(veh).x

"""
    posgy(roadway::Roadway, veh::Entity)
returns y position in the global frame 
"""
posgy(roadway::Roadway, veh::Entity) = posg(veh).y

"""
    posgθ(roadway::Roadway, veh::Entity)
returns heading in the global frame
"""
posgθ(roadway::Roadway, veh::Entity) = posg(veh).θ

"""
    posfs(roadway::Roadway, veh::Entity)
returns longitudinal position in Frenet frame
"""
posfs(roadway::Roadway, veh::Entity) = posf(veh).s

"""
    posft(roadway::Roadway, veh::Entity)
returns lateral position in Frenet frame
"""
posft(roadway::Roadway, veh::Entity) = posf(veh).t

"""
    posfϕ(roadway::Roadway, veh::Entity)
returns heading relative to centerline of the lane
"""
posfϕ(roadway::Roadway, veh::Entity) = posf(veh).ϕ

"""
    vel(roadway::Roadway, veh::Entity)
returns the velocity of `veh`
"""
vel(roadway::Roadway, veh::Entity) = vel(veh)

"""
    velfs(roadway::Roadway, veh::Entity)
returns the longitudinal velocity in the Frenet frame
"""
velfs(roadway::Roadway, veh::Entity) = velf(veh).s

"""
    velft(roadway::Roadway, veh::Entity)
returns the lateral velocity in the Frenet frame
"""
velft(roadway::Roadway, veh::Entity) = velf(veh).t

"""
    velgx(roadway::Roadway, veh::Entity)
returns the velocity in the x direction of the global frame
"""
velgx(roadway::Roadway, veh::Entity) = velg(veh).x

"""
    velgy(roadway::Roadway, veh::Entity)
returns the velocity in the y direction of the global frame
"""
velgy(roadway::Roadway, veh::Entity) = velg(veh).y

"""
    time_to_crossing_right(roadway::Roadway, veh::Entity)
time before crossing the right lane marker assuming constant lateral speed.
"""
function time_to_crossing_right(roadway::Roadway, veh::Entity)
    d_mr = markerdist_right(roadway, veh)
    velf_t = velf(veh).t
    if d_mr > 0.0 && velf_t < 0.0
        return -d_mr / velf_t 
    else 
        return Inf 
    end
end

"""
    time_to_crossing_left(roadway::Roadway, veh::Entity)
time before crossing the left lane marker assuming constant lateral speed.
"""
function time_to_crossing_left(roadway::Roadway, veh::Entity)
    d_mr = markerdist_left(roadway, veh)
    velf_t = velf(veh).t
    if d_mr > 0.0 && velf_t > 0.0
        return -d_mr / velf_t 
    else 
        return Inf 
    end
end

"""
    estimated_time_to_lane_crossing(roadway::Roadway, veh::Entity)
minimum of the time to crossing left and time to crossing right.
"""
function estimated_time_to_lane_crossing(roadway::Roadway, veh::Entity)
    ttcr_left = time_to_crossing_left(roadway, veh)
    ttcr_right = time_to_crossing_right(roadway, veh)
    return min(ttcr_left, ttcr_right)
end

"""
    iswaiting(roadway::Roadway, veh::Entity)
returns true if the vehicle is waiting (if the velocity is 0.0)
"""
iswaiting(roadway::Roadway, veh::Entity) = vel(veh) ≈ 0.0

## scene features 

"""
    iscolliding(roadway::Roadway, scene::Scene, veh::Entity) 
returns true if the vehicle is colliding with another vehicle in the scene.
"""
function iscolliding(roadway::Roadway, scene::Scene, veh::Entity) 
    return collision_checker(scene, veh.id)
end

"""
    distance_to(egoid)
generate a feature function distance_to_\$egoid.

`distance_to_\$egoid(roadway, scene, veh)` returns the distance between veh and the vehicle of id egoid in the scene.
"""
function distance_to(egoid)
    @eval begin 
        function ($(Symbol("distance_to_$(egoid)")))(roadway::Roadway, scene::Scene, veh::Entity)
            ego = get_by_id(scene, $egoid)
            return norm(VecE2(posg(ego) - posg(veh)))
        end
        export $(Symbol("distance_to_$(egoid)"))
        $(Symbol("distance_to_$(egoid)"))
    end
end

# temporal features 

"""
    acc(roadway::Roadway, scenes::Vector{<:Scene}, vehid)
returns the history of acceleration of the entity of id vehid using finite differences on the velocity.
The first element is `missing`.
"""
function acc(roadway::Roadway, scenes::Vector{<:Scene}, vehid)
    result = zeros(Union{Missing,Float64},length(scenes))
    angles = broadcast(x->vel(get_by_id(x, vehid)), scenes)
    result[1] = missing
    result[2:end] .= diff(angles)
    return result
end

"""
    accfs(roadway::Roadway, scenes::Vector{<:Scene}, vehid)
returns the history of longitudinal acceleration in the Frenet frame of the entity of id vehid using finite differences on the velocity.
The first element is `missing`.
"""
function accfs(roadway::Roadway, scenes::Vector{<:Scene}, vehid)
    result = zeros(Union{Missing,Float64},length(scenes))
    angles = broadcast(x->velf(get_by_id(x, vehid)).s, scenes)
    result[1] = missing
    result[2:end] .= diff(angles)
    return result
end

"""
    accft(roadway::Roadway, scenes::Vector{<:Scene}, vehid)
returns the history of lateral acceleration in the Frenet frame of the entity of id vehid using finite differences on the velocity.
The first element is `missing`.
"""
function accft(roadway::Roadway, scenes::Vector{<:Scene}, vehid)
    result = zeros(Union{Missing,Float64},length(scenes))
    angles = broadcast(x->velf(get_by_id(x, vehid)).t, scenes)
    result[1] = missing
    result[2:end] .= diff(angles)
    return result
end

"""
    jerk(roadway::Roadway, scenes::Vector{<:Scene}, vehid)
returns the jerk history of the entity of id vehid using finite differences on acceleration (which uses finite differences on velocity).
The first two elements are missing. 
"""
function jerk(roadway::Roadway, scenes::Vector{<:Scene}, vehid)
    return vcat(missing, diff(acc(roadway, scenes, vehid)))
end

"""
    jerkfs(roadway::Roadway, scenes::Vector{<:Scene}, vehid)
returns the longitudinal jerk history in the Frenet frame of the entity of id vehid using finite differences on acceleration (which uses finite differences on velocity).
The first two elements are missing. 
"""
function jerkfs(roadway::Roadway, scenes::Vector{<:Scene}, vehid)
    return vcat(missing, diff(accfs(roadway, scenes, vehid)))
end

"""
    jerkft(roadway::Roadway, scenes::Vector{<:Scene}, vehid)
returns the lateral jerk history in the Frenet frame of the entity of id vehid using finite differences on acceleration (which uses finite differences on velocity).
The first two elements are missing. 
"""
function jerkft(roadway::Roadway, scenes::Vector{<:Scene}, vehid)
    return vcat(missing, diff(accft(roadway, scenes, vehid)))
end

"""
    turn_rate_g(roadway::Roadway, scenes::Vector{<:Scene}, vehid)
returns the turn rate history in the Frenet frame of the entity of id vehid using finite differences on global heading.
The first element is missing. 
"""
function turn_rate_g(roadway::Roadway, scenes::Vector{<:Scene}, vehid)
    result = zeros(Union{Missing,Float64},length(scenes))
    angles = broadcast(x->posg(get_by_id(x, vehid)).θ, scenes)
    result[1] = missing
    result[2:end] .= diff(angles)
    return result
end

"""
    turn_rate_g(roadway::Roadway, scenes::Vector{<:Scene}, vehid)
returns the turn rate history in the Frenet frame of the entity of id vehid using finite differences on heading in the Frenet frame.
The first element is missing. 
"""
function turn_rate_f(roadway::Roadway, scenes::Vector{<:Scene}, vehid)
    result = zeros(Union{Missing,Float64},length(scenes))
    angles = broadcast(x->posf(get_by_id(x, vehid)).ϕ, scenes)
    result[1] = missing
    result[2:end] .= diff(angles)
    return result
end

"""
    isbraking(roadway::Roadway, scenes::Vector{<:Scene}, vehid)
history of braking events: true when the acceleration is negative.
The first element is missing.
"""
function isbraking(roadway::Roadway, scenes::Vector{<:Scene}, vehid)
    return acc(roadway, scenes, vehid) .< 0.0
end

"""
    isaccelerating(roadway::Roadway, scenes::Vector{<:Scene}, vehid)
history of acceleration events: true when the acceleration is positive
The first element is missing.
"""
function isaccelerating(roadway::Roadway, scenes::Vector{<:Scene}, vehid)
    return acc(roadway, scenes, vehid) .> 0.0
end
