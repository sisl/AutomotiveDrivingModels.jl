## define features traits

# most general feature(roadway, scenes, actions, vehid)
abstract type AbstractFeature end

function featuretype end

# feature(roadway, veh)
struct EntityFeature <: AbstractFeature end
# feature(roadway, scene, veh)
struct FrameFeature <: AbstractFeature end
# feature(roadway, scenes, veh)
struct TemporalFeature <: AbstractFeature end
# feature(actions, veh)
# struct TemporalActionFeature <: AbstractFeature end 
# feature(action, veh)
# struct ActionFeature <: AbstractFeature end

# macro for automatic association of methods to traits
macro feature(fun)
    f = eval(fun) # evaluate in the scope of this module
    T = fun.args[1].args[1] # hacky ? get function name
    if hasmethod(f, Tuple{Roadway, Entity})
        @eval featuretype(::typeof($T)) = EntityFeature()
    elseif hasmethod(f, Tuple{Roadway, Frame, Entity})
        @eval featuretype(::typeof($T)) = FrameFeature()
    elseif hasmethod(f, Tuple{Roadway, Vector{<:Frame}, Any})
        @eval featuretype(::typeof($T)) = TemporalFeature()
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
function extract_features(features, roadway::Roadway, scenes::Vector{<:Frame}, ids::Vector{I}) where I 
    dfs_list = broadcast(f -> extract_feature(featuretype(f), f, roadway, scenes, ids), features)
    # join all the dataframes 
    dfs = Dict{I, DataFrame}()
    for id in ids 
        dfs[id] = DataFrame([Symbol(features[i]) => dfs_list[i][id] for i=1:length(features)])
    end
    return dfs
end


# feature is a general (temporal) feature, broadcast on id only
function extract_feature(ft::AbstractFeature, feature, roadway::Roadway, scenes::Vector{<:Frame}, ids)
    values = feature.(Ref(roadway), Ref(scenes), ids)  # vector of vectors
    return Dict(zip(ids, values))
end

# feature is a frame feature, broadcast on ids and scenes
function extract_feature(ft::FrameFeature, feature, roadway::Roadway, scenes::Vector{<:Frame}, ids)
    values = broadcast(ids) do id 
        veh_hist = get_by_id.(scenes, Ref(id))
        feature.(Ref(roadway), scenes, veh_hist)
    end
    return Dict(zip(ids, values))
end

# feature is an entity feature, broadcast on ids and scenes 
function extract_feature(ft::EntityFeature, feature, roadway::Roadway, scenes::Vector{<:Frame}, ids)
    values = broadcast(ids) do id
        veh_hist= get_by_id.(scenes, Ref(id))
        feature.(Ref(roadway), veh_hist)
    end
    return Dict(zip(ids, values))
end

## feature functions 

# vehicle only features 
@feature posgx(roadway::Roadway, veh::Entity) = posg(veh).x
@feature posgy(roadway::Roadway, veh::Entity) = posg(veh).y
@feature posgθ(roadway::Roadway, veh::Entity) = posg(veh).θ
@feature posfs(roadway::Roadway, veh::Entity) = posf(veh).s
@feature posft(roadway::Roadway, veh::Entity) = posf(veh).t
@feature posfϕ(roadway::Roadway, veh::Entity) = posf(veh).ϕ
@feature vel(roadway::Roadway, veh::Entity) = vel(veh)
@feature velfs(roadway::Roadway, veh::Entity) = velf(veh).s
@feature velft(roadway::Roadway, veh::Entity) = velf(veh).t
@feature velgx(roadway::Roadway, veh::Entity) = velg(veh).x
@feature velgy(roadway::Roadway, veh::Entity) = velg(veh).y


function time_to_crossing_right(roadway::Roadway, veh::Entity)
    d_mr = markerdist_right(roadway, veh)
    velf_t = velf(veh).t
    if d_mr > 0.0 && velf_t < 0.0
        return -d_mr / velf_t 
    else 
        return Inf 
    end
end

featuretype(::typeof(time_to_crossing_right)) = EntityFeature()

function time_to_crossing_left(roadway::Roadway, veh::Entity)
    d_mr = markerdist_left(roadway, veh)
    velf_t = velf(veh).t
    if d_mr > 0.0 && velf_t > 0.0
        return -d_mr / velf_t 
    else 
        return Inf 
    end
end

featuretype(::typeof(time_to_crossing_left)) = EntityFeature()

function estimated_time_to_lane_crossing(roadway::Roadway, veh::Entity)
    ttcr_left = time_to_crossing_left(roadway, veh)
    ttcr_right = time_to_crossing_right(roadway, veh)
    return min(ttcr_left, ttcr_right)
end

featuretype(::typeof(estimated_time_to_lane_crossing)) = EntityFeature()

@feature iswaiting(roadway::Roadway, veh::Entity) = vel(veh) ≈ 0.0

# scene features 
@feature function iscolliding(roadway::Roadway, scene::Frame, veh::Entity) 
    return collision_checker(scene, veh.id)
end

function distance_to(egoid)
    @eval begin 
        @feature function ($(Symbol("distance_to_$(egoid)")))(roadway::Roadway, scene::Frame, veh::Entity)
            ego = get_by_id(scene, $egoid)
            return norm(VecE2(posg(ego) - posg(veh)))
        end
        export $(Symbol("distance_to_$(egoid)"))
        $(Symbol("distance_to_$(egoid)"))
    end
end

# temporal features 

@feature function acc(roadway::Roadway, scenes::Vector{<:Frame}, vehid)
    result = zeros(Union{Missing,Float64},length(scenes))
    angles = broadcast(x->vel(get_by_id(x, vehid)), scenes)
    result[1] = missing
    result[2:end] .= diff(angles)
    return result
end

@feature function accfs(roadway::Roadway, scenes::Vector{<:Frame}, vehid)
    result = zeros(Union{Missing,Float64},length(scenes))
    angles = broadcast(x->velf(get_by_id(x, vehid)).s, scenes)
    result[1] = missing
    result[2:end] .= diff(angles)
    return result
end

@feature function accft(roadway::Roadway, scenes::Vector{<:Frame}, vehid)
    result = zeros(Union{Missing,Float64},length(scenes))
    angles = broadcast(x->velf(get_by_id(x, vehid)).t, scenes)
    result[1] = missing
    result[2:end] .= diff(angles)
    return result
end

@feature function jerk(roadway::Roadway, scenes::Vector{<:Frame}, vehid)
    return vcat(missing, diff(acc(roadway, scenes, vehid)))
end

@feature function jerkfs(roadway::Roadway, scenes::Vector{<:Frame}, vehid)
    return vcat(missing, diff(accfs(roadway, scenes, vehid)))
end

@feature function jerkft(roadway::Roadway, scenes::Vector{<:Frame}, vehid)
    return vcat(missing, diff(accft(roadway, scenes, vehid)))
end

@feature function turn_rate_g(roadway::Roadway, scenes::Vector{<:Frame}, vehid)
    result = zeros(Union{Missing,Float64},length(scenes))
    angles = broadcast(x->posg(get_by_id(x, vehid)).θ, scenes)
    result[1] = missing
    result[2:end] .= diff(angles)
    return result
end

@feature function turn_rate_f(roadway::Roadway, scenes::Vector{<:Frame}, vehid)
    result = zeros(Union{Missing,Float64},length(scenes))
    angles = broadcast(x->posf(get_by_id(x, vehid)).ϕ, scenes)
    result[1] = missing
    result[2:end] .= diff(angles)
    return result
end

@feature function isbraking(roadway::Roadway, scenes::Vector{<:Frame}, vehid)
    return acc(roadway, scenes, vehid) .< 0.0
end

@feature function isaccelerating(roadway::Roadway, scenes::Vector{<:Frame}, vehid)
    return acc(roadway, scenes, vehid) .> 0.0
end
