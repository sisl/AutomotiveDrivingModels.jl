generate_feature_functions("Dist_Front", :d_front, Float64, "m", lowerbound=0.0, can_be_missing=true)
function Base.get(::Feature_Dist_Front, rec::EntityQueueRecord{PosSpeed1D, BoundingBoxDef, Int}, roadway::Straight1DRoadway, vehicle_index::Int, pastframe::Int=0)

    scene = rec[pastframe]
    neighbor_fore = get_neighbor_fore(scene, vehicle_index, roadway)
    if neighbor_fore.ind != 0
        return FeatureValue(neighbor_fore.Δs)
    else
        return FeatureValue(200.0, FeatureState.MISSING)
    end
end


generate_feature_functions("Timegap", :timegap, Float64, "s", can_be_missing=true)
function Base.get(::Feature_Timegap, rec::EntityQueueRecord{PosSpeed1D, BoundingBoxDef, Int}, roadway::Straight1DRoadway, vehicle_index::Int, pastframe::Int=0)

    scene = rec[pastframe]
    neighbor_fore = get_neighbor_fore(scene, vehicle_index, roadway)
    if neighbor_fore.ind != 0
        v = scene[vehicle_index].state.v
        return FeatureValue(neighbor_fore.Δs / v)
    else
        return FeatureValue(15.0, FeatureState.MISSING)
    end
end