get_center(veh::Entity{PosSpeed1D, BoundingBoxDef, Int}) = veh.state.s
get_footpoint(veh::Entity{PosSpeed1D, BoundingBoxDef, Int}) = veh.state.s
get_front(veh::Entity{PosSpeed1D, BoundingBoxDef, Int}) = veh.state.s + veh.def.len/2
get_rear(veh::Entity{PosSpeed1D, BoundingBoxDef, Int}) = veh.state.s - veh.def.len/2

function get_headway(
    veh_rear::Entity{PosSpeed1D, BoundingBoxDef, Int},
    veh_fore::Entity{PosSpeed1D, BoundingBoxDef, Int},
    roadway::Straight1DRoadway,
    )

    return get_headway(get_front(veh_rear), get_rear(veh_fore), roadway)
end
function get_neighbor_fore(
    scene::EntityFrame{PosSpeed1D, BoundingBoxDef, Int},
    vehicle_index::Int,
    roadway::Straight1DRoadway,
    )

    ego = scene[vehicle_index]
    best_ind = 0
    best_gap = Inf
    for (i,veh) in enumerate(scene)
        if i != vehicle_index
            Δs = get_headway(ego, veh, roadway)
            if Δs < best_gap
                best_gap, best_ind = Δs, i
            end
        end
    end
    return NeighborLongitudinalResult(best_ind, best_gap)
end
function get_neighbor_rear(
    scene::EntityFrame{PosSpeed1D, BoundingBoxDef, Int},
    vehicle_index::Int,
    roadway::Straight1DRoadway,
    )

    ego = scene[vehicle_index]
    best_ind = 0
    best_gap = Inf
    for (i,veh) in enumerate(scene)
        if i != vehicle_index
            Δs = get_headway(veh, ego, roadway)
            if Δs < best_gap
                best_gap, best_ind = Δs, i
            end
        end
    end
    return NeighborLongitudinalResult(best_ind, best_gap)
end

include("propagate.jl")
include("features.jl")
include("behaviors.jl")