typealias MobiusVehicle Entity{State1D, VehicleDef, Int}
typealias MobiusScene Frame{MobiusVehicle}
MobiusScene(n::Int=100) = Frame(MobiusVehicle, n)
MobiusScene(arr::Vector{MobiusVehicle}) = Frame{MobiusVehicle}(arr, length(arr))

get_center(veh::MobiusVehicle) = veh.state.s
get_footpoint(veh::MobiusVehicle) = veh.state.s
get_front(veh::MobiusVehicle) = veh.state.s + veh.def.length/2
get_rear(veh::MobiusVehicle) = veh.state.s - veh.def.length/2

function get_headway(veh_rear::MobiusVehicle, veh_fore::MobiusVehicle, roadway::StraightRoadway)
    return get_headway(get_front(veh_rear), get_rear(veh_fore), roadway)
end
function get_neighbor_fore(scene::MobiusScene, vehicle_index::Int, roadway::StraightRoadway)
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
function get_neighbor_rear(scene::MobiusScene, vehicle_index::Int, roadway::StraightRoadway)
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