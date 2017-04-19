typealias Vehicle1D Entity{State1D, VehicleDef, Int}
typealias Scene1D Frame{Vehicle1D}
Scene1D(n::Int=100) = Frame(Vehicle1D, n)
Scene1D(arr::Vector{Vehicle1D}) = Frame{Vehicle1D}(arr, length(arr))

get_center(veh::Vehicle1D) = veh.state.s
get_footpoint(veh::Vehicle1D) = veh.state.s
get_front(veh::Vehicle1D) = veh.state.s + veh.def.length/2
get_rear(veh::Vehicle1D) = veh.state.s - veh.def.length/2

function get_headway(veh_rear::Vehicle1D, veh_fore::Vehicle1D, roadway::StraightRoadway)
    return get_headway(get_front(veh_rear), get_rear(veh_fore), roadway)
end
function get_neighbor_fore(scene::Scene1D, vehicle_index::Int, roadway::StraightRoadway)
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
function get_neighbor_rear(scene::Scene1D, vehicle_index::Int, roadway::StraightRoadway)
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