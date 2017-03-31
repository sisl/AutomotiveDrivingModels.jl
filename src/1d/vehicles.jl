typealias MobiusVehicle Entity{State1D, VehicleDef, Int}
typealias MobiusScene Frame{MobiusVehicle}
MobiusScene(n::Int=100) = Frame(MobiusVehicle, n)

function get_neighbor_fore(scene::MobiusScene, vehicle_index::Int, roadway::StraightRoadway)
    s_ego_fore = scene[vehicle_index].state.s + scene[vehicle_index].def.length/2
    best_ind = 0
    best_gap = Inf
    for (i,veh) in enumerate(scene)
        if i != vehicle_index
            s_oth_rear = veh.state.s - veh.def.length/2 # back point of car
            while s_oth_rear < s_ego_fore
                s_oth_rear += roadway.length
            end
            Δs = s_oth_rear - s_ego_fore
            if Δs < best_gap
                best_gap, best_ind = Δs, i
            end
        end
    end
    return NeighborLongitudinalResult(best_ind, best_gap)
end