"""
    PedestrianLatLonAccel
Pedestrian walking action. Acceleration in the Frenet frame, along with
desired lane after crossing the street.
"""
struct PedestrianLatLonAccel
    a_lat::Float64
    a_lon::Float64
    lane_des::Lane
end

function AutomotiveDrivingModels.propagate(ped::Entity{VehicleState, D, I},
                                           action::PedestrianLatLonAccel,
                                           roadway::Roadway,
                                           ΔT::Float64=0.1
                                           ) where {D,I}
    state = propagate(ped, LatLonAccel(action.a_lat, action.a_lon), roadway, ΔT)
    roadproj = proj(state.posG, roadway[action.lane_des.tag], roadway, move_along_curves=false)
    retval = VehicleState(Frenet(roadproj, roadway), roadway, state.v)
    return retval
end
