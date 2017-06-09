

typealias Vehicle Entity{VehicleState,VehicleDef,Int}

Base.show(io::IO, v::Vehicle) = print(io, "Vehicle(", v.id, ", ", v.state, ", ", v.def, ")")

get_center(veh::Vehicle) = veh.state.posG
get_footpoint(veh::Vehicle) = veh.state.posG + polar(veh.state.posF.t, veh.state.posG.θ-veh.state.posF.ϕ-π/2)
get_front(veh::Vehicle) = veh.state.posG + polar(veh.def.length/2, veh.state.posG.θ)
get_rear(veh::Vehicle) = veh.state.posG - polar(veh.def.length/2, veh.state.posG.θ)

function get_lane_width(veh::Vehicle, roadway::Roadway)

    lane = roadway[veh.state.posF.roadind.tag]

    if n_lanes_left(lane, roadway) > 0
        footpoint = get_footpoint(veh)
        lane_left = roadway[LaneTag(lane.tag.segment, lane.tag.lane + 1)]
        return -proj(footpoint, lane_left, roadway).curveproj.t
    else
        return lane.width
    end
end

function get_markerdist_left(veh::Vehicle, roadway::Roadway)
    t = veh.state.posF.t
    lane_width = get_lane_width(veh, roadway)
    return lane_width/2 - t
end
function get_markerdist_right(veh::Vehicle, roadway::Roadway)
    t = veh.state.posF.t
    lane_width = get_lane_width(veh, roadway)
    return lane_width/2 + t
end

function propagate(veh::Vehicle, action::LaneFollowingAccel, roadway::Roadway, ΔT::Float64)

    a_lon = action.a

    ds = veh.state.v

    ΔT² = ΔT*ΔT
    Δs = ds*ΔT + 0.5*a_lon*ΔT²

    v₂ = ds + a_lon*ΔT

    roadind = move_along(veh.state.posF.roadind, roadway, Δs)
    posG = roadway[roadind].pos
    VehicleState(posG, roadway, v₂)
end