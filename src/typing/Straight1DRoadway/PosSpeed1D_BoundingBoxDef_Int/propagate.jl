"""
Perfect integration of accel over Δt
"""
function propagate(veh::Entity{PosSpeed1D, BoundingBoxDef, Int}, action::Accel, roadway::Straight1DRoadway, Δt::Float64)

    a = action.a
    s, v = veh.state.s, veh.state.v

    s′ = s + v*Δt + a*Δt*Δt/2
    v′ = v + a*Δt

    s′ = mod_position_to_roadway(s′, roadway)

    return PosSpeed1D(s′, v′)
end