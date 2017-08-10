"""
Perfect integration of accel over Δt
"""
function propagate(veh::Entity{PosSpeed1D, BoundingBoxDef, Int}, action::Accel, roadway::Union{Curve, Straight1DRoadway}, Δt::Float64)

    a = action.a
    s, v = veh.state.s, veh.state.v

    s′ = s + v*Δt + a*Δt*Δt/2
    v′ = v + a*Δt

    return PosSpeed1D(s′, v′)
end
function propagate(veh::Entity{PosSpeed1D, BoundingBoxDef, Int}, action::StoppingAccel, roadway::Union{Curve, Straight1DRoadway}, Δt::Float64)

    a = action.a
    s, v = veh.state.s, veh.state.v

    v′ = v + a*Δt
    s′ = s + v*Δt + a*Δt*Δt/2

    if v′ < 0
        v′ = 0.0
        if a < 0
            Δt′ = clamp(-v/a,0.0,1.0)
            s′ = s + v*Δt′ + a*Δt′*Δt′/2
        end
    end

    return PosSpeed1D(s′, v′)
end
function propagate(
    veh::Entity{PosSpeed1D, BoundingBoxDef, Int},
    action::Union{Accel,StoppingAccel},
    roadway::Wraparound,
    Δt::Float64,
    )

    next_state = propagate(veh, action, roadway.road, Δt)
    s′ = mod_position_to_roadway(next_state.s, roadway)
    return PosSpeed1D(s′, next_state.v)
end