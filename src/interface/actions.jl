"""
    propagate(veh, action, roadway, Δt)

Take an entity of type {S,D,I} and move it over Δt seconds to produce a new
entity based on the action on the given roadway.
"""
propagate{S,D,I,A,R}(veh::Entity{S,D,I}, action::A, roadway::R, Δt::Float64) = error("propagate not implemented for Entity{$S, $D, $I}, actions $A, and roadway $R")


propagate{S,D,I,R}(veh::Entity{S,D,I}, state::S, roadway::R, Δt::Float64) = state