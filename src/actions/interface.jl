"""
    propagate(veh::Entity{S,D,I}, action::A, roadway::R, Δt::Float64) where {S,D,I,A,R}

Take an entity of type {S,D,I} and move it over Δt seconds to produce a new
entity based on the action on the given roadway.
"""
propagate(veh::Entity{S,D,I}, action::A, roadway::R, Δt::Float64) where {S,D,I,A,R} = error("propagate not implemented for Entity{$S, $D, $I}, actions $A, and roadway $R")


propagate(veh::Entity{S,D,I}, state::S, roadway::R, Δt::Float64) where {S,D,I,R}= state