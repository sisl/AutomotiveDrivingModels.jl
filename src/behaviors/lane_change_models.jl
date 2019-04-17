"""
    LaneChangeChoice
A choice of whether to change lanes, and what direction to do it in
"""
const DIR_RIGHT = -1
const DIR_MIDDLE =  0
const DIR_LEFT =  1
struct LaneChangeChoice
    dir::Int # -1, 0, 1
end
Base.show(io::IO, a::LaneChangeChoice) = @printf(io, "LaneChangeChoice(%d)", a.dir)

function get_lane_offset(a::LaneChangeChoice, rec::SceneRecord, roadway::Roadway, vehicle_index::Int, pastframe::Int=0)
    if a.dir == DIR_MIDDLE
        rec[pastframe][vehicle_index].state.posF.t
    elseif a.dir == DIR_LEFT
        convert(Float64, get(LANEOFFSETLEFT, rec, roadway, vehicle_index, pastframe))
    else
        @assert(a.dir == DIR_RIGHT)
        convert(Float64, get(LANEOFFSETRIGHT, rec, roadway, vehicle_index, pastframe))
    end
end

####################

abstract type LaneChangeModel end
get_name(::LaneChangeModel) = "???"
set_desired_speed!(model::LaneChangeModel, v_des::Float64) = model # # do nothing by default
reset_hidden_state!(model::LaneChangeModel) = model # do nothing by default
observe!(model::LaneChangeModel, scene::Scene, roadway::Roadway, egoid::Int) = model  # do nothing by default
Base.rand(model::LaneChangeModel) = error("rand not implemented for model $model")
