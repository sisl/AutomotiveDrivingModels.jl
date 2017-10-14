"""
    LaneChangeChoice
A choice of whether to change lanes, and what direction to do it in
"""
const DIR_RIGHT = -1
const DIR_MIDDLE =  0
const DIR_LEFT =  1
immutable LaneChangeChoice
    dir::Int # -1, 0, 1
end
Base.show(io::IO, a::LaneChangeChoice) = @printf(io, "LaneChangeChoice(%d)", dir)
Base.length(::Type{LaneChangeChoice}) = 1
Base.convert(::Type{LaneChangeChoice}, v::Vector{Float64}) = LaneChangeChoice(convert(Int, v[1]))
function Base.copy!(v::Vector{Float64}, a::LaneChangeChoice)
    v[1] = a.dir
    v
end
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

abstract LaneChangeModel
get_name(::LaneChangeModel) = "???"
set_desired_speed!(::LaneChangeModel, v_des::Float64) = model # # do nothing by default
reset_hidden_state!(model::LaneChangeModel) = model # do nothing by default
observe!(model::LaneChangeModel, scene::Scene, roadway::Roadway, egoid::Int) = model  # do nothing by default
Base.rand(model::LaneChangeModel) = error("rand not implemented for model $model")
