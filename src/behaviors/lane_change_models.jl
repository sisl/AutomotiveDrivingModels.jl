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
    get_lane_offset(a, rec[pastframe], roadway, vehicle_index)
end
function get_lane_offset(a::LaneChangeChoice, scene::Frame, roadway::Roadway, vehicle_index::Int)
    if a.dir == DIR_MIDDLE
        posf(scene[vehicle_index].state).t
    elseif a.dir == DIR_LEFT
        convert(Float64, get(LANEOFFSETLEFT, scene, roadway, vehicle_index))
    else
        @assert(a.dir == DIR_RIGHT)
        convert(Float64, get(LANEOFFSETRIGHT, scene, roadway, vehicle_index))
    end
end

####################

abstract type LaneChangeModel{A} <: DriverModel{A} end
