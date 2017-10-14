@with_kw mutable struct TimLaneChanger <: LaneChangeModel
    dir::Int = DIR_MIDDLE
    v_des::Float64 = 29.0
    threshold_fore::Float64 = 50.0
    threshold_lane_change_gap_fore::Float64 = 10.0
    threshold_lane_change_gap_rear::Float64 = 10.0
end
get_name(::TimLaneChanger) = "TimLaneChanger"
function set_desired_speed!(model::TimLaneChanger, v_des::Float64)
    model.v_des = v_des
    model
end
Base.rand(model::TimLaneChanger) = LaneChangeChoice(model.dir)