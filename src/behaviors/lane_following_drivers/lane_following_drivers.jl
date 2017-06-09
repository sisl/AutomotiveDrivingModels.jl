abstract LaneFollowingDriver <: DriverModel{Accel}
track_longitudinal!(model::LaneFollowingDriver, v_ego::Float64, v_oth::Float64, headway::Float64) = model # do nothing by default