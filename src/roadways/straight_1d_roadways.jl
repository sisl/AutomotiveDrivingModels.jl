struct Straight1DRoadway
    len::Float64 # length of roadway [m]
end

function get_headway(s_rear::Float64, s_fore::Float64, roadway::Straight1DRoadway)

    0 ≤ s_rear ≤ roadway.len || error("s_rear=$s_rear not within straight roadway bounds of $(roadway.len)")
    0 ≤ s_fore ≤ roadway.len || error("s_fore=$s_fore not within straight roadway bounds of $(roadway.len)")

    if s_fore ≥ s_rear
        return s_fore - s_rear # positive distance
    else
        return Inf
    end
end
