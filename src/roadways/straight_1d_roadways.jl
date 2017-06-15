type Straight1DRoadway
    len::Float64 # length of roadway [m]
end

function mod_position_to_roadway(s::Float64, roadway::Straight1DRoadway)
    while s > roadway.len
        s -= roadway.len
    end
    while s < 0
        s += roadway.len
    end
    return s
end

function get_headway(s_rear::Float64, s_fore::Float64, roadway::Straight1DRoadway)
    while s_fore < s_rear
        s_fore += roadway.len
    end
    while s_fore > s_rear + roadway.len
        s_fore -= roadway.len
    end
    return s_fore - s_rear # positive distance
end
