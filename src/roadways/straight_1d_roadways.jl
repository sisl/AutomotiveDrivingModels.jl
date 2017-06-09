type Straight1DRoadway
    len::Float64 # length of roadway [m]
end

function mod_position_to_roadway(s::Float64, roadway::Straight1DRoadway)
    while s > roadway.length
        s -= roadway.length
    end
    while s < 0
        s += roadway.length
    end
    return s
end

function get_headway(s_rear::Float64, s_fore::Float64, roadway::Straight1DRoadway)
    while s_fore < s_rear
        s_fore += roadway.length
    end
    while s_fore > s_rear + roadway.length
        s_fore -= roadway.length
    end
    return s_fore - s_rear # positive distance
end
