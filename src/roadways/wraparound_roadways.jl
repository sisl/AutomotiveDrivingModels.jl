type Wraparound{R}
    road::R # typically a Curve or a Straight1D roadway
end

get_s_max(roadway::Wraparound{Straight1DRoadway}) = roadway.road.len
get_s_max(roadway::Wraparound{Curve}) = roadway.road[end].s

function mod_position_to_roadway(s::Float64, s_max::Float64)
    while s > s_max
        s -= s_max
    end
    while s < 0
        s += s_max
    end
    return s
end
mod_position_to_roadway(s::Float64, roadway::Wraparound) = mod_position_to_roadway(s, get_s_max(roadway))

function get_headway(s_rear::Float64, s_fore::Float64, s_max::Float64)
    while s_fore < s_rear
        s_fore += s_max
    end
    while s_fore > s_rear + s_max
        s_fore -= s_max
    end
    return s_fore - s_rear # positive distance
end
get_headway(s_rear::Float64, s_fore::Float64, roadway::Wraparound) = get_headway(s_rear, s_fore, get_s_max(roadway))