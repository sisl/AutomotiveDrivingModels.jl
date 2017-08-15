struct Wraparound{R}
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

"""
    loop_order(s1, s2)

Returns 1 if s1 -> s2 is a shorter distance than s2 -> s1
       -1 if s2 -> s1 is a shorter distance than s1 -> s2
        0 if the distances are the same
"""
function loop_order(s1::Float64, s2::Float64, s_max::Float64)
    s1 = mod_position_to_roadway(s1, s_max)
    s2 = mod_position_to_roadway(s2, s_max)

    s12 = get_headway(s1, s2, s_max)
    s21 = get_headway(s2, s1, s_max)
    if s12 < s21
        return 1
    elseif s21 < s12
        return -1
    else
        return 0
    end
end
loop_order(s1::Float64, s2::Float64, roadway::Wraparound) = loop_order(s1, s2, get_s_max(roadway))