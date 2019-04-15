"""
    StraightRoadway
A simple type representing a one lane, one dimensional straight roadway
# Fields
- `length::Float64`
"""
struct StraightRoadway
    length::Float64
end

"""
    mod_position_to_roadway(s::Float64, roadway::StraightRoadway)
performs a modulo of the position `s` with the length of `roadway`
"""
function mod_position_to_roadway(s::Float64, roadway::StraightRoadway)
    while s > roadway.length
        s -= roadway.length
    end
    while s < 0.0
        s += roadway.length
    end
    return s
end

"""
    get_headway(s_rear::Float64, s_fore::Float64, roadway::StraightRoadway)
returns a positive distance between s_rear and s_fore.
"""
function get_headway(s_rear::Float64, s_fore::Float64, roadway::StraightRoadway)
    while s_fore < s_rear
        s_fore += roadway.length
    end
    while s_fore > s_rear + roadway.length
        s_fore -= roadway.length
    end
    return s_fore - s_rear # positive distance
end
