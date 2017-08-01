function is_colliding{S,D,I}(
    vehA::Entity{S,D,I},
    vehB::Entity{S,D,I},
    roadway::Union{Curve, Straight1DRoadway},
    )

    segA = LineSegment1D(get_rear(vehA), get_front(vehA))
    segB = LineSegment1D(get_rear(vehB), get_front(vehB))
    return intersects(segA, segB)
end
function is_colliding{S,D,I}(
    vehA::Entity{S,D,I},
    vehB::Entity{S,D,I},
    roadway::Wraparound,
    )

    # NOTE: rear and front are *not* wrapped around
    segA = LineSegment1D(get_rear(vehA), get_front(vehA))
    segB = LineSegment1D(get_rear(vehB), get_front(vehB))

    s_max = get_s_max(roadway)

    return intersects(segA, segB) || intersects(segA, segB + s_max) || intersects(segA, segB - s_max)
end