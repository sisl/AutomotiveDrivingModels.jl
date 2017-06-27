function is_colliding(
    vehA::Entity{PosSpeed1D, BoundingBoxDef, Int},
    vehB::Entity{PosSpeed1D, BoundingBoxDef, Int},
    roadway::Union{Curve, Straight1DRoadway},
    )

    segA = LineSegment1D(get_rear(vehA), get_front(vehA))
    segB = LineSegment1D(get_rear(vehB), get_front(vehB))
    return intersects(segA, segB)
end
function is_colliding(
    vehA::Entity{PosSpeed1D, BoundingBoxDef, Int},
    vehB::Entity{PosSpeed1D, BoundingBoxDef, Int},
    roadway::Wraparound,
    )

    # NOTE: rear and front are *not* wrapped around
    segA = LineSegment1D(get_rear(vehA), get_front(vehA))
    segB = LineSegment1D(get_rear(vehB), get_front(vehB))

    S = get_s_max(roadway)

    return intersects(segA, segB) || intersects(segA, segB + S) || intersects(segA, segB - S)
end

