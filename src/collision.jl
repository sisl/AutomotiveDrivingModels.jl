export
    intersects


const HALF_PI = 0.5π
const τ = 2π

type Point
    x::Float64
    y::Float64
end
type Projection
    min::Float64 # minimum coordinate along axis
    max::Float64 # maximum coordinate along axis
end

function intersects(simlog::Matrix{Float64};
    startframe::Int=1,
    endframe::Int=get_nframes(simlog)
    )

    # check whether any vehicles intersect anywhere along the trajectory

    carA = Vehicle()
    carB = Vehicle()

    ncars = get_ncars(simlog)

    for frameind in startframe : endframe
        for carA = 1 : ncars-1
            pull_vehicle!(carA, simlog, calc_logindexbase(carindA), frameind)
            for carB = carA + 1 : ncars
                pull_vehicle!(carB, simlog, calc_logindexbase(carindB), frameind)
                if intersects(carA, carB)
                    return true
                end
            end
        end
    end

    false
end
function intersects(carind::Int, simlog::Matrix{Float64};
    startframe::Int=1,
    endframe::Int=get_nframes(simlog)
    )

    # check whether any vehicles intersect the given vehicel anywhere along the trajectory

    carA = Vehicle()
    carB = Vehicle()

    baseindA = calc_logindexbase(carindA)

    ncars = get_ncars(simlog)

    for frameind in startframe : endframe
        pull_vehicle!(carA, simlog, basindA, frameind)
        for carB = 1 : ncars
            if carB != carA
                pull_vehicle!(carB, simlog, calc_logindexbase(carindB), frameind)
                if intersects(carA, carB)
                    return true
                end
            end
        end
    end

    false
end
function intersects(carindA::Int, carindB::Int, simlog::Matrix{Float64};
    startframe::Int=1,
    endframe::Int=get_nframes(simlog)
    )
    # check whether the two vehicles intersect anywhere along the trajectory

    carA = Vehicle()
    carB = Vehicle()

    baseindA = calc_logindexbase(carindA)
    baseindB = calc_logindexbase(carindB)

    for frameind in startframe : endframe
        pull_vehicle!(carA, simlog, baseindA, frameind)
        pull_vehicle!(carB, simlog, baseindA, frameind)
        if intersects(carA, carB)
            return true
        end
    end

    false
end
function intersects(carA::Vehicle, carB::Vehicle)
    # INTERSECTS: Given two car configurations, uses the separating axis theorem to determine
    #             whether their rectangular body-aligned bounding boxes intersect
    # OUTPUTS: <Bool> true if there is a collision
    #
    # Used approach outlined at http://www.codezealot.org/archives/55
    # Originally written by Julien Kawawa-Beaudan, Summer 2014

    ϕA = carA.pos.ϕ
    ϕB = carB.pos.ϕ

    axes = [ϕA, ϕB, ϕA + HALF_PI, ϕA + HALF_PI]

    cornersCarA = get_corners(carA)
    cornersCarB = get_corners(carB)

    for axis in axes
        p1 = get_projection(cornersCarA, axis)
        p2 = get_projection(cornersCarB, axis)
          if !overlap(p1, p2)
            return false
        end
    end

    true
end

function get_corners( car::Vehicle )

    # s = sin(car.pos.ϕ)
    # c = cos(car.pos.ϕ)

    # sa = boxA.length*s/2
    # sb = boxA.width*s/2
    # ca = boxA.length*c/2
    # cb = boxA.width*c/2

    # # Corners in order: front left, front right, back left, back right
    # corners = [Point(boxA.x + ca - sb, boxA.y + sa + cb),
    #            Point(boxA.x + ca - sb, boxA.y + sa + cb),
    #            Point(boxA.x + ca - sb, boxA.y + sa + cb),
    #            Point(boxA.x + ca - sb, boxA.y + sa + cb)]
    # return corners

    # Corners in order: front left, front right, back left, back right
    # car.pos.ϕ is in radians from x axis (direction of road)
    corners = Array(Point, 4);
    corners[1] = Point(car.pos.x + car.length*cos(car.pos.ϕ)/2 - car.width*sin(car.pos.ϕ)/2,
        car.pos.y + car.length*sin(car.pos.ϕ)/2 + car.width*cos(car.pos.ϕ)/2)
    corners[2] = Point(car.pos.x + car.length*cos(car.pos.ϕ)/2 + car.width*sin(car.pos.ϕ)/2,
        car.pos.y + car.length*sin(car.pos.ϕ)/2 - car.width*cos(car.pos.ϕ)/2)
    corners[3] = Point(car.pos.x - car.length*cos(car.pos.ϕ)/2 - car.width*sin(car.pos.ϕ)/2,
        car.pos.y - car.length*sin(car.pos.ϕ)/2 + car.width*cos(car.pos.ϕ)/2)
    corners[4] = Point(car.pos.x - car.length*cos(car.pos.ϕ)/2 + car.width*sin(car.pos.ϕ)/2,
        car.pos.y - car.length*sin(car.pos.ϕ)/2 - car.width*cos(car.pos.ϕ)/2)
    return corners
end
function get_projection( corners::Vector{Point}, axis::Real )
    # Calculate projection in direction of axis, the dot product of
    # v1 = (corner.x)i + (corner.y)j and
    # v2 = cos(axis)i + sin(axis)j

    p = corners[1].x * cos(axis) + corners[1].y *sin(axis)
    projection = Projection(p, p)

    for i = 2:4
        p = corners[i].x * cos(axis) + corners[i].y * sin(axis)
        if p < projection.min
            projection.min = p
        elseif p > projection.max
            projection.max = p
        end
    end
    return projection
end
function overlap(p1::Projection, p2::Projection)
    #OUTPUT: <bool> true if the two projections overlap
    (p1.min <= p2.min && p2.min <= p1.max) || (p1.min <= p2.max && p2.max <= p1.max) || (p2.min <= p1.min && p1.min <= p2.max)
end

# abstract Box
# immutable AABB <: Box
#     # axis-aligned bounding box
#     x::Float64 # center x-position
#     y::Float64 # center y-position
#     a::Float64 # semi-axis along the x-axis (half the width)
#     b::Float64 # semi-axis along the y-axis
# end
# immutable OBB <: Box
#     # oriented bounding box
#     x::Float64 # center x-position
#     y::Float64 # center y-position
#     ϕ::Float64 # orientation, counterclockwise rotation
#     a::Float64 # semi-axis along the x-axis (half the width)
#     b::Float64 # semi-axis along the y-axis
# end

# function collide(boxA::OBB, boxB::OBB)
#     # COLLIDE: Given two oriented bounding boxes, 
#     #          uses the separating axis theorem to determine
#     #          whether their rectangular body-aligned bounding boxes collide.
#     # OUTPUTS: <Bool> true if there is a collision
#     #
#     # Used approach outlined at http://www.gamedev.net/page/resources/_/technical/game-programming/2d-rotated-rectangle-collision-r2604

#     # axes = [boxA.ϕ, boxB.ϕ, boxA.ϕ + HALF_PI, boxB.ϕ + HALF_PI]

#     # # function get_corners( car::Vehicle )
#     # #     corners = Array(Point, 4);
#     # #     # Corners in order: front left, front right, back left, back right
#     # #     # car.pos.ϕ is in radians from x axis (direction of road)
#     # #     corners[1] = Point(car.x + car.length*cos(car.pos.ϕ)/2 - car.width*sin(car.pos.ϕ)/2,
#     # #         car.pos.y + car.length*sin(car.pos.ϕ)/2 + car.width*cos(car.pos.ϕ)/2)
#     # #     corners[2] = Point(car.x + car.length*cos(car.pos.ϕ)/2 + car.width*sin(car.pos.ϕ)/2,
#     # #         car.pos.y + car.length*sin(car.pos.ϕ)/2 - car.width*cos(car.pos.ϕ)/2)
#     # #     corners[3] = Point(car.x - car.length*cos(car.pos.ϕ)/2 - car.width*sin(car.pos.ϕ)/2,
#     # #         car.pos.y - car.length*sin(car.pos.ϕ)/2 + car.width*cos(car.pos.ϕ)/2)
#     # #     corners[4] = Point(car.x - car.length*cos(car.pos.ϕ)/2 + car.width*sin(car.pos.ϕ)/2,
#     # #         car.pos.y - car.length*sin(car.pos.ϕ)/2 - car.width*cos(car.pos.ϕ)/2)
#     # #     return corners
#     # # end

#     const URx = 1
#     const URy = 2
#     const ULx = 3
#     const ULy = 4
#     const LRx = 5
#     const LRy = 6
#     const LLx = 7
#     const LLy = 8

#     sinA = sin(boxA.ϕ)
#     cosA = cos(boxA.ϕ)
#     sinB = sin(boxB.ϕ)
#     cosB = cos(boxB.ϕ)

#     asinA = boxA.a*sin(boxA.ϕ)
#     bsinA = boxA.b*sin(boxA.ϕ)
#     acosA = boxA.a*cos(boxA.ϕ)
#     bcosA = boxA.b*cos(boxA.ϕ)

#     # Corners in order: front left, front right, back left, back right
#     cornersCarA = [boxA.x + acosA - bsinA,
#                    boxA.y + asinA + bcosA),
#                    boxA.x + acosA - bsinA,
#                    boxA.y + asinA + bcosA),
#                    boxA.x + acosA - bsinA,
#                    boxA.y + asinA + bcosA),
#                    boxA.x + acosA - bsinA,
#                    boxA.y + asinA + bcosA)]

#     # cornersCarB = get_corners(carB)

#     Axis1.x = A.UR.x - A.UL.x
#     Axis1.y = A.UR.y - A.UL.y
#     Axis2.x = A.UR.x - A.LR.x
#     Axis2.y = A.UR.y - A.LR.y
#     Axis3.x = B.UL.x - B.LL.x
#     Axis3.y = B.UL.y - B.LL.y
#     Axis4.x = B.UL.x - B.UR.x
#     Axis4.y = B.UL.y - B.UR.y

    

#     # for axis in axes
#     #     p1 = get_projection(cornersCarA, axis)
#     #     p2 = get_projection(cornersCarB, axis)
#     #       if !overlap(p1, p2)
#     #         return false
#     #     end
#     # end

#     true
# end
# function collide(boxA::AABB, boxB::OBB)
#     # COLLIDE: Given a AABB and an oriented bounding boxe, 
#     #          use the separating axis theorem to determine
#     #          whether they collide.
#     # OUTPUTS: <Bool> true if there is a collision






#     const URx = 1
#     const URy = 2
#     const ULx = 3
#     const ULy = 4
#     const LRx = 5
#     const LRy = 6
#     const LLx = 7
#     const LLy = 8

#     sinA = sin(boxA.ϕ)
#     cosA = cos(boxA.ϕ)
#     sinB = sin(boxB.ϕ)
#     cosB = cos(boxB.ϕ)

#     asinA = boxA.a*sin(boxA.ϕ)
#     bsinA = boxA.b*sin(boxA.ϕ)
#     acosA = boxA.a*cos(boxA.ϕ)
#     bcosA = boxA.b*cos(boxA.ϕ)

#     # Corners in order: front left, front right, back left, back right
#     cornersCarA = [boxA.x + acosA - bsinA,
#                    boxA.y + asinA + bcosA),
#                    boxA.x + acosA - bsinA,
#                    boxA.y + asinA + bcosA),
#                    boxA.x + acosA - bsinA,
#                    boxA.y + asinA + bcosA),
#                    boxA.x + acosA - bsinA,
#                    boxA.y + asinA + bcosA)]

#     # cornersCarB = get_corners(carB)

#     Axis1.x = A.UR.x - A.UL.x
#     Axis1.y = A.UR.y - A.UL.y
#     Axis2.x = A.UR.x - A.LR.x
#     Axis2.y = A.UR.y - A.LR.y
#     Axis3.x = B.UL.x - B.LL.x
#     Axis3.y = B.UL.y - B.LL.y
#     Axis4.x = B.UL.x - B.UR.x
#     Axis4.y = B.UL.y - B.UR.y

#     true
# end