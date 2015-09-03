export
    # Box,
    # AABB,
    # OBB,

    FourCorners,

    has_intersection,
    intersects

#=
abstract Box
immutable AABB <: Box
    # axis-aligned bounding box
    x::Float64 # center x-position
    y::Float64 # center y-position
    a::Float64 # semi-axis along the x-axis (half the width)
    b::Float64 # semi-axis along the y-axis
end
immutable OBB <: Box
    # oriented bounding box
    x::Float64 # center x-position
    y::Float64 # center y-position
    ϕ::Float64 # orientation, counterclockwise rotation
    a::Float64 # semi-axis along the x-axis (half the width)
    b::Float64 # semi-axis along the y-axis
end

function intersects(i::AABB, j::AABB)
    if i.x + i.a < j.x - j.a ||
       i.y + i.b < j.y - j.b ||
       i.x - i.a > j.x + j.a ||
       i.y - i.b > j.y + j.b
                
        false
    else
        true
    end
end
function intersects(
    A_x_lo::Float64, A_x_hi::Float64, A_y_lo::Float64, A_y_hi::Float64,
    B_x_lo::Float64, B_x_hi::Float64, B_y_lo::Float64, B_y_hi::Float64
    )

    if A_x_hi < B_x_lo ||
       A_y_hi < B_y_lo ||
       A_x_lo > B_x_hi ||
       A_y_lo > B_y_hi
                
        false
    else
        true
    end
end
=#

const HALF_PI = 0.5π

type FourCorners
    front_left  :: VecE2
    front_right :: VecE2
    back_left   :: VecE2
    back_right  :: VecE2

    FourCorners() = new()
    FourCorners(front_left::VecE2, front_right::VecE2, back_left::VecE2, back_right::VecE2) = 
        new(front_left, front_right, back_left, back_right)
end

function get_corners!(corners::FourCorners, car::Vehicle)

    #=
    Corners in order: front left, front right, back left, back right
    car.pos.θ is in radians from x axis (direction of road)
    =#

    half_s = sin(car.pos.θ)/2
    half_c = cos(car.pos.θ)/2

    L_half_s = car.length*half_s
    L_half_c = car.length*half_c
    W_half_s = car.width*half_s
    W_half_c = car.width*half_c

    corners.front_left  = VecE2(car.pos.x + L_half_c - W_half_s,
                                car.pos.y + L_half_s + W_half_c)
    corners.front_right = VecE2(car.pos.x + L_half_c + W_half_s,
                                car.pos.y + L_half_s - W_half_c)
    corners.back_left   = VecE2(car.pos.x - L_half_c - W_half_s,
                                car.pos.y - L_half_s + W_half_c)
    corners.back_right  = VecE2(car.pos.x - L_half_c + W_half_s,
                                car.pos.y - L_half_s - W_half_c)

    corners
end
get_corners(car::Vehicle) = get_corners!(FourCorners(), car)

function get_projection( corners::FourCorners, axis::Real )

    #=
    Calculate projection in direction of axis, the dot product of
        v₁ = (corner.x)i + (corner.y)j and
        v₂ =   cos(axis)i + sin(axis)j
    =#

    s = sin(axis)
    c = cos(axis)

    p = corners.front_left.x*c + corners.front_left.y*s
    project_min = p
    project_max = p
    
    p = corners.front_right.x*c + corners.front_right.y*s
    if p < project_min
        project_min = p
    elseif p > project_max
        project_max = p
    end

    p = corners.back_left.x*c + corners.back_left.y*s
    if p < project_min
        project_min = p
    elseif p > project_max
        project_max = p
    end

    p = corners.back_right.x*c + corners.back_right.y*s
    if p < project_min
        project_min = p
    elseif p > project_max
        project_max = p
    end
    
    return VecE2(project_min, project_max)
end
function overlap(p1::VecE2, p2::VecE2)
    # OUTPUT: <::Bool> true if the two projections overlap
    #  p.x -> min
    #  p.y -> max

    (p1.x <= p2.x && p2.x <= p1.y) || (p1.x <= p2.y && p2.y <= p1.y) || (p2.x <= p1.x && p1.x <= p2.y)
end

function intersects(ϕA::Real, cornersCarA::FourCorners, ϕB::Real, cornersCarB::FourCorners)
    #=
    INTERSECTS: Given two car configurations, uses the separating axis theorem to determine
                whether their rectangular body-aligned bounding boxes intersect

    OUTPUT: 
        <Bool> true if there is a collision
    
    Used approach outlined at http://www.codezealot.org/archives/55
    Originally written by Julien Kawawa-Beaudan, Summer 2014
    =#

    # retval = intersects(AABB(carA.pos.x, carA.pos.y, carA.length/2, carA.width/2),
    #            AABB(carB.pos.x, carB.pos.y, carB.length/2, carB.width/2))

    # if retval == true
    #     println("A: ", carA.pos)
    #     println("B: ", carB.pos)
    # end

    # retval

    if !overlap(get_projection(cornersCarA, ϕA), get_projection(cornersCarB, ϕA))
        return false
    end

    ϕA += HALF_PI
    if !overlap(get_projection(cornersCarA, ϕA), get_projection(cornersCarB, ϕA))
        return false
    end

    if !overlap(get_projection(cornersCarA, ϕB), get_projection(cornersCarB, ϕB))
        return false
    end

    if !overlap(get_projection(cornersCarA, ϕB), get_projection(cornersCarB, ϕB))
        return false
    end

    true
end
function intersects(carA::Vehicle, carB::Vehicle, cornersCarA::FourCorners=FourCorners(), cornersCarB::FourCorners=FourCorners())
    
    get_corners!(cornersCarA, carA)
    get_corners!(cornersCarB, carB)

    intersects(carA.pos.θ, cornersCarA, carB.pos.θ, cornersCarB)
end

function has_intersection(
    pdset::PrimaryDataset,
    validfind::Integer,
    carA::Vehicle = Vehicle(),
    carB::Vehicle = Vehicle(),
    cornersCarA::FourCorners=FourCorners(),
    cornersCarB::FourCorners=FourCorners(),
    )

    carind_max = get_maxcarind(pdset, validfind)
    for carindA in CARIND_EGO : carind_max-1
        get_vehicle!(carA, pdset, carindA, validfind)
        get_corners!(cornersCarA, carA)

        for carindB in carindA+1 : carind_max
            get_vehicle!(carB, pdset, carindB, validfind)
            get_corners!(cornersCarB, carB)

            if intersects(carA.pos.θ, cornersCarA, carB.pos.θ, cornersCarB)
                return true
            end
        end
    end

    false
end
function has_intersection(
    pdset::PrimaryDataset,
    validfind_start::Integer,
    validfind_end::Integer,
    carA::Vehicle = Vehicle(),
    carB::Vehicle = Vehicle(),
    cornersCarA::FourCorners=FourCorners(),
    cornersCarB::FourCorners=FourCorners(),
    )

    # check whether any vehicles intersect anywhere along the trajectory

    for validfind in validfind_start : validfind_end
        if has_intersection(pdset, validfind,
                            carA, carB, cornersCarA, cornersCarB)
            return true
        end
    end

    false
end
function has_intersection(
    pdset::PrimaryDataset,
    carA::Vehicle = Vehicle(),
    carB::Vehicle = Vehicle(),
    cornersCarA::FourCorners=FourCorners(),
    cornersCarB::FourCorners=FourCorners(),
    )

    has_intersection(pdset, 1, nvalidfinds(pdset), carA, carB, cornersCarA, cornersCarB)
end

# function intersects(carind::Int, simlog::Matrix{Float64};
#     startframe::Int=1,
#     endframe::Int=get_nframes(simlog)
#     )

#     # check whether any vehicles intersect the given vehicel anywhere along the trajectory

#     carA = Vehicle()
#     carB = Vehicle()

#     baseindA = calc_logindexbase(carindA)

#     ncars = get_ncars(simlog)

#     for frameind in startframe : endframe
#         pull_vehicle!(carA, simlog, basindA, frameind)
#         for carindB = 1 : ncars
#             if carB != carA
#                 pull_vehicle!(carB, simlog, calc_logindexbase(carindB), frameind)
#                 if intersects(carA, carB)
#                     return true
#                 end
#             end
#         end
#     end

#     false
# end
# function intersects(carindA::Int, carindB::Int, simlog::Matrix{Float64};
#     startframe::Int=1,
#     endframe::Int=get_nframes(simlog)
#     )
#     # check whether the two vehicles intersect anywhere along the trajectory

#     carA = Vehicle()
#     carB = Vehicle()

#     baseindA = calc_logindexbase(carindA)
#     baseindB = calc_logindexbase(carindB)

#     for frameind in startframe : endframe
#         pull_vehicle!(carA, simlog, baseindA, frameind)
#         pull_vehicle!(carB, simlog, baseindA, frameind)
#         if intersects(carA, carB)
#             return true
#         end
#     end

#     false
# end