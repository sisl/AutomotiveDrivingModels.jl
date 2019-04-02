"""
    collision_checker(veh_a::Vehicle, veh_b::Vehicle)
    collision_checker(veh_a::VehicleState, veh_b::VehicleState, veh_a_def::AbstractAgentDefinition, veh_b_def::AbstractAgentDefinition)
return True if `veh_a` and `veh_b` collides.
Relies on the parallel axis theorem.
"""
function collision_checker(veh_a::Entity{VehicleState, D, I}, veh_b::Entity{VehicleState, D, I}) where {D<:AbstractAgentDefinition, I}
    return collision_checker(veh_a.state, veh_b.state, veh_a.def, veh_b.def)
end

function collision_checker(veh_a::VehicleState, veh_b::VehicleState, veh_a_def::AbstractAgentDefinition, veh_b_def::AbstractAgentDefinition)
    center_a = veh_a.posG
    center_b = veh_b.posG
    l_a = length(veh_a_def)
    w_a = width(veh_a_def)
    l_b = length(veh_b_def)
    w_b = width(veh_b_def)
    # first fast check:
    @fastmath begin
        Δ = sqrt((veh_a.posG.x - veh_b.posG.x)^2 + (veh_a.posG.y - veh_b.posG.y)^2)
        r_a = sqrt(l_a*l_a/4 + w_a*w_a/4)
        r_b = sqrt(l_b*l_b/4 + w_b*w_b/4)
    end
    if Δ ≤ r_a + r_b
        # fast check is true, run parallel axis theorem
        Pa = polygon(center_a, veh_a_def)
        Pb = polygon(center_b, veh_b_def)
        return overlap(Pa, Pb)
    end
    return false
end

""" 
    polygon(pos::VecSE2{Float64}, veh_def::AbstractAgentDefinition)
    polygon(x::Float64,y::Float64,theta::Float64, length::Float64, width::Float64)
returns a 4x2 static matrix corresponding to a rectangle around a car
centered at `pos` and of dimensions specified by `veh_def`
"""
function polygon(pos::VecSE2{Float64}, veh_def::AbstractAgentDefinition)
    x, y ,θ = pos 
    return polygon(x, y, θ, length(veh_def), width(veh_def))
end

function polygon(x::Float64,y::Float64,theta::Float64, length::Float64, width::Float64)
    l_ = length/2
    w_ = width/2

    # compute each vertex:
    @fastmath begin 
        #top left
        Ax = x + l_*cos(theta) - w_*sin(theta)
        Ay = y + l_*sin(theta) + w_*cos(theta)

        #top right
        Bx = x + l_*cos(theta) + w_*sin(theta)
        By = y + l_*sin(theta) - w_*cos(theta)

        # bottom right
        Cx = x - l_*cos(theta) + w_*sin(theta)
        Cy = y - l_*sin(theta) - w_*cos(theta)

        # bttom left
        Dx = x - l_*cos(theta) - w_*sin(theta)
        Dy =  y - l_*sin(theta) + w_*cos(theta)
    end 
    P = @SMatrix [Ax Ay; 
                  Bx By;
                  Cx Cy;
                  Dx Dy]

    return P
end

### POLYGON INTERSECTION USING PARALLEL AXIS THEOREM

""" 
    overlap(poly_a::SMatrix{4, 2, Float64}, poly_b::SMatrix{4, 2, Float64})
Check if two convex polygons overlap, using the parallel axis theorem
a polygon is a nx2 matrix where n in the number of verteces
http://gamemath.com/2011/09/detecting-whether-two-convex-polygons-overlap/ 
  /!\\ edges needs to be ordered
"""
function overlap(poly_a::SMatrix{M, 2, Float64},
                poly_b::SMatrix{N, 2, Float64}) where {M,N}
  
    if find_separating_axis(poly_a, poly_b)
        return false
    end
    if find_separating_axis(poly_b, poly_a)
        return false
    end

    return true

end

""" 
    find_separating_axis(poly_a::SMatrix{4, 2, Float64}, poly_b::SMatrix{4, 2, Float64})
build a list of candidate separating axes from the edges of a
  /!\\ edges needs to be ordered
"""
function find_separating_axis(poly_a::SMatrix{M, 2, Float64},
                              poly_b::SMatrix{N, 2, Float64}) where {M, N}
    n_a = size(poly_a)[1]
    n_b = size(poly_b)[1]
    axis = zeros(2)
    @views previous_vertex = poly_a[end,:]
    for i=1:n_a # loop through all the edges n edges
        @views current_vertex = poly_a[i,:]
        # get edge vector
        edge = current_vertex - previous_vertex

        # rotate 90 degrees to get the separating axis
        axis[1] = edge[2]
        axis[2] = -edge[1]

        #  project polygons onto the axis
        a_min,a_max = polygon_projection(poly_a, axis)
        b_min,b_max = polygon_projection(poly_b, axis)

        # check separation
        if a_max < b_min
            return true#,current_vertex,previous_vertex,edge,a_max,b_min

        end
        if b_max < a_min
            return true#,current_vertex,previous_vertex,edge,a_max,b_min

        end

        @views previous_vertex = poly_a[i,:]
    end

    # no separation was found
    return false
end

"""
    polygon_projection(poly::SMatrix{4, 2, Float64}, axis::Vector{Float64})
return the projection interval for the polygon poly over the axis axis 
"""
function polygon_projection(poly::SMatrix{N, 2, Float64},
                            axis::Vector{Float64}) where {N}
        n_a = size(poly)[1]
        @inbounds @fastmath @views d1 = dot(poly[1,:],axis)
        @inbounds @fastmath @views d2 = dot(poly[2,:],axis)
        # initialize min and max
        if d1<d2
            out_min = d1
            out_max = d2
        else
            out_min = d2
            out_max = d1
        end

        for i=1:n_a
            @inbounds @fastmath @views d = dot(poly[i,:],axis)
            if d < out_min
                out_min = d
            elseif d > out_max
                out_max = d
            end
        end
        return out_min,out_max
end