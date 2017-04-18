export
        gen_straight_curve,
        gen_straight_segment,
        gen_straight_roadway,
        gen_stadium_roadway,
        gen_bezier_curve

function gen_straight_curve(A::VecE2, B::VecE2, nsamples::Int)

    θ = atan2(B-A)
    δ = abs(B-A)/(nsamples-1)

    s = 0.0
    curve = Array(CurvePt, nsamples)
    for i in 1 : nsamples
        t = (i-1)/(nsamples-1)
        P = lerp(A,B,t)
        curve[i] = CurvePt(VecSE2(P.x,P.y,θ), s, 0.0)
        s += δ
    end

    return curve
end

function gen_straight_segment(seg_id::Int, nlanes::Int, length::Float64=1000.0;
    origin::VecSE2 = VecSE2(0.0,0.0,0.0),
    lane_width::Float64=DEFAULT_LANE_WIDTH, # [m]
    lane_widths::Vector{Float64} = fill(lane_width, nlanes),
    boundary_leftmost::LaneBoundary=LaneBoundary(:solid, :white),
    boundary_rightmost::LaneBoundary=LaneBoundary(:solid, :white),
    boundary_middle::LaneBoundary=LaneBoundary(:broken, :white),
    )

    seg = RoadSegment(seg_id, Array(Lane, nlanes))
    y = -lane_widths[1]/2
    for i in 1 : nlanes
        y += lane_widths[i]/2
        seg.lanes[i] = Lane(LaneTag(seg_id,i), [CurvePt(body2inertial(VecSE2(   0.0,y,0.0), origin),    0.0),
                                                CurvePt(body2inertial(VecSE2(length,y,0.0), origin), length)],
                            width=lane_widths[i],
                            boundary_left=(i == nlanes ? boundary_leftmost : boundary_middle),
                            boundary_right=(i == 1 ? boundary_rightmost : boundary_middle)
                           )

        y += lane_widths[i]/2
    end

    return seg
end

"""
quadratic bezier lerp
"""
Vec.lerp(A::VecE2, B::VecE2, C::VecE2, t::Float64) = (1-t)^2*A + 2*(1-t)*t*B + t^2*B

"""
cubic bezier lerp
"""
Vec.lerp(A::VecE2, B::VecE2, C::VecE2, D::VecE2, t::Float64) = (1-t)^3*A + 3*(1-t)^2*t*B + 3*(1-t)*t^2*C + t^3*D

function gen_bezier_curve(A::VecSE2, B::VecSE2, rA::Float64, rB::Float64, nsamples::Int)

    a = convert(VecE2, A)
    d = convert(VecE2, B)
    b = a + polar( rA, A.θ)
    c = d + polar(-rB, B.θ)

    s = 0.0
    curve = Array(CurvePt, nsamples)
    for i in 1 : nsamples
        t = (i-1)/(nsamples-1)
        P = lerp(a,b,c,d,t)
        P′ = 3*(1-t)^2*(b-a) + 6*(1-t)*t*(c-b) + 3*t^2*(d-c)
        P′′ = 6*(1-t)*(c-2b+a) + 6t*(d-2*c+b)
        θ = atan2(P′)
        κ = (P′.x*P′′.y - P′.y*P′′.x)/(P′.x^2 + P′.y^2)^1.5 # signed curvature

        if i > 1
            s += abs(P - convert(VecE2, curve[i-1].pos)) # approximation, but should be good for many samples
        end

        curve[i] = CurvePt(VecSE2(P.x,P.y,θ), s, κ)
    end

    return curve
end

"""
    gen_straight_roadway(nlanes::Int, length::Float64)
Generate a roadway with a single straight segment whose rightmost lane center starts at starts at (0,0),
and proceeds in the positive x direction.
"""
function gen_straight_roadway(nlanes::Int, length::Float64=1000.0;
    origin::VecSE2 = VecSE2(0.0,0.0,0.0),
    lane_width::Float64=DEFAULT_LANE_WIDTH, # [m]
    lane_widths::Vector{Float64} = fill(lane_width, nlanes),
    boundary_leftmost::LaneBoundary=LaneBoundary(:solid, :white),
    boundary_rightmost::LaneBoundary=LaneBoundary(:solid, :white),
    boundary_middle::LaneBoundary=LaneBoundary(:broken, :white),
    )

    retval = Roadway()
    push!(retval.segments, gen_straight_segment(1, nlanes, length,
                                                origin=origin, lane_widths=lane_widths,
                                                boundary_leftmost=boundary_leftmost,
                                                boundary_rightmost=boundary_rightmost,
                                                boundary_middle=boundary_middle))
    retval
end

"""
    gen_stadium_roadway(nlanes::Int, length::Float64)
Generate a roadway that is a rectangular racetrack with rounded corners.
    length = length of the x-dim straight section for the innermost (leftmost) lane [m]
    width  = length of the y-dim straight section for the innermost (leftmost) lane [m]
    radius = turn radius [m]

      ______________________
     /                      \
    |                        |
    |                        |
     \______________________/
"""
function gen_stadium_roadway(nlanes::Int;
    length::Float64=100.0,
    width::Float64=10.0,
    radius::Float64=25.0,
    ncurvepts_per_turn::Int=25, # includes start and end
    lane_width::Float64=DEFAULT_LANE_WIDTH, # [m]
    boundary_leftmost::LaneBoundary=LaneBoundary(:solid, :white),
    boundary_rightmost::LaneBoundary=LaneBoundary(:solid, :white),
    boundary_middle::LaneBoundary=LaneBoundary(:broken, :white),
    )

    ncurvepts_per_turn ≥ 2 || error("must have at least 2 pts per turn")

    A = VecE2(length, radius)
    B = VecE2(length, width + radius)
    C = VecE2(0.0, width + radius)
    D = VecE2(0.0, radius)

    seg1 = RoadSegment(1, Array(Lane, nlanes))
    seg2 = RoadSegment(2, Array(Lane, nlanes))
    seg3 = RoadSegment(3, Array(Lane, nlanes))
    seg4 = RoadSegment(4, Array(Lane, nlanes))
    for i in 1 : nlanes
        curvepts1 = Array(CurvePt, ncurvepts_per_turn)
        curvepts2 = Array(CurvePt, ncurvepts_per_turn)
        curvepts3 = Array(CurvePt, ncurvepts_per_turn)
        curvepts4 = Array(CurvePt, ncurvepts_per_turn)

        r = radius + lane_width*(i-1)
        for j in 1:ncurvepts_per_turn
            t = (j-1)/(ncurvepts_per_turn-1) # ∈ [0,1]
            s = r*π/2*t
            curvepts1[j] = CurvePt(VecSE2(A + polar(r, lerp(-π/2,  0.0, t)), lerp(0.0,π/2,t)),  s)
            curvepts2[j] = CurvePt(VecSE2(B + polar(r, lerp( 0.0,  π/2, t)), lerp(π/2,π,  t)),  s)
            curvepts3[j] = CurvePt(VecSE2(C + polar(r, lerp( π/2,  π,   t)), lerp(π, 3π/2,t)),  s)
            curvepts4[j] = CurvePt(VecSE2(D + polar(r, lerp( π,   3π/2, t)), lerp(3π/2,2π,t)),  s)
        end

        laneindex = nlanes-i+1
        tag1 = LaneTag(1,laneindex)
        tag2 = LaneTag(2,laneindex)
        tag3 = LaneTag(3,laneindex)
        tag4 = LaneTag(4,laneindex)

        boundary_left = (laneindex == nlanes ? boundary_leftmost : boundary_middle)
        boundary_right = (laneindex == 1 ? boundary_rightmost : boundary_middle)
        curveind_lo = CurveIndex(1,0.0)
        curveind_hi = CurveIndex(ncurvepts_per_turn-1,1.0)

        seg1.lanes[laneindex] = Lane(tag1, curvepts1, width=lane_width,
                                      boundary_left=boundary_left, boundary_right=boundary_right,
                                      next = RoadIndex(curveind_lo, tag2),
                                      prev = RoadIndex(curveind_hi, tag4),
                                     )
        seg2.lanes[laneindex] = Lane(tag2, curvepts2, width=lane_width,
                                      boundary_left=boundary_left, boundary_right=boundary_right,
                                      next = RoadIndex(curveind_lo, tag3),
                                      prev = RoadIndex(curveind_hi, tag1),
                                     )
        seg3.lanes[laneindex] = Lane(tag3, curvepts3, width=lane_width,
                                      boundary_left=boundary_left, boundary_right=boundary_right,
                                      next = RoadIndex(curveind_lo, tag4),
                                      prev = RoadIndex(curveind_hi, tag2),
                                     )
        seg4.lanes[laneindex] = Lane(tag4, curvepts4, width=lane_width,
                                      boundary_left=boundary_left, boundary_right=boundary_right,
                                      next = RoadIndex(curveind_lo, tag1),
                                      prev = RoadIndex(curveind_hi, tag3),
                                     )
    end

    retval = Roadway()
    push!(retval.segments, seg1)
    push!(retval.segments, seg2)
    push!(retval.segments, seg3)
    push!(retval.segments, seg4)
    retval
end