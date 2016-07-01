export
        gen_straight_roadway,
        gen_stadium_roadway

"""
    gen_straight_roadway(nlanes::Int, length::Float64)
Generate a roadway with a single straight segment whose rightmost lane center starts at starts at (0,0),
and proceeds in the positive x direction.
"""
function gen_straight_roadway(nlanes::Int, length::Float64=1000.0;
    lane_width::Float64=DEFAULT_LANE_WIDTH, # [m]
    boundary_leftmost::LaneBoundary=LaneBoundary(:solid, :white),
    boundary_rightmost::LaneBoundary=LaneBoundary(:solid, :white),
    boundary_middle::LaneBoundary=LaneBoundary(:broken, :white),
    )

    seg = RoadSegment(1, Array(Lane, nlanes))
    for i in 1 : nlanes
        y = lane_width*(i-1)
        seg.lanes[i] = Lane(LaneTag(1,i), [CurvePt(VecSE2(0.0,y,0.0), 0.0), CurvePt(VecSE2(length,y,0.0), length)],
                            width=lane_width,
                            boundary_left=(i == nlanes ? boundary_leftmost : boundary_middle),
                            boundary_right=(i == 1 ? boundary_rightmost : boundary_middle)
                           )
    end

    retval = Roadway()
    push!(retval.segments, seg)
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

    npts_per_lane = ncurvepts_per_turn*4

    A = VecE2(length, radius)
    B = VecE2(length, width + radius)
    C = VecE2(0.0, width + radius)
    D = VecE2(0.0, radius)

    seg = RoadSegment(1, Array(Lane, nlanes))
    for i in 1 : nlanes
        curvepts = Array(CurvePt, npts_per_lane)
        r = radius + lane_width*(i-1)
        for j in 1:ncurvepts_per_turn
            t = (j-1)/(ncurvepts_per_turn-1) # ∈ [0,1]
            curvepts[0*ncurvepts_per_turn + j] = CurvePt(VecSE2(A + polar(r, lerp(-π/2,  0.0, t)), lerp(0.0,π/2,t)),  length +          r*π/2*t)
            curvepts[1*ncurvepts_per_turn + j] = CurvePt(VecSE2(B + polar(r, lerp( 0.0,  π/2, t)), lerp(π/2,π,  t)),  length +  width + r*π/2*(t+1))
            curvepts[2*ncurvepts_per_turn + j] = CurvePt(VecSE2(C + polar(r, lerp( π/2,  π,   t)), lerp(π, 3π/2,t)), 2length +  width + r*π/2*(t+2))
            curvepts[3*ncurvepts_per_turn + j] = CurvePt(VecSE2(D + polar(r, lerp( π,   3π/2, t)), lerp(3π/2,2π,t)), 2length + 2width + r*π/2*(t+3))
        end
        pop!(curvepts) # make the start the end of the last turn
        unshift!(curvepts, CurvePt(VecSE2(0.0,lane_width*(1-i),0.0), 0.0)) # set angle to 0 rather than 2π

        laneindex = nlanes-i+1
        tag = LaneTag(1,laneindex)
        lane = Lane(tag, curvepts, width=lane_width,
                    boundary_left=(laneindex == nlanes ? boundary_leftmost : boundary_middle),
                    boundary_right=(laneindex == 1 ? boundary_rightmost : boundary_middle),
                    prev = RoadIndex(CurveIndex(npts_per_lane-1,1.0), tag),
                    next = RoadIndex(CurveIndex(1,0.0),               tag),
                   )

        seg.lanes[laneindex] = lane
    end

    retval = Roadway()
    push!(retval.segments, seg)
    retval
end