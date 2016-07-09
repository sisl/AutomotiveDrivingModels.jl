function render!(
    rendermodel::RenderModel,
    boundary::LaneBoundary,
    pts::Matrix{Float64},
    lane_marking_width  :: Real=0.15, # [m]
    lane_dash_len       :: Real=0.91, # [m]
    lane_dash_spacing   :: Real=2.74, # [m]
    lane_dash_offset    :: Real=0.00  # [m]
    )

    marker_color = boundary.color == :yellow ? COLOR_LANE_MARKINGS_YELLOW : COLOR_LANE_MARKINGS_WHITE
    if boundary.style == :broken
        add_instruction!(rendermodel, render_dashed_line, (pts, marker_color, lane_marking_width, lane_dash_len, lane_dash_spacing, lane_dash_offset))
    else
        add_instruction!(rendermodel, render_line, (pts, marker_color, lane_marking_width))
    end
    rendermodel
end

function render!(rendermodel::RenderModel, lane::Lane, roadway::Roadway;
    color_asphalt       :: Colorant=COLOR_ASPHALT,
    )

    n = length(lane.curve)
    pts = Array(Float64, 2, n + has_next(lane))
    for (i,pt) in enumerate(lane.curve)
        pts[1,i] = pt.pos.x
        pts[2,i] = pt.pos.y
    end
    if has_next(lane)
        pt = next_lane_point(lane, roadway)
        pts[1,end] = pt.pos.x
        pts[2,end] = pt.pos.y
    end

    add_instruction!(rendermodel, render_line, (pts, color_asphalt, lane.width))
    rendermodel
end

function render!(rendermodel::RenderModel, roadway::Roadway;
    color_asphalt       :: Colorant=COLOR_ASPHALT,
    lane_marking_width  :: Real=0.15, # [m]
    lane_dash_len       :: Real=0.91, # [m]
    lane_dash_spacing   :: Real=2.74, # [m]
    lane_dash_offset    :: Real=0.00  # [m]
    )

    # render the asphalt between the leftmost and rightmost lane markers
    for seg in roadway.segments
        if !isempty(seg.lanes)
            laneR = seg.lanes[1]
            laneL = seg.lanes[end]

            pts = Array(Float64, 2, length(laneL.curve) + has_next(laneL) +
                                        length(laneR.curve) + has_next(laneR) +
                                        2*length(seg.lanes))
            pts_index = 0
            for pt in laneL.curve
                edgept = pt.pos + polar(laneL.width/2, pt.pos.θ + π/2)
                pts_index += 1
                pts[1, pts_index] = edgept.x
                pts[2, pts_index] = edgept.y
            end
            if has_next(laneL)
                pt = next_lane_point(laneL, roadway)
                edgept = pt.pos + polar(laneL.width/2, pt.pos.θ + π/2)
                pts_index += 1
                pts[1, pts_index] = edgept.x
                pts[2, pts_index] = edgept.y
            end
            for i in reverse(1:length(seg.lanes))
                lane = seg.lanes[i]
                if has_next(lane)
                    pt = next_lane_point(lane, roadway).pos
                else
                    pt = lane.curve[end].pos
                end
                pts_index += 1
                pts[1, pts_index] = pt.x
                pts[2, pts_index] = pt.y
            end

            if has_next(laneR)
                pt = next_lane_point(laneR, roadway)
                edgept = pt.pos + polar(laneR.width/2, pt.pos.θ - π/2)
                pts_index += 1
                pts[1, pts_index] = edgept.x
                pts[2, pts_index] = edgept.y
            end
            for j in length(laneR.curve) : -1 : 1
                pt = laneR.curve[j]
                edgept = pt.pos + polar(laneR.width/2, pt.pos.θ - π/2)
                pts_index += 1
                pts[1, pts_index] = edgept.x
                pts[2, pts_index] = edgept.y
            end
            for i in 1:length(seg.lanes)
                lane = seg.lanes[i]
                pt = lane.curve[1].pos
                pts_index += 1
                pts[1, pts_index] = pt.x
                pts[2, pts_index] = pt.y
            end

            add_instruction!(rendermodel, render_fill_region, (pts, color_asphalt))
        end
        # for lane in seg.lanes
        #     render!(rendermodel, lane, roadway)
        # end
    end

    # render the lane edges
    for seg in roadway.segments
        for lane in seg.lanes

            N = length(lane.curve)
            halfwidth = lane.width/2

            # always render the left lane marking
            pts_left = Array(Float64, 2, N)
            for (i,pt) in enumerate(lane.curve)
                p_left = pt.pos + polar(halfwidth, pt.pos.θ + π/2)

                pts_left[1,i] = p_left.x
                pts_left[2,i] = p_left.y
            end
            if has_next(lane)
                pt = next_lane_point(lane, roadway)
                p_left = pt.pos + polar(halfwidth, pt.pos.θ + π/2)
                pts_left = hcat(pts_left, [p_left.x, p_left.y])
            end

            render!(rendermodel, lane.boundary_left, pts_left, lane_marking_width, lane_dash_len, lane_dash_spacing, lane_dash_offset)

            # only render the right lane marking if this is the first lane
            if lane.tag.lane == 1
                pts_right = Array(Float64, 2, N)

                for (i,pt) in enumerate(lane.curve)
                    p_right = pt.pos - polar(halfwidth, pt.pos.θ + π/2)

                    pts_right[1,i] = p_right.x
                    pts_right[2,i] = p_right.y
                end

                if has_next(lane)
                    pt = next_lane_point(lane, roadway)
                    p_right = pt.pos - polar(halfwidth, pt.pos.θ + π/2)
                    pts_right = hcat(pts_right, [p_right.x, p_right.y])
                end

                render!(rendermodel, lane.boundary_right, pts_right, lane_marking_width, lane_dash_len, lane_dash_spacing, lane_dash_offset)
            end
        end
    end

    rendermodel
end

function render(roadway::Roadway;
    canvas_width::Int=DEFAULT_CANVAS_WIDTH,
    canvas_height::Int=DEFAULT_CANVAS_HEIGHT,
    rendermodel = RenderModel(),
    cam::Camera = FitToContentCamera(),
    )

    s = CairoRGBSurface(canvas_width, canvas_height)
    ctx = creategc(s)
    clear_setup!(rendermodel)

    render!(rendermodel, roadway)

    camera_set!(rendermodel, cam, canvas_width, canvas_height)
    render(rendermodel, ctx, canvas_width, canvas_height)
    s
end