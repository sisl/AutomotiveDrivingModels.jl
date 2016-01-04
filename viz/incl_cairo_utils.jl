using AutomotiveDrivingModels

using Cairo
using Interact
using Reel
Reel.set_output_type("gif")

# include(Pkg.dir("AutomotiveDrivingModels", "viz", "Renderer.jl")); using .Renderer
# include(Pkg.dir("AutomotiveDrivingModels", "viz", "ColorScheme.jl")); using .ColorScheme

colorscheme = getcolorscheme("monokai")

######################

abstract Overlay

type TextStatsOverlay <: Overlay
    draw_velFy::Bool
    draw_inv_timegap::Bool
    draw_Δv_front::Bool
    draw_inferred_context::Bool
    draw_actual_contexts::Bool

    function TextStatsOverlay(;
        draw_velFy::Bool = true,
        draw_inv_timegap::Bool = true,
        draw_Δv_front::Bool = true,
        draw_inferred_context::Bool = true,
        draw_actual_contexts::Bool = true,
        )

        new(draw_velFy, draw_inv_timegap, draw_Δv_front, draw_inferred_context, draw_actual_contexts)
    end
end
function render_overlay!(overlay::TextStatsOverlay,
    rendermodel::RenderModel,
    runlog::RunLog,
    sn::StreetNetwork,
    frame::Integer,
    active_carid::Integer,
    )

    colset = id2colset(runlog, active_carid, frame)
    velFy = get(FeaturesNew.VELFT, runlog, sn, colset, frame)
    inv_timegap_front = get(FeaturesNew.INV_TIMEGAP_FRONT, runlog, sn, colset, frame)
    Δv_front = get(FeaturesNew.DELTA_V_FRONT, runlog, sn, colset, frame)

    r = colorant"red"
    b = colorant"blue"
    color_freeflow = (inv_timegap_front < 1.0/3.0 || Δv_front > 0.5) ? r : b
    str_inferred_context = abs(velFy) > 0.1 ? "lanechange" : (inv_timegap_front < 1.0/3.0 || Δv_front > 0.5) ? "freeflow" : "following"

    behavior = get(runlog, colset, frame, :behavior)::UInt16

    str_actual_contexts = ""
    if (behavior & ContextClass.FREEFLOW) > 0
        str_actual_contexts *= "freeflow"
    end
    if (behavior & ContextClass.FOLLOWING) > 0
        if !isempty(str_actual_contexts)
            str_actual_contexts *= ", "
        end
        str_actual_contexts *= "following"
    end
    if (behavior & ContextClass.LANECHANGE) > 0
        if !isempty(str_actual_contexts)
            str_actual_contexts *= ", "
        end
        str_actual_contexts *= "lanechange"
    end
    if isempty(str_actual_contexts)
        str_actual_contexts = "NONE"
    end

    text_y = 15
    text_y_jump = 20

    if overlay.draw_velFy
        add_instruction!( rendermodel, render_text, (@sprintf("v_lat = %.4f", velFy), 10, text_y, 15, abs(velFy) > 0.1 ? b : r), incameraframe=false)
        text_y += text_y_jump
    end
    if overlay.draw_inv_timegap
        add_instruction!( rendermodel, render_text, (@sprintf("inv_timegap_front = %.4f", inv_timegap_front), 10, text_y, 15, color_freeflow), incameraframe=false)
        text_y += text_y_jump
    end
    if overlay.draw_Δv_front
        add_instruction!( rendermodel, render_text, (@sprintf("Δv_front = %.4f", Δv_front), 10, text_y, 15, color_freeflow), incameraframe=false)
        text_y += text_y_jump
    end
    if overlay.draw_inferred_context
        add_instruction!( rendermodel, render_text, (str_inferred_context, 10, text_y, 15, COLOR_CAR_EGO), incameraframe=false)
        text_y += text_y_jump
    end
    if overlay.draw_actual_contexts
        add_instruction!( rendermodel, render_text, (str_actual_contexts, 10, text_y, 15, colorscheme["foreground"]), incameraframe=false)
        text_y += text_y_jump
    end

    rendermodel
end

type TrajdataOverlay <: Overlay
    trajdata::DataFrame
end
function render_overlay!(overlay::TrajdataOverlay,
    rendermodel::RenderModel,
    runlog::RunLog,
    sn::StreetNetwork,
    frame::Integer,
    active_carid::Integer,
    )

    trajdata = overlay.trajdata

    # find frameind in trajdata
    t = get(runlog, frame, :time)::Float64
    t₀ = trajdata[1, :time]::Float64
    frameind_in_trajdata = -1
    for frameind in 1 : size(trajdata, 1)
        if abs(trajdata[frameind, :time]::Float64 - t- t₀) < 0.05
            frameind_in_trajdata = frameind
            break
        end
    end

    frameind_in_trajdata == -1 || render_scene!(rendermodel, trajdata, frameind_in_trajdata,
                                                    color_ego=RGBA(COLOR_CAR_EGO.r*1.1, COLOR_CAR_EGO.g*0.9, COLOR_CAR_EGO.b*0.9, 0.5),
                                                    color_oth=RGBA(COLOR_CAR_OTHER.r*0.9, COLOR_CAR_OTHER.g*1.1, COLOR_CAR_OTHER.b*1.1, 0.5))
end

######################

Renderer.camera_set_pos!(rm::RenderModel, car::Vehicle) = camera_set_pos!(rm, convert(VecE2, car.pos))

function render_trace!(
    rm::RenderModel,
    runlog::RunLog,
    carid::Integer,
    frame_start::Integer,
    frame_end::Integer;
    color::Colorant=RGB(0xBB,0xBB,0xFF),
    linewidth::Float64=0.25, # [m]
    arrowhead_len::Float64=1.0 # [m]
    )

    npts = validfind_end - validfind_start + 1
    pts = Array(Float64, 2, npts)
    pt_index = 0

    for frame in frame_start:frame_end
        if frame_inbounds(runlog, frame)

            colset = id2colset(runlog, carid, frame)
            if colset != COLSET_NULL
                pt_index += 1
                pos = get(runlog, colset, carid, :inertial)::VecSE2
                pts[1,pt_index] = pos.x
                pts[2,pt_index] = pos.y
            end
        end
    end

    pts = pts[:,1:pt_index]

    add_instruction!(rm, render_arrow, (pts, color, linewidth, arrowhead_len))
end
function render_trace!(
    rm::RenderModel,
    pdset::PrimaryDataset,
    carid::Integer,
    validfind_start::Int,
    validfind_end::Int;
    color::Colorant=RGB(0xBB,0xBB,0xFF),
    linewidth::Float64=0.25, # [m]
    arrowhead_len::Float64=1.0 # [m]
    )

    npts = validfind_end - validfind_start + 1
    pts = Array(Float64, 2, npts)
    pt_index = 0

    for validfind in validfind_start:validfind_end

        if validfind2frameind(pdset, validfind) != 0
            carind = carid2ind_or_negative_two_otherwise(pdset, carid, validfind)
            if carind != -2
                pt_index += 1
                pts[1,pt_index] = get(pdset, :posGx, carind, validfind)
                pts[2,pt_index] = get(pdset, :posGy, carind, validfind)
            end
        end
    end

    pts = pts[:,1:pt_index]

    add_instruction!(rm, render_arrow, (pts, color, linewidth, arrowhead_len))
end

function render_extracted_trajdef!(
    rendermodel::RenderModel,
    extracted::ExtractedTrajdef;
    color::Colorant=hexcolor(0xBB,0xBB,0xFF),
    linewidth::Float64=0.25, # [m]
    arrowhead_len::Float64=1.0, # [m]
    )

    npts = get_num_pdset_frames(extracted)
    pts = Array(Float64, 2, npts)
    pt_index = 0

    for i in 1 : npts
        pts[1,i] = extracted.df[i, :posGx]
        pts[2,i] = extracted.df[i, :posGy]
    end

    add_instruction!(rendermodel, render_arrow, (pts, color, linewidth, arrowhead_len))
end

function calc_subframe_interpolation_bounds(subframeind::Int, subframes_per_frame::Int)
    frameind_lo = int((subframeind-1 - mod(subframeind-1, subframes_per_frame))/subframes_per_frame)+1
    frameind_hi = frameind_lo + 1
    (frameind_lo, frameind_hi) # frameind low and high in stream
end
function calc_subframe_interpolation_scalar(subframeind::Int, subframes_per_frame::Int)
    # calc interpolation constant t for subframe (t ∈ [0,1])
    mod(subframeind-1,subframes_per_frame) / subframes_per_frame
end

function render_scene!( rm::RenderModel, s::Vector{Vehicle})

    if !isempty(s)
        render_car!(rm, s[1], COLOR_CAR_EGO)
    end
    for i = 2 : length(s)
        render_car!(rm, s[i], COLOR_CAR_OTHER)
    end
    rm
end

function render_lane_curve!(
    rm::RenderModel,
    lane::StreetLane;
    color :: Colorant = RGB(0xDD, 0x44, 0x44),
    line_width :: Real = 0.1
    )

    n = length(lane.curve.x)
    pts = Array(Float64, 2, n)
    pts[1,:] = lane.curve.x
    pts[2,:] = lane.curve.y # + rand(length(lane.curve.y))

    add_instruction!(rm, render_line, (pts, color, line_width))
end
function render_streetnet_nodes!(
    rm::RenderModel,
    sn::StreetNetwork,
    color_nodes :: Colorant = colorant"white",
    size_nodes  :: Real = 0.25, # [m]
    )

    npts = nv(sn.graph)

    pts = Array(Float64, 2, npts)

    for (i,node_index) in enumerate(vertices(sn.graph))
        node = sn.nodes[node_index]
        pts[1,i] = node.pos.x
        pts[2,i] = node.pos.y
    end

    add_instruction!(rm, render_point_trail, (pts, color_nodes, size_nodes))

    rm
end
function render_streetnet_edges!(
    rm::RenderModel,
    sn::StreetNetwork;
    color :: Colorant = RGB(0x88, 0x88, 0xFF),
    line_width :: Real = 0.1
    )

    for e in edges(sn.graph)

        a = sn.nodes[e.first]
        b = sn.nodes[e.second]

        pts = [a.pos.x b.pos.x;
               a.pos.y b.pos.y]

        Renderer.add_instruction!(rm, render_line, (pts, color, line_width))
    end

    rm
end
function render_streetnet_curves!(
    rm::RenderModel,
    sn::StreetNetwork;
    color :: Colorant = RGB(0xCC, 0x44, 0x44),
    line_width :: Real = 0.1
    )

    for tile in values(sn.tile_dict)
        for seg in values(tile.segments)
            for lane in values(seg.lanes)
                render_lane_curve!(rm, lane, color=color, line_width=line_width)
            end
        end
    end

    rm
end
function render_streetnet_roads!(
    rm::RenderModel,
    sn::StreetNetwork;
    color_asphalt       :: Colorant=COLOR_ASPHALT,
    color_lane_markings :: Colorant=COLOR_LANE_MARKINGS,
    lane_marking_width  :: Real=0.15, # [m]
    lane_dash_len       :: Real=0.91, # [m]
    lane_dash_spacing   :: Real=2.74, # [m]
    lane_dash_offset    :: Real=0.00  # [m]
    )

    # tf_1 = [lane_dash_len,     lane_dash_width]
    # tf_2 = [lane_dash_spacing, 0.0]
    # tf_3 = [lane_dash_offset,  0.0]
    # user_to_device_distance!(ctx, tf_1)
    # user_to_device_distance!(ctx, tf_2)
    # user_to_device_distance!(ctx, tf_3)
    # lane_dash_len,lane_dash_width = (tf_1[1],-tf_1[2])
    # lane_dash_spacing = tf_2[1]
    # lane_dash_offset = tf_3[1]

    # render the asphalt
    for tile in values(sn.tile_dict)
        for seg in values(tile.segments)
            for lane in values(seg.lanes)
                n = length(lane.curve.x)
                pts = vcat(lane.curve.x', lane.curve.y')
                @assert(size(pts,1) == 2)
                add_instruction!(rm, render_line, (pts, color_asphalt, lane.width))
            end
        end
    end

    # render the lane edges

    rotL = [0.0 -1.0;  1.0 0.0]
    rotR = [0.0  1.0; -1.0 0.0]

    for tile in values(sn.tile_dict)
        for seg in values(tile.segments)
            for lane in values(seg.lanes)
                n = length(lane.curve.x)
                pts_left = Array(Float64, 2, length(lane.node_indeces))
                pts_right = Array(Float64, 2, length(lane.node_indeces))

                for (i,node_index) in enumerate(lane.node_indeces)
                    node = sn.nodes[node_index]
                    θ = curve_at(lane.curve, node.extind).θ
                    v = [cos(θ), sin(θ)]
                    p = [node.pos.x, node.pos.y]
                    pts_left[:,i]   = p + node.marker_dist_left * rotL * v
                    pts_right[:,i]  = p + node.marker_dist_right * rotR * v
                end

                if has_next_lane(sn, lane)

                    nextlane = next_lane(sn, lane)
                    node_index_B = nextlane.node_indeces[1]
                    node = sn.nodes[node_index_B]

                    θ = curve_at(nextlane.curve, node.extind).θ
                    v = [cos(θ), sin(θ)]
                    p = [node.pos.x, node.pos.y]

                    pts_left = hcat(pts_left, p + node.marker_dist_left * rotL * v)
                    pts_right = hcat(pts_right, p + node.marker_dist_right * rotR * v)
                end

                add_instruction!(rm, render_line, (pts_left, color_lane_markings, lane_marking_width))
                add_instruction!(rm, render_line, (pts_right, color_lane_markings, lane_marking_width))
            end
        end
    end

    rm
end
function render_streetnet_segments!(
    rm::RenderModel,
    sn::StreetNetwork;
    line_width::Real = 0.1,
    seed::UInt=zero(UInt)
    )

    srand(seed)

    for tile in values(sn.tile_dict)
        for seg in values(tile.segments)
            color = RGB(rand(), rand(), rand())
            for lane in values(seg.lanes)
                render_lane_curve!(rm, lane, color=color, line_width=line_width)
            end
        end
    end

    rm
end
function render_streetnet_lanes!(
    rm::RenderModel,
    sn::StreetNetwork;
    line_width :: Real = 0.1,
    seed::UInt=zero(UInt)
    )

    srand(seed)

    for tile in values(sn.tile_dict)
        for seg in values(tile.segments)
            for lane in values(seg.lanes)
                color = RGB(rand(), rand(), rand())
                render_lane_curve!(rm, lane, color=color, line_width=line_width)
            end
        end
    end

    rm
end
function render_streetnet_marker_dist_left!(
    rm::RenderModel,
    sn::StreetNetwork;
    color       :: Colorant=RGB(1.0,0.0,0.0),
    )

    for node in sn.nodes

        lane = get_lane(sn, node)
        center = VecE2(node.pos.x, node.pos.y)
        θ = curve_at(lane.curve, node.extind).θ
        left = center + Vec.polar(1.0, θ + π/2)*node.marker_dist_left

        pts = [center.x left.x;
               center.y left.y]

        add_instruction!(rm, render_line, (pts, color, 0.1))
    end

    rm
end
function render_streetnet_marker_dist_right!(
    rm::RenderModel,
    sn::StreetNetwork;
    color       :: Colorant=RGB(0.0,0.0,1.0),
    )

    for node in sn.nodes

        lane = get_lane(sn, node)
        center = VecE2(node.pos.x, node.pos.y)
        θ = curve_at(lane.curve, node.extind).θ
        left = center + Vec.polar(1.0, θ - π/2)*node.marker_dist_right

        pts = [center.x left.x;
               center.y left.y]

        add_instruction!(rm, render_line, (pts, color, 0.1))
    end

    rm
end
function render_streetnet_n_lanes_left!(
    rm::RenderModel,
    sn::StreetNetwork;
    color       :: Colorant=RGB(1.0,0.0,0.0),
    lanewidth::Real = DEFAULT_LANE_WIDTH
    )

    for node in sn.nodes

        lane = get_lane(sn, node)
        center = VecE2(node.pos.x, node.pos.y)
        θ = curve_at(lane.curve, node.extind).θ
        left = center + Vec.polar(1.0, θ + π/2)*node.n_lanes_left*lanewidth

        pts = [center.x left.x;
               center.y left.y]

        add_instruction!(rm, render_line, (pts, color, 0.1))
    end

    rm
end
function render_streetnet_n_lanes_right!(
    rm::RenderModel,
    sn::StreetNetwork;
    color       :: Colorant=RGB(0.0,0.0,1.0),
    lanewidth::Real = DEFAULT_LANE_WIDTH
    )

    for node in sn.nodes

        lane = get_lane(sn, node)
        center = VecE2(node.pos.x, node.pos.y)
        θ = curve_at(lane.curve, node.extind).θ
        left = center + Vec.polar(1.0, θ - π/2)*node.n_lanes_right*lanewidth

        pts = [center.x left.x;
               center.y left.y]

        add_instruction!(rm, render_line, (pts, color, 0.1))
    end

    rm
end
function render_streetnet_d_end!(
    rm::RenderModel,
    sn::StreetNetwork;
    color_lo::Colorant=RGB(1.0,0.0,0.0),
    color_hi::Colorant=RGB(0.0,1.0,0.0),
    )

    for node in sn.nodes

        center = VecE2(node.pos.x, node.pos.y)

        @assert(node.d_end ≥ 0.0)
        t = node.d_end / StreetNetworks.SATURATION_DISTANCE_TO_LANE_END
        color = lerp_color(color_lo, color_hi, t)

        add_instruction!(rm, render_circle, (center.x, center.y, 1.0, color))
    end

    rm
end
function render_streetnet_next_nodes!(
    rm::RenderModel,
    sn::StreetNetwork;
    color::Colorant=colorant"white",
    )

    for node_index in 1 : length(sn.nodes)
        nextnode_index = next_node_index(sn, node_index)
        if nextnode_index != node_index
            node = sn.nodes[nextnode_index]
            center = VecE2(node.pos.x, node.pos.y)
            add_instruction!(rm, render_circle, (center.x, center.y, 1.0, color))
        end
    end

    rm
end
function render_streetnet_lane_nodes!(
    rm::RenderModel,
    sn::StreetNetwork;
    color::Colorant=colorant"green",
    )

    for tile in values(sn.tile_dict)
        for seg in values(tile.segments)
            for lane in values(seg.lanes)
                for node_index in lane.node_indeces
                    node = sn.nodes[node_index]
                    center = VecE2(node.pos.x, node.pos.y)
                    add_instruction!(rm, render_circle, (center.x, center.y, 0.3, color))
                end
            end
        end
    end

    rm
end
function render_streetnet_lane_first_nodes!(
    rm::RenderModel,
    sn::StreetNetwork;
    color::Colorant=colorant"blue",
    )


    for tile in values(sn.tile_dict)
        for seg in values(tile.segments)
            for lane in values(seg.lanes)
                node_index = lane.node_indeces[1]
                node = sn.nodes[node_index]
                center = VecE2(node.pos.x, node.pos.y)
                add_instruction!(rm, render_circle, (center.x, center.y, 1.0, color))
            end
        end
    end

    rm
end
function render_streetnet_lane_last_nodes!(
    rm::RenderModel,
    sn::StreetNetwork;
    color::Colorant=colorant"red",
    )

    for tile in values(sn.tile_dict)
        for seg in values(tile.segments)
            for lane in values(seg.lanes)
                node_index = lane.node_indeces[end]
                node = sn.nodes[node_index]
                center = VecE2(node.pos.x, node.pos.y)
                add_instruction!(rm, render_circle, (center.x, center.y, 1.0, color))
            end
        end
    end

    rm
end
function render_streetnet_lane_connections!(
    rm::RenderModel,
    sn::StreetNetwork;
    color::Colorant=colorant"cyan",
    )

    for tile in values(sn.tile_dict)
        for seg in values(tile.segments)
            for lane in values(seg.lanes)

                if has_next_lane(sn, lane)

                    nextlane = next_lane(sn, lane)

                    node_index_A = lane.node_indeces[end]
                    node_A = sn.nodes[node_index_A]

                    node_index_B = nextlane.node_indeces[1]
                    node_B = sn.nodes[node_index_B]

                    pts = [node_A.pos.x node_B.pos.x;
                           node_A.pos.y node_B.pos.y]

                    add_instruction!(rm, render_line, (pts, color, 0.3))
                end
            end
        end
    end

    rm
end
function render_streetnet_headings!(
    rm::RenderModel,
    sn::StreetNetwork;
    color  :: Colorant=colorant"magenta",
    radius::Real=2.0
    )

    for node in sn.nodes

        lane = get_lane(sn, node)
        center = VecE2(node.pos.x, node.pos.y)
        θ = curve_at(lane.curve, node.extind).θ
        left = center + Vec.polar(1.0, θ)*radius

        pts = [center.x left.x;
               center.y left.y]

        add_instruction!(rm, render_line, (pts, color, 0.1))
    end

    rm
end
function render_streetnet_curve_headings!(
    rm::RenderModel,
    sn::StreetNetwork;
    color :: Colorant = RGB(0xCC, 0x44, 0x44),
    dist::Real=0.1,
    seed::UInt=zero(UInt)
    )

    color2 = RGB(0xCC, 0x88, 0x88)

    srand(seed)

    for tile in values(sn.tile_dict)
        for seg in values(tile.segments)
            for lane in values(seg.lanes)

                n = length(lane.curve.x)

                for (x,y,θ) in zip(lane.curve.x, lane.curve.y, lane.curve.t)

                    center = VecE2(x, y) # + VecE2(0.0, rand())
                    farther = center + Vec.polar(1.0, θ)*dist

                    pts = [center.x farther.x;
                           center.y farther.y]

                    add_instruction!(rm, render_circle, (center.x, center.y, 0.1, color2))
                    add_instruction!(rm, render_line, (pts, color, 0.05))
                end

            end
        end
    end

    rm
end

function render_car!(
    rm::RenderModel,
    car::Vehicle,
    color :: Colorant = RGB(rand(), rand(), rand())
    )

    add_instruction!(rm, render_car, (car.pos.x, car.pos.y, car.pos.ϕ, color))
    rm
end

function render_car!(
    rm::RenderModel,
    runlog::RunLog,
    colset::Integer,
    frame::Integer;
    color::Colorant = RGB(rand(), rand(), rand())
    )

    pos = get(runlog, colset, frame, :inertial)::VecSE2
    add_instruction!(rm, render_car, (pos.x, pos.y, pos.θ, color))
    rm
end
function render_car!(
    rm::RenderModel,
    runlog::RunLog,
    colset::Integer,
    frame::Integer,
    color_fill::Colorant,
    color_stroke::Colorant = color_fill
    )

    pos = get(runlog, colset, frame, :inertial)::VecSE2
    add_instruction!(rm, render_car, (pos.x, pos.y, pos.θ, color_fill, color_stroke))
    rm
end

function render_car!(
    rm::RenderModel,
    pdset::PrimaryDataset,
    carind::Int,
    frameind::Int;
    color::Colorant = RGB(rand(), rand(), rand())
    )

    posGx = posGy = posGθ = 0.0

    if carind == CARIND_EGO
        posGx = gete(pdset, :posGx, frameind)
        posGy = gete(pdset, :posGy, frameind)
        posGθ = gete(pdset, :posGyaw, frameind)
    else
        validfind = frameind2validfind(pdset, frameind)
        posGx = getc(pdset, :posGx, carind, validfind)
        posGy = getc(pdset, :posGy, carind, validfind)
        posGθ = getc(pdset, :posGyaw, carind, validfind)
    end

    # add_instruction!(rm, render_circle, (posGx, posGy, 0.25, color))
    add_instruction!(rm, render_car, (posGx, posGy, posGθ, color))
    rm
end
function render_car!(
    rm::RenderModel,
    pdset::PrimaryDataset,
    carind::Int,
    frameind::Int,
    color_fill::Colorant,
    color_stroke::Colorant = color_fill
    )

    posGx = posGy = posGθ = 0.0

    if carind == CARIND_EGO
        posGx = gete(pdset, :posGx, frameind)
        posGy = gete(pdset, :posGy, frameind)
        posGθ = gete(pdset, :posGyaw, frameind)
    else
        validfind = frameind2validfind(pdset, frameind)
        posGx = getc(pdset, :posGx, carind, validfind)
        posGy = getc(pdset, :posGy, carind, validfind)
        posGθ = getc(pdset, :posGyaw, carind, validfind)
    end

    # add_instruction!(rm, render_circle, (posGx, posGy, 0.25, color_fill, color_stroke))
    add_instruction!(rm, render_car, (posGx, posGy, posGθ, color_fill, color_stroke))
    rm
end

function render_car!(
    rm::RenderModel,
    trajdata::DataFrame,
    carind::Int,
    frameind::Int;
    color :: Colorant = RGB(rand(), rand(), rand())
    )

    posGx = trajdata[frameind, :posGx]
    posGy = trajdata[frameind, :posGy]
    posGθ = trajdata[frameind, :yawG]

    if carind != CARIND_EGO
        posEx = getc(trajdata, "posEx", carind, frameind)
        posEy = getc(trajdata, "posEy", carind, frameind)
        velEx = getc(trajdata, "velEx", carind, frameind)
        velEy = getc(trajdata, "velEy", carind, frameind)

        posGx, posGy = Trajdata.ego2global(posGx, posGy, posGθ, posEx, posEy)



        velGx, velGy = Trajdata.ego2global(0.0, 0.0, posGθ, velEx, velEy)

        if hypot(velGx, velGy) > 3.0
            posGθ        = atan2(velGy, velGx)
        end
    end

    add_instruction!(rm, render_car, (posGx, posGy, posGθ, color))
    rm
end

function render_scene!(
    rendermodel::RenderModel,
    runlog::RunLog,
    frame::Integer;
    active_carid::Integer = RunLog.ID_EGO,
    color_active::Colorant = COLOR_CAR_EGO,
    color_oth::Colorant = COLOR_CAR_OTHER,
    )

    @assert(frame_inbounds(runlog, frame))

    for colset in get_colset_range(runlog, frame)

        id = colset2id(runlog, colset, frame)
        color = (id == active_carid) ? color_active : color_oth

        render_car!(rendermodel, runlog, colset, frame, color=color)
    end

    rendermodel
end
function render_scene!(
    rm::RenderModel,
    pdset::PrimaryDataset,
    frameind::Int;
    active_carid::Integer=CARID_EGO,
    color_active :: Colorant = COLOR_CAR_EGO,
    color_oth :: Colorant = COLOR_CAR_OTHER
    )

    @assert(frameind_inbounds(pdset, frameind))

    # render other cars first
    validfind = frameind2validfind(pdset, frameind)
    for carind in IterAllCarindsInFrame(pdset, validfind)
        carid = carind2id(pdset, carind, validfind)
        color = carid == active_carid ? color_active : color_oth

        render_car!(rm, pdset, carind, frameind, color=color)
    end

    rm
end
function render_scene!(
    rm::RenderModel,
    trajdata::DataFrame,
    frameind::Int;
    color_ego :: Colorant = COLOR_CAR_EGO,
    color_oth :: Colorant = COLOR_CAR_OTHER
    )

    @assert(0 < frameind ≤ size(trajdata, 1))

    # render other cars first
    for i = 1 : get_num_other_cars_in_frame(trajdata, frameind)
        carind = i - 1
        render_car!(rm, trajdata, carind, frameind, color=color_oth)
    end

    # render ego car
    render_car!(rm, trajdata, CARIND_EGO, frameind, color=color_ego)

    rm
end

function camera_center_on_ego!(
    rendermodel::RenderModel,
    runlog::RunLog,
    frame::Int,
    zoom::Real = rendermodel.camera_zoom # [pix/m]
    )

    colset = id2colset(runlog, RunLog.ID_EGO, frame)
    pos = get(runlog, colset, frame, :inertial)::VecSE2
    camera_set!(rendermodel, pos.x, pos.y, zoom)
    rendermodel
end
function camera_center_on_ego!(
    rendermodel::RenderModel,
    pdset::PrimaryDataset,
    frameind::Int,
    zoom::Real = rendermodel.camera_zoom # [pix/m]
    )

    posGx = gete(pdset, :posGx, frameind)
    posGy = gete(pdset, :posGy, frameind)

    camera_set!(rendermodel, posGx, posGy, zoom)
    rendermodel
end
function camera_center_on_ego!(
    rendermodel::RenderModel,
    trajdata::DataFrame,
    frameind::Int,
    zoom::Real = rendermodel.camera_zoom # [pix/m]
    )

    posGx = trajdata[frameind, :posGx]
    posGy = trajdata[frameind, :posGy]

    camera_set!(rendermodel, posGx, posGy, zoom)
    rendermodel
end

function camera_center_on_carid!(
    rendermodel::RenderModel,
    pdset::PrimaryDataset,
    carid::Integer,
    frameind::Integer,
    zoom::Real = rendermodel.camera_zoom # [pix/m]
    )

    if carid == CARID_EGO
        camera_center_on_ego!(rendermodel, pdset, frameind, zoom)
    else
        validfind = frameind2validfind(pdset, frameind)
        carind = carid2ind(pdset, carid, validfind)
        posGx = get(pdset, :posGx, carind, validfind)
        posGy = get(pdset, :posGy, carind, validfind)

        camera_set!(rendermodel, posGx, posGy, zoom)
    end

    rendermodel
end

function set_camera_in_front_of_carid!(
    rendermodel::RenderModel,
    runlog::RunLog,
    colset::UInt,
    frame::Integer,
    camera_forward_offset::Real,
    )

    pos = get(runlog, colset, frame, :inertial)::VecSE2
    pos_camera = convert(VecE2, pos) + Vec.polar(camera_forward_offset, pos.θ)
    camera_set_pos!(rendermodel, pos_camera.x, pos_camera.y)
end
function set_camera_in_front_of_carid!(
    rendermodel::RenderModel,
    pdset::PrimaryDataset,
    carid::Integer,
    frameind::Integer,
    camera_forward_offset::Real,
    )

    if carid == CARID_EGO
        inertial = get_inertial_ego(pdset, frameind)
    else
        validfind = frameind2validfind(pdset, frameind)
        carind = carid2ind(pdset, carid, validfind)
        inertial = get_inertial(pdset, carind, validfind)
    end

    posG = convert(VecE2, inertial) + Vec.polar(camera_forward_offset, inertial.θ)
    camera_set_pos!(rendermodel, posG.x, posG.y)
end

function plot_scene(
    runlog::RunLog,
    sn::StreetNetwork,
    frame::Integer,
    active_carid::Integer=RunLog.ID_EGO;

    canvas_width::Integer=1100,
    canvas_height::Integer=300,
    rendermodel::RenderModel=RenderModel(),

    camera_forward_offset::Float64=60.0, # [m]
    camerazoom::Real=6.5, # [pix/m]

    overlays::AbstractVector{Overlay}=Overlay[],
    )

    s = CairoRGBSurface(canvas_width, canvas_height)
    ctx = creategc(s)
    clear_setup!(rendermodel)

    render_streetnet_roads!(rendermodel, sn)
    render_scene!(rendermodel, runlog, frame, active_carid=active_carid)
    camera_setzoom!(rendermodel, camerazoom)
    set_camera_in_front_of_carid!(rendermodel, runlog, active_carid, frame, camera_forward_offset)

    for overlay in overlays
        render_overlay!(overlay, rendermodel, runlog, sn, frame, active_carid)
    end

    render(rendermodel, ctx, canvas_width, canvas_height)
    s
end
function plot_scene(
    pdset::PrimaryDataset,
    sn::StreetNetwork,
    validfind::Integer,
    active_carid::Integer=CARID_EGO;

    canvas_width::Integer=1100,
    canvas_height::Integer=300,
    rendermodel::RenderModel=RenderModel(),

    camera_forward_offset::Float64=60.0, # [m]
    camerazoom::Real=6.5, # [pix/m]
    )

    s = CairoRGBSurface(canvas_width, canvas_height)
    ctx = creategc(s)
    clear_setup!(rendermodel)

    basics = FeatureExtractBasicsPdSet(pdset, sn)

    frameind = validfind2frameind(pdset, validfind)
    render_streetnet_roads!(rendermodel, sn)
    render_scene!(rendermodel, pdset, frameind, active_carid=active_carid)

    camera_setzoom!(rendermodel, camerazoom)
    set_camera_in_front_of_carid!(rendermodel, pdset, active_carid, frameind, camera_forward_offset)

    render(rendermodel, ctx, canvas_width, canvas_height)
    s
end
function plot_scene(
    trajdata::DataFrame,
    sn::StreetNetwork,
    frame::Integer,
    active_carid::Integer=CARID_EGO;

    canvas_width::Integer=1100,
    canvas_height::Integer=300,
    rendermodel::RenderModel=RenderModel(),

    camera_forward_offset::Float64=60.0, # [m]
    camerazoom::Real=6.5, # [pix/m]
    )

    s = CairoRGBSurface(canvas_width, canvas_height)
    ctx = creategc(s)
    clear_setup!(rendermodel)

    render_streetnet_roads!(rendermodel, sn)
    render_scene!(rendermodel, trajdata, frame)

    camera_center_on_ego!(rendermodel, trajdata, frame, camerazoom)

    render(rendermodel, ctx, canvas_width, canvas_height)
    s
end

function plot_scene_and_front_rear(
    runlog::RunLog,
    sn::StreetNetwork,
    frame::Integer,
    active_carid::Integer=RunLog.ID_EGO;

    canvas_width::Integer=1100,
    canvas_height::Integer=300,
    rendermodel::RenderModel=RenderModel(),

    camera_forward_offset::Float64=60.0, # [m]
    camerazoom::Real=6.5, # [pix/m]
    )

    s = CairoRGBSurface(canvas_width, canvas_height)
    ctx = creategc(s)
    clear_setup!(rendermodel)

    render_streetnet_roads!(rendermodel, sn)
    render_scene!(rendermodel, runlog, frame, active_carid=active_carid)

    id = convert(UInt, active_carid)
    colset = id2colset(runlog, id, frame)
    if colset != COLSET_NULL

        ego = get(runlog, colset, frame, :inertial)::VecSE2

        idfront = get(runlog, colset, frame, :idfront)
        if idfront != COLSET_NULL
            pts = Array(Float64, 2, 2)
            pts[1,1] = ego.x + 2.0*cos(ego.θ)
            pts[2,1] = ego.y + 2.0*sin(ego.θ)

            oth = get(runlog, idfront, frame, :inertial)::VecSE2
            pts[1,2] = oth.x - 2.0*cos(oth.θ)
            pts[2,2] = oth.y - 2.0*sin(oth.θ)

            add_instruction!(rendermodel, render_line, (pts, RGBA(0.0, 1.0, 0.0, 1.0), 0.5))
            render_car!(rendermodel, runlog, idfront, frame, color=RGBA(1.0, 1.0, 1.0, 0.5))
        end

        idrear = get(runlog, colset, frame, :idrear)
        if idrear != COLSET_NULL
            pts = Array(Float64, 2, 2)
            pts[1,1] = ego.x - 2.0*cos(ego.θ)
            pts[2,1] = ego.y - 2.0*sin(ego.θ)

            oth = get(runlog, idrear, frame, :inertial)::VecSE2
            pts[1,2] = oth.x + 2.0*cos(oth.θ)
            pts[2,2] = oth.y + 2.0*sin(oth.θ)

            add_instruction!(rendermodel, render_line, (pts, RGBA(0.0, 0.0, 1.0, 1.0), 0.5))
            render_car!(rendermodel, runlog, idrear, frame, color=RGBA(1.0, 1.0, 1.0, 0.5))
        end
    end

    camera_setzoom!(rendermodel, camerazoom)
    set_camera_in_front_of_carid!(rendermodel, runlog, active_carid, frame, camera_forward_offset)

    render(rendermodel, ctx, canvas_width, canvas_height)
    s
end
function plot_scene_and_footpoint(
    runlog::RunLog,
    sn::StreetNetwork,
    frame::Integer,
    active_carid::Integer=RunLog.ID_EGO;

    canvas_width::Integer=1100,
    canvas_height::Integer=300,
    rendermodel::RenderModel=RenderModel(),

    camera_forward_offset::Float64=60.0, # [m]
    camerazoom::Real=6.5, # [pix/m]
    )

    s = CairoRGBSurface(canvas_width, canvas_height)
    ctx = creategc(s)
    clear_setup!(rendermodel)

    render_streetnet_roads!(rendermodel, sn)
    render_scene!(rendermodel, runlog, frame, active_carid=active_carid)

    id = convert(UInt, active_carid)
    colset = id2colset(runlog, id, frame)
    if colset != COLSET_NULL

        inertial = get(runlog, colset, frame, :inertial)::VecSE2
        footpoint = get(runlog, colset, frame, :footpoint)::CurvePt

        pts = Array(Float64, 2, 2)
        pts[1,1] = inertial.x
        pts[2,1] = inertial.y

        pts[1,2] = footpoint.x
        pts[2,2] = footpoint.y

        add_instruction!(rendermodel, render_line, (pts, RGBA(0.0, 0.0, 0.0, 1.0), 0.3))
    end

    camera_setzoom!(rendermodel, camerazoom)
    set_camera_in_front_of_carid!(rendermodel, runlog, active_carid, frame, camera_forward_offset)

    render(rendermodel, ctx, canvas_width, canvas_height)
    s
end


function plot_traces(
    runlog::RunLog,
    sn::StreetNetwork,
    frame::Integer,
    horizon::Integer,
    history::Integer;
    active_carid::Integer=RunLog.ID_EGO,
    canvas_width::Integer=1100, # [pix]
    canvas_height::Integer=500, # [pix]
    rendermodel::RenderModel=RenderModel(),
    camerazoom::Real=6.5,

    colset::Integer = id2colset(runlog, active_carid, frame),
    camerax::Real = get(runlog, colset, frame :inertial).x,
    cameray::Real = get(runlog, colset, frame :inertial).y,
    color_history::Colorant=RGBA(0.7,0.3,0.0,0.8),
    color_horizon::Colorant=RGBA(0.3,0.3,0.7,0.8),
    )

    s = CairoRGBSurface(canvas_width, canvas_height)
    ctx = creategc(s)
    clear_setup!(rendermodel)

    render_streetnet_roads!(rendermodel, sn)

    for colset in get_colset_range(runlog, frame)
        id = colset2id(runlog, colset, frame)
        render_trace!(rendermodel, runlog, id, frame - history, frame, color=color_history)
        render_trace!(rendermodel, runlog, id, frame, frame + horizon, color=color_horizon)
    end

    # render car positions
    render_scene!(rendermodel, runlog, frame, active_carid=active_carid)

    camera_setzoom!(rendermodel, camerazoom)
    camera_set_pos!(rendermodel, camerax, cameray)
    render(rendermodel, ctx, canvas_width, canvas_height)
    s
end
function plot_traces(
    pdset::PrimaryDataset,
    sn::StreetNetwork,
    validfind::Integer,
    horizon::Integer,
    history::Integer;
    active_carid::Integer=CARID_EGO,
    canvas_width::Integer=1100, # [pix]
    canvas_height::Integer=500, # [pix]
    rendermodel::RenderModel=RenderModel(),
    camerazoom::Real=6.5,
    camerax::Real=get(pdset, :posGx, active_carid, validfind),
    cameray::Real=get(pdset, :posGy, active_carid, validfind),
    color_history::Colorant=RGBA(0.7,0.3,0.0,0.8),
    color_horizon::Colorant=RGBA(0.3,0.3,0.7,0.8),
    )


    s = CairoRGBSurface(canvas_width, canvas_height)
    ctx = creategc(s)
    clear_setup!(rendermodel)

    render_streetnet_roads!(rendermodel, sn)

    for carind in -1 : get_maxcarind(pdset, validfind)
        carid = carind2id(pdset, carind, validfind)
        render_trace!(rendermodel, pdset, carid, validfind - history, validfind, color=color_history)
        render_trace!(rendermodel, pdset, carid, validfind, validfind + horizon, color=color_horizon)
    end

    # render car positions
    render_scene!(rendermodel, pdset, validfind, active_carid=active_carid)

    camera_setzoom!(rendermodel, camerazoom)
    camera_set_pos!(rendermodel, camerax, cameray)
    render(rendermodel, ctx, canvas_width, canvas_height)
    s
end

function plot_extracted_trajdefs(
    pdset::PrimaryDataset,
    sn::StreetNetwork,
    extracted_trajdefs::Vector{ExtractedTrajdef},
    validfind::Integer,
    active_carid::Integer=CARID_EGO;

    canvas_width::Integer=1100, # [pix]
    canvas_height::Integer=500, # [pix]
    rendermodel::RenderModel=RenderModel(),
    camerazoom::Real=6.5,
    camera_forward_offset::Float64=60.0, # [m]

    color_trace::Colorant=RGBA(0.7,0.0,0.3,0.8),
    )


    s = CairoRGBSurface(canvas_width, canvas_height)
    ctx = creategc(s)
    clear_setup!(rendermodel)

    render_streetnet_roads!(rendermodel, sn)

    for extracted in extracted_trajdefs
        render_extracted_trajdef!(rendermodel, extracted, color=color_trace)
    end

    render_scene!(rendermodel, pdset, validfind, active_carid=active_carid)

    frameind = validfind2frameind(pdset, validfind)
    camera_setzoom!(rendermodel, camerazoom)
    set_camera_in_front_of_carid!(rendermodel, pdset, active_carid, frameind, camera_forward_offset)

    render(rendermodel, ctx, canvas_width, canvas_height)
    s
end
function plot_extracted_trajdefs(
    pdset::PrimaryDataset,
    sn::StreetNetwork,
    extracted_trajdefs::Vector{ExtractedTrajdef},
    trajdef_color_scalars::Vector{Float64},
    validfind::Integer,
    active_carid::Integer=CARID_EGO;

    canvas_width::Integer=1100, # [pix]
    canvas_height::Integer=500, # [pix]
    rendermodel::RenderModel=RenderModel(),
    camerazoom::Real=6.5,
    camera_forward_offset::Float64=60.0, # [m]

    trajdef_color_lo::Colorant=RGB(0.0,0.0,1.0),
    trajdef_color_hi::Colorant=RGB(1.0,0.0,0.0),
    )


    s = CairoRGBSurface(canvas_width, canvas_height)
    ctx = creategc(s)
    clear_setup!(rendermodel)

    render_streetnet_roads!(rendermodel, sn)

    for (extracted, color_scalar) in zip(extracted_trajdefs, trajdef_color_scalars)
        color = lerp_color_rgb(trajdef_color_lo, trajdef_color_hi, color_scalar, 1.0)
        render_extracted_trajdef!(rendermodel, extracted, color=color)
    end

    render_scene!(rendermodel, pdset, validfind, active_carid=active_carid)

    frameind = validfind2frameind(pdset, validfind)
    camera_setzoom!(rendermodel, camerazoom)
    set_camera_in_front_of_carid!(rendermodel, pdset, active_carid, frameind, camera_forward_offset)

    render(rendermodel, ctx, canvas_width, canvas_height)
    s
end

function lerp_color(a::Colorant, b::Colorant, t::Real)

    ra = red(a)
    rb = red(b)
    ga = green(a)
    gb = green(b)
    ba = blue(a)
    bb = blue(b)
    aa = alpha(a)
    ab = alpha(b)

    r = ra + (rb - ra)*t
    g = ga + (gb - ga)*t
    b = ba + (bb - ba)*t
    a = aa + (ab - aa)*t

    RGBA(r,g,b,a)
end
function lerp_color_rgb(a::Colorant, b::Colorant, t::Real)

    ra = red(a)
    rb = red(b)
    ga = green(a)
    gb = green(b)
    ba = blue(a)
    bb = blue(b)

    r = ra + (rb - ra)*t
    g = ga + (gb - ga)*t
    b = ba + (bb - ba)*t

    RGB(r,g,b)
end
function lerp_color_rgb(a::Colorant, b::Colorant, t::Real, alpha::Real)

    ra = red(a)
    rb = red(b)
    ga = green(a)
    gb = green(b)
    ba = blue(a)
    bb = blue(b)

    r = ra + (rb - ra)*t
    g = ga + (gb - ga)*t
    b = ba + (bb - ba)*t

    RGBA(r,g,b, alpha)
end

function plot_manipulable_trajdata{I<:Integer}(
    trajdata::DataFrame,
    sn::StreetNetwork,
    active_carid::Integer=CARID_EGO;

    canvas_width::Integer=1100, # [pix]
    canvas_height::Integer=300, # [pix]
    rendermodel::RenderModel=RenderModel(),
    camerazoom::Real=6.5,
    camera_forward_offset::Real=0.0,

    frames::AbstractVector{I} = 1:nrow(trajdata),
    )

    @manipulate for frame in frames

        plot_scene(trajdata, sn, frame, active_carid,
                   canvas_width=canvas_width,
                   canvas_height=canvas_height,
                   rendermodel=rendermodel,
                   camera_forward_offset=camera_forward_offset,
                   camerazoom=camerazoom)
    end
end

function plot_manipulable_runlog{I<:Integer}(
    runlog::RunLog,
    sn::StreetNetwork,
    active_carid::Integer=RunLog.ID_EGO;

    canvas_width::Integer=1100, # [pix]
    canvas_height::Integer=300, # [pix]
    rendermodel::RenderModel=RenderModel(),
    camerazoom::Real=6.5,
    camera_forward_offset::Real=0.0,

    frames::AbstractVector{I} = 1:nframes(runlog),

    overlays::AbstractVector{Overlay} = Overlay[],
    )

    @manipulate for frame in frames

        plot_scene(runlog, sn, frame, active_carid,
                   canvas_width=canvas_width,
                   canvas_height=canvas_height,
                   rendermodel=rendermodel,
                   camera_forward_offset=camera_forward_offset,
                   camerazoom=camerazoom,
                   overlays=overlays)
    end
end
function plot_manipulable_runlog_and_front_rear{I<:Integer}(
    runlog::RunLog,
    sn::StreetNetwork,
    active_carid::Integer=RunLog.ID_EGO;

    canvas_width::Integer=1100, # [pix]
    canvas_height::Integer=300, # [pix]
    rendermodel::RenderModel=RenderModel(),
    camerazoom::Real=6.5,
    camera_forward_offset::Real=0.0,

    frames::AbstractVector{I} = 1:nframes(runlog),
    )

    @manipulate for frame in frames

        plot_scene_and_front_rear(runlog, sn, frame, active_carid,
                   canvas_width=canvas_width,
                   canvas_height=canvas_height,
                   rendermodel=rendermodel,
                   camera_forward_offset=camera_forward_offset,
                   camerazoom=camerazoom)
    end
end
function plot_manipulable_runlog_and_footpoint{I<:Integer}(
    runlog::RunLog,
    sn::StreetNetwork,
    active_carid::Integer=RunLog.ID_EGO;

    canvas_width::Integer=1100, # [pix]
    canvas_height::Integer=300, # [pix]
    rendermodel::RenderModel=RenderModel(),
    camerazoom::Real=15.0,
    camera_forward_offset::Real=0.0,

    frames::AbstractVector{I} = 1:nframes(runlog),
    )

    @manipulate for frame in frames

        plot_scene_and_footpoint(runlog, sn, frame, active_carid,
                   canvas_width=canvas_width,
                   canvas_height=canvas_height,
                   rendermodel=rendermodel,
                   camera_forward_offset=camera_forward_offset,
                   camerazoom=camerazoom)
    end
end

function plot_manipulable_pdset(
    pdset::PrimaryDataset,
    sn::StreetNetwork,
    active_carid::Integer=CARID_EGO;

    canvas_width::Integer=1100, # [pix]
    canvas_height::Integer=300, # [pix]
    rendermodel::RenderModel=RenderModel(),
    camerazoom::Real=6.5,
    camera_forward_offset::Real=0.0,

    validfind_start::Integer = 1,
    validfind_end::Integer = nvalidfinds(pdset)
    )

    @manipulate for validfind in validfind_start : validfind_end

        plot_scene(pdset, sn, validfind, active_carid,
                   canvas_width=canvas_width,
                   canvas_height=canvas_height,
                   rendermodel=rendermodel,
                   camera_forward_offset=camera_forward_offset,
                   camerazoom=camerazoom)
    end
end

function plot_manipulable_generated_future_traces(
    behavior::AbstractVehicleBehavior,
    pdset::PrimaryDataset,
    sn::StreetNetwork,
    active_carid::Integer,
    validfind_start::Integer,
    history::Integer,
    horizon::Integer;

    nsimulations::Integer=100,
    canvas_width::Integer=1100, # [pix]
    canvas_height::Integer=150, # [pix]
    rendermodel::RenderModel=RenderModel(),
    camerax::Real=get(pdset, :posGx, CARIND_EGO, validfind_start),
    cameray::Real=get(pdset, :posGy, CARIND_EGO, validfind_start),
    camerazoom::Real=6.5,
    color_history::Colorant=RGB(0.7,0.3,0.0,0.8),
    color_horizon::Colorant=RGB(0.3,0.3,0.7,0.8),
    color_dot::Colorant=RGB(0.7,0.3,0.3,0.5),
    color_stroke::Colorant=RGB(0.0,0.0,0.0,0.0),
    dot_radius::Real=0.25, # [m]
    )

    pdset_sim = deepcopy(pdset)
    basics = FeatureExtractBasicsPdSet(pdset, sn)
    validfind_end = validfind_start+horizon
    positions = Array(Float64, nsimulations, horizon, 2) # {posGx, posGy}
    for i = 1 : nsimulations
        simulate!(basics, behavior, active_carid, validfind_start, validfind_end)

        for (j,validfind_final) in enumerate(validfind_start+1 : validfind_start+horizon)

            carind = carid2ind(pdset_sim, active_carid, validfind_final)
            positions[i,j,1] = get(pdset_sim, :posGx, carind, validfind_final)
            positions[i,j,2] = get(pdset_sim, :posGy, carind, validfind_final)
        end
    end

    validfind = validfind_start
    @manipulate for validfind = validfind_start-history : validfind_end

        s = CairoRGBSurface(canvas_width, canvas_height)
        ctx = creategc(s)
        clear_setup!(rendermodel)

        render_streetnet_roads!(rendermodel, sn)

        for carind in -1 : get_maxcarind(pdset, validfind)
            carid = carind2id(pdset, carind, validfind)
            if carid != active_carid || validfind ≤ validfind_start
                frameind = validfind2frameind(pdset, validfind)
                render_car!(rendermodel, pdset, carind, frameind, color=(carid==active_carid?COLOR_CAR_EGO:COLOR_CAR_OTHER))
            end
        end

        if validfind > validfind_start
            j = validfind - validfind_start
            for i = 1 : nsimulations
                x, y = positions[i,j,1], positions[i,j,2]
                add_instruction!(rendermodel, render_circle, (x,y,dot_radius,color_dot,color_stroke))
            end
        end

        camera_setzoom!(rendermodel, camerazoom)
        camera_set_pos!(rendermodel, camerax, cameray)
        render!(ctx, canvas_width, canvas_height, rendermodel)
        s
    end
end
function plot_manipulable_pdset_with_traces(
    pdset::PrimaryDataset,
    sn::StreetNetwork;
    canvas_width::Integer=1100, # [pix]
    canvas_height::Integer=500, # [pix]
    rendermodel::RenderModel=RenderModel(),
    camerazoom::Real=6.5,
    active_carid::Integer=CARID_EGO,
    color_history::Colorant=RGBA(0.7,0.3,0.0,0.8),
    color_horizon::Colorant=RGBA(0.3,0.3,0.7,0.8),
    history::Int=20,
    )

    nvalidfinds_total = nvalidfinds(pdset)
    validfind = 1
    @manipulate for validfind = 1 : nvalidfinds_total

        s = CairoRGBSurface(canvas_width, canvas_height)
        ctx = creategc(s)
        clear_setup!(rendermodel)

        frameind = validfind2frameind(pdset, validfind)
        render_streetnet_roads!(rendermodel, sn)

        for carind in -1 : get_maxcarind(pdset, validfind)
            carid = carind2id(pdset, carind, validfind)
            render_trace!(rendermodel, pdset, carid, validfind - history, validfind, color=color_history)
            render_trace!(rendermodel, pdset, carid, validfind, validfind + horizon, color=color_horizon)
        end

        render_scene!(rendermodel, pdset, frameind, active_carid=active_carid)
        camera_center_on_carid!(rendermodel, pdset, active_carid, frameind, camerazoom)

        render(rendermodel, ctx, canvas_width, canvas_height)
        s
    end
end
function plot_manipulable_pdset_with_extracted_trajdefs(
    pdset::PrimaryDataset,
    sn::StreetNetwork,
    extracted_trajdefs::Vector{ExtractedTrajdef};
    canvas_width::Integer=1100, # [pix]
    canvas_height::Integer=500, # [pix]
    rendermodel::RenderModel=RenderModel(),
    camerazoom::Real=6.5,
    active_carid::Integer=CARID_EGO,
    color_trace::Colorant=RGBA(0.7,0.3,0.0,0.8),
    color_extracted::Colorant=RGB(0.7,0.3,0.0),
    color_extracted_fill::Colorant=RGBA(0,0,0,0)
    )



    nvalidfinds_total = nvalidfinds(pdset)
    validfind = 1
    @manipulate for validfind = 1 : nvalidfinds_total

        s = CairoRGBSurface(canvas_width, canvas_height)
        ctx = creategc(s)
        clear_setup!(rendermodel)

        frameind = validfind2frameind(pdset, validfind)
        render_streetnet_roads!(rendermodel, sn)

        for extracted in extracted_trajdefs
            render_extracted_trajdef!(rendermodel, extracted, color=color_trace)
        end
        for extracted in extracted_trajdefs
            df_frame = validfind - extracted.df[1,:frame] + 1
            if 1 ≤ df_frame ≤ nrow(extracted.df)
                posGx = extracted.df[df_frame, :posGx]
                posGy = extracted.df[df_frame, :posGy]
                posGθ = extracted.df[df_frame, :posGyaw]
                add_instruction!(rendermodel, render_car, (posGx, posGy, posGθ, color_extracted_fill, color_extracted))
            end
        end
        for carind in CARIND_EGO : get_maxcarind(pdset, validfind)
            carid = carind2id(pdset, carind, validfind)
            if carid != active_carid
                color = carind == CARID_EGO ? COLOR_CAR_EGO : COLOR_CAR_OTHER
                render_car!(rendermodel, pdset, carind, frameind, color=color)
            end
        end

        camera_center_on_carid!(rendermodel, pdset, active_carid, frameind, camerazoom)

        render(rendermodel, ctx, canvas_width, canvas_height)
        s
    end
end
function plot_manipulable_pdset_with_extracted_trajdefs(
    pdset::PrimaryDataset,
    sn::StreetNetwork,
    candtrajs::Vector{CandidateTrajectory},
    ntrajs::Int;
    canvas_width::Integer=1100, # [pix]
    canvas_height::Integer=500, # [pix]
    rendermodel::RenderModel=RenderModel(),
    camerazoom::Real=6.5,
    active_carid::Integer=CARID_EGO,
    color_trace::Colorant=RGBA(0.7,0.3,0.0,0.8),
    color_extracted::Colorant=RGB(0.7,0.3,0.0),
    color_extracted_fill::Colorant=RGBA(0,0,0,0)
    )


    nvalidfinds_total = nvalidfinds(pdset)
    validfind = 1
    @manipulate for validfind = 1 : nvalidfinds_total

        s = CairoRGBSurface(canvas_width, canvas_height)
        ctx = creategc(s)
        clear_setup!(rendermodel)

        frameind = validfind2frameind(pdset, validfind)
        render_streetnet_roads!(rendermodel, sn)

        for candtraj in candtrajs
            render_extracted_trajdef!(rendermodel, cantraj, color=color_trace)
        end
        for candtraj in candtrajs
            df_frame = validfind - extracted.df[1,:frame] + 1
            if 1 ≤ df_frame ≤ nrow(extracted.df)
                posGx = extracted.df[df_frame, :posGx]
                posGy = extracted.df[df_frame, :posGy]
                posGθ = extracted.df[df_frame, :posGyaw]
                add_instruction!(rendermodel, render_car, (posGx, posGy, posGθ, color_extracted_fill, color_extracted))
            end
        end
        for carind in CARIND_EGO : get_maxcarind(pdset, validfind)
            carid = carind2id(pdset, carind, validfind)
            if carid != active_carid
                color = carind == CARID_EGO ? COLOR_CAR_EGO : COLOR_CAR_OTHER
                render_car!(rendermodel, pdset, carind, frameind, color=color)
            end
        end

        camera_center_on_carid!(rendermodel, pdset, active_carid, frameind, camerazoom)

        render(rendermodel, ctx, canvas_width, canvas_height)
        s
    end
end

function plot_manipulable_pdset_with_extracted_trajdefs(
    pdset::PrimaryDataset,
    sn::StreetNetwork,
    extracted_trajdefs::Vector{ExtractedTrajdef},
    collision_risk::Vector{Float64};
    canvas_width::Integer=1100, # [pix]
    canvas_height::Integer=500, # [pix]
    rendermodel::RenderModel=RenderModel(),
    camerazoom::Real=6.5,
    active_carid::Integer=CARID_EGO,
    color_risk_lo::Colorant=RGB(0.0,0.0,1.0),
    color_risk_hi::Colorant=RGB(1.0,0.0,0.0),
    )

    nvalidfinds_total = nvalidfinds(pdset)
    validfind = 1
    @manipulate for validfind = 1 : nvalidfinds_total

        s = CairoRGBSurface(canvas_width, canvas_height)
        ctx = creategc(s)
        clear_setup!(rendermodel)

        frameind = validfind2frameind(pdset, validfind)
        render_streetnet_roads!(rendermodel, sn)

        for (collision_prob,extracted) in zip(collision_risk, extracted_trajdefs)
            color = lerp_color_rgb(color_risk_lo, color_risk_hi, collision_prob, 0.5)
            render_extracted_trajdef!(rendermodel, extracted, color=color)
        end
        for (collision_prob,extracted) in zip(collision_risk, extracted_trajdefs)
            df_frame = validfind - extracted.df[1,:frame] + 1
            if 1 ≤ df_frame ≤ nrow(extracted.df)
                posGx = extracted.df[df_frame, :posGx]
                posGy = extracted.df[df_frame, :posGy]
                posGθ = extracted.df[df_frame, :posGyaw]
                color = lerp_color_rgb(color_risk_lo, color_risk_hi, collision_prob, 0.5)
                add_instruction!(rendermodel, render_car, (posGx, posGy, posGθ, color))
            end
        end
        for carind in CARIND_EGO : get_maxcarind(pdset, validfind)
            active_carind = carid2ind(pdset, active_carid, validfind)
            if carind != active_carind
                color = carind == CARID_EGO ? COLOR_CAR_EGO : COLOR_CAR_OTHER
                render_car!(rendermodel, pdset, carind, frameind, color=color)
            end
        end

        camera_center_on_carid!(rendermodel, pdset, active_carid, frameind, camerazoom)

        render(rendermodel, ctx, canvas_width, canvas_height)
        s
    end
end
function plot_manipulable_pdset_with_extracted_trajdefs(
    pdset::PrimaryDataset,
    sn::StreetNetwork,
    active_carid::Integer,
    validfind_start::Integer,
    history::Integer,
    horizon::Integer,
    extracted_trajdefs::Vector{ExtractedTrajdef},
    collision_risk::Vector{Float64},
    gridcounts::Vector{Vector{Matrix{Float64}}},
    start_points::Vector{VecE2},
    histobin_params::ParamsHistobin;

    canvas_width::Integer=1100, # [pix]
    canvas_height::Integer=150, # [pix]
    rendermodel::RenderModel=RenderModel(),
    camerazoom::Real=6.5,
    color_risk_lo::Colorant=RGB(0.0,0.0,1.0),
    color_risk_hi::Colorant=RGB(1.0,0.0,0.0),
    color_gridcount_lo::Colorant=RGBA(0.0,0.0,0.0,0.0),
    color_gridcount_hi::Colorant=RGBA(1.0,0.0,0.0,1.0),
    count_adjust_exponent::Float64=0.25, # to make low probs stand out more
    )

    @assert(length(start_points) == length(gridcounts))

    # -----------

    gridcounts_copy = deepcopy(gridcounts)
    for i in 1 : length(gridcounts_copy)
        for gridcount in gridcounts_copy[i]
            maxcount = maximum(gridcount)
            for i = 1 : length(gridcount)
                gridcount[i] = (gridcount[i]/maxcount)^count_adjust_exponent
            end
        end
    end

    # -----------

    car_color_fill = RGBA(0,0,0,0)

    validfind = validfind_start
    validfind_end = validfind_start+horizon
    @manipulate for validfind = validfind_start-history : validfind_end

        s = CairoRGBSurface(canvas_width, canvas_height)
        ctx = creategc(s)
        clear_setup!(rendermodel)

        render_streetnet_roads!(rendermodel, sn)

        # for (collision_prob,extracted) in zip(collision_risk, extracted_trajdefs)
        #     color = lerp_color_rgb(color_risk_lo, color_risk_hi, collision_prob, 0.5)
        #     render_extracted_trajdef!(rendermodel, extracted, color=color)
        # end
        for (collision_prob,extracted) in zip(collision_risk, extracted_trajdefs)
            df_frame = validfind - extracted.df[1,:frame] + 1
            if 1 ≤ df_frame ≤ nrow(extracted.df)
                posGx = extracted.df[df_frame, :posGx]
                posGy = extracted.df[df_frame, :posGy]
                posGθ = extracted.df[df_frame, :posGyaw]
                color = lerp_color_rgb(color_risk_lo, color_risk_hi, collision_prob, 1.0)
                add_instruction!(rendermodel, render_car, (posGx, posGy, posGθ, car_color_fill, color))
            end
        end

        if validfind > validfind_start
            for i in 1 : length(gridcounts)

                gridcount = gridcounts_copy[i][validfind - validfind_start]

                posGx_start = start_points[i].x
                posGy_start = start_points[i].y

                # TODO(tim): change this to transform relative to frenet frame
                #            (maybe make it relative to original heading?)
                add_instruction!(rendermodel, render_colormesh, (gridcount,
                                  histobin_params.discx.binedges .+ posGx_start,
                                  histobin_params.discy.binedges .+ posGy_start,
                                  color_gridcount_lo, color_gridcount_hi))
            end
        end

        if validfind ≤ validfind_start
            for carind in CARIND_EGO : get_maxcarind(pdset, validfind)
                carid = carind2id(pdset, carind, validfind)
                frameind = validfind2frameind(pdset, validfind)
                color_stroke = (carid==active_carid?COLOR_CAR_EGO:COLOR_CAR_OTHER)
                render_car!(rendermodel, pdset, carind, frameind, color=color_stroke)
            end
        end

        active_carind = carid2ind(pdset, active_carid, validfind)
        camerax = get(pdset, :posGx, active_carind, validfind)
        cameray = get(pdset, :posGy, active_carind, validfind)

        camera_setzoom!(rendermodel, camerazoom)
        camera_set_pos!(rendermodel, camerax, cameray)
        render(rendermodel, ctx, canvas_width, canvas_height)
        s
    end
end

function plot_manipulable_gridcount_set(
    pdset::PrimaryDataset,
    sn::StreetNetwork,
    active_carid::Integer,
    validfind_start::Integer,
    history::Integer,
    horizon::Integer,
    gridcounts::Vector{Matrix{Float64}},
    histobin_params::ParamsHistobin;

    canvas_width::Integer=1100, # [pix]
    canvas_height::Integer=150, # [pix]
    rendermodel::RenderModel=RenderModel(),
    camerazoom::Real=6.5,
    color_prob_lo::Colorant=RGBA(0.0,0.0,0.0,0.0),
    color_prob_hi::Colorant=RGBA(1.0,0.0,0.0,1.0),
    count_adjust_exponent::Float64=0.25, # to make low probs stand out more
    )

    # -----------

    gridcounts_copy = deepcopy(gridcounts)
    for gridcount in gridcounts_copy
        maxcount = maximum(gridcount)
        for i = 1 : length(gridcount)
            gridcount[i] = (gridcount[i]/maxcount)^count_adjust_exponent
        end
    end

    # -----------

    carind_start = carid2ind(pdset, active_carid, validfind_start)
    posGx_start = get(pdset, :posGx, carind_start, validfind_start)
    posGy_start = get(pdset, :posGy, carind_start, validfind_start)

    validfind = validfind_start
    validfind_end = validfind_start+horizon
    @manipulate for validfind = validfind_start-history : validfind_end

        s = CairoRGBSurface(canvas_width, canvas_height)
        ctx = creategc(s)
        clear_setup!(rendermodel)

        render_streetnet_roads!(rendermodel, sn)

        for carind in -1 : get_maxcarind(pdset, validfind)
            carid = carind2id(pdset, carind, validfind)
            if carid == active_carid
                if validfind > validfind_start
                    gridcount = gridcounts[validfind - validfind_start]

                    # TODO(tim): change this to transform relative to frenet frame
                    add_instruction!(rendermodel, render_colormesh, (gridcount,
                                      histobin_params.discx.binedges .+ posGx_start,
                                      histobin_params.discy.binedges .+ posGy_start,
                                      color_prob_lo, color_prob_hi))
                end
            end
        end

        for carind in -1 : get_maxcarind(pdset, validfind)
            carid = carind2id(pdset, carind, validfind)
            if carid != active_carid || validfind ≤ validfind_start
                frameind = validfind2frameind(pdset, validfind)
                render_car!(rendermodel, pdset, carind, frameind, color=(carid==active_carid?COLOR_CAR_EGO:COLOR_CAR_OTHER))
            end
        end

        active_carind = carid2ind(pdset, active_carid, validfind)
        camerax = get(pdset, :posGx, active_carind, validfind)
        cameray = get(pdset, :posGy, active_carind, validfind)

        camera_setzoom!(rendermodel, camerazoom)
        camera_set_pos!(rendermodel, camerax, cameray)
        render(rendermodel, ctx, canvas_width, canvas_height)
        s
    end
end
function plot_manipulable_gridcount_sets(
    pdset::PrimaryDataset,
    sn::StreetNetwork,
    active_carid::Integer,
    validfind_start::Integer,
    history::Integer,
    horizon::Integer,
    gridcounts::Vector{Vector{Matrix{Float64}}},
    start_points::Vector{VecE2},
    histobin_params::ParamsHistobin;

    canvas_width::Integer=1100, # [pix]
    canvas_height::Integer=150, # [pix]
    rendermodel::RenderModel=RenderModel(),
    camerazoom::Real=6.5,
    color_prob_lo::Colorant=RGBA(0.0,0.0,0.0,0.0),
    color_prob_hi::Colorant=RGBA(1.0,0.0,0.0,1.0),
    count_adjust_exponent::Float64=0.25, # to make low probs stand out more
    )

    @assert(length(start_points) == length(gridcounts))

    # -----------

    gridcounts_copy = deepcopy(gridcounts)
    for i in 1 : length(gridcounts_copy)
        for gridcount in gridcounts_copy[i]
            maxcount = maximum(gridcount)
            for i = 1 : length(gridcount)
                gridcount[i] = (gridcount[i]/maxcount)^count_adjust_exponent
            end
        end
    end

    # -----------

    validfind = validfind_start
    validfind_end = validfind_start+horizon
    @manipulate for validfind = validfind_start-history : validfind_end

        s = CairoRGBSurface(canvas_width, canvas_height)
        ctx = creategc(s)
        clear_setup!(rendermodel)

        render_streetnet_roads!(rendermodel, sn)

        if validfind > validfind_start
            for i in 1 : length(gridcounts)

                gridcount = gridcounts_copy[i][validfind - validfind_start]

                posGx_start = start_points[i].x
                posGy_start = start_points[i].y

                # TODO(tim): change this to transform relative to frenet frame
                #            (maybe make it relative to original heading?)
                add_instruction!(rendermodel, render_colormesh, (gridcount,
                                  histobin_params.discx.binedges .+ posGx_start,
                                  histobin_params.discy.binedges .+ posGy_start,
                                  color_prob_lo, color_prob_hi))
            end
        end

        for carind in -1 : get_maxcarind(pdset, validfind)
            carid = carind2id(pdset, carind, validfind)
            if carid == active_carid || validfind ≤ validfind_start
                frameind = validfind2frameind(pdset, validfind)
                render_car!(rendermodel, pdset, carind, frameind, color=(carid==active_carid?COLOR_CAR_EGO:COLOR_CAR_OTHER))
            end
        end

        active_carind = carid2ind(pdset, active_carid, validfind)
        camerax = get(pdset, :posGx, active_carind, validfind)
        cameray = get(pdset, :posGy, active_carind, validfind)

        camera_setzoom!(rendermodel, camerazoom)
        camera_set_pos!(rendermodel, camerax, cameray)
        render(rendermodel, ctx, canvas_width, canvas_height)
        s
    end
end

function reel_pdset(
    pdset::PrimaryDataset,
    sn::StreetNetwork,
    active_carid::Integer=CARID_EGO;

    canvas_width::Integer=1100, # [pix]
    canvas_height::Integer=300, # [pix]
    rendermodel::RenderModel=RenderModel(),
    camerazoom::Real=6.5,
    camera_forward_offset::Real=0.0,
    fps::Integer=40,
    )

    frames = Array(CairoSurface, nvalidfinds(pdset))
    for (roll_index, validfind) in enumerate(1 : nvalidfinds(pdset))

        frames[roll_index] = plot_scene(pdset, sn, validfind, active_carid,
                                        canvas_width=canvas_width,
                                        canvas_height=canvas_height,
                                        rendermodel=rendermodel,
                                        camera_forward_offset=camera_forward_offset,
                                        camerazoom=camerazoom)
    end

    frames
    # roll(frames, fps=fps) # write("output.gif", film)
end
function reel_scenario_playthrough(
    scenario::Scenario,
    policy::RiskEstimationPolicy;

    pdset::PrimaryDataset=create_scenario_pdset(scenario),
    validfind_start::Integer=scenario.history,
    validfind_end::Integer=nvalidfinds(pdset),

    canvas_width::Integer=1100, # [pix]
    canvas_height::Integer=300, # [pix]
    rendermodel::RenderModel=RenderModel(),
    camerazoom::Real=8.0,
    camera_forward_offset::Real=60.0,

    fps::Integer=3,
    sec_per_frame::Float64=DEFAULT_SEC_PER_FRAME,
    )

    frames = CairoSurface[]

    push!(frames, plot_scene(pdset, sn, validfind_start, active_carid,
                                        canvas_width=canvas_width,
                                        canvas_height=canvas_height,
                                        rendermodel=rendermodel,
                                        camera_forward_offset=camera_forward_offset,
                                        camerazoom=camerazoom))

    validfind = validfind_start
    while validfind < validfind_end

        candidate_trajectories = generate_candidate_trajectories(basics, policy, active_carid, validfind)
        extracted_trajdefs, extracted_polies = extract_trajdefs(basics, candidate_trajectories, active_carid, validfind)

        push!(frames, plot_extracted_trajdefs(pdset, sn, extracted_trajdefs, validfind, active_carid,
                                              canvas_width=canvas_width, canvas_height=canvas_height,
                                              rendermodel=rendermodel, camerazoom=camerazoom,
                                              camera_forward_offset=camera_forward_offset))

        collision_risk = calc_collision_risk_monte_carlo!(basics, policy, candidate_trajectories, active_carid, validfind, sec_per_frame)
        push!(frames, plot_extracted_trajdefs(pdset, sn, extracted_trajdefs, collision_risk, validfind, active_carid,
                                              canvas_width=canvas_width, canvas_height=canvas_height,
                                              rendermodel=rendermodel, camerazoom=camerazoom,
                                              camera_forward_offset=camera_forward_offset))

        costs = calc_costs(policy, extracted_polies, collision_risk)
        max_cost = maximum(costs)
        push!(frames, plot_extracted_trajdefs(pdset, sn, extracted_trajdefs, costs./max_cost, validfind, active_carid,
                                              canvas_width=canvas_width, canvas_height=canvas_height,
                                              rendermodel=rendermodel, camerazoom=camerazoom,
                                              camera_forward_offset=camera_forward_offset))

        best = indmin(costs)
        extracted_best = extracted_trajdefs[best]
        extracted_best_color_factor_arr = [costs[best]/max_cost]

        push!(frames, plot_extracted_trajdefs(pdset, sn, [extracted_best], extracted_best_color_factor_arr, validfind, active_carid,
                                              canvas_width=canvas_width, canvas_height=canvas_height,
                                              rendermodel=rendermodel, camerazoom=camerazoom,
                                              camera_forward_offset=camera_forward_offset))

        insert!(pdset, extracted_best, validfind+1, validfind+N_FRAMES_PER_SIM_FRAME)
        for validfind_viz in validfind+1 : min(validfind+N_FRAMES_PER_SIM_FRAME, validfind_end)
            push!(frames, plot_extracted_trajdefs(pdset, sn, [extracted_best], extracted_best_color_factor_arr, validfind_viz, active_carid,
                                              canvas_width=canvas_width, canvas_height=canvas_height,
                                              rendermodel=rendermodel, camerazoom=camerazoom,
                                              camera_forward_offset=camera_forward_offset))
        end

        validfind += N_FRAMES_PER_SIM_FRAME
    end

    # roll(frames, fps=fps)
    frames
end

function generate_and_plot_manipulable_gridcount_set(
    behavior::AbstractVehicleBehavior,
    pdset::PrimaryDataset,
    sn::StreetNetwork,
    active_carid::Integer,
    validfind_start::Integer,
    history::Integer,
    horizon::Integer;
    disc_bounds_s::Tuple{Float64, Float64}=(0.0,150.0),
    disc_bounds_t::Tuple{Float64, Float64}=(-10.0, 10.0),
    nbinsx::Integer=101,
    nbinsy::Integer=51,
    nsimulations::Integer=1000,
    canvas_width::Integer=1100, # [pix]
    canvas_height::Integer=150, # [pix]
    rendermodel::RenderModel=RenderModel(),
    camerazoom::Real=6.5,
    color_prob_lo::Colorant=RGBA(0.0,0.0,0.0,0.0),
    color_prob_hi::Colorant=RGBA(1.0,0.0,0.0,1.0),
    count_adjust_exponent::Float64=0.25,
    )

    histobin_params = ParamsHistobin(LinearDiscretizer(linspace(disc_bounds_s..., nbinsx+1)),
                                     LinearDiscretizer(linspace(disc_bounds_t..., nbinsy+1)))

    # ncars = get_num_cars_in_frame(pdset, validfind_start)

    pdset_sim = deepcopy(pdset)

    gridcounts = allocate_gridcounts!(horizon, histobin_params)

    calc_future_grid_counts!(gridcounts, histobin_params, pdset_sim, sn, behavior,
                             active_carid, validfind_start, validfind_start + horizon, nsimulations)

    # -----------

    plot_manipulable_gridcount_set(pdset, sn, active_carid,
                                   validfind_start, history, horizon,
                                   gridcounts, histobin_params,
                                   canvas_width=canvas_width,
                                   canvas_height=canvas_height,
                                   rendermodel=rendermodel,
                                   camerazoom=camerazoom,
                                   color_prob_lo=color_prob_lo,
                                   color_prob_hi=color_prob_hi,
                                   count_adjust_exponent=count_adjust_exponent)
end

nothing