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
    velFy = get(VELFT, runlog, sn, colset, frame)
    inv_timegap_front = get(INV_TIMEGAP_FRONT, runlog, sn, colset, frame)
    Δv_front = get(DELTA_V_FRONT, runlog, sn, colset, frame)

    r = colorant"red"
    b = colorant"blue"
    color_freeflow = (isnan(inv_timegap_front) || inv_timegap_front > 1.0/3.0 || Δv_front > 1.0) ? r : b
    str_inferred_context = abs(velFy) > 0.1 ? "lanechange" : (isnan(inv_timegap_front) || inv_timegap_front < 1.0/3.0 || Δv_front > 1.0) ? "freeflow" : "following"

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

    # add_instruction!( rendermodel, render_text, (@sprintf("inv_timegap_front = %.4f", inv_timegap_front), 10, text_y, 15, colorscheme["foreground"]), incameraframe=false)
    # text_y += text_y_jump

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

    rendermodel
end

type EgoToRearCarOverlay <: Overlay
    color::RGBA
    line_width::Float64 # [m]

    function EgoToRearCarOverlay(;
        color::RGBA=RGBA(0x5a/0xFF,0xd4/0xFF,0xed/0xFF,0.8),
        line_width::Float64=0.5,
        )

        new(color, line_width)
    end
end
function render_overlay!(overlay::EgoToRearCarOverlay,
    rendermodel::RenderModel,
    runlog::RunLog,
    sn::StreetNetwork,
    frame::Integer,
    active_carid::Integer,
    )

    colset_ego = id2colset(runlog, active_carid, frame)
    if colset_ego != COLSET_NULL
        colset_rear = get(runlog, colset_ego, frame, :colset_rear)::UInt
        if colset_rear != COLSET_NULL
            @assert(colset_rear != colset_ego)
            pos_ego = get(runlog, colset_ego,  frame, :inertial)::VecSE2
            pos_oth = get(runlog, colset_rear, frame, :inertial)::VecSE2

            pts = Array(Float64, 2, 2)
            pts[1,1] = pos_ego.x
            pts[2,1] = pos_ego.y
            pts[1,2] = pos_oth.x
            pts[2,2] = pos_oth.y

            add_instruction!(rendermodel, render_line, (pts, overlay.color, overlay.line_width))
            add_instruction!( rendermodel, render_text, (@sprintf("dx_rear: %10.6f", get(DIST_REAR, runlog, sn, colset_ego, frame)), 10, 485, 15, colorant"white"), incameraframe=false)
        else
            add_instruction!( rendermodel, render_text, ("colset_rear is NULL", 10, 485, 15, colorant"white"), incameraframe=false)
        end
    else
        add_instruction!( rendermodel, render_text, ("colset_rear is NULL", 10, 485, 15, colorant"white"), incameraframe=false)
    end

    rendermodel
end

type EgoToLeftCarOverlay <: Overlay
    color::RGBA
    line_width::Float64 # [m]

    function EgoToLeftCarOverlay(;
        color::RGBA=RGBA(0x5a/0xFF,0xd4/0xFF,0xed/0xFF,0.8),
        line_width::Float64=0.5,
        )

        new(color, line_width)
    end
end
function render_overlay!(overlay::EgoToLeftCarOverlay,
    rendermodel::RenderModel,
    runlog::RunLog,
    sn::StreetNetwork,
    frame::Integer,
    active_carid::Integer,
    )

    colset_ego = id2colset(runlog, active_carid, frame)
    if colset_ego != COLSET_NULL
        colset_left = Features._get_vehicle_to_left(runlog, sn, colset_ego, frame)
        if colset_left != COLSET_NULL
            @assert(colset_left != colset_ego)
            pos_ego = get(runlog, colset_ego,  frame, :inertial)::VecSE2
            pos_oth = get(runlog, colset_left, frame, :inertial)::VecSE2

            pts = Array(Float64, 2, 2)
            pts[1,1] = pos_ego.x
            pts[2,1] = pos_ego.y
            pts[1,2] = pos_oth.x
            pts[2,2] = pos_oth.y

            add_instruction!(rendermodel, render_line, (pts, overlay.color, overlay.line_width))
        else
            add_instruction!( rendermodel, render_text, ("colset_oth is NULL", 10, 500, 15, colorant"white"), incameraframe=false)
        end
    else
        add_instruction!( rendermodel, render_text, ("colset_ego is NULL", 10, 500, 15, colorant"white"), incameraframe=false)
    end

    rendermodel
end

type RunLogSimOverlay <: Overlay
    runlog::RunLog
    frame_offset::Int
    color::Colorant

    RunLogSimOverlay(runlog::RunLog, frame_offset::Int, color::Colorant=RGBA(0.8,0.2,0.2,0.5)) = new(runlog, frame_offset, color)
end
function render_overlay!(overlay::RunLogSimOverlay,
    rendermodel::RenderModel,
    runlog::RunLog,
    sn::StreetNetwork,
    frame::Integer,
    active_carid::Integer,
    )

    runlog_sim = overlay.runlog
    frame_sim = frame+overlay.frame_offset

    if frame_inbounds(runlog_sim, frame_sim)
        colset = id2colset(runlog_sim, active_carid, frame_sim)
        render_car!(rendermodel, runlog_sim, colset, frame_sim, color=overlay.color)
    end

    rendermodel
end

##################

abstract CellEntry
type CellOverlay <: Overlay
    entries::Vector{CellEntry}
    font_size::Float64
    top_margin::Float64
    side_margin::Float64
    row_spacing::Float64 # distance between cells
end
function render_overlay!(overlay::CellOverlay,
    rendermodel::RenderModel,
    runlog::RunLog,
    sn::StreetNetwork,
    frame::Integer,
    active_carid::Integer,
    )

    text_y = overlay.top_margin
    for entry in overlay.entries
        text_y = render_entry(text_y, entry, overlay, rendermodel, runlog, sn, frame, active_carid)
    end

    rendermodel
end

#####################

type ModelNameCellEntry <: CellEntry
    model_name::AbstractString
    color::Colorant
end
function render_entry(
    text_y::Float64,
    entry::ModelNameCellEntry,
    overlay::CellOverlay,
    rendermodel::RenderModel,
    runlog::RunLog,
    sn::StreetNetwork,
    frame::Integer,
    active_carid::Integer,
    )


    add_instruction!(rendermodel, render_text, ("Yo yo yo", overlay.side_margin, text_y, overlay.font_size, entry.color), incameraframe=false)

    text_y + overlay.row_spacing
end
