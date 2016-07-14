export
        SceneOverlay,
        LineToCenterlineOverlay,
        LineToFrontOverlay,
        CarFollowingStatsOverlay

abstract SceneOverlay

type LineToCenterlineOverlay <: SceneOverlay
    target_id::Int # if -1 does it for all
    line_width::Float64
    color::Colorant

    function LineToCenterline(target_id::Int=-1;
        line_width::Float64=0.5, #[m]
        color::Colorant=colorant"blue",
        )

        new(target_id, line_width, color)
    end
end
function render!(rendermodel::RenderModel, overlay::LineToCenterlineOverlay, scene::Scene, roadway::Roadway)

    if overlay.target_id < 0
        target_inds = 1:length(scene)
    else
        target_inds = overlay.target_id:overlay.target_id
    end

    for ind in target_inds
        veh = scene[ind]
        footpoint = get_footpoint(veh)
        add_instruction!(rendermodel, render_line_segment,
            (veh.state.posG.x, veh.state.posG.y, footpoint.x, footpoint.y, overlay.color, overlay.line_width))
    end

    rendermodel
end

type LineToFrontOverlay <: SceneOverlay
    target_id::Int # if -1 does it for all
    line_width::Float64
    color::Colorant

    function LineToFrontOverlay(target_id::Int=-1;
        line_width::Float64=0.5, #[m]
        color::Colorant=colorant"blue",
        )

        new(target_id, line_width, color)
    end
end
function render!(rendermodel::RenderModel, overlay::LineToFrontOverlay, scene::Scene, roadway::Roadway)

    if overlay.target_id < 0
        target_inds = 1:length(scene)
    else
        target_inds = overlay.target_id:overlay.target_id
    end

    for ind in target_inds
        veh = scene[ind]
        veh_ind_front = get_neighbor_fore_along_lane(scene, ind, roadway).ind
        if veh_ind_front != 0
            v2 = scene[veh_ind_front]
            add_instruction!(rendermodel, render_line_segment,
                (veh.state.posG.x, veh.state.posG.y, v2.state.posG.x, v2.state.posG.y, overlay.color, overlay.line_width))
        end
    end

    rendermodel
end

type CarFollowingStatsOverlay <: SceneOverlay
    target_id::Int
    color::Colorant

    function CarFollowingStatsOverlay(target_id::Int;
        color::Colorant=colorant"white",
        )

        new(target_id, color)
    end
end
function render!(rendermodel::RenderModel, overlay::CarFollowingStatsOverlay, scene::Scene, roadway::Roadway)

    text_y = 10
    text_y_jump = 12
    font_size = 10

    add_instruction!( rendermodel, render_text, (@sprintf("id = %d", overlay.target_id), 10, text_y, font_size, overlay.color), incameraframe=false)
        text_y += text_y_jump

    veh_index = get_index_of_first_vehicle_with_id(scene, overlay.target_id)
    if veh_index != 0
        veh = scene[veh_index]

        add_instruction!( rendermodel, render_text, (@sprintf("v  = %10.3f m/s", veh.state.v), 10, text_y, font_size, overlay.color), incameraframe=false)
        text_y += text_y_jump
        add_instruction!( rendermodel, render_text, (@sprintf("t  = %10.3f m/s", veh.state.posF.t), 10, text_y, font_size, overlay.color), incameraframe=false)
        text_y += text_y_jump
        add_instruction!( rendermodel, render_text, ("posG: " * string(veh.state.posG), 10, text_y, font_size, overlay.color), incameraframe=false)
        text_y += text_y_jump
        add_instruction!( rendermodel, render_text, ("posF: " * string(veh.state.posF), 10, text_y, font_size, overlay.color), incameraframe=false)
        text_y += text_y_jump


        foreinfo = get_neighbor_fore_along_lane(scene, veh_index, roadway; max_distance_fore=Inf)
        if foreinfo.ind != 0
            v2 = scene[foreinfo.ind]
            rel_speed = v2.state.v - veh.state.v
            add_instruction!( rendermodel, render_text, (@sprintf("Δv = %10.3f m/s", rel_speed), 10, text_y, font_size, overlay.color), incameraframe=false)
            text_y += text_y_jump
            add_instruction!( rendermodel, render_text, (@sprintf("Δs = %10.3f m/s", foreinfo.Δs), 10, text_y, font_size, overlay.color), incameraframe=false)
            text_y += text_y_jump
            add_instruction!( rendermodel, render_text, ("posG: " * string(v2.state.posG), 10, text_y, font_size, overlay.color), incameraframe=false)
            text_y += text_y_jump
            add_instruction!( rendermodel, render_text, ("posF: " * string(v2.state.posF), 10, text_y, font_size, overlay.color), incameraframe=false)
            text_y += text_y_jump
        else
            add_instruction!( rendermodel, render_text, (@sprintf("no front vehicle"), 10, text_y, font_size, overlay.color), incameraframe=false)
        end
    else
        add_instruction!( rendermodel, render_text, (@sprintf("vehicle %d not found", overlay.target_id), 10, text_y, font_size, overlay.color), incameraframe=false)
    end

    rendermodel
end

function render{O<:SceneOverlay}(scene::Scene, roadway::Roadway, overlays::AbstractVector{O};
    canvas_width::Int=DEFAULT_CANVAS_WIDTH,
    canvas_height::Int=DEFAULT_CANVAS_HEIGHT,
    rendermodel::RenderModel=RenderModel(),
    cam::Camera=SceneFollowCamera(),
    )

    s = CairoRGBSurface(canvas_width, canvas_height)
    ctx = creategc(s)
    clear_setup!(rendermodel)

    render!(rendermodel, roadway)
    render!(rendermodel, scene)

    for overlay in overlays
        render!(rendermodel, overlay, scene, roadway)
    end

    camera_set!(rendermodel, cam, scene, roadway, canvas_width, canvas_height)

    render(rendermodel, ctx, canvas_width, canvas_height)
    s
end

