abstract Overlay
function render!(rendermodel::RenderModel, overlay::Overlay, trajdata::Trajdata, frame::Int)
    # does nothing
    rendermodel
end

type LineToCenterline <: Overlay
    target_id::Int # if -1 does it for all
end
function render!(rendermodel::RenderModel, overlay::LineToCenterline, trajdata::Trajdata, frame::Int,
    color::Colorant = colorant"white",
    )

    line_width = 0.5

    if overlay.target_id < 0
        target_ids = carsinframe(trajdata, frame)
    else
        target_ids = [overlay.t
        arget_id]
    end

    veh = Vehicle()
    for carid in target_ids
        get_vehicle!(veh, trajdata, carid, frame)
        footpoint = get_footpoint(veh)
        Renderer.add_instruction!(rendermodel, render_line_segment, (veh.state.posG.x, veh.state.posG.y, footpoint.x, footpoint.y, color, line_width))
    end

    rendermodel
end

type FrenetDisplay <: Overlay
    target_id::Int # if -1 does it for all
end
function render!(rendermodel::RenderModel, overlay::FrenetDisplay, trajdata::Trajdata, frame::Int,
    color::Colorant = colorant"white",
    )

    render!(rendermodel, LineToCenterline(overlay.target_id), trajdata, frame)

    if overlay.target_id > 0
        veh = get_vehicle(trajdata, overlay.target_id, frame)
        posF, laneid = project_to_closest_lane(veh.state.posG, trajdata.roadway)

        text_y = -5
        Renderer.add_instruction!(rendermodel, render_text, (@sprintf("extind: %.6f", posF.x), 10, text_y+=20, 15, color), incameraframe=false)
        Renderer.add_instruction!(rendermodel, render_text, (@sprintf("laneid: %d", laneid), 10, text_y+=20, 15, color), incameraframe=false)
        Renderer.add_instruction!(rendermodel, render_text, (@sprintf("d_cl: %.6f", posF.y), 10, text_y+=20, 15, color), incameraframe=false)
        Renderer.add_instruction!(rendermodel, render_text, (@sprintf("theta: %.5f", rad2deg(posF.θ)), 10, text_y+=20, 15, color), incameraframe=false)
        text_y+=20
        posF = veh.state.posF
        Renderer.add_instruction!(rendermodel, render_text, (@sprintf("extind: %.6f", posF.extind), 10, text_y+=20, 15, color), incameraframe=false)
        Renderer.add_instruction!(rendermodel, render_text, (@sprintf("laneid: %d", posF.laneid), 10, text_y+=20, 15, color), incameraframe=false)
        Renderer.add_instruction!(rendermodel, render_text, (@sprintf("d_cl: %.6f", posF.t), 10, text_y+=20, 15, color), incameraframe=false)
        Renderer.add_instruction!(rendermodel, render_text, (@sprintf("theta: %.5f", rad2deg(posF.ϕ)), text_y+=20, 75, 15, color), incameraframe=false)
    end

    rendermodel
end

type SubSceneBox <: Overlay
    scene_extract::Union{SceneExtractParams, SubScene1DExtractParams}
end
render!(rendermodel::RenderModel, overlay::SubSceneBox, trajdata::Trajdata, frame::Int) = render!(rendermodel, overlay.scene_extract)
