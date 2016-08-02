export
        SceneOverlay,
        LineToCenterlineOverlay,
        LineToFrontOverlay,
        NeighborsOverlay,
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
            println((veh.state.posG.x, veh.state.posG.y, v2.state.posG.x, v2.state.posG.y, overlay.color, overlay.line_width))
            add_instruction!(rendermodel, render_line_segment,
                (veh.state.posG.x, veh.state.posG.y, v2.state.posG.x, v2.state.posG.y, overlay.color, overlay.line_width))
        end
    end

    rendermodel
end

type CarFollowingStatsOverlay <: SceneOverlay
    target_id::Int
    verbosity::Int
    color::Colorant
    font_size::Int

    function CarFollowingStatsOverlay(target_id::Int, verbosity::Int=1;
        color::Colorant=colorant"white",
        font_size::Int=10,
        )

        new(target_id, verbosity, color,font_size)
    end
end
function render!(rendermodel::RenderModel, overlay::CarFollowingStatsOverlay, scene::Scene, roadway::Roadway)

    font_size = overlay.font_size
    text_y = font_size
    text_y_jump = round(Int, font_size*1.2)

    add_instruction!( rendermodel, render_text, (@sprintf("id = %d", overlay.target_id), 10, text_y, font_size, overlay.color), incameraframe=false)
        text_y += text_y_jump

    veh_index = get_index_of_first_vehicle_with_id(scene, overlay.target_id)
    if veh_index != 0
        veh = scene[veh_index]

        if overlay.verbosity ≥ 2
            add_instruction!( rendermodel, render_text, ("posG: " * string(veh.state.posG), 10, text_y, font_size, overlay.color), incameraframe=false)
            text_y += text_y_jump
            add_instruction!( rendermodel, render_text, ("posF: " * string(veh.state.posF), 10, text_y, font_size, overlay.color), incameraframe=false)
            text_y += text_y_jump
        end
        add_instruction!( rendermodel, render_text, (@sprintf("speed: %.3f", veh.state.v), 10, text_y, font_size, overlay.color), incameraframe=false)
        text_y += text_y_jump


        foreinfo = get_neighbor_fore_along_lane(scene, veh_index, roadway; max_distance_fore=Inf)
        if foreinfo.ind != 0
            v2 = scene[foreinfo.ind]
            rel_speed = v2.state.v - veh.state.v
            add_instruction!( rendermodel, render_text, (@sprintf("Δv = %10.3f m/s", rel_speed), 10, text_y, font_size, overlay.color), incameraframe=false)
            text_y += text_y_jump
            add_instruction!( rendermodel, render_text, (@sprintf("Δs = %10.3f m/s", foreinfo.Δs), 10, text_y, font_size, overlay.color), incameraframe=false)
            text_y += text_y_jump

            if overlay.verbosity ≥ 2
                add_instruction!( rendermodel, render_text, ("posG: " * string(v2.state.posG), 10, text_y, font_size, overlay.color), incameraframe=false)
                text_y += text_y_jump
                add_instruction!( rendermodel, render_text, ("posF: " * string(v2.state.posF), 10, text_y, font_size, overlay.color), incameraframe=false)
                text_y += text_y_jump
                add_instruction!( rendermodel, render_text, (@sprintf("speed: %.3f", v2.state.v), 10, text_y, font_size, overlay.color), incameraframe=false)
                text_y += text_y_jump
            end
        else
            add_instruction!( rendermodel, render_text, (@sprintf("no front vehicle"), 10, text_y, font_size, overlay.color), incameraframe=false)
        end
    else
        add_instruction!( rendermodel, render_text, (@sprintf("vehicle %d not found", overlay.target_id), 10, text_y, font_size, overlay.color), incameraframe=false)
    end

    rendermodel
end

type NeighborsOverlay <: SceneOverlay
    target_id::Int
    color_L::Colorant
    color_M::Colorant
    color_R::Colorant
    line_width::Float64
    function NeighborsOverlay(target_id::Int;
        color_L::Colorant=colorant"blue",
        color_M::Colorant=colorant"green",
        color_R::Colorant=colorant"red",
        line_width::Float64=0.5, # [m]
        )

        new(target_id, color_L, color_M, color_R, line_width)
    end
end
function render!(rendermodel::RenderModel, overlay::NeighborsOverlay, scene::Scene, roadway::Roadway)

    vehicle_index = get_index_of_first_vehicle_with_id(scene, overlay.target_id)

    if vehicle_index != 0

        veh_ego = scene[vehicle_index]
        t = veh_ego.state.posF.t
        ϕ = veh_ego.state.posF.ϕ
        v = veh_ego.state.v
        len_ego = veh_ego.def.length

        # line from me to lead
        fore_M = get_neighbor_fore_along_lane(scene, vehicle_index, roadway, VehicleTargetPointFront(), VehicleTargetPointRear(), VehicleTargetPointFront())
        if fore_M.ind != 0
            veh_oth = scene[fore_M.ind]
            A = get_footpoint(veh_ego) + polar(veh_ego.def.length/2, veh_ego.state.posG.θ)
            B = get_footpoint(veh_oth) - polar(veh_oth.def.length/2, veh_oth.state.posG.θ)
            add_instruction!(rendermodel, render_line_segment,
                (A.x, A.y, B.x, B.y, overlay.color_M, overlay.line_width))
        end

        fore_L = get_neighbor_fore_along_left_lane(scene, vehicle_index, roadway, VehicleTargetPointRear(), VehicleTargetPointRear(), VehicleTargetPointFront())
        if fore_L.ind != 0
            veh_oth = scene[fore_L.ind]
            A = get_footpoint(veh_ego) - polar(veh_ego.def.length/2, veh_ego.state.posG.θ)
            B = get_footpoint(veh_oth) - polar(veh_oth.def.length/2, veh_oth.state.posG.θ)
            add_instruction!(rendermodel, render_line_segment,
                (A.x, A.y, B.x, B.y, overlay.color_L, overlay.line_width))

            font_size = 14
            text_y = font_size
            text_y_jump = round(Int, font_size*1.2)

            add_instruction!( rendermodel, render_text, (string(fore_L), 10, text_y, font_size, colorant"white"), incameraframe=false)
        end

        rear_L = get_neighbor_rear_along_left_lane(scene, vehicle_index, roadway, VehicleTargetPointFront(), VehicleTargetPointFront(), VehicleTargetPointRear())
        if rear_L.ind != 0
            veh_oth = scene[rear_L.ind]
            A = get_footpoint(veh_ego) - polar(veh_ego.def.length/2, veh_ego.state.posG.θ)
            B = get_footpoint(veh_oth) + polar(veh_oth.def.length/2, veh_oth.state.posG.θ)
            add_instruction!(rendermodel, render_line_segment,
                (A.x, A.y, B.x, B.y, overlay.color_L, overlay.line_width))
        end


        fore_R = get_neighbor_fore_along_right_lane(scene, vehicle_index, roadway, VehicleTargetPointRear(), VehicleTargetPointRear(), VehicleTargetPointFront())
        if fore_R.ind != 0
            veh_oth = scene[fore_R.ind]
            A = get_footpoint(veh_ego) - polar(veh_ego.def.length/2, veh_ego.state.posG.θ)
            B = get_footpoint(veh_oth) - polar(veh_oth.def.length/2, veh_oth.state.posG.θ)
            add_instruction!(rendermodel, render_line_segment,
                (A.x, A.y, B.x, B.y, overlay.color_R, overlay.line_width))
        end

        rear_R = get_neighbor_rear_along_right_lane(scene, vehicle_index, roadway, VehicleTargetPointFront(), VehicleTargetPointFront(), VehicleTargetPointRear())
        if rear_R.ind != 0
            veh_oth = scene[rear_R.ind]
            A = get_footpoint(veh_ego) - polar(veh_ego.def.length/2, veh_ego.state.posG.θ)
            B = get_footpoint(veh_oth) + polar(veh_oth.def.length/2, veh_oth.state.posG.θ)
            add_instruction!(rendermodel, render_line_segment,
                (A.x, A.y, B.x, B.y, overlay.color_R, overlay.line_width))
        end
    end

    rendermodel
end


function render{O<:SceneOverlay}(scene::Scene, roadway::Roadway, overlays::AbstractVector{O};
    canvas_width::Int=DEFAULT_CANVAS_WIDTH,
    canvas_height::Int=DEFAULT_CANVAS_HEIGHT,
    rendermodel::RenderModel=RenderModel(),
    cam::Camera=SceneFollowCamera(),
    special_car_colors::Dict{Int,Colorant}=Dict{Int,Colorant}(),
    )

    s = CairoRGBSurface(canvas_width, canvas_height)
    ctx = creategc(s)
    clear_setup!(rendermodel)

    render!(rendermodel, roadway)
    render!(rendermodel, scene, special_car_colors=special_car_colors)

    for overlay in overlays
        render!(rendermodel, overlay, scene, roadway)
    end

    camera_set!(rendermodel, cam, scene, roadway, canvas_width, canvas_height)

    render(rendermodel, ctx, canvas_width, canvas_height)
    s
end

