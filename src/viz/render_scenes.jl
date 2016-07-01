function render!(
    rendermodel::RenderModel,
    scene::Scene;
    car_color::Colorant=COLOR_CAR_OTHER,
    special_car_colors::Dict{Int,Colorant}=Dict{Int,Colorant}(), #  id -> color
    )

    for veh in scene
        render!(rendermodel, veh, get(special_car_colors, veh.def.id, car_color))
    end

    rendermodel
end

function render(scene::Scene, roadway::Roadway;
    canvas_width::Int=DEFAULT_CANVAS_WIDTH,
    canvas_height::Int=DEFAULT_CANVAS_HEIGHT,
    rendermodel = RenderModel(),
    )

    s = CairoRGBSurface(canvas_width, canvas_height)
    ctx = creategc(s)
    clear_setup!(rendermodel)

    render!(rendermodel, roadway)
    render!(rendermodel, scene)

    camera_fit_to_content!(rendermodel, canvas_width, canvas_height)
    render(rendermodel, ctx, canvas_width, canvas_height)
    s
end