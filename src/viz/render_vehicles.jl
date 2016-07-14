function render!(
    rendermodel::RenderModel,
    veh::Vehicle,
    color::Colorant=RGB(rand(), rand(), rand())
    )

    p = veh.state.posG
    add_instruction!(rendermodel, render_vehicle, (p.x, p.y, p.θ, veh.def.length, veh.def.width, color))
    rm
end