function render!(
    rendermodel::RenderModel,
    veh::Vehicle,
    color::Colorant=RGB(rand(), rand(), rand())
    )

    color = veh.def.id == 1 ? color : RGB(0.0,1.0,0.0)

    p = veh.state.posG
    add_instruction!(rendermodel, render_vehicle, (p.x, p.y, p.θ, veh.def.length, veh.def.width, color))
    rm
end