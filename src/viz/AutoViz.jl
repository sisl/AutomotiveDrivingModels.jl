"""
    AutoViz
Provides visualization tools for AutomotiveDrivingModels
Built off of PGFPlots and Cairo

To use:
    include(Pkg.dir("AutomotiveDrivingModels", "src", "viz", "AutoViz.jl"))
    using AutoViz
"""
module AutoViz

using Reexport

using AutomotiveDrivingModels

@reexport using Colors
@reexport using Cairo

using Reactive
import Gtk
# using PGFPlots

export
        DEFAULT_CANVAS_WIDTH,
        DEFAULT_CANVAS_HEIGHT,
        render!

const DEFAULT_CANVAS_WIDTH = 1000
const DEFAULT_CANVAS_HEIGHT = 600

include(Pkg.dir("AutomotiveDrivingModels", "src", "viz", "colorscheme.jl"))
include(Pkg.dir("AutomotiveDrivingModels", "src", "viz", "rendermodels.jl"))

include(Pkg.dir("AutomotiveDrivingModels", "src", "viz", "render_roadways.jl"))
include(Pkg.dir("AutomotiveDrivingModels", "src", "viz", "render_vehicles.jl"))
include(Pkg.dir("AutomotiveDrivingModels", "src", "viz", "render_scenes.jl"))

include(Pkg.dir("AutomotiveDrivingModels", "src", "viz", "overlays.jl"))

include(Pkg.dir("AutomotiveDrivingModels", "src", "viz", "gtk.jl"))

end # module