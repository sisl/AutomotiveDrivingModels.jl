export
        DEFAULT_GTK_CANVAS_WIDTH,
        DEFAULT_GTK_CANVAS_HEIGHT,
        DEFAULT_GTK_WINDOW_IS_RESIZABLE,
        GTK_RENDERMODEL,
        GTK_CANVAS,
        GTK_WINDOW,
        GTK_SCENE,
        GTK_ROADWAY,

        show_gtk_window,
        hide_gtk_window,
        set_gtk_scene,
        set_gtk_roadway,
        render_gtk_video_1d

const DEFAULT_GTK_CANVAS_WIDTH = 800 # [pix]
const DEFAULT_GTK_CANVAS_HEIGHT = 600 # [pix]
const DEFAULT_GTK_WINDOW_IS_RESIZABLE = false

const GTK_RENDERMODEL = RenderModel()

GTK_CANVAS = Gtk.@GtkCanvas()
GTK_WINDOW = Gtk.@GtkWindow(GTK_CANVAS, "Automotive Driving Models Simulation",
    DEFAULT_GTK_CANVAS_WIDTH, DEFAULT_GTK_CANVAS_HEIGHT, DEFAULT_GTK_WINDOW_IS_RESIZABLE)
Gtk.show(GTK_CANVAS)
Gtk.visible(GTK_WINDOW, false)

GTK_SCENE = Scene()
GTK_ROADWAY = Roadway()

Gtk.@guarded Gtk.draw(GTK_CANVAS) do widget
    ctx = getgc(GTK_CANVAS)
    canvas_height = height(GTK_CANVAS)
    canvas_width = width(GTK_CANVAS)

    clear_setup!(GTK_RENDERMODEL)

    render!(GTK_RENDERMODEL, GTK_ROADWAY)
    render!(GTK_RENDERMODEL, GTK_SCENE)
    render!(GTK_RENDERMODEL, CarFollowingStatsOverlay(2), GTK_SCENE, GTK_ROADWAY)

    camera_fit_to_content!(GTK_RENDERMODEL, canvas_width, canvas_height)
    render(GTK_RENDERMODEL, ctx, canvas_width, canvas_height)
end

function show_gtk_window()
    Gtk.visible(GTK_WINDOW, true)
end
function hide_gtk_window()
    Gtk.visible(GTK_WINDOW, false)
end

function set_gtk_scene(scene::Scene)
    global GTK_SCENE = scene
end
function set_gtk_roadway(roadway::Roadway)
    global GTK_ROADWAY = roadway
end

function render_gtk_video_1d(
    veh2_actions::Vector{Float64}, # actions are for model2, just longitudinal
    scene::Scene, # initial scene, is overwritten
    model1::DriverModel{LatLonAccel, IntegratedContinuous}, # IDM
    model2::DriverModel{LatLonAccel, IntegratedContinuous}, # StaticLognitudinal
    roadway::Roadway;
    framerate::Int=10,
    )

    global GTK_ROADWAY = roadway
    actions = Array(LatLonAccel, length(scene))
    models = Dict{Int, DriverModel}()
    models[1] = model1
    models[2] = model2

    frame_index = length(veh2_actions)
    function drawsim(t::Float64)

        frame_index += 1
        if frame_index > length(veh2_actions)
            copy!(GTK_SCENE, scene)
            frame_index = 1
        end

        actions[1] = rand(observe!(model1, GTK_SCENE, roadway, 1))

        observe!(model2, GTK_SCENE, roadway, 1)
        model2.mlon.a = veh2_actions[frame_index]
        actions[2] = rand(model2)

        tick!(GTK_SCENE, GTK_ROADWAY, actions, models)
        Gtk.draw(GTK_CANVAS)
    end

    drawsim(1.0)

    show_gtk_window()

    ticks = fps(framerate)
    timestamps = map(_ -> time(), ticks)
    map(drawsim, timestamps)
end



