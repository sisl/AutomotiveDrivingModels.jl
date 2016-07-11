export reel_drive_1d

function reel_drive_1d(
    gif_filename::AbstractString,
    veh2_actions::Vector{Float64}, # actions are for model2, just longitudinal
    scene::Scene, # initial scene, is overwritten
    model1::DriverModel{LatLonAccel, IntegratedContinuous}, # IDM
    model2::DriverModel{LatLonAccel, IntegratedContinuous}, # StaticLognitudinal
    roadway::Roadway;
    framerate::Int=10,
    overlays::Vector{SceneOverlay}=SceneOverlay[], # CarFollowingStatsOverlay(2)
    cam::Camera=FitToContentCamera(),
    )

    actions = Array(LatLonAccel, length(scene))
    models = Dict{Int, DriverModel}()
    models[1] = model1
    models[2] = model2

    frames = Reel.Frames(MIME("image/png"), fps=framerate)

    push!(frames, render(scene, roadway, overlays, cam=cam))
    for frame_index in 1:length(veh2_actions)

        actions[1] = rand(observe!(model1, scene, roadway, 1))

        observe!(model2, scene, roadway, 1)
        model2.mlon.a = veh2_actions[frame_index]
        actions[2] = rand(model2)

        tick!(scene, roadway, actions, models)
        push!(frames, render(scene, roadway, overlays, cam=cam))
    end

    Reel.write(gif_filename, frames) # Write to a gif file
end