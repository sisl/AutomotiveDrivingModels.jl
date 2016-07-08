export
    Camera,
    FitToContentCamera,
    CarFollowCamera,
    SceneFollowCamera

abstract Camera
camera_set!(::RenderModel, cam::Camera, ::Scene, ::Roadway, canvas_width::Int, canvas_height::Int) = error("camera_set! not implemented for Camera $cam")

type FitToContentCamera <: Camera
    percent_border::Float64
    FitToContentCamera(percent_border::Float64=0.1) = new(percent_border)
end
function camera_set!(rendermodel::RenderModel, cam::FitToContentCamera, scene::Scene, roadway::Roadway, canvas_width::Int, canvas_height::Int)
    camera_fit_to_content!(rendermodel, canvas_width, canvas_height, cam.percent_border)
    rendermodel
end

type CarFollowCamera <: Camera
    targetid::Int
    zoom::Float64 # [pix/meter]
    CarFollowCamera(targetid::Int, zoom::Float64=1.0) = new(targetid, zoom)
end
function camera_set!(rendermodel::RenderModel, cam::CarFollowCamera, scene::Scene, roadway::Roadway, canvas_width::Int, canvas_height::Int)

    veh_index = get_index_of_first_vehicle_with_id(scene, cam.id)
    if veh_index != 0
        camera_set_pos!(rendermodel, scene[veh_index].pos)
        camera_setzoom!(rendermodel, cam.zoom)
    else
        add_instruction!( rendermodel, render_text, (@sprintf("CarFollowCamera did not find id %d", cam.targetid), 10, 15, 15, colorant"white"), incameraframe=false)
        camera_fit_to_content!(rendermodel, canvas_width, canvas_height, cam.percent_border)
    end

    rendermodel
end

type SceneFollowCamera <: Camera
    zoom::Float64 # [pix/meter]
end
function camera_set!(rendermodel::RenderModel, cam::SceneFollowCamera, scene::Scene, roadway::Roadway, canvas_width::Int, canvas_height::Int)


    if length(scene) > 0

        # get camera center
        C = VecE2(0.0,0.0)
        for veh in scene
            C += convert(VecE2, veh.state.posG)
        end
        C = C / length(scene)

        camera_set_pos!(rendermodel, C)
        camera_setzoom!(rendermodel, cam.zoom)
    else
        add_instruction!( rendermodel, render_text, ("SceneFollowCamera did not find any vehicles"), 10, 15, 15, colorant"white"), incameraframe=false)
        camera_fit_to_content!(rendermodel, canvas_width, canvas_height, cam.percent_border)
    end

    rendermodel
end


