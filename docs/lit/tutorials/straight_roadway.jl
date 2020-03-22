# # Driving on a Straight Roadway

#md # [![](https://img.shields.io/badge/show-nbviewer-579ACA.svg)](@__NBVIEWER_ROOT_URL__/notebooks/straight_roadway.ipynb)

# This example demonstrates a simple, one-dimensional driving simulation in which
# cars drive along a straight roadway.
# The vehicles are represented using the `Entity` type with the followin attributes:
#
# - `state` - `VehicleState`, containing the vehicle position and speed
# - `def` - `VehicleDef`, containing length, width, and class
# - `id` - `Int`, a unique label for each vehicle`
# 
# A driving situation with different vehicles at a given time is referred to as a scene or a frame. It is represented by the `Frame` object. 
# A `Frame` can be thought of as a vector of vehicles. However, in addition to simple vectors it allows to query vehicles by ID using the `get_by_id` function.
# We use a straight roadway with 1 lane as the environment.

using AutomotiveDrivingModels
using AutoViz # for rendering
AutoViz.colortheme["background"] = colorant"white"; # hide

roadway = gen_straight_roadway(1, 2000.0)  # 200m long straight roadway with 1 lane
scene = Frame([
    Entity(VehicleState(VecSE2(10.0,0.0,0.0), roadway, 8.0), VehicleDef(), 1),
    Entity(VehicleState(VecSE2(50.0,0.0,0.0), roadway, 12.5), VehicleDef(), 2),
    Entity(VehicleState(VecSE2(150.0,0.0,0.0), roadway, 6.0), VehicleDef(), 3),
])

veh_1 = get_by_id(scene, 1) # note that the order of the vehicles in the scene does not necessarily match the id

camera = StaticCamera(position=VecE2(100.0,0.0), zoom=4.75, canvas_height=100)
snapshot = render([roadway, scene], camera=camera)
#md write("straight_roadway.svg", snapshot) # hide
#md # ![three cars on road](straight_roadway.svg)

# In this call to the `render` function, we use the default rendering behavior for
# entities. More advanced examples will show how the rendering of entities can be customized.

# We can add an overlay that displays the car id:

idoverlay = IDOverlay(scene=scene, color=colorant"black", font_size=20, y_off=1.)
snapshot = render([roadway, scene, idoverlay], camera=camera)
#md write("straight_roadway_with_id.svg", snapshot) # hide
#md # ![three cars with id](straight_roadway_with_id.svg)

# To run a simulation we need driving models that produce actions.
# For this we will use `LaneFollowingDriver`s that produce `LaneFollowingAccel`s.
# For this demo, we will give each car a different model.

models = Dict{Int, LaneFollowingDriver}(
    1 => StaticLaneFollowingDriver(0.0), # always produce zero acceleration
    2 => IntelligentDriverModel(v_des=12.0), # default IDM with a desired speed of 12 m/s
    3 => PrincetonDriver(v_des = 10.0), # default Princeton driver with a desired speed of 10m/s
)

nticks = 100
timestep = 0.1
scenes = simulate(scene, roadway, models, nticks, timestep)
#md nothing # hide

# We can visualize the simulation as a sequence of images, for example using the
# `Reel` package

using Reel

animation = roll(fps=1.0/timestep, duration=nticks*timestep) do t, dt
    i = Int(floor(t/dt)) + 1
    idoverlay.scene = scenes[i]
    renderables = [roadway, scenes[i], idoverlay]
    render(renderables, camera=camera)
end
#md write("straight_roadway_animated.gif", animation) # hide
#md nothing # hide
#md # ![three vehicles animated](straight_roadway_animated.gif)

# In order to inspect the simulation interactively, we can use the `Interact` package

#md using Interact
#md using Blink
#md using ElectronDisplay
#md
#md w = Window()
#md viz = @manipulate for step in 1 : length(scenes)
#md     renderables = [roadway, scenes[step], IDOverlay(scene=scenes[step], roadway=roadway)]
#md     render(renderables, camera=camera)
#md end
#md body!(w, viz)
