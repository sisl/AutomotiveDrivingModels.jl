# Driving in a Stadium

This example demonstrates a 2D driving simulation where cars drive around a three-lane stadium.
The entities are defined by the types:

- `S` - `VehicleState`, containing the vehicle position (both globally and relative to the lane) and speed 
- `D` - `VehicleDef`, containing length, width, and class
- `I` - `Symbol`, a unique label for each vehicle

The environment is represented by a `Roadway` object which 
allows to define roads consisting of multiple lanes based on the RNDF format.

We load relevant modules and generate a 3-lane stadium roadway:

```@example driving_in_circles
using AutomotiveDrivingModels
using AutoViz
using Distributions

roadway = gen_stadium_roadway(3)
snapshot = render([roadway], camera=FitToContentCamera(.1))
write("stadium.svg", snapshot) # hide
```
![three lane stadium](stadium.svg)

As a next step, let's populate a scene with vehicles

```@example driving_in_circles
w = DEFAULT_LANE_WIDTH
scene = Frame([
    Entity(VehicleState(VecSE2(10.0,  -w, 0.0), roadway, 29.0), VehicleDef(), :alice),
    Entity(VehicleState(VecSE2(40.0, 0.0, 0.0), roadway, 22.0), VehicleDef(), :bob),
    Entity(VehicleState(VecSE2(30.0, -2w, 0.0), roadway, 27.0), VehicleDef(), :charlie),
])
car_colors = get_pastel_car_colors(scene)
renderables = [
    roadway,
    (FancyCar(car=veh, color=car_colors[veh.id]) for veh in scene)...
]
snapshot = render(renderables, camera=FitToContentCamera(.1))
write("stadium_with_cars.svg", snapshot) # hide
```
![stadium with cars](stadium_with_cars.svg)

We can assign driver models to each agent and simulate the scenario.

```@example driving_in_circles
timestep = 0.1
nticks = 300

models = Dict{Symbol, DriverModel}()
models[:alice] = LatLonSeparableDriver( # produces LatLonAccels
    ProportionalLaneTracker(), # lateral model
    IntelligentDriverModel(), # longitudinal model
)
# TODO: Tim2DDriver makes use of QueueRecord structure for scenes
# see tim_2d_driver.jl line 47
# see issue https://github.com/sisl/AutomotiveDrivingModels.jl/issues/62
# models[:bob] = Tim2DDriver(
#     timestep, mlane = MOBIL(timestep),
# )
models[:bob] = LatLonSeparableDriver( # produces LatLonAccels
    ProportionalLaneTracker(), # lateral model
    IntelligentDriverModel(), # longitudinal model
)
models[:charlie] = StaticDriver{AccelTurnrate, MvNormal}(
    MvNormal([0.0,0.0], [1.0,0.1])
)

set_desired_speed!(models[:alice],   12.0)
set_desired_speed!(models[:bob],     10.0)
set_desired_speed!(models[:charlie],  8.0)

scenes = simulate(scene, roadway, models, nticks, timestep)
nothing # hide
```

An animation of the simulation can be rendered using the `Reel` package

```@example driving_in_circles
using Reel
using Printf

camera = TargetFollowCamera(:alice; zoom=10.)

animation = roll(fps=1.0/timestep, duration=nticks*timestep) do t, dt
    i = Int(floor(t/dt)) + 1
    update_camera!(camera, scenes[i])
    renderables = [
        roadway,
        (FancyCar(car=veh, color=car_colors[veh.id]) for veh in scenes[i])...,
        RenderableOverlay(IDOverlay(x_off=-2, y_off=1), scenes[i], roadway),
        TextOverlay(text=[@sprintf("time: %.1fs", t)], pos=VecE2(40,40), font_size=24)
    ]
    render(renderables, camera=camera)
end

write("animated_stadium.gif", animation) # hide
```

![animated stadium with cars](animated_stadium.gif)

Alternatively, one can also use the `Interact` framework to inspect the simulation record interactively.

```julia
using Interact
using Reel
using Blink

w = Window()
camera = FitToContentCamera(.1)
viz = @manipulate for step in 1 : length(scenes)
    render([roadway, scenes[step]], camera=camera)
end
body!(w, viz)
```

The simulation results can be saved to a text file. We achieve this by first converting the list of scene to a `Trajdata` type and then exporting it.


### TODO: fix writing of Trajdata
```@example driving_in_circles
open("2Dstadium_listrec.txt", "w") do io
    @warn "TODO: need to fix bug in write(trajdata)"
    # write(io, MIME"text/plain"(), Trajdata(scenes, timestep))
end
```
The trajectory data file can be loaded in a similar way.

```@example driving_in_circles
# listrec = open("2Dstadium_listrec.txt", "r") do io
#     @warn "TODO: need to fix bug in write(trajdata)"
#     read(io, MIME"text/plain"(), Trajdata)
# end

# p = plot(listrec)
# TODO: maybe do something useful with the loaded data, like plot the speed over time or something
# write(p) # hide
```

