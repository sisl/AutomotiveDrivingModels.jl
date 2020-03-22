# # Feature Extraction 

#md # [![](https://img.shields.io/badge/show-nbviewer-579ACA.svg)](@__NBVIEWER_ROOT_URL__/notebooks/feature_extraction.ipynb)

# In this example we demonstrate how to extract feature from trajectories using AutomotiveDrivingModels 

# **Load a Dataset**
# First let's create a synthetic dataset.

using AutomotiveDrivingModels
using AutoViz
AutoViz.colortheme["background"] = colorant"white"; # hide
using Random

roadway = roadway = gen_straight_roadway(3, 1000.0)
veh_state = VehicleState(Frenet(roadway[LaneTag(1,2)], 0.0), roadway, 10.)
veh1 = Entity(veh_state, VehicleDef(), "bob")
veh_state = VehicleState(Frenet(roadway[LaneTag(1,2)], 20.0), roadway, 2.)
veh2 = Entity(veh_state, VehicleDef(), "alice")

dt = 0.5
n_steps = 10
models = Dict{String, DriverModel}()
models["bob"] = Tim2DDriver(mlane=MOBIL())
set_desired_speed!(models["bob"], 10.0)
models["alice"] = Tim2DDriver(mlane=MOBIL())
set_desired_speed!(models["alice"], 2.0)

scene = Scene([veh1, veh2])
scenes = simulate(scene, roadway, models, n_steps, dt)
camera = SceneFollowCamera()
update_camera!(camera, scene)
snapshot = render([roadway, scene], camera=camera)
#md write("feature_initial.svg", snapshot) # hide
#md # ![initial state of feature extraction scenario](feature_initial.svg)

#md nothing # hide

# One can also load the data from the Stadium tutorial

#md load data from stadium tutorial 
#md scenes = open("2Dstadium_listrec.txt", "r") do io 
#md     read(io, Vector{EntityScene{VehicleState, VehicleDef, String}})
#md end

# **Extract features from a recorded trajectory** 
# Recorded trajectories are expected to be vectors of `Scene`s where each element correspond to one time step. 
# To extract features, one can use the `extract_features` function which takes as input 
# a list of feature we want to extract and the list of vehicle ids for which we want those features.
# For this example, let's first query two features, the longitudinal and lateral position of Bob, and whether or not Bob is colliding:

dfs = extract_features((posfs, posft, iscolliding), roadway, scenes, ["bob"])
dfs["bob"]

# To query features for all traffic participants we can just add their ID to the list: 

dfs = extract_features((posfs, posft, iscolliding), roadway, scenes, ["bob", "alice"])
dfs["alice"]

# The output is a dictionary mapping ID to dataframes. To learn more about DataFrames visit [DataFrames.jl](https://github.com/JuliaData/DataFrames.jl).

# For the list of all possible features available see the documentation. 
# Features are generally just function. AutomotiveDrivingModels provides some convenience
# to automatically generate feature function like `distance_to_$x`
# The `distance_to` function takes as input a vehicle ID and returns a function to extract 
# the distance between the queried vehicle and the vehicle given to `distance_to`

distance_to("alice")

# we can use this newly generated funciton in the feature extraction pipeline 

dfs = extract_features((distance_to_alice, posfs), roadway, scenes, ["bob"])
dfs["bob"].distance_to_alice[1] # distance between Bob and Alice in the first scene.
