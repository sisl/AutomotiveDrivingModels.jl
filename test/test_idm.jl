using Revise
using AutomotiveDrivingModels
using AutoViz
using Random

roadway = gen_straight_roadway(1, 500.0)

num_veh = 2
scene = Scene(num_veh)

models = Dict{Int, DriverModel}()

k_spd = 1.0 
politeness = 3.0 

# 1: first vehicle, moving the fastest
mlane = MOBIL(.1, politeness = politeness)
mlon = IntelligentDriverModel(k_spd = k_spd, σ = 0.0)
models[1] = Tim2DDriver(.1, mlane = mlane, mlon = mlon)
road_idx = RoadIndex(proj(VecSE2(0.0, 0.0, 0.0), roadway))
base_speed = 10.
veh_state = VehicleState(Frenet(road_idx, roadway), roadway, base_speed)
veh_def = VehicleDef(AgentClass.CAR, 5., 2.)
push!(scene, Vehicle(veh_state, veh_def, 1))
# 2: second vehicle, in the middle, moving at intermediate speed
mlane = MOBIL(.1, politeness = politeness)
mlon = IntelligentDriverModel(k_spd = k_spd, σ = 0.0)
models[2] = Tim2DDriver(.1, mlane = mlane, mlon = mlon)
base_speed = 0.
road_pos = 8.
veh_state = VehicleState(Frenet(road_idx, roadway), roadway, base_speed)
veh_state = move_along(veh_state, roadway, road_pos)
veh_def = VehicleDef(AgentClass.CAR, 5., 2.)
push!(scene, Vehicle(veh_state, veh_def, 2))

rec = SceneRecord(500, 0.1, num_veh)
prime_time = .2
rng = MersenneTwister(1)

s = deepcopy(scene)
simulate!(Any, rec, s, roadway, models, 1)
render(s, roadway, cam=CarFollowCamera(1, 10.0))

