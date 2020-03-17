using Revise 
using AutomotiveDrivingModels
using DataFrames
using Test

roadway = gen_straight_roadway(4, 100.0)

scene = Scene([Vehicle(VehicleState(VecSE2( 0.0,0.0,0.0), roadway, 10.0), VehicleDef(AgentClass.CAR, 5.0, 2.0), 1),
       Vehicle(VehicleState(VecSE2(10.0,0.0,0.0), roadway, 10.0), VehicleDef(AgentClass.CAR, 5.0, 2.0), 2),
        ])

# test each feature individually 
pos1 = extract_feature(featuretype(posgx), posgx, roadway, [scene], 1)
pos2 = extract_feature(featuretype(posgx), posgx, roadway, [scene], 2)
poss = extract_feature(featuretype(posgx), posgx, roadway, [scene], [1,2])
@test poss[1][1] == pos1[1]
@test poss[2][1] == pos2[2]
@test pos1[1] == 0.0
@test pos2[2] == 10.0

scene = Scene([Vehicle(VehicleState(VecSE2(1.1,1.2,0.0), roadway, 10.0), VehicleDef(AgentClass.CAR, 5.0, 2.0), 1)])
posy = extract_feature(featuretype(posgy), posgy, roadway, [scene], 1)
@test posy[1] == 1.2
posθ = extract_feature(featuretype(posgθ), posgθ, roadway, [scene], 1)
@test posθ[1] == 0.0
poss = extract_feature(featuretype(posfs), posfs, roadway, [scene], 1)
@test poss[1] == 1.1
post = extract_feature(featuretype(posft), posft, roadway, [scene], 1)
@test post[1] == 1.2
posϕ = extract_feature(featuretype(posfϕ), posfϕ, roadway, [scene], 1)
@test posϕ[1] == 0.0


scene = Scene([Vehicle(VehicleState(VecSE2(1.1,1.2,0.0), roadway, 10.0), VehicleDef(AgentClass.CAR, 5.0, 2.0), 1),
               Vehicle(VehicleState(VecSE2(1.5,1.2,0.0), roadway, 10.0), VehicleDef(AgentClass.CAR, 5.0, 2.0), 2)])
coll = extract_feature(featuretype(iscolliding), iscolliding, roadway, [scene], 1)
@test coll[1]

d = extract_feature(featuretype(distance_to(1)), distance_to(1), roadway, [scene], 1)
@test d[1] == 0.0
d = extract_feature(featuretype(distance_to(2)), distance_to(2), roadway, [scene], 1)
@test d[1] ≈ 0.4

df = extract_feature(featuretype(turn_rate_g), turn_rate_g, roadway, [scene, scene], [1])
@test df[1][1] === missing
@test df[1][2] == 0.0

# TODO add neigh_dist_front, dist_front

# extract multiple features 
roadway = gen_straight_roadway(3, 1000.0, lane_width=1.0)
scene = Scene([
            Vehicle(VehicleState(VecSE2( 0.0,0.0,0.0), roadway, 10.0), VehicleDef(AgentClass.CAR, 5.0, 2.0), 1),
            Vehicle(VehicleState(VecSE2(10.0,0.0,0.0), roadway, 10.0), VehicleDef(AgentClass.CAR, 5.0, 2.0), 2),
        ])

dfs = extract_features((iscolliding, markerdist_left, markerdist_right), roadway, [scene], [1,2])
@test isapprox(dfs[1].markerdist_right[1], 0.5)
@test isapprox(dfs[2].markerdist_right[1], 0.5)
@test isapprox(dfs[1].markerdist_left[1], 0.5)
@test isapprox(dfs[2].markerdist_left[1], 0.5)

# integration testing, all the features 
feature_list = (posgx, posgy, posgθ, posfs, posft, posfϕ, vel, velfs, velft, velgx, velgy, 
                time_to_crossing_right, time_to_crossing_left, 
                estimated_time_to_lane_crossing, iswaiting, acc, accfs, accft, jerk, 
                jerkft, turn_rate_g, turn_rate_f, isbraking, isaccelerating)
dfs = extract_features(feature_list, roadway, [scene, scene], [1,2])
for id=[1,2]
    @test ncol(dfs[id]) == length(feature_list)
    @test nrow(dfs[id]) == 2
end
