using Revise 
using AutomotiveDrivingModels

roadway = gen_straight_roadway(4, 100.0)

scene = Scene([Vehicle(VehicleState(VecSE2( 0.0,0.0,0.0), roadway, 10.0), VehicleDef(AgentClass.CAR, 5.0, 2.0), 1),
       Vehicle(VehicleState(VecSE2(10.0,0.0,0.0), roadway, 10.0), VehicleDef(AgentClass.CAR, 5.0, 2.0), 2),
        ])

pos1 = extract_feature(AutomotiveDrivingModels.PosGFeature(), scene, scene[1])
pos2 = extract_feature(AutomotiveDrivingModels.PosGFeature(), scene, scene[2])
poss = extract_feature(AutomotiveDrivingModels.PosGFeature(), scene, [1,2])
poss[1] == pos1
poss[2] == pos2

dfs1 = extract_feature("posg", [scene, scene], [1,2]) # dataframes
dfs2 = extract_feature("posf", [scene, scene], [1,2]) # dataframes

dfs = AutomotiveDrivingModels.extract_features(["posg", "posf"], [scene, scene], [1,2])

df1 = DataFrame(posg = [f[1] for f in feature_dict])


dfs = extract_features((posgx, posgy, distance_to(1)), [scene, scene], [1,2])



using DataFrames
