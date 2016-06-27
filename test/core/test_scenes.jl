# push!(trajdata.states, TrajdataState(1, VehicleState(VecSE2(0.0,0.0,0.0), roadway, 10.0))) # car 1, frame 1
# push!(trajdata.states, TrajdataState(2, VehicleState(VecSE2(3.0,0.0,0.0), roadway, 20.0))) # car 2, frame 1
# push!(trajdata.states, TrajdataState(1, VehicleState(VecSE2(1.0,0.0,0.0), roadway, 10.0))) # car 1, frame 2
# push!(trajdata.states, TrajdataState(2, VehicleState(VecSE2(5.0,0.0,0.0), roadway, 20.0))) # car 2, frame 2

let
    trajdata = get_test_trajdata()

    scene = get!(Scene(), trajdata, 1)
    @test length(scene) == 2
    for (i,veh) in enumerate(scene)
        @test scene[i].state == get_vehiclestate(trajdata, i, 1)
        @test scene[i].def == get_vehicledef(trajdata, i)
    end

    scene2 = Scene(-1, deepcopy(scene.vehicles), 2)
    @test length(scene2) == 2
    for (i,veh) in enumerate(scene2)
        @test scene2[i].state == get_vehiclestate(trajdata, i, 1)
        @test scene2[i].def == get_vehicledef(trajdata, i)
    end

    empty!(scene2)
    @test length(scene2) == 0

    copy!(scene2, scene)
    @test length(scene2) == 2
    for (i,veh) in enumerate(scene2)
        @test scene2[i].state == get_vehiclestate(trajdata, i, 1)
        @test scene2[i].def == get_vehicledef(trajdata, i)
    end

    deleteat!(scene2, 1)
    @test length(scene2) == 1
    @test scene2[1].state == get_vehiclestate(trajdata, 2, 1)
    @test scene2[1].def == get_vehicledef(trajdata, 2)

    @test get_index_of_first_vehicle_with_id(scene, 1) == 1
    @test get_index_of_first_vehicle_with_id(scene, 2) == 2
end