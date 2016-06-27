let
    trajdata = get_test_trajdata()

    scene = get!(Scene(), trajdata, 1)
    @test length(scene) == 2
    for (i,veh) in enumerate(scene)
        @test scene[i].state == get_vehiclestate(trajdata, i, 1)
        @test scene[i].def == get_vehicledef(trajdata, i)
    end

    scene2 = Scene(deepcopy(scene.vehicles), 2)
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

    @test iscarinframe(scene, 1)
    @test iscarinframe(scene, 2)
    @test !iscarinframe(scene, 3)

    veh = get_vehicle(scene, 2)
    @test veh.state == get_vehiclestate(trajdata, 2, 1)
    @test veh.def == get_vehicledef(trajdata, 2)
end
