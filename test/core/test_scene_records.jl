let
    trajdata = get_test_trajdata()

    rec = SceneRecord(5)
    @test record_length(rec) == 5
    @test length(rec) == 0

    scene = get!(Scene(), trajdata, 1)
    update!(rec, scene)
    @test length(rec) == 1
    @test rec[1,0].state == get_vehiclestate(trajdata, 1, 1)
    @test rec[1,0].def == get_vehicledef(trajdata, 1)
    @test rec[2,0].state == get_vehiclestate(trajdata, 2, 1)
    @test rec[2,0].def == get_vehicledef(trajdata, 2)

    get!(scene, trajdata, 2)
    update!(rec, scene)
    @test length(rec) == 2
    @test rec[1,0].state == get_vehiclestate(trajdata, 1, 2)
    @test rec[1,0].def == get_vehicledef(trajdata, 1)
    @test rec[2,0].state == get_vehiclestate(trajdata, 2, 2)
    @test rec[2,0].def == get_vehicledef(trajdata, 2)
    @test rec[1,-1].state == get_vehiclestate(trajdata, 1, 1)
    @test rec[1,-1].def == get_vehicledef(trajdata, 1)
    @test rec[2,-1].state == get_vehiclestate(trajdata, 2, 1)
    @test rec[2,-1].def == get_vehicledef(trajdata, 2)

    @test  iscarinframe(rec, 1, 0)
    @test !iscarinframe(rec, 10, 0)

    @test get_index_of_first_vehicle_with_id(rec, 1, 0) == 1
    @test get_index_of_first_vehicle_with_id(rec, 2, 0) == 2
    @test get_index_of_first_vehicle_with_id(rec, 5, 0) == 0

    scene2 = get!(Scene(), rec, 0)
    @test scene2[1].state == get_vehiclestate(trajdata, 1, 2)
    @test scene2[1].def == get_vehicledef(trajdata, 1)
    @test scene2[2].state == get_vehiclestate(trajdata, 2, 2)
    @test scene2[2].def == get_vehicledef(trajdata, 2)

    get!(scene2, rec, -1)
    @test scene2[1].state == get_vehiclestate(trajdata, 1, 1)
    @test scene2[1].def == get_vehicledef(trajdata, 1)
    @test scene2[2].state == get_vehiclestate(trajdata, 2, 1)
    @test scene2[2].def == get_vehicledef(trajdata, 2)

    empty!(rec)
    @test length(rec) == 0
end
