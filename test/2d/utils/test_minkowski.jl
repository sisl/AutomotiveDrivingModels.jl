let
    @test AutomotiveDrivingModels.cyclic_shift_left!([1,2,3,4], 1, 4) == [2,3,4,1]
    @test AutomotiveDrivingModels.cyclic_shift_left!([1,2,3,4], 2, 4) == [3,4,1,2]
    @test AutomotiveDrivingModels.cyclic_shift_left!([1,2,3,4,5,6,7], 1, 4) == [2,3,4,1,5,6,7]
    @test AutomotiveDrivingModels.cyclic_shift_left!([1,2,3,4,5,6,7], 2, 4) == [3,4,1,2,5,6,7]

    roadway = get_test_roadway()
    trajdata = get_test_trajdata(roadway)
    scene = Scene()

    col = get_first_collision(get!(scene, trajdata, 1))
    @test col.is_colliding
    @test col.A == 1
    @test col.B == 2

    @test get_first_collision(get!(scene, trajdata, 2), CPAMemory()).is_colliding == true
end