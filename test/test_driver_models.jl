type FakeDriverModel <: DriverModel{FakeDriveAction} end

let
    roadway = get_test_roadway()
    trajdata = get_test_trajdata(roadway)
    veh = get(trajdata, 1, 1)

    model = FakeDriverModel()
    reset_hidden_state!(model)
    observe!(model, Scene(), roadway, 1)
    prime_with_history!(model, trajdata, roadway, 1, 2, 1)

    @test get_name(model) == "???"
    @test action_type(model) <: FakeDriveAction
    @test_throws ErrorException rand(model)
    @test_throws ErrorException pdf(model, FakeDriveAction())
    @test_throws ErrorException logpdf(model, FakeDriveAction())
end