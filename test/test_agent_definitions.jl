struct DummyDefinition <: AbstractAgentDefinition end
@testset "agent definition" begin 
    d = DummyDefinition()
    @test_throws ErrorException length(d)
    @test_throws ErrorException width(d)
    @test_throws ErrorException class(d)

    d = VehicleDef()
    @test class(d) == AgentClass.CAR
    @test length(d) == 4.0
    @test width(d) == 1.8

    d2 = BicycleModel(VehicleDef())
    @test class(d2) == class(d)
    @test length(d2) == length(d)
    @test width(d2) == width(d)
end