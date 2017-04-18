type DumbFeatureExtractor <: AbstractFeatureExtractor
end

let

    roadway = gen_straight_roadway(3, 1000.0, lane_width=1.0)
    rec = SceneRecord(1, 0.1, 5)
    update!(rec, Scene([
            Vehicle(VehicleState(VecSE2( 0.0,0.0,0.0), roadway, 10.0), VehicleDef(AgentClass.CAR, 5.0, 2.0), 1),
            Vehicle(VehicleState(VecSE2(10.0,0.0,0.0), roadway, 10.0), VehicleDef(AgentClass.CAR, 5.0, 2.0), 2),
        ]))

    ext = DumbFeatureExtractor()
    @test rec_length(ext) == 1
    @test_throws Exception length(ext)
    @test_throws Exception pull_features!(ext, Array(Float64, 0), rec, roadway, 1)

    ext = FeatureExtractor(AbstractFeature[SPEED, POSFT], 2)
    @test rec_length(ext) == 2
    @test length(ext) == 2
    @test pull_features!(ext, [NaN,NaN], rec, roadway, 1) == [10.0, 0.0]

    subset_ext = SubsetExtractor(ext, [1])
    @test rec_length(subset_ext) == 2
    @test length(subset_ext) == 1
    @test pull_features!(subset_ext, [NaN], rec, roadway, 1) == [10.0]

    subset_ext.subset[1] = 2
    @test pull_features!(subset_ext, [NaN], rec, roadway, 1) == [0.0]

    stan_ext = StandardizingExtractor(ext, [1.0, 2.0], [1.0, 4.0])
    @test rec_length(stan_ext) == 2
    @test length(stan_ext) == 2
    @test pull_features!(stan_ext, [NaN,NaN], rec, roadway, 1) == [(10.0-1.0)/1.0, (0.0-2.0)/4.0]
end