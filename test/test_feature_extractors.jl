type DumbFeatureExtractor <: AbstractFeatureExtractor
end

let

    roadway = gen_straight_roadway(3, 1000.0, lane_width=1.0)
    rec = SceneRecord(1, 0.1, 5)
    update!(rec, Scene([
            Vehicle(VehicleState(VecSE2( 0.0,0.0,0.0), roadway, 10.0), VehicleDef(1, AgentClass.CAR, 5.0, 2.0)),
            Vehicle(VehicleState(VecSE2(10.0,0.0,0.0), roadway, 10.0), VehicleDef(2, AgentClass.CAR, 5.0, 2.0)),
        ]))

    ext = DumbFeatureExtractor()
    @test_throws Exception length(ext)
    @test_throws Exception pull_features!(ext, Array(Float64, 0), rec, roadway, 1)

    ext = FeatureExtractor(AbstractFeature[SPEED, POSFT])
    @test length(ext) == 2
    @test pull_features!(ext, [NaN,NaN], rec, roadway, 1) == [10.0, 0.0]
end