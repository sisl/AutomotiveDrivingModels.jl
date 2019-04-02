const ROADWAY = gen_straight_roadway(1, 20.0)

function create_vehicle(x::Float64, y::Float64, θ::Float64 = 0.0; id::Int64=1)
    s = VehicleState(VecSE2(x, y, θ), ROADWAY, 0.0)
    return Vehicle(s, VehicleDef(), id)
end

const VEH_REF = create_vehicle(0.0, 0.0, 0.0, id=1)

function no_collisions(xspace, yspace, thetaspace)
    for (x,y,th) in zip(xspace, yspace, thetaspace)
        veh2 = create_vehicle(x,y,th,id=2)
        @assert !collision_checker(VEH_REF, veh2) "Error for veh ", veh2
    end
    return true
end

function collisions(xspace, yspace, thetaspace)
    for (x,y,th) in zip(xspace, yspace, thetaspace)
        veh2 = create_vehicle(x,y,th,id=2)
        @assert collision_checker(VEH_REF, veh2) "Error for veh ", veh2
    end
    return true
end

@testset "Minkowski" begin
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
    scene = Scene()
    @test is_collision_free(get!(scene, trajdata, 1)) == false
    @test is_collision_free(get!(scene, trajdata, 1), [1]) == false
    @test is_colliding(scene[1], scene[2])
    @test get_distance(scene[1], scene[2]) == 0
    @test is_collision_free(get!(scene, trajdata, 3)) 
    get_distance(scene[1], scene[2])
    
    roadway = gen_straight_roadway(2, 100.0)
    veh1 = Vehicle(VehicleState(VecSE2(0.0, 0.0, 0.0), roadway, 10.0), VehicleDef(), 1)
    veh2 = Vehicle(VehicleState(VecSE2(10.0, 0.0, 0.0), roadway, 5.0), VehicleDef(), 2)
    scene = Scene([veh1, veh2])
    @test is_collision_free(scene)
    @test get_distance(veh1, veh2) ≈ 6.0
end

@testset "parallel axis" begin 
    ## Series of test 1: Far field 
    thetaspace = LinRange(0.0, 2*float(pi), 10)
    xspace = LinRange(15.0, 300.0, 10)
    yspace = LinRange(10.0, 300.0, 10)
    @test no_collisions(xspace, yspace, thetaspace)

    ## Series of test 2: Superposition 
    thetaspace = LinRange(0.0, 2*float(pi), 10)
    xspace = LinRange(-2.0, 2.0, 10)
    yspace = LinRange(1.5, 1.5, 10)

    @test collisions(xspace,yspace,thetaspace)

    ## Close but no collisions 
    veh2 = create_vehicle(0.0, 2.2, 0.0, id=2)
    @test !collision_checker(VEH_REF, veh2)
    veh2 = create_vehicle(3.1, 2.2, 1.0pi, id=2)
    @test !collision_checker(VEH_REF, veh2)
end