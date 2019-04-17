#=
 Benchmark two collision checkers:
collision_checker(veh1::Vehicle, veh2::Vehicle) from AutomotivePOMDPs using parallel axis theorem
is_colliding(veh1::Vehicle, veh2::Vehicle) from AutomotiveDrivingModels using Minkowski sums
=#

using AutomotiveDrivingModels
using BenchmarkTools

## Helpers

const ROADWAY = gen_straight_roadway(1, 20.0)

function create_vehicle(x::Float64, y::Float64, θ::Float64 = 0.0; id::Int64=1)
    s = VehicleState(VecSE2(x, y, θ), ROADWAY, 0.0)
    return Vehicle(s, VehicleDef(), id)
end

const VEH_REF = create_vehicle(0.0, 0.0, 0.0, id=1)

function no_collisions_PA(xspace, yspace, thetaspace)
    for (x,y,th) in zip(xspace, yspace, thetaspace)
        veh2 = create_vehicle(x,y,th,id=2)
        @assert !collision_checker(VEH_REF, veh2) "Error for veh ", veh2
    end
end

function collisions_PA(xspace, yspace, thetaspace)
    for (x,y,th) in zip(xspace, yspace, thetaspace)
        veh2 = create_vehicle(x,y,th,id=2)
        @assert collision_checker(VEH_REF, veh2) "Error for veh ", veh2
    end
end

function no_collisions_MI(xspace, yspace, thetaspace)
    for (x,y,th) in zip(xspace, yspace, thetaspace)
        veh2 = create_vehicle(x,y,th,id=2)
        @assert !is_colliding(VEH_REF, veh2) "Error for veh ", veh2
    end
end

function collisions_MI(xspace, yspace, thetaspace)
    for (x,y,th) in zip(xspace, yspace, thetaspace)
        veh2 = create_vehicle(x,y,th,id=2)
        @assert is_colliding(VEH_REF, veh2) "Error for veh ", veh2
    end
end

function consistency(xspace, yspace, thetaspace)
    for (x,y,th) in zip(xspace, yspace, thetaspace)
        veh2 = create_vehicle(x,y,th,id=2)
        @assert is_colliding(VEH_REF, veh2) == collision_checker(VEH_REF, veh2) "Error for veh ", veh2
    end
end

## Series of test 1: Far field 
thetaspace = LinRange(0.0, 2*float(pi), 10)
xspace = LinRange(15.0, 300.0, 10)
yspace = LinRange(10.0, 300.0, 10)

no_collisions_PA(xspace, yspace, thetaspace)
no_collisions_MI(xspace, yspace, thetaspace)
consistency(xspace, yspace, thetaspace)

println(" -- TEST 1 : Far Range Collision Detection -- ")
println(" ")
println("Parallel Axis performance on far range negative collisions: ")
@btime no_collisions_PA($xspace, $yspace, $thetaspace)
println(" ")
println("Minkowski Sum performance on far range negative collisions: ")
@btime no_collisions_MI($xspace, $yspace, $thetaspace)
println(" ")
println(" ------------------------------------------")
println(" ")
## Series of test 2: Superposition 
thetaspace = LinRange(0.0, 2*float(pi), 10)
xspace = LinRange(-2.0, 2.0, 10)
yspace = LinRange(1.5, 1.5, 10)

collisions_PA(xspace, yspace, thetaspace)
collisions_MI(xspace, yspace, thetaspace)
consistency(xspace, yspace, thetaspace)

println(" -- TEST 2 : Superposition Collision Detection -- ")
println(" ")
println("Parallel Axis performance on positive collisions: ")
@btime collisions_PA($xspace, $yspace, $thetaspace)
println(" ")
println("Minkowski Sum performance on positive collisions: ")
@btime collisions_MI($xspace, $yspace, $thetaspace)
println(" ")
println(" ------------------------------------------")
println(" ")
## Series of test 3: Close field 
thetaspace = [0.0]
yspace = [2.5]
xspace = LinRange(-2.0, 2.0, 10)

no_collisions_PA(xspace, yspace, thetaspace)
no_collisions_MI(xspace, yspace, thetaspace)
consistency(xspace, yspace, thetaspace)

println(" -- TEST 3 : Close Range Collision Detection -- ")
println(" ")
println("Parallel Axis performance on close range negative collisions: ")
@btime no_collisions_PA($xspace, $yspace, $thetaspace)
println(" ")
println("Minkowski Sum performance on close range negative collisions: ")
@btime no_collisions_MI($xspace, $yspace, $thetaspace)
println(" ")
println(" ------------------------------------------")
println(" ")