import LinearAlgebra: norm, normalize, dot
import Random: seed!

@test invlerp(0.0,1.0,0.5) ≈ 0.5
@test invlerp(0.0,1.0,0.4) ≈ 0.4
@test invlerp(0.0,1.0,0.7) ≈ 0.7
@test invlerp(10.0,20.0,12.0) ≈ 0.2

@testset "VecE2" begin
    include("test_vecE2.jl")
end

@testset "VecE3" begin
    include("test_vecE3.jl")
end

@testset "VecSE2" begin
    include("test_vecSE2.jl")
end

@testset "quaternions" begin
    include("test_quat.jl")
end

@testset "coordinate transforms" begin 
    include("test_coordinate_transforms.jl")
end

@testset "geom" begin 
    include("test_geomE2.jl")
end

@testset "diff" begin 
    include("test_diff.jl")
end
