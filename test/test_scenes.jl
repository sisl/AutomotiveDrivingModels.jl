@testset "Scene" begin
    @testset begin
        scene = Scene([1,2,3])
        @test length(scene) == 3
        @test capacity(scene) == 3
        for i in 1 : 3
            @test scene[i] == i
        end

        scene = Scene([1,2,3], capacity=5)
        @test length(scene) == 3
        @test capacity(scene) == 5
        for i in 1 : 3
            @test scene[i] == i
        end

        @test_throws ErrorException Scene([1,2,3], capacity=2)

        scene = Scene(Int)
        @test length(scene) == 0
        @test capacity(scene) > 0
        @test lastindex(scene) == scene.n

        scene = Scene(Int, 2)
        @test length(scene) == 0
        @test capacity(scene) == 2

        scene[1] = 999
        scene[2] = 888

        @test scene[1] == 999
        @test scene[2] == 888
        @test length(scene) == 0 # NOTE: length does not change
        @test capacity(scene) == 2

        empty!(scene)
        @test length(scene) == 0
        @test capacity(scene) == 2

        push!(scene, 999)
        push!(scene, 888)
        @test length(scene) == 2
        @test capacity(scene) == 2

        @test_throws BoundsError push!(scene, 777)

        scene = Scene([999,888])
        deleteat!(scene, 1)
        @test length(scene) == 1
        @test capacity(scene) == 2
        @test scene[1] == 888

        deleteat!(scene, 1)
        @test length(scene) == 0
        @test capacity(scene) == 2

        scene = Scene([1,2,3])
        scene2 = copy(scene)
        for i in 1 : 3
            @test scene[i] == scene2[i]
        end
        scene[1] = 999
        @test scene2[1] == 1
    end

    @testset begin 
        scene = EntityScene(Int, Float64, String)
        scene = EntityScene(Int, Float64, String, 10)
        @test eltype(scene) == Entity{Int,Float64,String}
        @test capacity(scene) == 10

        scene = Scene([Entity(1,1,"A"),Entity(2,2,"B"),Entity(3,3,"C")])
        @test  in("A", scene)
        @test  in("B", scene)
        @test  in("C", scene)
        @test !in("D", scene)
        @test findfirst("A", scene) == 1
        @test findfirst("B", scene) == 2
        @test findfirst("C", scene) == 3
        @test findfirst("D", scene) == nothing
        @test id2index(scene, "A") == 1
        @test id2index(scene, "B") == 2
        @test id2index(scene, "C") == 3
        @test_throws BoundsError id2index(scene, "D")

        scene = Scene([Entity(1,1,"A"),Entity(2,2,"B"),Entity(3,3,"C")])
        @test get_by_id(scene, "A") == scene[1]
        @test get_by_id(scene, "B") == scene[2]
        @test get_by_id(scene, "C") == scene[3]

        delete!(scene, Entity(2,2,"B"))
        @test scene[1] == Entity(1,1,"A")
        @test scene[2] == Entity(3,3,"C")
        @test length(scene) == 2

        delete!(scene, "A")
        @test scene[1] == Entity(3,3,"C")
        @test length(scene) == 1

        scene = Scene([Entity(1,1,1),Entity(2,2,2)], capacity=3)
        @test get_first_available_id(scene) == 3

    end
end
