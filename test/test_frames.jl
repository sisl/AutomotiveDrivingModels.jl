@testset "Frame" begin
    @testset begin
        frame = Frame([1,2,3])
        @test length(frame) == 3
        @test capacity(frame) == 3
        for i in 1 : 3
            @test frame[i] == i
        end

        frame = Frame([1,2,3], capacity=5)
        @test length(frame) == 3
        @test capacity(frame) == 5
        for i in 1 : 3
            @test frame[i] == i
        end

        @test_throws ErrorException Frame([1,2,3], capacity=2)

        frame = Frame(Int)
        @test length(frame) == 0
        @test capacity(frame) > 0

        frame = Frame(Int, 2)
        @test length(frame) == 0
        @test capacity(frame) == 2

        frame[1] = 999
        frame[2] = 888

        @test frame[1] == 999
        @test frame[2] == 888
        @test length(frame) == 0 # NOTE: length does not change
        @test capacity(frame) == 2

        empty!(frame)
        @test length(frame) == 0
        @test capacity(frame) == 2

        push!(frame, 999)
        push!(frame, 888)
        @test length(frame) == 2
        @test capacity(frame) == 2

        @test_throws BoundsError push!(frame, 777)

        frame = Frame([999,888])
        deleteat!(frame, 1)
        @test length(frame) == 1
        @test capacity(frame) == 2
        @test frame[1] == 888

        deleteat!(frame, 1)
        @test length(frame) == 0
        @test capacity(frame) == 2

        frame = Frame([1,2,3])
        frame2 = copy(frame)
        for i in 1 : 3
            @test frame[i] == frame2[i]
        end
        frame[1] = 999
        @test frame2[1] == 1
    end

    @testset begin 
        frame = EntityFrame(Int, Float64, String)
        @test eltype(frame) == Entity{Int,Float64,String}

        frame = Frame([Entity(1,1,"A"),Entity(2,2,"B"),Entity(3,3,"C")])
        @test  in("A", frame)
        @test  in("B", frame)
        @test  in("C", frame)
        @test !in("D", frame)
        @test findfirst("A", frame) == 1
        @test findfirst("B", frame) == 2
        @test findfirst("C", frame) == 3
        @test findfirst("D", frame) == nothing
        @test id2index(frame, "A") == 1
        @test id2index(frame, "B") == 2
        @test id2index(frame, "C") == 3
        @test_throws BoundsError id2index(frame, "D")

        frame = Frame([Entity(1,1,"A"),Entity(2,2,"B"),Entity(3,3,"C")])
        @test get_by_id(frame, "A") == frame[1]
        @test get_by_id(frame, "B") == frame[2]
        @test get_by_id(frame, "C") == frame[3]

        delete!(frame, Entity(2,2,"B"))
        @test frame[1] == Entity(1,1,"A")
        @test frame[2] == Entity(3,3,"C")
        @test length(frame) == 2

        delete!(frame, "A")
        @test frame[1] == Entity(3,3,"C")
        @test length(frame) == 1

        frame = Frame([Entity(1,1,1),Entity(2,2,2)], capacity=3)
        @test get_first_available_id(frame) == 3
    end
end
