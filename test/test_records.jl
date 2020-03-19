@testset "Conversions" begin 
    @testset "list" begin
        lrec = ListRecord(1.0, Float64, Bool, Int)
        append!(lrec.frames, [RecordFrame(1,2), RecordFrame(3,4)])
        append!(lrec.states, [RecordState(1.0,1),RecordState(2.0,2),RecordState(3.0,1),RecordState(4.0,3)])
        lrec.defs[1] = true
        lrec.defs[2] = true
        lrec.defs[3] = false

        sparsemat, id_lookup = get_sparse_lookup(lrec)
        @test sparsemat[1,1] == 1.0
        @test sparsemat[2,1] == 3.0
        @test sparsemat[1,2] == 2.0
        @test sparsemat[2,2] == 0.0
        @test sparsemat[1,3] == 0.0
        @test sparsemat[2,3] == 4.0
        @test id_lookup == Dict(2=>2,3=>3,1=>1)

        qrec = convert(QueueRecord{Entity{Float64, Bool, Int}}, lrec)
        @test qrec[-1][1] == Entity(1.0,true,1)
        @test qrec[-1][2] == Entity(2.0,true,2)
        @test qrec[ 0][1] == Entity(3.0,true,1)
        @test qrec[ 0][2] == Entity(4.0,false,3)
        @test nframes(qrec) == 2
    end

    @testset "queue" begin
        qrec = QueueRecord(Entity{Float64, Bool, Int}, 2, 0.1)
        @test nframes(qrec) == 0

        update!(qrec, Frame([Entity(1.0,true,1), Entity(2.0,true,2)]))
        update!(qrec, Frame([Entity(3.0,true,1), Entity(4.0,false,3)]))

        @test nframes(qrec) == 2

        lrec = convert(ListRecord{Float64, Bool, Int}, qrec)
        @test nframes(lrec) == 2
        @test lrec.frames == [RecordFrame(1,2), RecordFrame(3,4)]
        @test lrec.states == [RecordState(1.0,1),RecordState(2.0,2),RecordState(3.0,1),RecordState(4.0,3)]
        @test lrec.defs[1] == true
        @test lrec.defs[2] == true
        @test lrec.defs[3] == false
        @test length(lrec.defs) == 3
    end
end

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

@testset "IO" begin 
    @testset begin
        lrec = ListRecord(1.0, Float64, Bool, Int)
        append!(lrec.frames, [RecordFrame(1,2), RecordFrame(3,4)])
        append!(lrec.states, [RecordState(1.0,1),RecordState(2.0,2),RecordState(3.0,1),RecordState(4.0,3)])
        lrec.defs[1] = true
        lrec.defs[2] = true
        lrec.defs[3] = false

        file = tempname()
        open(file, "w") do io
            write(io, MIME"text/plain"(), lrec)
        end

        lrec2 = open(file, "r") do io
            read(io, MIME"text/plain"(), ListRecord{Float64, Bool, Int})
        end
        @test nframes(lrec2) == 2
        @test lrec2.frames == [RecordFrame(1,2), RecordFrame(3,4)]
        @test lrec2.states == [RecordState(1.0,1),RecordState(2.0,2),RecordState(3.0,1),RecordState(4.0,3)]
        @test lrec2.defs[1] == true
        @test lrec2.defs[2] == true
        @test lrec2.defs[3] == false
        @test length(lrec2.defs) == 3

        rm(file)
    end

    @testset begin
        frames = [
                Frame([Entity(1.0,true,1), Entity(2.0, true,2)]),
                Frame([Entity(3.0,true,1), Entity(4.0,false,3)]),
                ]

        file = tempname()
        open(file, "w") do io
            write(io, MIME"text/plain"(), frames)
        end

        frames2 = open(file, "r") do io
            read(io, MIME"text/plain"(), Vector{Frame{Entity{Float64, Bool, Int}}})
        end
        @test frames2[1][1] == Entity(1.0,true,1)
        @test frames2[1][2] == Entity(2.0,true,2)
        @test frames2[2][1] == Entity(3.0,true,1)
        @test frames2[2][2] == Entity(4.0,false,3)
        @test length(frames2) == 2

        rm(file)
    end
end

@testset "ListRecord" begin 
    @testset begin 
        rec = ListRecord(1.0, Float64, Bool, Int)
        @test get_statetype(rec) == Float64
        @test get_deftype(rec) == Bool
        @test get_idtype(rec) == Int

        append!(rec.frames, [RecordFrame(1,2), RecordFrame(3,4)])
        append!(rec.states, [RecordState(1.0,1),RecordState(2.0,2),RecordState(3.0,1),RecordState(4.0,3)])
        rec.defs[1] = true
        rec.defs[2] = true
        rec.defs[3] = false

        @test nframes(rec) == 2
        @test nstates(rec) == 4
        @test nids(rec) == 3

        @test sort!(get_ids(rec)) == [1,2,3]
        @test nth_id(rec, 1, 1) == 1
        @test nth_id(rec, 1, 2) == 2
        @test nth_id(rec, 2, 1) == 1
        @test nth_id(rec, 2, 2) == 3

        @test length(ListRecordStateByIdIterator(rec, 1)) == 2
        @test length(ListRecordStateByIdIterator(rec, 2)) == 1
        @test length(ListRecordStateByIdIterator(rec, 3)) == 1

        @test collect(ListRecordStateByIdIterator(rec, 1)) == [(1,1.0),(2,3.0)]
        @test collect(ListRecordStateByIdIterator(rec, 2)) == [(1,2.0)]
        @test collect(ListRecordStateByIdIterator(rec, 3)) == [(2,4.0)]

        @test get_time(rec, 1) == 0.0
        @test get_time(rec, 2) == 1.0
        @test get_time(rec, 3) == 2.0
        @test get_timestep(rec) == 1.0
        @test get_elapsed_time(rec, 1, 10) == 9*1.0

        @test findfirst_frame_with_id(rec, 1) == 1
        @test findfirst_frame_with_id(rec, 3) == 2
        @test findfirst_frame_with_id(rec, 4) == nothing

        @test findlast_frame_with_id(rec, 2) == 1
        @test findlast_frame_with_id(rec, 3) == 2
        @test findlast_frame_with_id(rec, 4) == nothing

        @test length(ListRecordFrameIterator(rec)) == 2
        len = 0
        for frame in ListRecordFrameIterator(rec)
            len += 1
        end
        @test len == 2

        subrec = get_subinterval(rec, 2, 2)
        @test nframes(subrec) == 1
        @test get(subrec, 1, 1) == Entity(3.0, true, 1)
        @test get(subrec, 3, 1) == Entity(4.0, false, 3)
    end


    @testset begin 
        rec = ListRecord(1.0, Float64, Bool, Int)
        @test nframes(rec) == 0

        scene = EntityFrame(Float64, Bool, Int, 2)
        push!(scene, Entity(1.0, true, 1))
        push!(scene, Entity(2.0, true, 2))
        push!(rec, scene)

        @test nframes(rec) == 1

        empty!(scene)
        push!(scene, Entity(3.0, true, 1))
        push!(scene, Entity(4.0, false, 3))
        push!(rec, scene)
        @test nframes(rec) == 2

        @test get(rec, 1, 1) == Entity(1.0, true, 1)
        @test get(rec, 1, 2) == Entity(3.0, true, 1)
        @test get(rec, 2, 1) == Entity(2.0, true, 2)
        @test get(rec, 3, 2) == Entity(4.0, false, 3)
    end
end

@testset "records_iterator" begin
    q = QueueRecord(Int64, 100, 1., 10)
    data = [[1,2,3],[4,5,6],[7,8,9]]
    for d in data
        update!(q, Frame(d))
    end
    @test all([data[i] == [x[1], x[2], x[3]] for (i, x) in enumerate(q)])
    @test iterate(q,1) === nothing
end
