type DumbFoldAssigner <: FoldAssigner end
let
    @test_throws Exception assign_folds(DumbFoldAssigner(), 5, 2)

    assignment = assign_folds(RandomFoldAssigner(), 4, 2)
    @test length(find(assignment .== 1)) == 2
    @test length(find(assignment .== 2)) == 2

    assignment = assign_folds(RandomFoldAssigner(true), 5, 2)
    @test length(find(assignment .== 1)) == 3
    @test length(find(assignment .== 2)) == 2

    assignment = assign_folds(RandomFoldAssigner(false), 5, 2)
    @test length(find(assignment .== 1)) == 2
    @test length(find(assignment .== 2)) == 2

    @test assign_folds(OrderedFoldAssigner(), 4, 2) == [1, 1, 2, 2]
    @test assign_folds(OrderedFoldAssigner(true), 4, 3) == [1, 1, 2, 3]
    @test assign_folds(OrderedFoldAssigner(true), 5, 2) == [1, 1, 1, 2, 2]
    @test assign_folds(OrderedFoldAssigner(false), 5, 2) == [1, 1, 2, 2, 0]
end