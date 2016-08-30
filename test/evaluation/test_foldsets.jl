let
    assignment = [1, 2, 1, 2, 1, 3]
    foldset1 = foldset_match(assignment, 1)
    foldset2 = foldset_withhold(assignment, 2)

    @test collect(foldset1) == find(assignment .== 1)
    @test collect(foldset2) == find(v-> v != 2, assignment)
end